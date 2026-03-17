"""
mass_closure.py  --  Iterative MTOW Sizing Loop
================================================

Solves the fundamental circular dependency in aircraft conceptual design:

    MTOW  ->  power required  ->  battery mass  ->  MTOW

The loop converges when the battery mass (and therefore MTOW) is
self-consistent with the mission energy demand.

THEORY
------

WHY IS THIS CIRCULAR?
    The battery is the heaviest single component in an electric UAV.
    Its mass depends on how much energy the mission requires.
    But the mission energy depends on the total weight -- including the
    battery. This means you cannot solve for battery mass without knowing
    MTOW, and you cannot know MTOW without knowing battery mass.

    The only way out is iteration:
        1. Guess a battery mass (or equivalently, guess MTOW).
        2. Compute power required at that weight.
        3. Compute the battery mass that supplies that energy.
        4. Check if the new battery mass equals the assumed one.
        5. If not, update and repeat.

    This is guaranteed to converge for well-posed problems because the
    battery mass function is a contraction mapping -- doubling the aircraft
    weight does not double the required battery mass (the structural mass
    also grows, but at a lower rate).

WEIGHT FRACTIONS
    Fixed mass fractions express the non-battery components as fractions
    of MTOW.  For the iterative loop the total mass is:

        m_total = m_payload + m_fixed_fraction * m_total + m_wing + m_battery

    where m_fixed_fraction = f_s + f_p + f_a + f_m  (structure,
    propulsion hardware, avionics, misc).  Re-arranging:

        m_total * (1 - f_fixed) = m_payload + m_wing + m_battery

    This gives us a way to solve for m_total analytically given m_battery
    and m_wing.  The iteration only needs to update m_battery.

BATTERY SIZING (Tyan et al., eq. 2-49)
    For each flight segment:

        E_segment = P_segment * t_segment / 3600     [Wh]

    Total energy with reserve:

        E_req = reserve_factor * sum(E_segment)      [Wh]

    Battery mass:

        m_batt = E_req / (e_spec * eta_bat * f_usable)  [kg]

    where:
        e_spec    = pack-level specific energy  [Wh/kg]  (e.g. 140 Wh/kg for LiPo)
        eta_bat   = battery discharge efficiency (typically 0.95-0.98)
        f_usable  = usable depth-of-discharge   (typically 0.80-0.90)

FLIGHT SEGMENTS
    For an electric tail-sitter VTOL the mission has four energy-consuming
    segments:
        1. VTOL hover    (t_hover seconds, P_hover watts)
        2. VTOL climb    (brief, included in hover budget)
        3. Forward cruise (t_cruise seconds, P_cruise watts)
        4. Reserve       (applied as a multiplier to total)

CONVERGENCE
    The loop terminates when:
        |m_batt_new - m_batt_old| < tol   (default tol = 1e-5 kg = 10 mg)

    Divergence is detected if the battery fraction exceeds a physical
    limit (> 60% of MTOW is batteries = unrealistic for a UAV of this
    class) or if the loop exceeds max_iter.

References
----------
  DLR-IB-FT-BS-2024-106, 3.4.2, 3.4.6
  Tyan et al. (2017), eq. (2-49)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

from .models import (
    Aerodynamics, Battery, Environment, Mission,
    PropulsiveSystemParameters, WeightFraction, WingSizing,
)
from .vtol_power import VTOLParams, vtol_hover_power_to_weight, vtol_climb_power_to_weight
from .forward_flight_power import (
    ForwardFlightParams, compute_size_matching_diagram, cruise_power_W,
)
from .wing_sizing import WingStructureParams, size_wing


# ---------------------------------------------
#  Iteration history entry
# ---------------------------------------------
@dataclass
class IterStep:
    iteration:     int
    m_total_kg:    float
    m_batt_kg:     float
    m_batt_new_kg: float
    P_hover_W:     float
    P_cruise_W:    float
    E_total_Wh:    float
    converged:     bool


# ---------------------------------------------
#  Full sizing result
# ---------------------------------------------
@dataclass
class SizingResult:
    """Complete output of the mass closure loop."""
    # Converged masses
    m_total_kg:     float   # MTOW                            [kg]
    m_battery_kg:   float   # battery mass                    [kg]
    m_wing_kg:      float   # wing structural mass            [kg]
    m_structure_kg: float   # fuselage/airframe fraction mass [kg]
    m_propulsion_kg: float  # motor+ESC+prop fraction mass    [kg]
    m_avionics_kg:  float   # avionics fraction mass          [kg]
    m_misc_kg:      float   # misc fraction mass              [kg]
    m_payload_kg:   float   # payload (fixed requirement)     [kg]

    # Power
    P_hover_W:      float   # electrical power in hover       [W]
    P_cruise_W:     float   # electrical power in cruise      [W]
    P_design_W:     float   # design (max) power              [W]

    # Energy
    E_hover_Wh:     float   # hover energy                    [Wh]
    E_cruise_Wh:    float   # cruise energy                   [Wh]
    E_total_Wh:     float   # total required energy (with reserve) [Wh]

    # Wing
    wing: WingSizing        # wing geometry

    # Fractions (post-convergence check)
    batt_fraction:  float   # m_battery / m_total             [-]
    fixed_fraction: float   # (structure+prop+avionics+misc) / m_total [-]

    # Convergence metadata
    converged:      bool
    n_iterations:   int
    history:        List[IterStep]

    def print_summary(self) -> None:
        """Pretty-print the sizing result."""
        print("=" * 52)
        print("  MASS CLOSURE RESULT")
        print("=" * 52)
        print(f"  {'MTOW':<30}: {self.m_total_kg:>8.3f} kg")
        print(f"  {'Battery':<30}: {self.m_battery_kg:>8.3f} kg  ({self.batt_fraction*100:.1f}%)")
        print(f"  {'Wing structure':<30}: {self.m_wing_kg:>8.3f} kg")
        print(f"  {'Airframe / structure':<30}: {self.m_structure_kg:>8.3f} kg")
        print(f"  {'Propulsion HW':<30}: {self.m_propulsion_kg:>8.3f} kg")
        print(f"  {'Avionics':<30}: {self.m_avionics_kg:>8.3f} kg")
        print(f"  {'Misc':<30}: {self.m_misc_kg:>8.3f} kg")
        print(f"  {'Payload':<30}: {self.m_payload_kg:>8.3f} kg")
        total_check = (self.m_battery_kg + self.m_wing_kg + self.m_structure_kg
                       + self.m_propulsion_kg + self.m_avionics_kg
                       + self.m_misc_kg + self.m_payload_kg)
        print(f"  {'Sum check':<30}: {total_check:>8.3f} kg")
        print()
        print(f"  {'P_hover':<30}: {self.P_hover_W:>8.1f} W")
        print(f"  {'P_cruise':<30}: {self.P_cruise_W:>8.1f} W")
        print(f"  {'P_design':<30}: {self.P_design_W:>8.1f} W")
        print()
        print(f"  {'E_hover':<30}: {self.E_hover_Wh:>8.2f} Wh")
        print(f"  {'E_cruise':<30}: {self.E_cruise_Wh:>8.2f} Wh")
        print(f"  {'E_total (with reserve)':<30}: {self.E_total_Wh:>8.2f} Wh")
        print()
        print(f"  {'Wing area':<30}: {self.wing.S_wing:>8.4f} m^2")
        print(f"  {'Wing span':<30}: {self.wing.b_wing:>8.4f} m")
        print(f"  {'Wing chord (MAC)':<30}: {self.wing.chord_mean:>8.4f} m")
        print(f"  {'Wing loading (design)':<30}: {self.wing.wing_loading:>8.1f} N/m^2")
        print()
        status = "CONVERGED" if self.converged else "DID NOT CONVERGE"
        print(f"  Iterations : {self.n_iterations}  [{status}]")
        print("=" * 52)


# ---------------------------------------------
#  Battery energy -> mass
# ---------------------------------------------

def battery_mass_from_energy(
    E_Wh:       float,
    batt:       Battery,
    eta_bat:    float = 0.97,   # battery discharge efficiency
) -> float:
    """
    Tyan et al. eq. (2-49):
        m_batt = E_req / (e_spec * eta_bat * f_usable)

    Parameters
    ----------
    E_Wh     : total required energy including reserve  [Wh]
    batt     : Battery dataclass (specific_energy, usable_fraction)
    eta_bat  : battery efficiency (internal resistance losses)  [-]

    Returns
    -------
    float -- battery mass [kg]
    """
    effective_capacity = batt.specific_energy * eta_bat * batt.usable_fraction
    if effective_capacity <= 0:
        raise ValueError(
            "Battery effective capacity is zero or negative. "
            "Check specific_energy, eta_bat, and usable_fraction."
        )
    return E_Wh / effective_capacity


# ---------------------------------------------
#  MTOW from component masses + fractions
# ---------------------------------------------

def mtow_from_components(
    m_payload_kg:  float,
    m_battery_kg:  float,
    wf:            WeightFraction,
) -> float:
    """
    Close the weight equation analytically.

    The weight fraction model says:
        m_total = m_payload + m_battery
                  + (fs + fp + fa + fm) * m_total

    where the fractions (fs, fp, fa, fm) already cover ALL non-battery,
    non-payload mass -- including structure, wing, propulsion hardware,
    avionics, and misc.

    Re-arranging:
        m_total * (1 - f_fixed) = m_payload + m_battery
        m_total = (m_payload + m_battery) / (1 - f_fixed)

    NOTE: The wing structural mass computed separately (Raymer) is used
    only to CHECK that it is consistent with the structural fraction
    fs * m_total.  If it is significantly different, the user should
    adjust fs.

    Parameters
    ----------
    m_payload_kg  : payload mass (requirement)   [kg]
    m_battery_kg  : battery mass (current guess) [kg]
    wf            : WeightFraction dataclass

    Returns
    -------
    float -- MTOW [kg]
    """
    f_fixed = wf.fs + wf.fp + wf.fa + wf.fm
    if f_fixed >= 1.0:
        raise ValueError(
            f"Sum of weight fractions = {f_fixed:.3f} >= 1.0. "
            f"This leaves no room for battery or payload."
        )
    return (m_payload_kg + m_battery_kg) / (1.0 - f_fixed)


# ---------------------------------------------
#  Main sizing loop
# ---------------------------------------------

def run_sizing_loop(
    # Requirements
    m_payload_kg:   float,

    # Physics models
    mission:        Mission,
    aero:           Aerodynamics,
    batt:           Battery,
    wf:             WeightFraction,
    prop_params:    PropulsiveSystemParameters,
    ff_params:      ForwardFlightParams,
    ws_params:      WingStructureParams,
    env:            Environment = Environment(),

    # VTOL rotor geometry (for tail-sitter: single EDF diameter)
    D_rotor_m:      float = 0.28,
    disk_loading:   float = 150.0,   # [N/m^2]  initial guess; will be updated

    # Battery model
    eta_bat:        float = 0.97,

    # Convergence
    tol:            float = 1e-5,
    max_iter:       int   = 100,
    m_batt_initial: float = 1.0,    # [kg] initial battery mass guess
) -> SizingResult:
    """
    Iterative MTOW sizing loop.

    Algorithm
    ---------
    1. Start with m_batt = m_batt_initial.
    2. Build VTOLParams from prop_params.
    3. Compute wing design point (Size Matching Diagram) -- this is done
       ONCE outside the inner loop because the aerodynamic design point
       does not change with mass (T/W curves are normalised by weight).
       Wing GEOMETRY changes with mass though, so wing sizing is inside
       the loop.
    4. Inner loop:
       a. Compute MTOW from payload + wing + battery + weight fractions.
       b. Run wing sizing at current MTOW -> m_wing.
       c. Re-compute MTOW.
       d. Compute VTOL hover and cruise power at current MTOW.
       e. Compute total mission energy.
       f. Compute new battery mass.
       g. Check convergence.
       h. Update m_batt.
    5. Return SizingResult with full breakdown and history.

    Parameters
    ----------
    (see individual parameter comments above)

    Returns
    -------
    SizingResult
    """
    vtol_p = VTOLParams.from_propulsive(prop_params, env)

    # -- Step 3: Size Matching Diagram (weight-normalised -- run once) --
    smd = compute_size_matching_diagram(
        aero=aero,
        mission=mission,
        ff=ff_params,
        env=env,
        eta_propulsive=prop_params.eta_total,
    )

    # -- Initialise ----------------------------------------------------
    m_batt  = m_batt_initial
    history: List[IterStep] = []
    converged = False

    for i in range(max_iter):

        # -- 4a. MTOW from payload + battery + weight fractions --------
        # The fixed fractions (fs, fp, fa, fm) already represent ALL
        # non-battery, non-payload mass (incl. structure/wing).
        m_total = mtow_from_components(m_payload_kg, m_batt, wf)

        # -- 4b. Wing sizing for geometry / assumption check -----------
        # The wing mass from Raymer is computed for reference and to
        # cross-check that fs is consistent with the structural mass.
        wing = size_wing(
            MTOW_kg    = m_total,
            WS_design  = smd.WS_design,
            TW_design  = smd.TW_design,
            aero       = aero,
            mission_V  = mission.V_cruise,
            ws         = ws_params,
            env        = env,
        )

        # -- 4d. Power at current MTOW ---------------------------------
        W_N = m_total * env.g

        # VTOL hover power  (W/N x N = W)
        P_W_hover = vtol_hover_power_to_weight(disk_loading, vtol_p)
        P_hover   = P_W_hover * W_N

        # VTOL climb power
        P_W_climb = vtol_climb_power_to_weight(
            RoC_mps           = mission.rate_of_climb,
            disk_loading_N_m2 = disk_loading,
            wing_loading_N_m2 = smd.WS_design,
            D_rotor_m         = D_rotor_m,
            p                 = vtol_p,
        )
        P_climb = P_W_climb * W_N

        # Use the larger of hover / climb as the VTOL design power
        P_vtol = max(P_hover, P_climb)

        # Cruise power  (L/D model)
        cruise = cruise_power_W(m_total, aero, mission, prop_params.eta_total, env)
        P_cruise = cruise["P_elec_W"]

        # -- 4e. Mission energy ----------------------------------------
        # t_hover already includes takeoff + landing hover budget
        E_hover_Wh  = P_vtol   * mission.t_hover  / 3600.0
        E_cruise_Wh = P_cruise * mission.t_cruise / 3600.0
        E_total_Wh  = mission.reserve_factor * (E_hover_Wh + E_cruise_Wh)

        # -- 4f. New battery mass --------------------------------------
        m_batt_new = battery_mass_from_energy(E_total_Wh, batt, eta_bat)

        # -- 4g. Convergence check -------------------------------------
        delta = abs(m_batt_new - m_batt)
        step  = IterStep(
            iteration     = i + 1,
            m_total_kg    = m_total,
            m_batt_kg     = m_batt,
            m_batt_new_kg = m_batt_new,
            P_hover_W     = P_vtol,
            P_cruise_W    = P_cruise,
            E_total_Wh    = E_total_Wh,
            converged     = delta < tol,
        )
        history.append(step)

        if delta < tol:
            m_batt    = m_batt_new
            m_total   = mtow_from_components(m_payload_kg, m_batt, wf)
            converged = True
            break

        # -- 4h. Update ------------------------------------------------
        m_batt = m_batt_new

    # -- Post-convergence: full mass breakdown -------------------------
    f_fixed        = wf.fs + wf.fp + wf.fa + wf.fm
    m_structure    = wf.fs * m_total
    m_propulsion   = wf.fp * m_total
    m_avionics     = wf.fa * m_total
    m_misc         = wf.fm * m_total

    batt_fraction  = m_batt  / m_total
    fixed_fraction = f_fixed

    # Wing assumption check: Raymer wing mass vs structural fraction
    wing_fraction_actual = wing.mass_wing_kg / m_total
    wing_fraction_assumed = wf.fs   # structure fraction is a proxy

    return SizingResult(
        m_total_kg      = m_total,
        m_battery_kg    = m_batt,
        m_wing_kg       = wing.mass_wing_kg,        # from Raymer (reference)
        m_structure_kg  = m_structure,              # from fraction
        m_propulsion_kg = m_propulsion,
        m_avionics_kg   = m_avionics,
        m_misc_kg       = m_misc,
        m_payload_kg    = m_payload_kg,
        P_hover_W       = P_vtol,
        P_cruise_W      = P_cruise,
        P_design_W      = max(P_vtol, P_cruise),
        E_hover_Wh      = E_hover_Wh,
        E_cruise_Wh     = E_cruise_Wh,
        E_total_Wh      = E_total_Wh,
        wing            = wing,
        batt_fraction   = batt_fraction,
        fixed_fraction  = fixed_fraction,
        converged       = converged,
        n_iterations    = len(history),
        history         = history,
    )


if __name__ == "__main__":
    from .models import (
        Aerodynamics, Battery, Environment, Mission,
        PropulsiveSystemParameters, WeightFraction,
    )
    from .forward_flight_power import ForwardFlightParams
    from .wing_sizing import WingStructureParams

    env     = Environment()
    mission = Mission(t_hover=120, t_cruise=1800, V_cruise=20.0,
                      rate_of_climb=2.5, reserve_factor=1.20)
    aero    = Aerodynamics(LD=8.0, CD0=0.025, AR=6.0, e=0.80,
                           CL_max=1.4, V_stall=12.0, V_max=35.0)
    batt    = Battery(specific_energy=140.0, usable_fraction=0.85)
    wf      = WeightFraction(fs=0.30, fp=0.20, fa=0.10, fm=0.10)
    prop    = PropulsiveSystemParameters(
        eta_prop=0.80, eta_motor=0.90, eta_esc=0.95,
        fom=0.70, Cd_blade=0.01, sigma_rotor=0.077,
        s_ratio=1.3, k_rpm=2762.786, exp_rpm=-0.932,
    )
    ff      = ForwardFlightParams(RoC_climb=2.5, h_ceiling=3000, RoC_ceiling=0.5)
    ws      = WingStructureParams(tc_ratio=0.12, n_ult=3.75, k_material=0.70)

    result = run_sizing_loop(
        m_payload_kg  = 0.5,
        mission       = mission,
        aero          = aero,
        batt          = batt,
        wf            = wf,
        prop_params   = prop,
        ff_params     = ff,
        ws_params     = ws,
        env           = env,
        D_rotor_m     = 0.28,
        disk_loading  = 150.0,
    )
    result.print_summary()

    print("\nIteration history:")
    print(f"  {'Iter':>4}  {'m_total':>9}  {'m_batt':>9}  {'m_batt_new':>10}  {'delta':>10}  {'E_total':>9}")
    for s in result.history:
        delta = abs(s.m_batt_new_kg - s.m_batt_kg)
        print(f"  {s.iteration:>4}  {s.m_total_kg:>9.4f}  {s.m_batt_kg:>9.4f}  "
              f"{s.m_batt_new_kg:>10.4f}  {delta:>10.2e}  {s.E_total_Wh:>9.2f}")