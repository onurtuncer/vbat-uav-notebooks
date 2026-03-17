"""
forward_flight_power.py  --  Forward Flight Power-to-Weight & Size Matching Diagram
====================================================================================

Implements Gudmundsson / Sadraey T/W-wing-loading equations
(2-50 through 2-60) as documented in DLR-IB-FT-BS-2024-106 2.6.

THEORY
------
In forward flight the aircraft's propulsion system must overcome drag.
The key insight of the Size Matching Diagram is that ALL performance
requirements can be expressed as:

        T/W  =  f( W/S, aero params, mission params )

i.e. each flight phase yields a CURVE of required T/W as a function of
wing loading.  Plotting all curves together reveals a feasible region
(above every curve, left of the stall line).  The design point is where
the MAXIMUM required T/W is MINIMISED -- this simultaneously minimises
propulsion system size and battery mass.

PARABOLIC DRAG POLAR
    The entire method rests on a parabolic polar:
        CD = CD0 + k*CL^2
        k  = 1 / (pi*AR*e)

    This is valid for subsonic, attached-flow conditions -- exactly the
    regime of a small UAV in cruise.

FLIGHT-PHASE EQUATIONS
-----------------------

1. TAKE-OFF  (ground roll, eq. 2-50 / Gudmundsson):
        T/W = q*CD_To/(W/S)  +  mu*(1 - q*CL_To/(W/S))  +  V_To^2/(2g*s_g)
   where q = 0.5rhoV_To^2 evaluated at 0.7*V_To (representative ground speed),
   CD_To, CL_To are lift/drag coefficients at liftoff attitude,
   mu = rolling resistance, s_g = take-off ground run.
   For a tail-sitter with VTOL capability this phase is replaced by
   VTOL hover; it is still included for completeness.

2. CLIMB  (eq. 2-51 / Gudmundsson):
        T/W = RoC/V_climb  +  q*CD_min/(W/S)  +  k*(W/S)/q
   where V_climb is the best-climb speed,
         q = 0.5rhoV_climb^2,
         CD_min ~ CD0 (minimum drag, wings level).

   The best-climb speed can be estimated as V_climb = sqrt(2/rho * sqrt(k/3CD0) * W/S)
   -- this speed actually DEPENDS on W/S so the climb curve is not trivial.

3. CRUISE  (eq. 2-52):
        T/W = q*CD0/(W/S)  +  k*(W/S)/q
   where q = 0.5rhoV_cruise^2.  This is the pure drag equation in level flight.

4. SERVICE CEILING  (eq. 2-53 / Gudmundsson):
        T/W = RoC_ceiling/V_ceiling  +  4*sqrt(k*CD0/3)
   Minimum power-required speed at ceiling altitude.
   Simplified to: T/W ~ RoC_ceil/V_mp_ceiling  +  4*sqrt(k*CD0/3)

5. MAXIMUM SPEED  (eq. 2-59 / Sadraey):
        T/W = rho*Vmax^2*CD0 / (2*W/S)  +  2k*(W/S) / (rho*Vmax^2)

6. STALL BOUNDARY  (eq. 2-55 / 2-60):
        (W/S)_stall = 0.5*rho*V_stall^2*CL_max
   This is a VERTICAL LINE on the diagram -- all chosen wing loadings must
   be left of this line.

DESIGN POINT SELECTION
    After computing all curves over a sweep of W/S values, the design point
    is found analytically:  the wing loading that minimises the UPPER ENVELOPE
    (max of all T/W curves) is the optimal design wing loading.

References
----------
  Gudmundsson (2014), General Aviation Aircraft Design, eqs as above.
  Sadraey (2013), Aircraft Design, eqs as above.
  DLR-IB-FT-BS-2024-106, 2.6.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

from .models import Aerodynamics, Environment, Mission


# ---------------------------------------------
#  Additional forward-flight inputs not in Aerodynamics
# ---------------------------------------------
@dataclass
class ForwardFlightParams:
    """
    Extra parameters needed only for the forward-flight power module.
    These are early-design estimates; update them as the design matures.
    Loaded from config/forward_flight_params.yaml via from_yaml().
    """
    # Take-off / ground roll (ignored for pure VTOL, kept for completeness)
    mu:          float = 0.04    # runway rolling resistance coefficient [-]
    CL_To:       float = 0.8     # lift coeff at take-off rotation       [-]
    CD_To:       float = 0.05    # drag coeff at take-off (gear down)    [-]
    s_g:         float = 50.0    # take-off ground run                   [m]
    V_To_factor: float = 1.2     # V_To = factor x V_stall               [-]

    # Climb
    RoC_climb:   float = 2.5     # forward-flight climb rate             [m/s]

    # Service ceiling
    h_ceiling:   float = 3000.0  # service ceiling                       [m]
    RoC_ceiling: float = 0.5     # rate of climb at ceiling              [m/s]

    # Wing loading sweep range for the Size Matching Diagram
    WS_min: float = 30.0    # [N/m^2]
    WS_max: float = 500.0   # [N/m^2]
    WS_n:   int   = 1000    # number of points in sweep

    @classmethod
    def from_yaml(cls, path) -> "ForwardFlightParams":
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(
            mu          = float(data["mu"]),
            CL_To       = float(data["CL_To"]),
            CD_To       = float(data["CD_To"]),
            s_g         = float(data["s_g"]),
            V_To_factor = float(data["V_To_factor"]),
            RoC_climb   = float(data["RoC_climb"]),
            h_ceiling   = float(data["h_ceiling"]),
            RoC_ceiling = float(data["RoC_ceiling"]),
            WS_min      = float(data["WS_min"]),
            WS_max      = float(data["WS_max"]),
            WS_n        = int(data["WS_n"]),
        )


# ---------------------------------------------
#  Individual T/W constraint functions
# ---------------------------------------------

def _q(rho: float, V: float) -> float:
    """Dynamic pressure 0.5rhoV^2  [N/m^2 = Pa]."""
    return 0.5 * rho * V**2


def tw_takeoff(
    WS: np.ndarray,
    aero: Aerodynamics,
    ff: ForwardFlightParams,
    env: Environment,
) -> np.ndarray:
    """
    Eq. (2-50): T/W for take-off ground roll as a function of W/S.

    T/W = q*CD_To/(W/S)  +  mu*(1 - q*CL_To/(W/S))  +  VTo^2/(2g*sg)

    Dynamic pressure evaluated at 0.7*V_To (average ground speed).
    V_To = V_To_factor * V_stall.
    """
    V_stall = ff.V_To_factor * env.g   # placeholder -- actual stall speed depends on W/S
    # stall speed per wing loading: V_s = sqrt(2*WS/(rho*CL_max))
    V_s   = np.sqrt(2.0 * WS / (env.rho * aero.CL_max))
    V_To  = ff.V_To_factor * V_s
    q_To  = _q(env.rho, 0.7 * V_To)
    tw    = (q_To * ff.CD_To / WS
             + ff.mu * (1.0 - q_To * ff.CL_To / WS)
             + V_To**2 / (2.0 * env.g * ff.s_g))
    return np.maximum(tw, 0.0)


def tw_climb(
    WS: np.ndarray,
    aero: Aerodynamics,
    ff: ForwardFlightParams,
    env: Environment,
    V_climb: float | None = None,
) -> np.ndarray:
    """
    Eq. (2-51): T/W for climb phase.

        T/W = RoC/V  +  q*CD0/(W/S)  +  k*(W/S)/q

    If V_climb is None, the best-climb speed is computed from W/S:
        V_best_climb = sqrt( (2/rho) * sqrt(k/(3*CD0)) * W/S )

    This makes the climb curve depend on W/S in a non-trivial way.
    """
    k = aero.k
    if V_climb is None:
        # best-climb speed varies with W/S -- compute element-wise
        V = np.sqrt((2.0 / env.rho) * np.sqrt(k / (3.0 * aero.CD0)) * WS)
    else:
        V = V_climb * np.ones_like(WS)

    q = _q(env.rho, V)
    return ff.RoC_climb / V + q * aero.CD0 / WS + k * WS / q


def tw_cruise(
    WS: np.ndarray,
    aero: Aerodynamics,
    mission: Mission,
    env: Environment,
) -> np.ndarray:
    """
    Eq. (2-52): T/W for cruise (steady level flight).

        T/W = q*CD0/(W/S)  +  k*(W/S)/q
    """
    q = _q(env.rho, mission.V_cruise)
    return q * aero.CD0 / WS + aero.k * WS / q


def tw_ceiling(
    WS: np.ndarray,
    aero: Aerodynamics,
    ff: ForwardFlightParams,
    env: Environment,
) -> np.ndarray:
    """
    Eq. (2-53) simplified: T/W at service ceiling.

        T/W ~ RoC_ceiling / V_mp_ceiling  +  4*sqrt(k*CD0/3)

    V_mp (minimum power speed) at ceiling altitude depends on density
    at ceiling.  We use ISA lapse: rho_h ~ rho0*(1 - 2.256e-5*h)^4.256
    """
    # ISA density at ceiling
    rho_h = env.rho * (1.0 - 2.256e-5 * ff.h_ceiling)**4.256
    rho_h = max(rho_h, 0.1)   # guard against extreme altitudes

    k = aero.k
    # Minimum-power speed (depends on W/S)
    V_mp = np.sqrt((2.0 / rho_h) * np.sqrt(k / (3.0 * aero.CD0)) * WS)
    tw   = ff.RoC_ceiling / V_mp + 4.0 * np.sqrt(k * aero.CD0 / 3.0)
    return tw


def tw_vmax(
    WS: np.ndarray,
    aero: Aerodynamics,
    env: Environment,
) -> np.ndarray:
    """
    Eq. (2-59): T/W at maximum speed (Sadraey).

        T/W = rho*Vmax^2*CD0 / (2*W/S)  +  2k*(W/S) / (rho*Vmax^2)
    """
    Vmax  = aero.V_max
    rho   = env.rho
    return (rho * Vmax**2 * aero.CD0 / (2.0 * WS)
            + 2.0 * aero.k * WS / (rho * Vmax**2))


def ws_stall(aero: Aerodynamics, env: Environment) -> float:
    """
    Eq. (2-55/2-60): Maximum allowable wing loading from stall constraint.

        (W/S)_stall = 0.5*rho*V_stall^2*CL_max

    This is a HARD UPPER LIMIT on W/S (vertical boundary on the diagram).
    """
    return 0.5 * env.rho * aero.V_stall**2 * aero.CL_max


# ---------------------------------------------
#  Size Matching Diagram computation
# ---------------------------------------------
@dataclass
class SizeMatchingResult:
    """Output of compute_size_matching_diagram."""
    WS_array:       np.ndarray   # wing loading sweep          [N/m^2]
    tw_takeoff:     np.ndarray   # T/W curve -- take-off        [-]
    tw_climb:       np.ndarray   # T/W curve -- climb           [-]
    tw_cruise:      np.ndarray   # T/W curve -- cruise          [-]
    tw_ceiling:     np.ndarray   # T/W curve -- ceiling         [-]
    tw_vmax:        np.ndarray   # T/W curve -- max speed       [-]
    tw_envelope:    np.ndarray   # upper envelope (max of all) [-]
    WS_stall:       float        # stall limit                 [N/m^2]
    WS_design:      float        # optimal wing loading        [N/m^2]
    TW_design:      float        # T/W at design point         [-]
    P_W_design:     float        # required P/W at design pt   [W/N]


def compute_size_matching_diagram(
    aero:    Aerodynamics,
    mission: Mission,
    ff:      ForwardFlightParams,
    env:     Environment = Environment(),
    eta_propulsive: float = 0.75,   # overall propulsive efficiency (motorxESCxprop)
) -> SizeMatchingResult:
    """
    Sweep wing loading and compute all T/W constraint curves.

    The design point is chosen as the W/S that minimises the upper
    envelope of all T/W curves, subject to  W/S <= (W/S)_stall.

    Parameters
    ----------
    aero            : Aerodynamics dataclass
    mission         : Mission dataclass
    ff              : ForwardFlightParams dataclass
    env             : Environment dataclass
    eta_propulsive  : overall chain efficiency used to convert T/W -> P/W
                      P/W = T/W * V_cruise / eta_propulsive

    Returns
    -------
    SizeMatchingResult
    """
    WS = np.linspace(ff.WS_min, ff.WS_max, ff.WS_n)

    curves = {
        "takeoff": tw_takeoff(WS, aero, ff, env),
        "climb":   tw_climb(WS, aero, ff, env),
        "cruise":  tw_cruise(WS, aero, mission, env),
        "ceiling": tw_ceiling(WS, aero, ff, env),
        "vmax":    tw_vmax(WS, aero, env),
    }

    WS_stall_val = ws_stall(aero, env)

    # Upper envelope
    envelope = np.maximum.reduce(list(curves.values()))

    # Feasible region: W/S <= W/S_stall
    feasible_mask = WS <= WS_stall_val
    if not np.any(feasible_mask):
        raise ValueError(
            f"No feasible wing loading found below stall limit "
            f"{WS_stall_val:.1f} N/m^2. Check CL_max or V_stall."
        )

    feasible_envelope = np.where(feasible_mask, envelope, np.inf)
    idx_design = int(np.argmin(feasible_envelope))

    WS_design = float(WS[idx_design])
    TW_design = float(envelope[idx_design])

    # Power-to-weight:  P/W = (T/W * V) / eta
    P_W_design = TW_design * mission.V_cruise / eta_propulsive

    return SizeMatchingResult(
        WS_array    = WS,
        tw_takeoff  = curves["takeoff"],
        tw_climb    = curves["climb"],
        tw_cruise   = curves["cruise"],
        tw_ceiling  = curves["ceiling"],
        tw_vmax     = curves["vmax"],
        tw_envelope = envelope,
        WS_stall    = WS_stall_val,
        WS_design   = WS_design,
        TW_design   = TW_design,
        P_W_design  = P_W_design,
    )


# ---------------------------------------------
#  Cruise power -- convenience function
# ---------------------------------------------

def cruise_power_W(
    mass_total_kg: float,
    aero:          Aerodynamics,
    mission:       Mission,
    eta_propulsive: float = 0.75,
    env:           Environment = Environment(),
) -> Dict[str, float]:
    """
    Simple cruise power calculation from L/D model (legacy sizing_old approach).

        P_aero  = W * V / (L/D)
        P_elec  = P_aero / eta

    Returns dict with P_aero_W, P_elec_W, thrust_N, drag_N.
    """
    W_N     = mass_total_kg * env.g
    P_aero  = W_N * mission.V_cruise / aero.LD
    P_elec  = P_aero / eta_propulsive
    D_N     = W_N / aero.LD
    return {
        "P_aero_W":  P_aero,
        "P_elec_W":  P_elec,
        "thrust_N":  D_N,
        "drag_N":    D_N,
    }


if __name__ == "__main__":
    from .models import Aerodynamics, Mission, Environment, ForwardFlightParams

    env  = Environment()
    aero = Aerodynamics(LD=8.0, CD0=0.025, AR=6.0, e=0.80,
                        CL_max=1.4, V_stall=12.0, V_max=35.0)
    mission = Mission(t_hover=120, t_cruise=1800, V_cruise=20.0,
                      rate_of_climb=2.5, reserve_factor=1.2)
    ff = ForwardFlightParams(RoC_climb=2.5, h_ceiling=3000, RoC_ceiling=0.5)

    res = compute_size_matching_diagram(aero, mission, ff, env)
    print(f"Design wing loading : {res.WS_design:.1f} N/m^2")
    print(f"Design T/W          : {res.TW_design:.4f}")
    print(f"Design P/W          : {res.P_W_design:.4f} W/N")
    print(f"Stall limit W/S     : {res.WS_stall:.1f} N/m^2")