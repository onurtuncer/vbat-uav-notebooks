# conceptual_design/mission_profile.py
"""Mission-profile timeline and battery-state track for the design summary (NB16).

Pure post-processing of an already-converged design point: it decomposes the
flight into the legs the vehicle actually flies, and tracks pack state-of-charge
leg-by-leg.  There is NO new physics here -- the leg powers are exactly the
``P_hover_W`` (= P_vtol) and ``P_cruise_W`` the mass closure already sized
against, and the leg energies re-sum to the loop's ``E_hover + E_transition +
E_cruise``.  The only geometry added is kinematic: the vertical legs climb /
descend the cruise altitude at ``rate_of_climb``, and cruise range is
``V_cruise * t_cruise``.

Convention: state-of-charge is referenced to the SIZED pack energy
(``E_total_Wh``, which already carries the reserve factor).  So a nominal
mission starts at 100 % and ends at the reserve fraction ``1 - 1/reserve``
(~17 % for the 1.20 reserve) -- the untouched reserve is what is left on the
plot, not a depleted pack.
"""

from __future__ import annotations

from dataclasses import dataclass

from .mass_closure import SizingResult
from .models import Mission

#: One takeoff (hover->cruise) and one landing (cruise->hover) transition.
N_TRANSITIONS = 2

#: Energy-reconstruction tolerance [Wh]: the leg sum must reproduce the sizing
#: loop's segment energies to well within a rounding step.
_E_RECON_TOL_WH = 1e-6


@dataclass
class MissionLeg:
    """One flown leg with its kinematic and battery state at start/end."""

    name:        str
    phase:       str      # "vtol" | "transition" | "cruise"
    t_start_s:   float
    t_end_s:     float
    power_W:     float
    alt_start_m: float
    alt_end_m:   float
    range_start_m: float
    range_end_m:   float
    E_leg_Wh:    float
    soc_start:   float    # pack state-of-charge fraction at leg start [-]
    soc_end:     float    # pack state-of-charge fraction at leg end   [-]

    @property
    def duration_s(self) -> float:
        return self.t_end_s - self.t_start_s


@dataclass
class MissionProfile:
    """The ordered legs plus the mission-level rollup NB16 reports."""

    legs:          list[MissionLeg]
    E_pack_Wh:     float   # sized pack energy (with reserve) = result.E_total_Wh
    E_mission_Wh:  float   # consumed by the nominal (no-reserve) mission
    reserve_frac:  float   # fraction of the pack still held at end of mission
    range_km:      float   # cruise ground track
    endurance_min: float   # total mission time
    h_cruise_m:    float   # cruise altitude reached
    v_climb_mps:   float   # vertical rate of climb
    t_vclimb_s:    float   # time to climb to h_cruise
    soc_end:       float   # pack state-of-charge at end of mission [-]


def build_mission_profile(result: SizingResult, mission: Mission) -> MissionProfile:
    """Decompose the converged design point into flown legs + a battery track.

    ``result`` supplies the leg powers/energies (``P_hover_W`` is P_vtol, the
    max of hover/climb the closure used); ``mission`` supplies the timeline
    split and the cruise altitude.  Raises if the config's hover split is
    inconsistent or the climb does not fit inside the takeoff budget -- the
    profile fails loudly rather than drawing a physically impossible sketch.
    """
    P_vtol   = result.P_hover_W          # SizingResult.P_hover_W already = max(hover, climb)
    P_cruise = result.P_cruise_W
    E_pack   = result.E_total_Wh         # sized with the reserve factor

    t_to   = mission.t_hover_takeoff
    t_land = mission.t_hover_landing
    t_tr   = mission.t_transition / N_TRANSITIONS
    if abs((t_to + t_land) - mission.t_hover) > 1e-6:
        raise ValueError(
            f"hover split {t_to:.1f} + {t_land:.1f} s != t_hover "
            f"{mission.t_hover:.1f} s (fix config/mission.yaml)")

    v_climb = mission.rate_of_climb
    h_cru   = mission.h_cruise_m
    t_vclimb = h_cru / v_climb
    if t_vclimb > t_to + 1e-6:
        raise ValueError(
            f"vertical climb to {h_cru:.0f} m needs {t_vclimb:.0f} s at "
            f"{v_climb:.1f} m/s but the takeoff budget is only {t_to:.0f} s "
            f"-- lower h_cruise or raise t_hover_takeoff/rate_of_climb")

    range_cruise = mission.V_cruise * mission.t_cruise

    # (name, phase, duration, power, dalt, drange) in flown order.
    specs = [
        ("takeoff climb + hover", "vtol",       t_to,   P_vtol,   +h_cru, 0.0),
        ("transition to cruise",  "transition", t_tr,   P_vtol,   0.0,    0.0),
        ("cruise",                "cruise",     mission.t_cruise, P_cruise, 0.0, range_cruise),
        ("transition to hover",   "transition", t_tr,   P_vtol,   0.0,    0.0),
        ("descent + landing",     "vtol",       t_land, P_vtol,   -h_cru, 0.0),
    ]

    legs: list[MissionLeg] = []
    t = 0.0
    alt = 0.0
    rng = 0.0
    E_consumed = 0.0
    for name, phase, dt, power, dalt, drange in specs:
        E_leg = power * dt / 3600.0
        soc0  = 1.0 - E_consumed / E_pack
        E_consumed += E_leg
        soc1  = 1.0 - E_consumed / E_pack
        legs.append(MissionLeg(
            name=name, phase=phase,
            t_start_s=t, t_end_s=t + dt, power_W=power,
            alt_start_m=alt, alt_end_m=alt + dalt,
            range_start_m=rng, range_end_m=rng + drange,
            E_leg_Wh=E_leg, soc_start=soc0, soc_end=soc1,
        ))
        t   += dt
        alt += dalt
        rng += drange

    # The leg sum must reproduce the closure's mission energy (no reserve).
    E_mission = E_consumed
    E_segments = result.E_hover_Wh + result.E_transition_Wh + result.E_cruise_Wh
    if abs(E_mission - E_segments) > _E_RECON_TOL_WH:
        raise ValueError(
            f"leg energy {E_mission:.4f} Wh != sizing segments "
            f"{E_segments:.4f} Wh -- profile decomposition drifted from the "
            f"mass closure")

    return MissionProfile(
        legs=legs,
        E_pack_Wh=E_pack,
        E_mission_Wh=E_mission,
        reserve_frac=1.0 - E_mission / E_pack,
        range_km=range_cruise / 1000.0,
        endurance_min=t / 60.0,
        h_cruise_m=h_cru,
        v_climb_mps=v_climb,
        t_vclimb_s=t_vclimb,
        soc_end=legs[-1].soc_end,
    )
