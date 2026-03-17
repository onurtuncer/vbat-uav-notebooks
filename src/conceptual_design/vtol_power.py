"""
vtol_power.py  --  VTOL Power-to-Weight ratio estimator
=======================================================

Implements equations (2-61) through (2-64) from Jae-Hyun et al.,
as documented in DLR-IB-FT-BS-2024-106 (Poelma, 2024).

THEORY
------
During vertical flight a VTOL aircraft must generate thrust equal to
(or greater than) its weight.  The electrical power required to do this
comes from three physical phenomena:

  1. INDUCED POWER  --  work done to accelerate air downward through the
     rotor disk.  From momentum (actuator disk) theory:

         v_i = (RoC/2) + sqrt( (RoC/2)^2 + (W/S_rotor)/(2rho) )

         P_induced / W  =  RoC/2  +  (1/2)*sqrt( RoC^2 + 2*DL/rho )

     where DL = W/S_rotor is the disk loading.  At hover RoC=0, this
     collapses to the classic hover result:

         P_hover / W  =  sqrt( DL / (2rho) )

     This term is divided by Figure-of-Merit (FoM) to convert ideal
     momentum power to real rotor power.

  2. PROFILE POWER  --  drag on the spinning blades.  For a rotor with
     solidity sigma (blade area / disk area) and blade drag coefficient CD_blade:

         P_profile / W  =  rho * Vtip^3 * sigma * CD_blade / (8 * DL)

     Vtip comes from an empirical RPM-diameter relationship:
         RPM = k_rpm * D^exp_rpm          (eq. 2-64)
         Vtip = pi * RPM * D / 60          (eq. 2-63)

  3. PARASITE / BODY DRAG TERMS  --  extra aerodynamic drag on the
     fuselage and wing during climb.  Two terms appear in eq. (2-61):

         C1 = rho * RoC^3 / DL
         C2 = rho * RoC^3 / (S_ratio * WL)

     where S_ratio = S_total/S_wing accounts for the total wetted area
     exposed to the climb airstream.

The full VTOL takeoff P/W is therefore:
    P/W_VTOL = (Induced term) + (Profile term) + C1 + C2

All outputs are in [W/N]  ==  [m/s]  (power per unit weight).
Multiply by MTOW in [N] to get absolute power in [W].

References
----------
  Jae-Hyun et al.  (eq. 2-61 to 2-64)
  DLR-IB-FT-BS-2024-106, Poelma 2024, 2.7.1
"""

from __future__ import annotations
from dataclasses import dataclass, field
from math import sqrt, pi
from typing import Dict

from .models import Environment, PropulsiveSystemParameters


# ---------------------------------------------
#  Parameter container  (mirrors PropulsiveSystemParameters
#  but adds rho0 for convenience)
# ---------------------------------------------
@dataclass
class VTOLParams:
    """
    Physical constants and empirical rotor parameters used in the
    VTOL power equations.

    Sensible defaults are taken from the DLR report 2.7.1 and from
    Jae-Hyun et al.
    """
    rho0:     float = 1.225   # air density at sea level     [kg/m^3]
    FoM:      float = 0.70    # Figure of Merit              [-]  (0.65-0.80 typical)
    CD_blade: float = 0.01    # blade drag coefficient       [-]
    sigma:    float = 0.077   # rotor solidity               [-]
    S_ratio:  float = 1.3     # S_total / S_wing             [-]  (1.3-1.4 for UAVs)
    rpm_k:    float = 2762.786  # RPM coefficient            [rpm*m^|rpm_exp|]
    rpm_exp:  float = -0.932    # RPM diameter exponent      [-]

    @classmethod
    def from_propulsive(cls, p: PropulsiveSystemParameters,
                        env: Environment | None = None) -> "VTOLParams":
        """Build VTOLParams from the project's PropulsiveSystemParameters dataclass."""
        rho0 = env.rho if env is not None else 1.225
        return cls(
            rho0     = rho0,
            FoM      = p.fom,
            CD_blade = p.Cd_blade,
            sigma    = p.sigma_rotor,
            S_ratio  = p.s_ratio,
            rpm_k    = p.k_rpm,
            rpm_exp  = p.exp_rpm,
        )


# ---------------------------------------------
#  Helper functions
# ---------------------------------------------

def rpm_from_diameter(D_rotor_m: float, p: VTOLParams) -> float:
    """
    Equation (2-64): empirical RPM-diameter relationship.

        RPM = k_rpm * D^rpm_exp

    For small rotors (D ~ 0.2-1.0 m) this gives RPM in the typical
    2 000-8 000 range.
    """
    if D_rotor_m <= 0:
        raise ValueError("D_rotor_m must be > 0")
    return p.rpm_k * (D_rotor_m ** p.rpm_exp)


def vtip_from_rpm_and_diameter(rpm: float, D_rotor_m: float) -> float:
    """
    Equation (2-63): blade tip speed from RPM and rotor diameter.

        Vtip = pi * RPM * D / 60

    Units: [m/s].  Tip speeds of 150-220 m/s are typical for small rotors.
    """
    return pi * rpm * D_rotor_m / 60.0


# ---------------------------------------------
#  Core P/W functions
# ---------------------------------------------

def vtol_hover_power_to_weight(
    disk_loading_N_m2: float,
    p: VTOLParams = VTOLParams(),
) -> float:
    """
    Equation (2-62): hover power-to-weight ratio.

        P/W_hover = sqrt( DL / (2*rho) ) / FoM

    This is the ideal actuator-disk hover power (momentum theory) divided
    by the Figure of Merit to account for real rotor losses.

    Parameters
    ----------
    disk_loading_N_m2 : float
        Disk loading  W/S_rotor  [N/m^2].  Typical range: 80-300 N/m^2 for
        small VTOL UAVs.
    p : VTOLParams

    Returns
    -------
    float  -- P/W in [W/N]  (= m/s numerically)
    """
    if disk_loading_N_m2 <= 0:
        raise ValueError("disk_loading_N_m2 must be > 0")
    if p.FoM <= 0:
        raise ValueError("FoM must be > 0")

    return sqrt(disk_loading_N_m2 / (2.0 * p.rho0)) / p.FoM


def vtol_climb_power_to_weight(
    RoC_mps: float,
    disk_loading_N_m2: float,
    wing_loading_N_m2: float,
    D_rotor_m: float,
    p: VTOLParams = VTOLParams(),
) -> float:
    """
    Equation (2-61): VTOL climb / takeoff power-to-weight ratio.

    Four additive terms:

        Term A  [induced + climb]
            = RoC/2 + (1/2)*sqrt( RoC^2 + 2*DL/rho )

            The full expression is (T*v_i)/W where v_i is the inflow
            velocity.  For climb at RoC, actuator disk theory gives:
                v_i = RoC/2 + sqrt( (RoC/2)^2 + T/(2rhoA) )
            which simplifies to the form above when T~W.

        Term B  [blade profile drag]
            = rho*Vtip^3*sigma*CD_blade / (8*DL)

        Term C1  [fuselage parasite drag in climb airstream]
            = rho*RoC^3 / DL

        Term C2  [wing parasite drag in climb airstream]
            = rho*RoC^3 / (S_ratio * WL)

    Parameters
    ----------
    RoC_mps           : vertical rate of climb   [m/s]
    disk_loading_N_m2 : W/S_rotor                [N/m^2]
    wing_loading_N_m2 : W/S_wing                 [N/m^2]
    D_rotor_m         : rotor diameter            [m]
    p                 : VTOLParams

    Returns
    -------
    float  -- P/W in [W/N]
    """
    if disk_loading_N_m2 <= 0:
        raise ValueError("disk_loading_N_m2 must be > 0")
    if wing_loading_N_m2 <= 0:
        raise ValueError("wing_loading_N_m2 must be > 0")

    rpm  = rpm_from_diameter(D_rotor_m, p)
    Vtip = vtip_from_rpm_and_diameter(rpm, D_rotor_m)

    # Term A -- induced + climb velocity  [m/s = W/N]
    A = 0.5 * RoC_mps + 0.5 * sqrt(RoC_mps**2 + 2.0 * disk_loading_N_m2 / p.rho0)

    # Term B -- rotor profile drag  [W/N]
    B = (p.rho0 * Vtip**3 * p.sigma * p.CD_blade) / (8.0 * disk_loading_N_m2)

    # Term C1 -- fuselage drag in vertical flow  [W/N]
    C1 = (p.rho0 * RoC_mps**3) / disk_loading_N_m2

    # Term C2 -- wing drag in vertical flow  [W/N]
    C2 = (p.rho0 * RoC_mps**3) / (p.S_ratio * wing_loading_N_m2)

    return A + B + C1 + C2


# ---------------------------------------------
#  Convenience wrapper
# ---------------------------------------------

def vtol_power_requirements(
    weight_N:          float,
    RoC_mps:           float,
    disk_loading_N_m2: float,
    wing_loading_N_m2: float,
    D_rotor_m:         float,
    p:                 VTOLParams = VTOLParams(),
) -> Dict[str, float]:
    """
    Compute both hover and climb P/W and return absolute powers [W].

    Returns a dict with keys:
        P_W_hover        [W/N]   hover power-to-weight
        P_W_climb        [W/N]   VTOL climb power-to-weight
        P_hover_W        [W]     absolute hover power
        P_climb_W        [W]     absolute VTOL climb power
        P_design_W       [W]     design power = max(hover, climb)
        rpm_rotor        [rpm]   rotor RPM
        Vtip_mps         [m/s]   blade tip speed
        disk_loading     [N/m^2]  (echo back)
    """
    if weight_N <= 0:
        raise ValueError("weight_N must be > 0")

    P_W_hover = vtol_hover_power_to_weight(disk_loading_N_m2, p)
    P_W_climb = vtol_climb_power_to_weight(
        RoC_mps, disk_loading_N_m2, wing_loading_N_m2, D_rotor_m, p
    )

    rpm  = rpm_from_diameter(D_rotor_m, p)
    Vtip = vtip_from_rpm_and_diameter(rpm, D_rotor_m)

    return {
        "P_W_hover_W_per_N":  P_W_hover,
        "P_W_climb_W_per_N":  P_W_climb,
        "P_hover_W":          P_W_hover * weight_N,
        "P_climb_W":          P_W_climb * weight_N,
        "P_design_W":         max(P_W_hover, P_W_climb) * weight_N,
        "rpm_rotor":          rpm,
        "Vtip_mps":           Vtip,
        "disk_loading_N_m2":  disk_loading_N_m2,
    }


if __name__ == "__main__":
    # Quick smoke-test with representative V-BAT-scale numbers
    p = VTOLParams(FoM=0.70, CD_blade=0.01, sigma=0.077, S_ratio=1.3)
    W_N = 5.0 * 9.80665   # 5 kg MTOW
    res = vtol_power_requirements(
        weight_N=W_N, RoC_mps=2.5,
        disk_loading_N_m2=150.0,
        wing_loading_N_m2=200.0,
        D_rotor_m=0.28,
        p=p,
    )
    print("VTOL Power Requirements (5 kg UAV)")
    print("-" * 40)
    for k, v in res.items():
        print(f"  {k:<28s}: {v:>10.4f}")