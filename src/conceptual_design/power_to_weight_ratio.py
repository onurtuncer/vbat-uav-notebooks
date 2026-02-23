"""
VTOL power-to-weight estimator based on the (2-61)–(2-64) equations
you pasted (Jae-Hyun et al. style).

IMPORTANT NOTES (read me):
- Your pasted (2-61) appears to be a *sum of several terms* that include:
  1) induced + climb component (with RoC)
  2) profile power term ~ rho * Vtip^3 * sigma * CDblade / (8 * disk_loading)
  3) parasite-ish terms ~ rho * RoC^3 / disk_loading and / (S_ratio * wing_loading)
- The text has some duplication / line-break artifacts (e.g., RoC^3 terms repeated).
  I implement a faithful, explicit form below and keep it easy to adjust.

- All inputs are SI: N/m^2 for loadings, kg/m^3 for density, m/s for RoC, m for diameters.
- Outputs:
    * P_W_to_vtol   [W/N]  (power-to-weight for VTOL takeoff/climb segment)
    * P_W_hover_vtol[W/N]  (hover power-to-weight)
    * P_to, P_hover [W]    (absolute power if you provide weight_N)
"""

from __future__ import annotations
from dataclasses import dataclass
from math import sqrt, pi
from typing import Optional, Dict


@dataclass
class VTOLParams:
    rho0: float = 1.225              # kg/m^3, sea-level density
    CD_blade: float = 0.01           # blade drag coefficient
    sigma: float = 0.077             # rotor solidity
    S_ratio: float = 1.35            # Stot / Swing (typ 1.3-1.4)
    FoM: float = 0.75                # figure of merit (typ ~0.6-0.85 for small rotors)
    # Rotor speed model (2-64): rpm = 2762.786 * D^{-0.932}
    rpm_k: float = 2762.786
    rpm_exp: float = -0.932


def rpm_from_diameter(D_rotor_m: float, p: VTOLParams) -> float:
    """Equation (2-64)."""
    if D_rotor_m <= 0:
        raise ValueError("D_rotor_m must be > 0")
    return p.rpm_k * (D_rotor_m ** p.rpm_exp)


def vtip_from_rpm_and_diameter(rpm: float, D_rotor_m: float) -> float:
    """Equation (2-63): Vtip = pi * rpm * D / 60."""
    return pi * rpm * D_rotor_m / 60.0


def power_to_weight_vtol_takeoff(
    RoC_mps: float,
    disk_loading_N_m2: float,         # W / S_rotor  (N/m^2)
    wing_loading_N_m2: float,         # W / S_wing   (N/m^2)
    D_rotor_m: float,
    p: VTOLParams = VTOLParams(),
) -> float:
    """
    Implements a practical version of your equation (2-61).

    Returns P/W in [W/N].

    Interpreted structure:

      Term A: induced+climb style term
          A = sqrt( RoC^2 + 2*(W/S_rotor)/rho ) / 2  +  RoC/2
        (Your pasted line shows: (RoC)^2 + 1/2 * sqrt(RoC^2 + 2*(W/S_rotor)/rho0)
         which is dimensionally inconsistent if RoC^2 is outside the sqrt.
         A common form is: (RoC/2) + (1/2)*sqrt(RoC^2 + 2*T/(rho*A)).
         So we implement that dimensionally-consistent form.)

      Term B: profile power term
          B = rho * Vtip^3 * sigma * CD_blade / (8 * disk_loading)

      Term C: climb/parasite-ish terms (as in your pasted text)
          C1 = rho * RoC^3 / disk_loading
          C2 = rho * RoC^3 / ( S_ratio * wing_loading )

    If you want to match your PDF *exactly*, just edit the terms below—this layout is meant
    to be transparent and easy to change.
    """
    if disk_loading_N_m2 <= 0:
        raise ValueError("disk_loading_N_m2 must be > 0")
    if wing_loading_N_m2 <= 0:
        raise ValueError("wing_loading_N_m2 must be > 0")
    if p.rho0 <= 0:
        raise ValueError("rho0 must be > 0")
    if p.S_ratio <= 0:
        raise ValueError("S_ratio must be > 0")

    rpm = rpm_from_diameter(D_rotor_m, p)
    Vtip = vtip_from_rpm_and_diameter(rpm, D_rotor_m)

    # Term A (induced + climb) [m/s]
    A = 0.5 * RoC_mps + 0.5 * sqrt(RoC_mps**2 + 2.0 * (disk_loading_N_m2) / p.rho0)

    # Term B (profile) [W/N] after dividing by disk_loading
    B = (p.rho0 * (Vtip**3) * p.sigma * p.CD_blade) / (8.0 * disk_loading_N_m2)

    # Term C terms [W/N] (as written in your excerpt)
    C1 = (p.rho0 * (RoC_mps**3)) / (disk_loading_N_m2)
    C2 = (p.rho0 * (RoC_mps**3)) / (p.S_ratio * wing_loading_N_m2)

    # Convert Term A [m/s] to [W/N] by dividing by (??)
    # In many rotorcraft relations, induced/climb term is already a *P/T* ratio (m/s).
    # Since P/W in W/N numerically equals (m/s) because:
    #    (W) / (N) = (N*m/s)/N = m/s
    # So it is consistent to add A + B + C1 + C2.
    P_W = A + B + C1 + C2
    return P_W


def power_to_weight_vtol_hover(
    disk_loading_N_m2: float,         # W / S_rotor
    p: VTOLParams = VTOLParams(),
) -> float:
    """
    Equation (2-62) as pasted:
        P/W_hover = sqrt(disk_loading / (2*rho0)) / FoM
    Returns [W/N] (numerically m/s).
    """
    if disk_loading_N_m2 <= 0:
        raise ValueError("disk_loading_N_m2 must be > 0")
    if p.rho0 <= 0:
        raise ValueError("rho0 must be > 0")
    if p.FoM <= 0:
        raise ValueError("FoM must be > 0")

    return sqrt(disk_loading_N_m2 / (2.0 * p.rho0)) / p.FoM


def vtol_power_requirements(
    weight_N: float,
    RoC_mps: float,
    disk_loading_N_m2: float,
    wing_loading_N_m2: float,
    D_rotor_m: float,
    p: VTOLParams = VTOLParams(),
) -> Dict[str, float]:
    """
    Convenience wrapper: returns both P/W and absolute power [W] for takeoff and hover.
    """
    if weight_N <= 0:
        raise ValueError("weight_N must be > 0")

    P_W_to = power_to_weight_vtol_takeoff(
        RoC_mps=RoC_mps,
        disk_loading_N_m2=disk_loading_N_m2,
        wing_loading_N_m2=wing_loading_N_m2,
        D_rotor_m=D_rotor_m,
        p=p,
    )
    P_W_hover = power_to_weight_vtol_hover(disk_loading_N_m2=disk_loading_N_m2, p=p)

    return {
        "P_W_to_vtol_W_per_N": P_W_to,
        "P_to_vtol_W": P_W_to * weight_N,
        "P_W_hover_W_per_N": P_W_hover,
        "P_hover_W": P_W_hover * weight_N,
        "rpm_rotor": rpm_from_diameter(D_rotor_m, p),
        "Vtip_mps": vtip_from_rpm_and_diameter(rpm_from_diameter(D_rotor_m, p), D_rotor_m),
    }


""" if __name__ == "__main__":
    # Example (replace with your UAV numbers)
    # Suppose:
    #   MTOW = 6.0 kg => W = 6*9.81 = 58.86 N
    #   disk loading = 120 N/m^2 (typ small multicopter-ish)
    #   wing loading = 200 N/m^2 (small fixed wing)
    #   vertical RoC = 2.0 m/s
    #   rotor diameter = 0.35 m
    W_N = 6.0 * 9.81
    results = vtol_power_requirements(
        weight_N=W_N,
        RoC_mps=2.0,
        disk_loading_N_m2=120.0,
        wing_loading_N_m2=200.0,
        D_rotor_m=0.35,
        p=VTOLParams(FoM=0.75, CD_blade=0.01, sigma=0.077, S_ratio=1.35),
    )

    for k, v in results.items():
        print(f"{k:>24s}: {v:.4f}") """