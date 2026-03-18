"""
airfoil_selection.py  --  Airfoil Analysis and Selection Module
===============================================================

PURPOSE
-------
Bridges airfoil choice to the rest of the sizing toolchain.

The DLR report (Poelma 2024, Table 5) treats airfoil t/c and Cl_max as
**required user inputs**.  This module goes one step further: given a NACA
4-digit designation it computes those numbers from first principles so the
user has a physics-backed starting point rather than a lookup table.

SCOPE
-----
  1. Parse a NACA 4-digit designation  (e.g. "NACA 2412", "0012", "4415")
  2. Compute section geometry          (camber line, thickness distribution)
  3. Estimate section aerodynamics     (Cl_alpha, Cl_max, Cd_0, Cl/Cd)
  4. Apply 3D finite-wing corrections  (CL_max, effective L/D, Oswald e)
  5. Check design constraints          (stall margin, spar depth)
  6. Generate coordinate file          (.dat, Selig format)
  7. Write config/airfoil.yaml         (consumed by wing_sizing and aerodynamics)

THEORY
------

NACA 4-DIGIT GEOMETRY
    A NACA MPxx airfoil is defined by:
        M  = maximum camber as fraction of chord   (1st digit / 100)
        P  = location of max camber from LE        (2nd digit / 10)
        t  = maximum thickness as fraction of chord (last 2 digits / 100)

    Thickness distribution (NACA standard):
        y_t = 5t * (0.2969*sqrt(x) - 0.1260*x - 0.3516*x^2
                    + 0.2843*x^3 - 0.1015*x^4)

    Camber line for 0 <= x <= P:
        y_c = (M/P^2) * (2*P*x - x^2)
    for P <= x <= 1:
        y_c = (M/(1-P)^2) * (1 - 2*P + 2*P*x - x^2)

SECTION LIFT-CURVE SLOPE  (thin airfoil theory)
    Cl_alpha = 2*pi  [per radian]  (symmetric or cambered, thin airfoil limit)
    Corrected for thickness by Prandtl-Glauert / empirical factor:
        Cl_alpha_corrected = 2*pi * (1 + 0.77*t)   (Abbott & von Doenhoff)

SECTION MAXIMUM LIFT COEFFICIENT
    For NACA 4-digit sections in the Reynolds number range relevant to small
    UAVs (Re ~ 3e5 - 1e6), empirical data gives:
        Cl_max ~ 1.0 + 2.3*(M/0.04) + correction for thickness
    Base fit (Abbott & von Doenhoff, low-speed data):
        Cl_max_sym   ~ 0.96 + 8.0*t   (symmetric sections, t = 0.06..0.18)
        Cl_max_cam   ~ Cl_max_sym + 2*M / (1 - 2*P + P^2)^0.5  (camber add)
    This is an engineering approximation; for higher accuracy use XFOIL or
    experimental data.

SECTION ZERO-LIFT ANGLE
    alpha_L0 = -2*M*(1 - P)^2 / (1 - 2*P + P^2)  [radians, thin airfoil]
    For symmetric sections: alpha_L0 = 0.

SECTION ZERO-LIFT DRAG (profile drag)
    Empirical fit for NACA 4-digit at Re ~ 5e5:
        Cd_0 ~ 0.006 + 0.007*t + 0.4*t^2
    This captures the strong thickness penalty on profile drag.

3D FINITE-WING CORRECTIONS  (Prandtl lifting-line)
    Lift-curve slope:
        CL_alpha_3D = Cl_alpha / (1 + Cl_alpha/(pi*AR*e))

    Maximum lift coefficient:
        CL_max = 0.9 * Cl_max * cos(sweep_c4)
        (Raymer eq. 3-8, also used in DLR toolchain)

    Induced drag factor:
        k = 1 / (pi * AR * e)
        where e = Oswald span efficiency

    Cruise lift coefficient:
        CL_cruise = W / (q * S)  = (W/S) / q

    Lift-to-drag ratio:
        L/D = CL_cruise / (CD0_total + k*CL_cruise^2)
        CD0_total = fuselage + wing profile drag contributions

OSWALD EFFICIENCY
    Raymer empirical (straight wing, eq. 3-5):
        e = 1.78 * (1 - 0.045*AR^0.68) - 0.64

DESIGN CONSTRAINTS CHECKED
    1. CL_max >= (W/S) / (0.5 * rho * V_stall^2)     (stall speed met)
    2. t/c >= 0.09                                     (spar depth minimum)
    3. Cl_max_section >= 1.2 * CL_max_3D              (adequate stall margin)

References
----------
  Abbott & von Doenhoff (1959), Theory of Wing Sections
  Raymer (2018), Aircraft Design: A Conceptual Approach, eqs 3-5, 3-8
  DLR-IB-FT-BS-2024-106, Poelma (2024), Table 5, eq. 3-8
"""

from __future__ import annotations

import math
import re
import yaml
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Tuple, Optional


# ─────────────────────────────────────────────
#  NACA 4-digit parser
# ─────────────────────────────────────────────

def parse_naca4(designation: str) -> Tuple[float, float, float]:
    """
    Parse a NACA 4-digit designation string and return (M, P, t).

        M  = max camber / chord        [-]   e.g. 0.02 for NACA 2xxx
        P  = max camber location/chord [-]   e.g. 0.40 for NACAxX4xx
        t  = thickness / chord         [-]   e.g. 0.12 for NACAxxx12

    Accepts:  "NACA 2412", "naca2412", "2412", "0012", "4415"
    """
    # Strip "NACA" prefix and whitespace
    digits = re.sub(r'(?i)naca\s*', '', designation).strip()
    if not re.fullmatch(r'\d{4}', digits):
        raise ValueError(
            f"'{designation}' is not a valid NACA 4-digit designation. "
            f"Expected 4 digits, e.g. '2412' or 'NACA 0012'."
        )
    M = int(digits[0]) / 100.0
    P = int(digits[1]) / 10.0
    t = int(digits[2:]) / 100.0
    return M, P, t


# ─────────────────────────────────────────────
#  Geometry
# ─────────────────────────────────────────────

def naca4_thickness(x: float, t: float) -> float:
    """NACA standard thickness distribution y_t at chord station x in [0,1]."""
    x = max(x, 0.0)
    return 5.0 * t * (
        0.2969 * math.sqrt(x)
        - 0.1260 * x
        - 0.3516 * x**2
        + 0.2843 * x**3
        - 0.1015 * x**4
    )


def naca4_camber(x: float, M: float, P: float) -> Tuple[float, float]:
    """
    Camber line y_c and its slope dy_c/dx at chord station x.
    Returns (y_c, dyc_dx).
    """
    if M == 0.0 or P == 0.0:
        return 0.0, 0.0
    if x < P:
        y_c    = (M / P**2) * (2*P*x - x**2)
        dyc_dx = (M / P**2) * (2*P - 2*x)
    else:
        y_c    = (M / (1-P)**2) * (1 - 2*P + 2*P*x - x**2)
        dyc_dx = (M / (1-P)**2) * (2*P - 2*x)
    return y_c, dyc_dx


def naca4_coordinates(
    M: float, P: float, t: float, n: int = 200
) -> Tuple[List[float], List[float], List[float], List[float]]:
    """
    Compute upper and lower surface coordinates in [0,1] chord.

    Returns (x_upper, y_upper, x_lower, y_lower).
    Uses cosine spacing for good leading-edge resolution.
    """
    # Cosine-spaced chord stations
    betas = [math.pi * i / (n - 1) for i in range(n)]
    xs    = [(1.0 - math.cos(b)) / 2.0 for b in betas]

    xu, yu, xl, yl = [], [], [], []
    for x in xs:
        y_c, dyc_dx = naca4_camber(x, M, P)
        y_t         = naca4_thickness(x, t)
        theta       = math.atan(dyc_dx)

        xu.append(x  - y_t * math.sin(theta))
        yu.append(y_c + y_t * math.cos(theta))
        xl.append(x  + y_t * math.sin(theta))
        yl.append(y_c - y_t * math.cos(theta))

    return xu, yu, xl, yl


# ─────────────────────────────────────────────
#  Section aerodynamics
# ─────────────────────────────────────────────

def section_Cl_alpha(t: float) -> float:
    """
    2D lift-curve slope [per radian], corrected for thickness.
    Abbott & von Doenhoff: Cl_alpha = 2*pi*(1 + 0.77*t)
    """
    return 2.0 * math.pi * (1.0 + 0.77 * t)


def section_alpha_L0(M: float, P: float) -> float:
    """
    Zero-lift angle of attack [radians].
    Thin airfoil theory: alpha_L0 = -2*M*(1-P)^2 / (1 - 2P + P^2)^0.5
    For symmetric (M=0): alpha_L0 = 0.
    """
    if M == 0.0 or P == 0.0:
        return 0.0
    # Standard thin airfoil result for NACA 4-digit camber line
    denom = math.sqrt(1.0 - 2*P + P**2)
    if denom < 1e-9:
        return 0.0
    return -2.0 * M * (1.0 - P)**2 / denom


def section_Cl_max(M: float, P: float, t: float) -> float:
    """
    Empirical section maximum lift coefficient.

    Calibrated against Abbott & von Doenhoff (1959) smooth-surface data
    at Re ~ 3e5..6e6 (relevant range for small UAVs):

      Symmetric baseline:  Cl_max_sym = 0.750 + 2.533*t
        NACA 0006: 0.90   0009: 1.03   0012: 1.05   0015: 1.13   0018: 1.21

      Camber correction:   delta_Cl = 15*M
        NACA 2412: 1.05 + 0.30 = 1.35  (actual ~1.40, conservative)
        NACA 4412: 1.05 + 0.60 = 1.65  (matches data)
        NACA 2415: 1.13 + 0.30 = 1.43  (actual ~1.44, good)

    Valid for t = 0.06..0.18, M <= 0.06, Re ~ 3e5..1e6.
    At lower Re (< 2e5) real Cl_max will be lower due to laminar
    separation; add ~15% penalty if flying in that regime.
    """
    Cl_max_sym = 0.750 + 2.533 * t
    if M == 0.0 or P == 0.0:
        return Cl_max_sym
    return Cl_max_sym + 15.0 * M


def section_Cd0(t: float) -> float:
    """
    Section profile drag coefficient at zero lift.
    Empirical fit for NACA 4-digit at Re ~ 5e5:
        Cd_0 ~ 0.006 + 0.007*t + 0.4*t^2
    """
    return 0.006 + 0.007 * t + 0.4 * t**2


def section_Cl_Cd_max(M: float, P: float, t: float) -> Tuple[float, float, float]:
    """
    Maximum section lift-to-drag ratio and the CL at which it occurs.
    Using the parabolic polar:
        Cd = Cd0 + (Cl - Cl_design)^2 / (pi * ... )
    Simplified: best L/D occurs near Cl_design ~ 2*pi*M (camber term).
    Returns (Cl_at_best_LD, best_LD_2D, Cd_at_best_LD).
    """
    cd0  = section_Cd0(t)
    # Approximate: treat as parabolic polar centered at Cl_design
    Cl_design = 2.0 * math.pi * M   # lift at zero incidence due to camber
    # For thin parabolic polar Cd = Cd0 + k2D * Cl^2:
    # k2D is very small for a 2D section; best L/D ~ sqrt(Cl_design^2 / Cd0)
    # We use a simple fit: k2D ~ Cd0 / (section_Cl_max * 0.5)^2
    Cl_half = max(section_Cl_max(M, P, t) * 0.5, 0.1)
    k2d  = cd0 / (Cl_half**2)
    Cl_best = math.sqrt(cd0 / k2d) if k2d > 0 else 0.5
    Cl_best = max(Cl_best, Cl_design)
    Cd_best = cd0 + k2d * Cl_best**2
    LD_best = Cl_best / Cd_best if Cd_best > 0 else 0.0
    return Cl_best, LD_best, Cd_best


# ─────────────────────────────────────────────
#  3D finite-wing corrections
# ─────────────────────────────────────────────

def oswald_efficiency(AR: float, sweep_rad: float = 0.0) -> float:
    """
    Raymer empirical Oswald span efficiency (eq. 3-5 / 3-6).
    Straight wing: e = 1.78*(1 - 0.045*AR^0.68) - 0.64
    """
    e = 1.78 * (1.0 - 0.045 * AR**0.68) - 0.64
    return max(min(e, 0.95), 0.5)   # clip to physical range


def wing_CL_max(Cl_max_section: float, sweep_c4_rad: float = 0.0) -> float:
    """
    3D wing maximum lift coefficient.
    Raymer eq. 3-8:  CL_max = 0.9 * Cl_max * cos(sweep_c4)
    """
    return 0.9 * Cl_max_section * math.cos(sweep_c4_rad)


def wing_CL_alpha(
    Cl_alpha_section: float, AR: float, e: float
) -> float:
    """
    3D lift-curve slope [per radian] from Prandtl lifting-line.
        CL_alpha = Cl_alpha / (1 + Cl_alpha / (pi * AR * e))
    """
    denom = 1.0 + Cl_alpha_section / (math.pi * AR * e)
    return Cl_alpha_section / denom


def wing_cruise_LD(
    WS_N_m2:    float,
    rho:        float,
    V_cruise:   float,
    CD0_total:  float,
    AR:         float,
    e:          float,
) -> Tuple[float, float, float]:
    """
    Cruise lift-to-drag ratio from parabolic polar.

    Parameters
    ----------
    WS_N_m2   : design wing loading  [N/m^2]
    rho       : air density          [kg/m^3]
    V_cruise  : cruise speed         [m/s]
    CD0_total : total zero-lift drag (wing + fuselage estimate)
    AR        : wing aspect ratio
    e         : Oswald efficiency

    Returns
    -------
    (CL_cruise, CD_cruise, LD_cruise)
    """
    q        = 0.5 * rho * V_cruise**2
    CL       = WS_N_m2 / q
    k        = 1.0 / (math.pi * AR * e)
    CD       = CD0_total + k * CL**2
    LD       = CL / CD if CD > 0 else 0.0
    return CL, CD, LD


# ─────────────────────────────────────────────
#  Design constraint checker
# ─────────────────────────────────────────────

def check_constraints(
    CL_max_3D:  float,
    WS_N_m2:    float,
    V_stall:    float,
    rho:        float,
    tc_ratio:   float,
    Cl_max_2D:  float,
) -> List[str]:
    """
    Run design constraint checks.  Returns a list of warning strings;
    empty list means all constraints are satisfied.
    """
    warnings = []

    # 1. Stall speed constraint
    WS_stall_max = 0.5 * rho * V_stall**2 * CL_max_3D
    if WS_N_m2 > WS_stall_max * 1.05:   # 5% margin
        warnings.append(
            f"STALL: Wing loading {WS_N_m2:.1f} N/m^2 exceeds stall limit "
            f"{WS_stall_max:.1f} N/m^2 at V_stall={V_stall} m/s. "
            f"Increase CL_max or reduce V_stall."
        )

    # 2. Spar depth (structural minimum t/c)
    if tc_ratio < 0.09:
        warnings.append(
            f"STRUCTURE: t/c = {tc_ratio:.3f} is below 0.09 minimum for "
            f"adequate spar depth. Consider NACA xx12 or thicker."
        )

    # 3. 2D to 3D margin (section must have headroom above 3D CL_max)
    required_2D = CL_max_3D / 0.9   # inverse of Raymer 3-8
    if Cl_max_2D < required_2D * 1.05:
        warnings.append(
            f"MARGIN: Section Cl_max = {Cl_max_2D:.3f} has less than 5% "
            f"headroom above required {required_2D:.3f}. "
            f"Choose a higher-lift section or add camber."
        )

    return warnings


# ─────────────────────────────────────────────
#  Coordinate file writer  (Selig .dat format)
# ─────────────────────────────────────────────

def write_dat_file(
    designation: str,
    M: float, P: float, t: float,
    path: str,
    n: int = 200,
    chord: float = 1.0,
) -> None:
    """
    Write airfoil coordinates in Selig format (upper surface TE->LE->TE lower).
    Coordinates are scaled to the given chord length.

    Parameters
    ----------
    designation : human-readable name for the file header
    M, P, t     : NACA 4-digit parameters
    path        : output file path (e.g. "data/airfoils/naca2412.dat")
    n           : number of points per surface
    chord       : chord length for scaling [m] (default 1.0 = unit chord)
    """
    xu, yu, xl, yl = naca4_coordinates(M, P, t, n=n)

    with open(path, 'w') as f:
        f.write(f"{designation.upper()}\n")
        # Upper surface from LE to TE
        for x, y in zip(xu, yu):
            f.write(f"  {x*chord:.6f}  {y*chord:.6f}\n")
        # Lower surface from TE to LE (skip duplicate LE point)
        for x, y in zip(reversed(xl), reversed(yl)):
            f.write(f"  {x*chord:.6f}  {y*chord:.6f}\n")


# ─────────────────────────────────────────────
#  Main result dataclass
# ─────────────────────────────────────────────

@dataclass
class AirfoilResult:
    """Complete output of analyse_airfoil()."""
    # Identity
    designation:       str     # e.g. "NACA 2412"
    M:                 float   # max camber ratio
    P:                 float   # max camber location
    t:                 float   # thickness ratio (t/c)

    # 2D section properties
    Cl_alpha_rad:      float   # section lift-curve slope  [/rad]
    Cl_max_2D:         float   # section max lift coeff    [-]
    Cd0_section:       float   # section profile drag      [-]
    alpha_L0_deg:      float   # zero-lift angle           [deg]
    Cl_at_best_LD:     float   # section Cl at best L/D    [-]
    LD_best_2D:        float   # section best L/D          [-]

    # 3D wing properties
    AR:                float   # wing aspect ratio         [-]
    e_oswald:          float   # Oswald efficiency         [-]
    CL_alpha_rad:      float   # 3D lift-curve slope       [/rad]
    CL_max_3D:         float   # 3D max lift coeff         [-]
    CL_cruise:         float   # cruise lift coefficient   [-]
    CD_cruise:         float   # cruise drag coefficient   [-]
    LD_cruise:         float   # cruise lift-to-drag ratio [-]
    k_induced:         float   # induced drag factor       [-]

    # Constraint check
    warnings:          List[str] = field(default_factory=list)

    def print_summary(self) -> None:
        print("=" * 56)
        print(f"  AIRFOIL ANALYSIS:  {self.designation}")
        print("=" * 56)
        print(f"  Geometry:")
        print(f"    t/c            : {self.t:.4f}  ({self.t*100:.1f}%)")
        print(f"    Camber (M)     : {self.M:.4f}  ({self.M*100:.1f}% chord)")
        print(f"    Max camber loc : {self.P:.2f}  ({self.P*100:.0f}% chord)")
        print()
        print(f"  Section (2D) aerodynamics:")
        print(f"    Cl_alpha       : {self.Cl_alpha_rad:.4f} /rad  "
              f"({math.degrees(self.Cl_alpha_rad):.4f} /deg)")
        print(f"    alpha_L0       : {self.alpha_L0_deg:.3f} deg")
        print(f"    Cl_max         : {self.Cl_max_2D:.4f}")
        print(f"    Cd_0           : {self.Cd0_section:.5f}")
        print(f"    Best L/D (2D)  : {self.LD_best_2D:.1f}  at Cl = {self.Cl_at_best_LD:.3f}")
        print()
        print(f"  3D wing (AR = {self.AR:.1f}):")
        print(f"    Oswald e       : {self.e_oswald:.4f}")
        print(f"    CL_alpha       : {self.CL_alpha_rad:.4f} /rad")
        print(f"    CL_max (3D)    : {self.CL_max_3D:.4f}")
        print(f"    k (induced)    : {self.k_induced:.5f}")
        print(f"    CL_cruise      : {self.CL_cruise:.4f}")
        print(f"    CD_cruise      : {self.CD_cruise:.5f}")
        print(f"    L/D (cruise)   : {self.LD_cruise:.2f}")
        print()
        if self.warnings:
            print(f"  CONSTRAINT WARNINGS:")
            for w in self.warnings:
                print(f"    ! {w}")
        else:
            print(f"  All design constraints: OK")
        print("=" * 56)


# ─────────────────────────────────────────────
#  Main analysis function
# ─────────────────────────────────────────────

def analyse_airfoil(
    designation:  str,
    AR:           float,
    WS_N_m2:      float,
    V_cruise:     float,
    V_stall:      float,
    rho:          float  = 1.225,
    sweep_c4_rad: float  = 0.0,
    CD0_fuselage: float  = 0.010,   # fuselage + misc drag contribution to CD0
) -> AirfoilResult:
    """
    Full airfoil analysis and 3D wing performance estimate.

    Parameters
    ----------
    designation  : NACA 4-digit string, e.g. "NACA 2412" or "0012"
    AR           : wing aspect ratio               [-]
    WS_N_m2      : design wing loading             [N/m^2]
    V_cruise     : cruise airspeed                 [m/s]
    V_stall      : required stall speed            [m/s]
    rho          : air density                     [kg/m^3]
    sweep_c4_rad : quarter-chord sweep angle       [rad]
    CD0_fuselage : fuselage + misc CD0 contribution[-]

    Returns
    -------
    AirfoilResult dataclass
    """
    M, P, t = parse_naca4(designation)

    # -- 2D section --
    Cl_a     = section_Cl_alpha(t)
    Cl_max2D = section_Cl_max(M, P, t)
    Cd0_sec  = section_Cd0(t)
    aL0      = section_alpha_L0(M, P)
    Cl_bld, LD2D, _ = section_Cl_Cd_max(M, P, t)

    # -- 3D wing --
    e       = oswald_efficiency(AR, sweep_c4_rad)
    CLa_3D  = wing_CL_alpha(Cl_a, AR, e)
    CLmax3D = wing_CL_max(Cl_max2D, sweep_c4_rad)
    k       = 1.0 / (math.pi * AR * e)
    CD0_tot = Cd0_sec + CD0_fuselage
    CL_cr, CD_cr, LD_cr = wing_cruise_LD(WS_N_m2, rho, V_cruise, CD0_tot, AR, e)

    # -- constraints --
    warns = check_constraints(CLmax3D, WS_N_m2, V_stall, rho, t, Cl_max2D)

    return AirfoilResult(
        designation   = designation.upper(),
        M=M, P=P, t=t,
        Cl_alpha_rad  = Cl_a,
        Cl_max_2D     = Cl_max2D,
        Cd0_section   = Cd0_sec,
        alpha_L0_deg  = math.degrees(aL0),
        Cl_at_best_LD = Cl_bld,
        LD_best_2D    = LD2D,
        AR=AR,
        e_oswald      = e,
        CL_alpha_rad  = CLa_3D,
        CL_max_3D     = CLmax3D,
        CL_cruise     = CL_cr,
        CD_cruise     = CD_cr,
        LD_cruise     = LD_cr,
        k_induced     = k,
        warnings      = warns,
    )


# ─────────────────────────────────────────────
#  Output writers  (all go to out/, never to config/)
# ─────────────────────────────────────────────
#
# Directory convention
# --------------------
#   config/   -- user-edited INPUT files  (never written by code)
#   out/      -- code-generated OUTPUT files  (never edited by hand)
#
# Two files are generated:
#   out/airfoil.yaml          -- aerodynamic properties fed to wing_sizing
#   out/airfoils/<name>.dat   -- coordinate file fed to cad/wing_profile.py

def write_airfoil_yaml(result: "AirfoilResult", path) -> None:
    """
    Write out/airfoil.yaml.

    Contains the computed aerodynamic properties that downstream modules
    (wing_sizing, aerodynamics) need.  This file is auto-generated --
    do not edit it by hand.  Change config/airfoil_selection.yaml and
    re-run the analysis cell to regenerate.

    Parameters
    ----------
    result : AirfoilResult from analyse_airfoil()
    path   : destination path, e.g. Path("out/airfoil.yaml")
    """
    Path(path).parent.mkdir(parents=True, exist_ok=True)

    data = {
        "designation":  result.designation,
        "tc_ratio":     round(result.t, 4),
        "Cl_max_2D":    round(result.Cl_max_2D, 4),
        "CL_max_3D":    round(result.CL_max_3D, 4),
        "Cd0_section":  round(result.Cd0_section, 5),
        "alpha_L0_deg": round(result.alpha_L0_deg, 3),
        "LD_cruise":    round(result.LD_cruise, 2),
        "e_oswald":     round(result.e_oswald, 4),
        "k_induced":    round(result.k_induced, 6),
    }

    with open(path, "w", encoding="utf-8") as f:
        f.write("# AUTO-GENERATED -- do not edit by hand.\n")
        f.write("# Source : src/conceptual_design/airfoil_selection.py\n")
        f.write("# Input  : config/airfoil_selection.yaml\n")
        f.write("# Regen  : re-run the airfoil selection notebook cell.\n\n")
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)


def write_dat_file(
    designation: str,
    M: float, P: float, t: float,
    path,
    n: int = 200,
    chord: float = 1.0,
) -> None:
    """
    Write airfoil coordinates to out/airfoils/<name>.dat in Selig format.

    The .dat file is consumed by cad/wing_profile.py to generate the 3D
    wing geometry.  Coordinates are upper surface LE->TE then lower
    surface TE->LE (Selig convention).

    Parameters
    ----------
    designation : header label for the file
    M, P, t     : NACA 4-digit parameters
    path        : destination, e.g. Path("out/airfoils/naca2412.dat")
    n           : points per surface half (default 200)
    chord       : chord scaling in metres (default 1.0 = unit chord)
    """
    Path(path).parent.mkdir(parents=True, exist_ok=True)

    xu, yu, xl, yl = naca4_coordinates(M, P, t, n=n)

    with open(path, "w") as f:
        f.write(f"{designation.upper()}\n")
        # Upper surface LE -> TE
        for x, y in zip(xu, yu):
            f.write(f"  {x*chord:.6f}  {y*chord:.6f}\n")
        # Lower surface TE -> LE (skip duplicate LE point at index 0)
        for x, y in zip(reversed(xl[1:]), reversed(yl[1:])):
            f.write(f"  {x*chord:.6f}  {y*chord:.6f}\n")


def write_outputs(
    result:     "AirfoilResult",
    out_dir,
    n_coords:   int   = 200,
    chord:      float = 1.0,
) -> None:
    """
    Write all generated outputs to the out/ directory.

    Creates two files:
        <out_dir>/airfoil.yaml           -- aerodynamic properties
        <out_dir>/airfoils/<name>.dat    -- coordinate file

    Parameters
    ----------
    result  : AirfoilResult from analyse_airfoil()
    out_dir : root output directory, e.g. Path("out")
    n_coords: coordinate points per surface half
    chord   : chord length for .dat file scaling [m]
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1. airfoil.yaml
    yaml_path = out_dir / "airfoil.yaml"
    write_airfoil_yaml(result, yaml_path)

    # 2. coordinate .dat file
    slug = result.designation.replace(" ", "").lower()   # "NACA 2412" -> "naca2412"
    dat_path = out_dir / "airfoils" / f"{slug}.dat"
    write_dat_file(result.designation, result.M, result.P, result.t,
                   dat_path, n=n_coords, chord=chord)

    print(f"Outputs written to  {out_dir}/")
    print(f"  {yaml_path}")
    print(f"  {dat_path}")


# ─────────────────────────────────────────────
#  Convenience: compare candidate airfoils
# ─────────────────────────────────────────────

def compare_airfoils(
    candidates:   List[str],
    AR:           float,
    WS_N_m2:      float,
    V_cruise:     float,
    V_stall:      float,
    rho:          float = 1.225,
    sweep_c4_rad: float = 0.0,
) -> None:
    """
    Print a side-by-side comparison table for a list of airfoil designations.
    """
    results = [
        analyse_airfoil(d, AR, WS_N_m2, V_cruise, V_stall, rho, sweep_c4_rad)
        for d in candidates
    ]

    header = (f"{'Designation':<14} {'t/c':>6} {'Cl_max':>8} {'CL_max3D':>10} "
              f"{'Cd0':>8} {'L/D':>8} {'Warns':>6}")
    print(header)
    print("-" * len(header))
    for r in results:
        w = len(r.warnings)
        flag = "  OK" if w == 0 else f"  {w}!"
        print(f"{r.designation:<14} {r.t:>6.3f} {r.Cl_max_2D:>8.4f} "
              f"{r.CL_max_3D:>10.4f} {r.Cd0_section:>8.5f} "
              f"{r.LD_cruise:>8.2f}{flag}")