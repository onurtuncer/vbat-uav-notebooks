"""
aileron_design.py  --  Aileron Sizing for Cruise-Phase Roll Authority
======================================================================

Sizes wing ailerons that back up the jet control vanes
(control_vane_design.py) for roll control in wing-borne cruise.

MOTIVATION
----------
Jet-vane authority is sized from HOVER thrust and scales linearly with
it (actuator-disk momentum theory: q_jet = T / A_disk). Cruise thrust is
only a small fraction of hover thrust (set by cruise L/D), so jet-vane
roll authority collapses in cruise even though it has a large margin in
hover. Ailerons use wing dynamic pressure instead, which does not depend
on EDF throttle setting.

THEORY
------

1. GEOMETRY
    One aileron occupies the outboard `span_frac_wing` fraction of the
    wing HALF-span, with chord `chord_frac` of the wing MAC:

        S_aileron = (chord_frac * chord_mean) * (span_frac_wing * b_wing/2)
        y_arm     = spanwise station of the aileron's mid-span centroid

2. FLAP EFFECTIVENESS  (Glauert thin-airfoil theory)
        theta_f = arccos(2*chord_frac - 1)
        tau     = 1 - (theta_f - sin(theta_f)) / pi

    Cl_delta = CL_alpha_3D * tau, where CL_alpha_3D is the wing's own
    finite-span lift-curve slope (airfoil_selection.section_Cl_alpha +
    wing_CL_alpha) -- reused rather than re-derived.

3. ROLL MOMENT AND AUTHORITY
    Both ailerons deflect differentially (+delta / -delta):
        dL_per_aileron = q_cruise * S_aileron * Cl_delta * delta_design
        M_roll         = 2 * dL_per_aileron * y_arm
        ddot_roll      = M_roll / I_roll   [rad/s^2] -> deg/s^2

4. HINGE MOMENT AND SERVO TORQUE  (mirrors control_vane_design.py)
        Ch_max      = C_hm_coeff * Cl_delta * delta_max_rad
        M_hinge_max = q_cruise * S_aileron * c_aileron * Ch_max
        M_servo     = M_hinge_max * SF_servo * 1.2

References
----------
  Glauert, "The Elements of Aerofoil and Airscrew Theory", ch. 12
  (flap effectiveness parameter tau).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import yaml

from .airfoil_selection import section_Cl_alpha, wing_CL_alpha


# ---------------------------------------------
#  Input parameters (config/aileron.yaml)
# ---------------------------------------------
@dataclass
class AileronParams:
    span_frac_wing:   float   # aileron span / wing half-span      [-]
    chord_frac:       float   # aileron chord / wing MAC            [-]
    delta_max_deg:    float   # max deflection                      [deg]
    delta_design_deg: float   # design deflection                   [deg]
    servo_mass_kg:    float   # per-aileron servo + horn + wiring   [kg]
    linkage_mass_kg:  float   # per-aileron pushrod + hinge pin     [kg]
    C_hm_coeff:       float   # hinge-moment coefficient factor     [-]
    SF_servo:         float   # servo torque safety factor          [-]

    @classmethod
    def from_yaml(cls, path) -> "AileronParams":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(
            span_frac_wing   = float(data["span_frac_wing"]),
            chord_frac       = float(data["chord_frac"]),
            delta_max_deg    = float(data["delta_max_deg"]),
            delta_design_deg = float(data["delta_design_deg"]),
            servo_mass_kg    = float(data["servo_mass_kg"]),
            linkage_mass_kg  = float(data["linkage_mass_kg"]),
            C_hm_coeff       = float(data["C_hm_coeff"]),
            SF_servo         = float(data["SF_servo"]),
        )


# ---------------------------------------------
#  Sizing output container
# ---------------------------------------------
@dataclass
class AileronSizing:
    c_aileron_m:      float   # aileron chord                       [m]
    b_aileron_m:      float   # aileron span                        [m]
    S_aileron_m2:     float   # aileron planform area (one side)    [m^2]
    y_arm_m:          float   # spanwise moment arm to centroid     [m]
    tau:              float   # flap effectiveness factor           [-]
    Cl_delta_per_rad: float   # lift-curve slope w.r.t. deflection  [1/rad]
    q_cruise_Pa:      float   # cruise dynamic pressure             [Pa]
    F_aileron_design_N: float # per-aileron force at design defl.   [N]
    M_roll_design_Nm: float   # roll moment at design deflection     [Nm]
    ddot_roll_deg_s2: float   # roll acceleration (aileron alone)   [deg/s^2]
    M_hinge_max_Nmm:  float   # hinge moment at max deflection       [N*mm]
    servo_torque_req_gcm: float  # required servo torque            [g*cm]


def size_aileron(
    b_wing_m:     float,   # wing span [m]
    chord_mean_m: float,   # wing MAC [m]
    tc_ratio:     float,   # wing airfoil t/c [-] (out/airfoil.yaml)
    AR:           float,   # wing aspect ratio [-]
    e_oswald:     float,   # Oswald efficiency [-] (out/airfoil.yaml)
    V_cruise:     float,   # cruise airspeed [m/s]
    rho:          float,   # air density [kg/m^3]
    I_roll:       float,   # roll moment of inertia [kg m^2]
    p: AileronParams = None,
) -> AileronSizing:
    """Complete aileron sizing.  See module docstring for the model."""
    if p is None:
        raise ValueError("AileronParams required (config/aileron.yaml)")

    b_half = 0.5 * b_wing_m
    b_aileron = p.span_frac_wing * b_half
    c_aileron = p.chord_frac * chord_mean_m
    S_aileron = c_aileron * b_aileron

    y_inner = (1.0 - p.span_frac_wing) * b_half
    y_arm = 0.5 * (y_inner + b_half)

    # Glauert thin-airfoil flap-effectiveness parameter
    theta_f = math.acos(2.0 * p.chord_frac - 1.0)
    tau = 1.0 - (theta_f - math.sin(theta_f)) / math.pi

    Cl_alpha_2D = section_Cl_alpha(tc_ratio)
    Cl_alpha_3D = wing_CL_alpha(Cl_alpha_2D, AR, e_oswald)
    Cl_delta = Cl_alpha_3D * tau

    q_cruise = 0.5 * rho * V_cruise**2

    delta_design_rad = math.radians(p.delta_design_deg)
    F_aileron = q_cruise * S_aileron * Cl_delta * delta_design_rad
    M_roll = 2.0 * F_aileron * y_arm

    ddot_roll = math.degrees(M_roll / I_roll)

    delta_max_rad = math.radians(p.delta_max_deg)
    Ch_max = p.C_hm_coeff * Cl_delta * delta_max_rad
    M_hinge_max = q_cruise * S_aileron * c_aileron * Ch_max

    M_servo_Nm = M_hinge_max * p.SF_servo * 1.2
    M_servo_gcm = M_servo_Nm * 1e3 * 100.0 / 9.80665

    return AileronSizing(
        c_aileron_m=c_aileron, b_aileron_m=b_aileron, S_aileron_m2=S_aileron,
        y_arm_m=y_arm, tau=tau, Cl_delta_per_rad=Cl_delta,
        q_cruise_Pa=q_cruise, F_aileron_design_N=F_aileron,
        M_roll_design_Nm=M_roll, ddot_roll_deg_s2=ddot_roll,
        M_hinge_max_Nmm=M_hinge_max * 1e3,
        servo_torque_req_gcm=M_servo_gcm,
    )


# ---------------------------------------------
#  Residual jet-vane authority in cruise
# ---------------------------------------------
@dataclass
class VaneCruiseAuthority:
    """Jet-vane authority scaled from hover to cruise thrust.

    q_jet scales linearly with thrust (actuator disk), so every vane
    ddot scales by T_cruise/T_hover.  Shared by NB4 and the NB12 COTS
    re-solve so the law is derived exactly once.
    """
    s_ratio:      float
    LD_cruise:    float
    T_hover_N:    float
    T_cruise_N:   float
    thrust_ratio: float
    ddot_roll_hover:    float
    ddot_pitch_hover:   float
    ddot_roll_cruise:   float
    ddot_pitch_cruise:  float


def vane_cruise_authority(W_N: float, s_ratio: float, LD_cruise: float,
                          vanes: dict) -> VaneCruiseAuthority:
    """Scale the NB3 hover vane authority to the cruise thrust setting."""
    T_hover  = s_ratio * W_N
    T_cruise = W_N / LD_cruise
    thrust_ratio = T_cruise / T_hover
    return VaneCruiseAuthority(
        s_ratio=s_ratio, LD_cruise=LD_cruise,
        T_hover_N=T_hover, T_cruise_N=T_cruise, thrust_ratio=thrust_ratio,
        ddot_roll_hover   = vanes["ddot_roll_deg_s2"],
        ddot_pitch_hover  = vanes["ddot_pitch_deg_s2"],
        ddot_roll_cruise  = vanes["ddot_roll_deg_s2"]  * thrust_ratio,
        ddot_pitch_cruise = vanes["ddot_pitch_deg_s2"] * thrust_ratio,
    )


# ---------------------------------------------
#  Handoff writer
# ---------------------------------------------

def write_aileron_yaml(ail: AileronSizing, p: AileronParams, path,
                       ddot_roll_vane_cruise_deg_s2: float,
                       ddot_min_deg_s2: float,
                       regen_notebook: str = "notebooks/aileron_design.ipynb",
                       extra: dict = None) -> None:
    """
    Write out/aileron.yaml -- the handoff consumed by fuselage_design.py
    (servo/linkage mass) and read for reference by downstream notebooks.

    The post-freeze re-solve reuses this writer for out/aileron_cots.yaml
    (same schema) with its own `regen_notebook` provenance and an `extra`
    block (frozen-servo id/stall/margins) appended.
    """
    ddot_roll_total = ail.ddot_roll_deg_s2 + ddot_roll_vane_cruise_deg_s2
    data = {
        "span_frac_wing":   p.span_frac_wing,
        "chord_frac":       p.chord_frac,
        "c_aileron_m":      round(ail.c_aileron_m, 5),
        "b_aileron_m":      round(ail.b_aileron_m, 5),
        "S_aileron_m2":     round(ail.S_aileron_m2, 6),
        "y_arm_m":          round(ail.y_arm_m, 5),
        "tau":              round(ail.tau, 5),
        "Cl_delta_per_rad": round(ail.Cl_delta_per_rad, 5),
        "delta_max_deg":    p.delta_max_deg,
        "delta_design_deg": p.delta_design_deg,
        "q_cruise_Pa":      round(ail.q_cruise_Pa, 4),
        "F_aileron_design_N": round(ail.F_aileron_design_N, 5),
        "M_roll_design_Nm":   round(ail.M_roll_design_Nm, 5),
        "ddot_roll_aileron_deg_s2":     round(ail.ddot_roll_deg_s2, 3),
        "ddot_roll_vane_cruise_deg_s2": round(ddot_roll_vane_cruise_deg_s2, 3),
        "ddot_roll_total_cruise_deg_s2": round(ddot_roll_total, 3),
        "ddot_min_deg_s2":  ddot_min_deg_s2,
        "cruise_authority_ok": bool(ddot_roll_total >= ddot_min_deg_s2),
        "M_hinge_max_Nmm":       round(ail.M_hinge_max_Nmm, 4),
        "servo_torque_req_gcm":  round(ail.servo_torque_req_gcm, 2),
        "servo_mass_kg_each":    p.servo_mass_kg,
        "linkage_mass_kg_each":  p.linkage_mass_kg,
        "n_ailerons": 2,
    }
    if extra:
        data.update(extra)
    with open(path, "w", encoding="utf-8") as f:
        f.write("# AUTO-GENERATED -- do not edit by hand.\n")
        f.write("# Source : src/conceptual_design/aileron_design.py\n")
        f.write("# Input  : config/aileron.yaml + wing geometry + control vanes\n")
        f.write(f"# Regen  : re-run {regen_notebook}\n\n")
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
