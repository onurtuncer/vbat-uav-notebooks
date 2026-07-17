"""
control_vane_design.py  --  Jet Control Vane Sizing (hover/transition)
======================================================================

Sizes the four flat-plate jet vanes in the EDF efflux that provide
pitch/yaw/roll control in hover and transition (primary control path;
see aileron_design.py for the cruise-phase roll backup).

THEORY
------

1. JET FLOW (actuator-disk momentum theory)
        v_h   = sqrt(T / (2 rho A_disk))     induced velocity at the disk
        V_jet = 2 v_h                        fully-developed jet velocity
        q_jet = 0.5 rho V_jet^2              dynamic pressure at the vanes

2. VANE LIFT (thin flat plate at low Re)
        Cl_alpha = 2 pi * Re_correction      (Pelletier & Mueller 2000,
                                              Re < 60k -> ~0.85)
        F_vane   = q_jet S_vane Cl_alpha delta

3. CONTROL MOMENTS AND AUTHORITY (T/B/L/R vane arrangement)
        M_pitch = 2 F L_CG      (T+B pair)
        M_yaw   = 2 F L_CG      (L+R pair)
        M_roll  = 4 F R_mid     (all four, differential)
    Angular accelerations follow from slender-body inertia estimates
    (uniform-density cylinder of the vehicle's bulk dimensions); the
    same estimates are reused by aileron_design so the cruise check
    does not re-derive them.

4. HINGE MOMENT AND SERVO TORQUE (mirrors aileron_design.py)
        Ch_max      = C_hm_coeff * Cl_alpha * delta_max
        M_hinge_max = q_jet S_vane c_vane Ch_max
        M_servo     = M_hinge_max * SF_servo * servo_margin
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
import yaml

#: Dynamic viscosity of air at ISA sea level [Pa s].
MU_AIR = 1.789e-5

#: Thin-airfoil lift-curve slope [1/rad].
CL_ALPHA_2PI = 2.0 * math.pi

#: Vane mixing matrix: rows T/B/L/R, columns pitch/yaw/roll commands.
MIX = np.array([
    # pitch   yaw   roll
    [ +1.0,   0.0, +1.0],   # T
    [ +1.0,   0.0, -1.0],   # B
    [  0.0,  +1.0, -1.0],   # L
    [  0.0,  +1.0, +1.0],   # R
])
MIX_ROWS = ["T", "B", "L", "R"]
MIX_COLS = ["pitch", "yaw", "roll"]


# ---------------------------------------------
#  Input parameters (set by the design notebook)
# ---------------------------------------------
@dataclass(frozen=True)
class VaneParams:
    """Vane design choices and empirical factors (notebook-set)."""

    tc_vane:           float   # flat-plate thickness ratio            [-]
    Re_correction:     float   # low-Re Cl_alpha knockdown (P&M 2000)  [-]
    delta_stall_deg:   float   # soft flat-plate stall onset           [deg]
    delta_max_deg:     float   # mechanical deflection limit           [deg]
    delta_design_deg:  float   # authority-check deflection            [deg]
    kappa_hub:         float   # hub radius / tip radius               [-]
    AR_vane:           float   # vane aspect ratio h/c                 [-]
    n_vanes:           int     # number of vanes (T/B/L/R)             [-]
    span_frac:         float   # vane span / annulus height            [-]
    hinge_xc:          float   # hinge line chord fraction             [-]
    C_hm_coeff:        float   # hinge-moment coefficient factor       [-]
    SF_servo:          float   # servo torque safety factor            [-]
    servo_margin:      float   # extra servo torque margin factor      [-]
    body_len_factor:   float   # slender-body length / wing MAC        [-]
    cg_frac:           float   # CG station / body length              [-]
    fus_radius_factor: float   # bulk fuselage radius / R_tip          [-]


# ---------------------------------------------
#  Sizing output container
# ---------------------------------------------
@dataclass
class VaneDesign:
    """Complete vane design: geometry, jet flow, authority, actuation."""

    p: VaneParams

    # jet flow (momentum theory at hover thrust)
    R_tip:   float          # duct tip radius                       [m]
    A_disk:  float          # disk area                             [m^2]
    v_h:     float          # induced velocity at the disk          [m/s]
    V_jet:   float          # jet velocity at the vanes             [m/s]
    q_jet:   float          # jet dynamic pressure                  [Pa]
    q_fs:    float          # cruise free-stream dynamic pressure   [Pa]

    # geometry
    R_hub:   float          # hub radius                            [m]
    R_mid:   float          # annulus mid radius (roll arm)         [m]
    h_vane:  float          # vane span                             [m]
    c_vane:  float          # vane chord                            [m]
    S_vane:  float          # vane planform area                    [m^2]

    # slender-body inertia estimates (shared with aileron_design)
    L_body:  float          # bulk body length                      [m]
    L_CG:    float          # vane-to-CG moment arm                 [m]
    L_roll:  float          # roll moment arm (= R_mid)             [m]
    I_pitch: float          # pitch inertia estimate                [kg m^2]
    I_yaw:   float          # yaw inertia estimate                  [kg m^2]
    I_roll:  float          # roll inertia estimate                 [kg m^2]

    # aerodynamics and authority at design deflection
    Cl_alpha_eff: float     # effective lift-curve slope            [1/rad]
    Re_vane:      float     # vane chord Reynolds number            [-]
    F_vane:       float     # force per vane                        [N]
    M_pitch_Nm:   float
    M_yaw_Nm:     float
    M_roll_Nm:    float
    ddot_pitch:   float     # angular accelerations                 [deg/s^2]
    ddot_yaw:     float
    ddot_roll:    float
    ddot_min:     float     # shared requirement (config/aerodynamics.yaml)

    # actuation
    M_hinge_max:  float     # hinge moment at delta_max             [Nm]
    servo_torque_req_gcm: float

    mix: np.ndarray = field(default_factory=lambda: MIX.copy())

    def thrust_loss_pct(self, delta_deg):
        """Blockage thrust loss [%] of the vane set at deflection delta."""
        delta_rad = np.radians(delta_deg)
        return (self.p.n_vanes * self.S_vane
                * np.abs(np.sin(delta_rad)) / self.A_disk) * 100.0


def design_vanes(
    MTOW_kg:      float,   # converged MTOW [kg]
    D_rotor_m:    float,   # EDF rotor diameter [m]
    V_cruise:     float,   # cruise airspeed [m/s]
    rho:          float,   # air density [kg/m^3]
    g:            float,   # gravitational acceleration [m/s^2]
    chord_mean_m: float,   # wing MAC [m] (for the slender-body estimates)
    ddot_min_deg_s2: float,  # shared authority floor (config/aerodynamics.yaml)
    p: VaneParams,
) -> VaneDesign:
    """Complete jet-vane sizing.  See module docstring for the model."""
    W_N = MTOW_kg * g
    R_tip = D_rotor_m / 2.0
    A_disk = math.pi * R_tip**2

    # jet flow at hover thrust (T = W)
    T_hover = W_N
    v_h = math.sqrt(T_hover / (2.0 * rho * A_disk))
    V_jet = 2.0 * v_h
    q_jet = 0.5 * rho * V_jet**2
    q_fs = 0.5 * rho * V_cruise**2

    Cl_alpha_eff = CL_ALPHA_2PI * p.Re_correction
    delta_design_rad = math.radians(p.delta_design_deg)

    # geometry
    R_hub = p.kappa_hub * R_tip
    R_mid = 0.5 * (R_tip + R_hub)
    h_vane = p.span_frac * (R_tip - R_hub)
    c_vane = h_vane / p.AR_vane
    S_vane = c_vane * h_vane

    # slender-body inertia estimates
    L_body = p.body_len_factor * chord_mean_m
    L_CG = p.cg_frac * L_body
    L_roll = R_mid
    R_fus = R_tip * p.fus_radius_factor
    I_pitch = (MTOW_kg / 12.0) * (3 * R_fus**2 + L_body**2)
    I_yaw = I_pitch
    I_roll = (MTOW_kg / 2.0) * R_fus**2

    # authority at design deflection
    nu_air = MU_AIR / rho
    Re_vane = V_jet * c_vane / nu_air
    Cl_design = Cl_alpha_eff * delta_design_rad
    F_vane = q_jet * S_vane * Cl_design
    M_pitch_Nm = 2.0 * F_vane * L_CG
    M_yaw_Nm = 2.0 * F_vane * L_CG
    M_roll_Nm = 4.0 * F_vane * L_roll
    ddot_pitch = math.degrees(M_pitch_Nm / I_pitch)
    ddot_yaw = math.degrees(M_yaw_Nm / I_yaw)
    ddot_roll = math.degrees(M_roll_Nm / I_roll)

    # hinge moment and servo torque at delta_max
    delta_max_rad = math.radians(p.delta_max_deg)
    Ch_max = p.C_hm_coeff * Cl_alpha_eff * delta_max_rad
    M_hinge_max = q_jet * S_vane * c_vane * Ch_max
    M_servo_Nm = M_hinge_max * p.SF_servo * p.servo_margin
    M_servo_gcm = M_servo_Nm * 1e3 * 100.0 / 9.80665

    return VaneDesign(
        p=p,
        R_tip=R_tip, A_disk=A_disk, v_h=v_h, V_jet=V_jet, q_jet=q_jet, q_fs=q_fs,
        R_hub=R_hub, R_mid=R_mid, h_vane=h_vane, c_vane=c_vane, S_vane=S_vane,
        L_body=L_body, L_CG=L_CG, L_roll=L_roll,
        I_pitch=I_pitch, I_yaw=I_yaw, I_roll=I_roll,
        Cl_alpha_eff=Cl_alpha_eff, Re_vane=Re_vane, F_vane=F_vane,
        M_pitch_Nm=M_pitch_Nm, M_yaw_Nm=M_yaw_Nm, M_roll_Nm=M_roll_Nm,
        ddot_pitch=ddot_pitch, ddot_yaw=ddot_yaw, ddot_roll=ddot_roll,
        ddot_min=ddot_min_deg_s2,
        M_hinge_max=M_hinge_max, servo_torque_req_gcm=M_servo_gcm,
    )


# ---------------------------------------------
#  Mixing saturation check
# ---------------------------------------------

#: Command test vectors for the mixing saturation check (pitch, yaw, roll).
MIX_TESTS = [
    ("Pure pitch-up",   (1.0, 0.0, 0.0)),
    ("Pure yaw-right",  (0.0, 1.0, 0.0)),
    ("Pure roll-right", (0.0, 0.0, 1.0)),
    ("Pitch + yaw",     (0.5, 0.5, 0.0)),
    ("Full 3-axis",     (1.0, 1.0, 1.0)),
]


def mixing_check(d: VaneDesign, tests=MIX_TESTS):
    """Per-vane deflections for each command vector and a saturation flag.

    Returns [(label, [dT, dB, dL, dR], within_limits), ...].
    """
    rows = []
    for label, cmd in tests:
        defl = MIX @ np.array(cmd) * d.p.delta_design_deg
        ok = all(abs(x) <= d.p.delta_max_deg for x in defl)
        rows.append((label, defl, ok))
    return rows


# ---------------------------------------------
#  Handoff writer
# ---------------------------------------------

def write_control_vanes_yaml(d: VaneDesign, path) -> None:
    """Write out/control_vanes.yaml -- consumed by aileron, fuselage and
    mass-properties notebooks (geometry, authority, inertia estimates)."""
    p = d.p
    vane_data = {
        "section_type"         : "flat_plate",
        "tc_ratio"             : p.tc_vane,
        "hinge_xc"             : p.hinge_xc,
        "Cl_alpha_per_rad"     : d.Cl_alpha_eff,
        "Re_vane"              : d.Re_vane,
        "n_vanes"              : p.n_vanes,
        "R_tip_m"              : d.R_tip,
        "R_hub_m"              : d.R_hub,
        "R_mid_m"              : d.R_mid,
        "h_vane_m"             : d.h_vane,
        "c_vane_m"             : d.c_vane,
        "AR_vane"              : p.AR_vane,
        "S_vane_m2"            : d.S_vane,
        "V_jet_ms"             : d.V_jet,
        "q_jet_Pa"             : d.q_jet,
        "delta_max_deg"        : p.delta_max_deg,
        "delta_stall_deg"      : p.delta_stall_deg,
        "delta_design_deg"     : p.delta_design_deg,
        "F_vane_design_N"      : d.F_vane,
        "M_pitch_design_Nm"    : d.M_pitch_Nm,
        "M_yaw_design_Nm"      : d.M_yaw_Nm,
        "M_roll_design_Nm"     : d.M_roll_Nm,
        "ddot_pitch_deg_s2"    : d.ddot_pitch,
        "ddot_yaw_deg_s2"      : d.ddot_yaw,
        "ddot_roll_deg_s2"     : d.ddot_roll,
        "ddot_min_deg_s2"      : d.ddot_min,
        "M_hinge_max_Nmm"      : d.M_hinge_max * 1e3,
        "servo_torque_req_gcm" : d.servo_torque_req_gcm,
        "L_CG_m"               : d.L_CG,
        "L_roll_m"             : d.L_roll,
        # slender-body inertia estimates (hover authority above); reused as-is
        # by aileron_design.py so the cruise roll-authority check doesn't
        # re-derive the same approximation a second time
        "I_pitch_kgm2"         : d.I_pitch,
        "I_yaw_kgm2"           : d.I_yaw,
        "I_roll_kgm2"          : d.I_roll,
        "mixing_matrix"        : d.mix.tolist(),
        "mixing_rows"          : MIX_ROWS,
        "mixing_cols"          : MIX_COLS,
    }
    with open(path, "w", encoding="utf-8") as f:
        yaml.dump(vane_data, f, default_flow_style=False, sort_keys=False)
