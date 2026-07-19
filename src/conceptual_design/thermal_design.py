"""
thermal_design.py  --  Thermal-Path Sizing (ESC cold-plate + vented bay)
=========================================================================

Sizes the two heat-rejection paths of the tail-sitter, both realized as
STRUCTURE that also does thermal management (ADR-0009):

  1. ESC cold-plate  -- the ESC bonds to a conductive plate on the inner
     shell wall where the EDF inflow washes it; forced convection off the
     plate carries the ESC switching loss away.
  2. Vented battery bay -- inlet/outlet vents admit a through-flow that
     carries the battery discharge (I^2 R) loss out of the bay.

MOTIVATION
----------
The airframe had no thermal model.  These paths are nearly free to plan in
the current geometry (the ESC is already mid-body in the inflow field, the
battery bay wall is available to vent) and painful to retrofit once the
shell is frozen.  This module puts a first-order number on both.

HEAT LOADS  (derived, not configured)
-------------------------------------
    Q_esc  = P_hover * (1 - eta_esc)          switching/conduction loss
    Q_batt = I_hover^2 * R_pack_nominal       pack IR loss in hover
(Q_batt used P_hover*(1/eta_bat - 1) before the ADR-0014 amendment; the
I^2 R law at the nominal pack resistance is the same one the mission
transient uses, so the vent sizing and the transient see the same
hover heat.)

COOLING AIRFLOW
---------------
The fan draws its induced velocity in hover (actuator disk):
    v_h = sqrt(W / (2 rho A_disk))
Neither path sees the full v_h -- the ESC sits mid-body, vent flow is a
throttled bleed -- so each uses a configured fraction of it.

ESC COLD-PLATE  (flat-plate forced convection)
----------------------------------------------
Laminar Nusselt  Nu = 0.664 Re^0.5 Pr^(1/3),  h = Nu k / L.  For a square
plate (flow length L = side = sqrt(A)) this closes in one step:
    h(A) = C_h A^(-1/4),   C_h = 0.664 k_air Pr^(1/3) sqrt(V_cool / nu)
    T_plate(A) = T_ambient + Q_esc / (C_h A^(3/4))
The area to hold the ESC at its limit, and the temperature it actually
reaches on the available wall, both follow in closed form.

VENTED BATTERY BAY  (through-flow energy balance)
-------------------------------------------------
    Q_batt = mdot cp dT_air,   mdot = rho V_vent A_vent
The exit air (hence the pack) rises dT_air above ambient; size the vent
area for the pack to stay under its limit.

BATTERY PACK MISSION TRANSIENT  (ADR-0014; reported, never filtered on)
------------------------------------------------------------------------
The vent balance above is steady-state and AIR-SIDE only -- it checks
that the bay can carry the heat away, not how hot the pack itself gets.
An external review (C. Ucler, 2026-07) showed that over the short-hover
mission the pack integrates its own Joule heat I^2 R_pack faster than
the cruise leg can reject it.  ``battery_pack_transient`` reproduces
that lumped-capacitance model: adiabatic hover+transition legs, then
exponential cruise cooling off the exposed pack surface, evaluated for
the configured optimistic/nominal/conservative pack-resistance cases.
Since the ADR-0014 amendment the vent model above uses the same
nominal-R I^2 R law, and the closure's eta_bat is the mission-averaged
efficiency of that resistance -- see config/battery.yaml.

References
----------
  Incropera & DeWitt, "Fundamentals of Heat and Mass Transfer",
  ch. 7 (external forced convection, flat plate).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import yaml


# --- Air properties at ISA sea level (film ~ 300 K) [documented constants] --
MU_AIR = 1.789e-5   # dynamic viscosity [Pa s]
K_AIR  = 0.0263     # thermal conductivity [W/(m K)]
CP_AIR = 1005.0     # specific heat at constant pressure [J/(kg K)]
G0     = 9.80665    # standard gravity [m/s^2]


def _prandtl() -> float:
    return MU_AIR * CP_AIR / K_AIR


# ---------------------------------------------
#  Input parameters (config/thermal.yaml)
# ---------------------------------------------
@dataclass
class ThermalParams:
    T_ambient_C:           float
    T_esc_max_C:           float
    T_batt_max_C:          float
    esc_inflow_frac:       float   # V over ESC plate / v_h [-]
    vent_inflow_frac:      float   # through-vent V / v_h [-]
    plate_thickness_m:     float
    rho_plate:             float
    plate_material:        str
    esc_wall_usable_frac:  float
    vent_wall_usable_frac: float
    # pack lumped transient (ADR-0014)
    c_p_pack_J_kgK:        float          # effective pack specific heat
    R_pack_ohm:            dict           # {case label: pack DCR [Ohm]}
    pack_exposed_frac:     float          # envelope fraction in cruise flow
    pack_dims_m:           tuple          # (L, W, H) pack envelope

    @classmethod
    def from_yaml(cls, path) -> "ThermalParams":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(
            T_ambient_C           = float(data["T_ambient_C"]),
            T_esc_max_C           = float(data["T_esc_max_C"]),
            T_batt_max_C          = float(data["T_batt_max_C"]),
            esc_inflow_frac       = float(data["esc_inflow_frac"]),
            vent_inflow_frac      = float(data["vent_inflow_frac"]),
            plate_thickness_m     = float(data["plate_thickness_m"]),
            rho_plate             = float(data["rho_plate"]),
            plate_material        = str(data["plate_material"]),
            esc_wall_usable_frac  = float(data["esc_wall_usable_frac"]),
            vent_wall_usable_frac = float(data["vent_wall_usable_frac"]),
            c_p_pack_J_kgK        = float(data["c_p_pack_J_kgK"]),
            R_pack_ohm            = {str(k): float(v) * 1e-3
                                     for k, v in data["R_pack_mohm"].items()},
            pack_exposed_frac     = float(data["pack_exposed_frac"]),
            pack_dims_m           = tuple(float(v) for v in data["pack_dims_m"]),
        )


# ---------------------------------------------
#  Per-path sizing outputs
# ---------------------------------------------
@dataclass
class EscPlateResult:
    Q_W:              float   # ESC heat load
    V_cool_ms:        float   # airflow over the plate
    A_req_m2:         float   # plate area to hold T_esc_max
    plate_side_mm:    float   # sqrt(A_req)
    plate_mass_kg:    float   # A_req * t * rho_plate
    A_avail_m2:       float   # usable inflow-washed wall
    area_headroom:    float   # A_avail / A_req  (>= 1 fits)
    T_at_avail_C:     float   # ESC temp if plate uses the full wall
    temp_margin_C:    float   # T_esc_max - T_at_avail
    mass_within_alloc: bool   # plate mass < ESC allocation
    ok:               bool


@dataclass
class BatteryVentResult:
    Q_W:            float   # battery heat load
    V_vent_ms:      float   # through-vent air velocity
    A_req_m2:       float   # vent area to hold T_batt_max
    A_avail_m2:     float   # usable battery-bay wall
    area_headroom:  float   # A_avail / A_req
    T_at_avail_C:   float   # pack temp at the full available vent area
    temp_margin_C:  float   # T_batt_max - T_at_avail
    ok:             bool


# ---------------------------------------------
#  Battery pack lumped transient (ADR-0014)
# ---------------------------------------------

#: Flat-plate transition Reynolds number -- above this the laminar
#: average-Nusselt correlation no longer applies (Incropera ch. 7).
RE_X_TRANSITION = 5.0e5


@dataclass
class PackTransientCase:
    """One pack-resistance case of the mission temperature transient."""
    label:            str
    R_pack_ohm:       float
    Q_hover_W:        float   # I_hover^2 R
    Q_cruise_W:       float   # I_cruise^2 R
    dT_leg_C:         float   # adiabatic rise per hover+transition leg
    T_after_leg1_C:   float
    T_after_cruise_C: float
    T_final_C:        float   # end of second leg = mission peak average
    temp_margin_C:    float   # T_batt_max - T_final
    ok:               bool    # T_final <= T_batt_max


@dataclass
class PackTransientResult:
    """Lumped-capacitance pack transient over the design mission.

    Model (C. Ucler external review, 2026-07): the mission is two
    hover+transition legs (adiabatic Joule heating -- hover bay airflow
    conservatively neglected) separated by the cruise segment, where the
    pack cools by laminar flat-plate forced convection over the exposed
    fraction of its envelope at V_cruise:

        T(t) = T_ss + (T_start - T_ss) exp(-t/tau),
        T_ss = T_ambient + Q_cruise/UA,   tau = m c_p / UA
    """
    I_hover_A:   float
    I_cruise_A:  float
    C_th_J_K:    float   # m_bat * c_p
    UA_W_K:      float   # h * A_exposed (cruise)
    tau_s:       float   # C_th / UA
    Re_L:        float   # cruise Reynolds number on the pack length
    t_leg_s:     float   # per-leg time at hover current
    t_cruise_s:  float
    cases:       dict    # {label: PackTransientCase}
    ok_nominal:  bool    # 'nominal' case stays under T_batt_max


def battery_pack_transient(
    m_bat_kg:    float,
    I_hover_A:   float,   # wiring-law hover current (P_hover / V_pack)
    I_cruise_A:  float,   # wiring-law cruise current
    t_hover_s:   float,   # total mission hover budget (split into 2 legs)
    t_transition_s: float,  # total transition budget, billed at hover current
    t_cruise_s:  float,
    V_cruise_ms: float,
    rho:         float,   # ambient air density [kg/m^3]
    p:           ThermalParams,
) -> PackTransientResult:
    """Mission temperature transient of the battery pack (ADR-0014).

    The two vertical legs each carry half the hover budget plus half the
    transition budget (transitions are billed at hover power, the same
    convention as the mass-closure energy model).
    """
    L, W, H = p.pack_dims_m
    A_pack = 2.0 * (L * W + L * H + W * H)
    A_exp  = A_pack * p.pack_exposed_frac

    # cruise cooling: laminar flat-plate average h on the pack length
    nu = MU_AIR / rho
    Pr = _prandtl()
    Re_L = V_cruise_ms * L / nu
    if Re_L >= RE_X_TRANSITION:
        raise ValueError(
            f"Pack cruise Re_L={Re_L:.3g} above laminar transition "
            f"{RE_X_TRANSITION:.1e}; extend the correlation before use.")
    Nu = 0.664 * math.sqrt(Re_L) * Pr ** (1.0 / 3.0)
    h  = Nu * K_AIR / L
    UA = h * A_exp

    C_th  = m_bat_kg * p.c_p_pack_J_kgK
    tau   = C_th / UA
    t_leg = 0.5 * t_hover_s + 0.5 * t_transition_s

    cases = {}
    for label, R in p.R_pack_ohm.items():
        Q_h = I_hover_A ** 2 * R
        Q_c = I_cruise_A ** 2 * R
        dT_leg = Q_h * t_leg / C_th                    # adiabatic leg
        T1 = p.T_ambient_C + dT_leg
        T_ss = p.T_ambient_C + Q_c / UA                # cruise equilibrium
        T2 = T_ss + (T1 - T_ss) * math.exp(-t_cruise_s / tau)
        T3 = T2 + dT_leg
        cases[label] = PackTransientCase(
            label=label, R_pack_ohm=R, Q_hover_W=Q_h, Q_cruise_W=Q_c,
            dT_leg_C=dT_leg, T_after_leg1_C=T1, T_after_cruise_C=T2,
            T_final_C=T3, temp_margin_C=p.T_batt_max_C - T3,
            ok=T3 <= p.T_batt_max_C,
        )

    if "nominal" not in cases:
        raise ValueError("R_pack_mohm must define a 'nominal' case")
    return PackTransientResult(
        I_hover_A=I_hover_A, I_cruise_A=I_cruise_A, C_th_J_K=C_th,
        UA_W_K=UA, tau_s=tau, Re_L=Re_L, t_leg_s=t_leg,
        t_cruise_s=t_cruise_s, cases=cases,
        ok_nominal=cases["nominal"].ok,
    )


# ---------------------------------------------
#  Main sizing routine
# ---------------------------------------------

def size_thermal(
    P_hover_W:        float,
    eta_esc:          float,
    I_hover_A:        float,   # wiring-law hover current (P_hover / V_pack)
    MTOW_kg:          float,
    D_rotor_m:        float,
    rho:              float,   # ambient air density [kg/m^3]
    D_int_m:          float,   # fuselage internal diameter [m]
    L_mid_m:          float,   # mid-body cylinder length (ESC plate zone) [m]
    L_battery_bay_m:  float,   # battery bay axial length [m]
    m_esc_alloc_kg:   float,   # ESC/propulsion mounts allocation [kg]
    p:                ThermalParams = None,
    g:                float = G0,
) -> dict:
    """Size both thermal paths.  See module docstring for the model."""
    if p is None:
        raise ValueError("ThermalParams required (config/thermal.yaml)")

    nu = MU_AIR / rho
    Pr = _prandtl()
    dT_esc  = p.T_esc_max_C - p.T_ambient_C
    dT_batt = p.T_batt_max_C - p.T_ambient_C
    if dT_esc <= 0 or dT_batt <= 0:
        raise ValueError("T_*_max must exceed T_ambient")

    # induced (inflow) velocity, actuator disk in hover
    A_disk = math.pi * (D_rotor_m / 2.0) ** 2
    v_h = math.sqrt(MTOW_kg * g / (2.0 * rho * A_disk))

    # ---- ESC cold-plate -------------------------------------------------
    Q_esc = P_hover_W * (1.0 - eta_esc)
    V_cool = p.esc_inflow_frac * v_h
    C_h = 0.664 * K_AIR * Pr ** (1.0 / 3.0) * math.sqrt(V_cool / nu)
    A_req_esc = (Q_esc / (C_h * dT_esc)) ** (4.0 / 3.0)
    plate_mass = A_req_esc * p.plate_thickness_m * p.rho_plate

    A_avail_esc = math.pi * D_int_m * L_mid_m * p.esc_wall_usable_frac
    T_at_avail_esc = p.T_ambient_C + Q_esc / (C_h * A_avail_esc ** 0.75)
    esc = EscPlateResult(
        Q_W=Q_esc, V_cool_ms=V_cool, A_req_m2=A_req_esc,
        plate_side_mm=math.sqrt(A_req_esc) * 1e3, plate_mass_kg=plate_mass,
        A_avail_m2=A_avail_esc, area_headroom=A_avail_esc / A_req_esc,
        T_at_avail_C=T_at_avail_esc, temp_margin_C=p.T_esc_max_C - T_at_avail_esc,
        mass_within_alloc=plate_mass < m_esc_alloc_kg,
        ok=(A_req_esc <= A_avail_esc) and (plate_mass < m_esc_alloc_kg),
    )

    # ---- vented battery bay --------------------------------------------
    Q_batt = I_hover_A ** 2 * p.R_pack_ohm["nominal"]
    V_vent = p.vent_inflow_frac * v_h
    mdot_req = Q_batt / (CP_AIR * dT_batt)
    A_req_vent = mdot_req / (rho * V_vent)

    A_avail_vent = math.pi * D_int_m * L_battery_bay_m * p.vent_wall_usable_frac
    mdot_avail = rho * V_vent * A_avail_vent
    dT_at_avail = Q_batt / (mdot_avail * CP_AIR)
    T_at_avail_batt = p.T_ambient_C + dT_at_avail
    batt = BatteryVentResult(
        Q_W=Q_batt, V_vent_ms=V_vent, A_req_m2=A_req_vent,
        A_avail_m2=A_avail_vent, area_headroom=A_avail_vent / A_req_vent,
        T_at_avail_C=T_at_avail_batt, temp_margin_C=p.T_batt_max_C - T_at_avail_batt,
        ok=A_req_vent <= A_avail_vent,
    )

    return {
        "v_h_ms": v_h,
        "Pr": Pr,
        "esc": esc,
        "battery": batt,
        "all_ok": esc.ok and batt.ok,
    }


# ---------------------------------------------
#  Handoff writer
# ---------------------------------------------

def write_thermal_yaml(res: dict, p: ThermalParams, path,
                       trans: PackTransientResult | None = None) -> None:
    """Write out/thermal.yaml -- consumed by the CAD notebook (cold-plate
    + vent geometry) and read for reference downstream.  ``trans`` appends
    the ADR-0014 pack mission transient as a reported (never filtered-on)
    block."""
    esc: EscPlateResult = res["esc"]
    batt: BatteryVentResult = res["battery"]
    data = {
        "T_ambient_C":  p.T_ambient_C,
        "v_h_ms":       round(res["v_h_ms"], 4),
        # ESC cold-plate
        "esc": {
            "Q_W":             round(esc.Q_W, 3),
            "V_cool_ms":       round(esc.V_cool_ms, 4),
            "A_req_cm2":       round(esc.A_req_m2 * 1e4, 3),
            "plate_side_mm":   round(esc.plate_side_mm, 2),
            "plate_thickness_mm": round(p.plate_thickness_m * 1e3, 3),
            "plate_material":  p.plate_material,
            "plate_mass_g":    round(esc.plate_mass_kg * 1e3, 2),
            "A_avail_cm2":     round(esc.A_avail_m2 * 1e4, 3),
            "area_headroom":   round(esc.area_headroom, 3),
            "T_esc_max_C":     p.T_esc_max_C,
            "T_at_avail_C":    round(esc.T_at_avail_C, 2),
            "temp_margin_C":   round(esc.temp_margin_C, 2),
            "mass_within_alloc": bool(esc.mass_within_alloc),
            "ok":              bool(esc.ok),
        },
        # vented battery bay
        "battery": {
            "Q_W":            round(batt.Q_W, 3),
            "V_vent_ms":      round(batt.V_vent_ms, 4),
            "A_req_cm2":      round(batt.A_req_m2 * 1e4, 3),
            "A_avail_cm2":    round(batt.A_avail_m2 * 1e4, 3),
            "area_headroom":  round(batt.area_headroom, 3),
            "T_batt_max_C":   p.T_batt_max_C,
            "T_at_avail_C":   round(batt.T_at_avail_C, 2),
            "temp_margin_C":  round(batt.temp_margin_C, 2),
            "ok":             bool(batt.ok),
        },
        "all_ok": bool(res["all_ok"]),
    }
    if trans is not None:
        data["battery_transient"] = {
            "I_hover_A":    round(trans.I_hover_A, 2),
            "I_cruise_A":   round(trans.I_cruise_A, 2),
            "C_th_J_K":     round(trans.C_th_J_K, 1),
            "UA_W_K":       round(trans.UA_W_K, 4),
            "tau_s":        round(trans.tau_s, 1),
            "t_leg_s":      round(trans.t_leg_s, 1),
            "t_cruise_s":   round(trans.t_cruise_s, 1),
            "T_batt_max_C": p.T_batt_max_C,
            "cases": {
                label: {
                    "R_pack_mohm":      round(c.R_pack_ohm * 1e3, 1),
                    "Q_hover_W":        round(c.Q_hover_W, 2),
                    "Q_cruise_W":       round(c.Q_cruise_W, 3),
                    "dT_leg_C":         round(c.dT_leg_C, 2),
                    "T_after_leg1_C":   round(c.T_after_leg1_C, 2),
                    "T_after_cruise_C": round(c.T_after_cruise_C, 2),
                    "T_final_C":        round(c.T_final_C, 2),
                    "temp_margin_C":    round(c.temp_margin_C, 2),
                    "ok":               bool(c.ok),
                }
                for label, c in trans.cases.items()
            },
            "ok_nominal": bool(trans.ok_nominal),
        }
    with open(path, "w", encoding="utf-8") as f:
        f.write("# AUTO-GENERATED -- do not edit by hand.\n")
        f.write("# Source : src/conceptual_design/thermal_design.py\n")
        f.write("# Input  : config/thermal.yaml + hover power + fuselage layout\n")
        f.write("# Regen  : re-run notebooks/thermal_design.ipynb\n\n")
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
