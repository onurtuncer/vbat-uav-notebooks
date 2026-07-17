"""
mass_properties.py  --  Rigid-Body Mass Properties, CG Trim and BOM
===================================================================

Refines the NB6 (fuselage_design) point-mass layout into a rigid-body
model: each layout item becomes an inertia primitive (solid cylinder,
thin tube, flat plate, shell of revolution, radial cluster, spanwise
rod), the assembly is trimmed to the target static margin by sliding
the battery tray, and the result is the inertia tensor about the CG in
body FRD plus the bill of materials.

Axis convention: body FRD, origin at the nose tip; stations from the
nose map to x_body = -station.  Only x-offsets appear (all primitives
are centred on the body axis or symmetric about it), so the parallel-
axis transport is diagonal and no products of inertia arise.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import pandas as pd
import yaml

from .fuselage_design import fuselage_radius


# ---------------------------------------------
#  Inertia primitives
# ---------------------------------------------
@dataclass
class Component:
    """Rigid component on the body x-axis.

    x_s   : CG station from nose, +aft [m]  (body x = -x_s)
    I_cg  : (Ixx, Iyy, Izz) about own CG, axes parallel to body FRD [kg m^2]
    """
    name:     str
    mass_kg:  float
    x_s:      float
    I_cg:     tuple
    primitive: str


def solid_cylinder(name, m, x_s, r, L):
    Ixx = 0.5 * m * r**2
    Itr = m * (3.0 * r**2 + L**2) / 12.0
    return Component(name, m, x_s, (Ixx, Itr, Itr), "solid cylinder")


def thin_tube(name, m, x_s, r_mean, L):
    Ixx = m * r_mean**2
    Itr = m * (r_mean**2 / 2.0 + L**2 / 12.0)
    return Component(name, m, x_s, (Ixx, Itr, Itr), "thin tube")


def flat_plate_wing(name, m, x_s, b, c, t):
    Ixx = m * (b**2 + t**2) / 12.0
    Iyy = m * (c**2 + t**2) / 12.0
    Izz = m * (b**2 + c**2) / 12.0
    return Component(name, m, x_s, (Ixx, Iyy, Izz), "flat plate")


def shell_of_revolution(name, m, x_cg_layout, D, L, f_nose, f_tail, r_hub, n=800):
    """Thin monocoque shell using the NB4/NB5 meridian, scaled to mass m.

    The meridian integration provides the *shape* (second moments about the
    shell's own CG); the CG station itself is pinned to the NB4 layout value
    so the vehicle-level mass bookkeeping closes exactly.  (The layout CG
    includes the frame/carry-through allocation (explicit members since ADR-0010), which the bare
    skin integral does not know about.)
    """
    x  = np.linspace(0.0, L, n + 1)
    r  = np.array([fuselage_radius(xi, D, L, f_nose, f_tail, r_hub) for xi in x])
    dr = np.gradient(r, x)
    ds = np.sqrt(1.0 + dr**2)                    # arc-length factor
    w  = 2.0 * np.pi * r * ds                    # dA/dx  (sigma = 1)

    A      = np.trapezoid(w, x)                  # total area  -> sigma = m/A
    x_cg   = np.trapezoid(w * x, x) / A          # skin-only CG (shape)
    Ixx    = (m / A) * np.trapezoid(w * r**2, x)
    Iyy_0  = (m / A) * np.trapezoid(w * (r**2 / 2.0 + x**2), x)   # about nose
    Iyy_cg = Iyy_0 - m * x_cg**2                 # parallel axis -> shell CG
    return Component(name, m, x_cg_layout, (Ixx, Iyy_cg, Iyy_cg),
                     "shell of revolution")


def radial_cluster(name, m_each, n, x_s, d, primitive="radial cluster"):
    """n identical point masses at radius d, distributed symmetrically
    about the body x-axis at station x_s (vanes, servos, linkages).

    For a symmetric set (e.g. n = 4 at T/B/L/R):
        Ixx = n m d^2,   Iyy = Izz = (n/2) m d^2
    Products of inertia cancel by symmetry.
    """
    m = n * m_each
    Ixx = m * d**2
    Itr = 0.5 * m * d**2
    return Component(name, m, x_s, (Ixx, Itr, Itr), primitive)


def thin_spanwise_rod(name, m, x_s, length_y):
    """Slender rod along the body y-axis (wing spar tube):
    Ixx = Izz = m L^2/12 (bending about x and z), Iyy ~ 0.
    """
    I_long = m * length_y**2 / 12.0
    return Component(name, m, x_s, (I_long, 0.0, I_long), "spanwise rod")


def assemble(components):
    """Total mass, CG station, and inertia tensor about the CG (body FRD)."""
    m_tot = sum(c.mass_kg for c in components)
    x_cg  = sum(c.mass_kg * c.x_s for c in components) / m_tot
    I_tot = np.zeros((3, 3))
    for c in components:
        d = c.x_s - x_cg                         # offset along x only
        I_tot += np.diag(c.I_cg)
        I_tot += c.mass_kg * np.diag([0.0, d**2, d**2])   # parallel axis
    return m_tot, x_cg, I_tot


# ---------------------------------------------
#  Assembly result container
# ---------------------------------------------
@dataclass
class MassProperties:
    """Rigid-body assembly: components, battery-tray trim, inertia tensor."""

    components: list          # as-trimmed Component list

    # NB6 geometry pulled through for plotting / BOM
    b_wing_m:     float
    chord_wing_m: float
    t_wing_m:     float
    D_fus:        float
    L_fus:        float
    R_int:        float       # internal bay radius
    r_eff:        float       # packing-equivalent bay radius

    # control / aileron hardware split (per unit)
    m_vane_each:  float
    m_servo_each: float
    m_link_each:  float
    m_ctl_total:  float
    m_aileron_servo:    float
    m_aileron_linkage:  float
    m_aileron_hw_total: float
    x_vane_te:  float
    x_hinge:    float         # vane hinge station
    r_servo_cg: float         # recessed servo-case CG radius

    # as-packaged (nominal battery station)
    m_tot0:  float
    x_cg0:   float
    I_cg0:   np.ndarray
    SM_ref0: float
    dx_cg0:  float            # as-packaged CG - NB4 handoff CG

    # battery-tray trim
    SM_nb4:         float     # NB4 target static margin
    travel:         float     # rail travel limit [m]
    x_battery_trim: float     # required slide, +aft [m]
    trim_ok:        bool

    # as-trimmed
    m_tot:     float
    x_cg:      float
    I_cg:      np.ndarray
    r_cg_body: np.ndarray     # [-x_cg, 0, 0] body FRD
    k:         np.ndarray     # radii of gyration
    SM_ref:    float

    @property
    def Ixx(self):
        return float(self.I_cg[0, 0])

    @property
    def Iyy(self):
        return float(self.I_cg[1, 1])

    @property
    def Izz(self):
        return float(self.I_cg[2, 2])


def compute_mass_properties(
    result,                # SizingResult (mass closure)
    fus: dict,             # out/fuselage.yaml
    vanes: dict,           # out/control_vanes.yaml
    ail: dict,             # out/aileron.yaml
    fus_cfg: dict,         # config/fuselage.yaml
    tc_ratio: float,       # wing t/c (config/wing_structure_params.yaml)
    servo_recess_m: float,     # servo case recess below hub surface [m]
    servo_case_h_m: float,     # servo case height [m]
) -> MassProperties:
    """Build the component table from the NB6 layout, trim the battery
    tray to the NB4 target static margin, and assemble the inertia tensor.

    Raises AssertionError if the notebook-level mass bookkeeping breaks
    (control/aileron splits vs the NB6 layout, symmetry of the tensor).
    """
    layout = {c["name"]: c for c in fus["layout"]}
    m_layout = sum(c["mass_kg"] for c in fus["layout"])

    b_wing_m     = result.wing.b_wing
    chord_wing_m = result.wing.chord_mean
    t_wing_m     = tc_ratio * chord_wing_m

    D_fus, L_fus = fus["D_fus_m"], fus["L_fus_m"]
    R_int = D_fus / 2.0 - fus_cfg["t_shell_m"]     # internal bay radius
    pack  = fus_cfg["packing_factor"]
    r_eff = math.sqrt(pack) * R_int                # preserves packed volume

    # ---- control hardware -- NB4 already places this at its true station,
    # re-derive the same per-vane split to give each cluster its own radius
    n_vanes      = vanes["n_vanes"]
    m_vane_each  = vanes["S_vane_m2"] * fus_cfg["t_vane_plate_m"] * fus_cfg["rho_shell"]
    m_servo_each = fus_cfg["servo_mass_kg"]
    m_link_each  = fus_cfg["linkage_mass_kg"]

    # NB4's handoff layout rounds masses to 4 d.p. (0.1 g) before writing
    # YAML, so compare with 1 g slack rather than exact equality.
    m_ctl_total = n_vanes * (m_vane_each + m_servo_each + m_link_each)
    assert abs(m_ctl_total - layout["control_hw"]["mass_kg"]) < 1e-3, (
        "control-hw mass split does not match the NB4 layout allocation"
    )

    # stations & radii, matching the NB5 CAD placement
    x_vane_te  = fus["x_vane_m"] + 0.5 * vanes["c_vane_m"]
    x_hinge    = x_vane_te - (1.0 - vanes["hinge_xc"]) * vanes["c_vane_m"]
    r_servo_cg = vanes["R_hub_m"] - servo_recess_m + servo_case_h_m / 2.0

    # ---- aileron hardware -- NB4 books this at the wing station
    n_ailerons        = ail["n_ailerons"]
    m_aileron_servo   = ail["servo_mass_kg_each"]
    m_aileron_linkage = ail["linkage_mass_kg_each"]
    m_aileron_hw_total = n_ailerons * (m_aileron_servo + m_aileron_linkage)
    assert abs(m_aileron_hw_total - layout["aileron_hw"]["mass_kg"]) < 1e-3, (
        "aileron-hw mass split does not match the NB4 layout allocation"
    )

    m_isolation_hw = layout["isolation_hw"]["mass_kg"]
    m_joint_hw = layout["joint_hw"]["mass_kg"]
    m_spar_hw  = layout["spar_hw"]["mass_kg"]

    components = [
        solid_cylinder("payload",  layout["payload"]["mass_kg"],
                       layout["payload"]["x_cg_m"],  r_eff, layout["payload"]["length_m"]),
        solid_cylinder("avionics", layout["avionics"]["mass_kg"],
                       layout["avionics"]["x_cg_m"], r_eff, layout["avionics"]["length_m"]),
        solid_cylinder("battery",  layout["battery"]["mass_kg"],
                       layout["battery"]["x_cg_m"],  r_eff, layout["battery"]["length_m"]),
        solid_cylinder("battery_tray", layout["battery_tray"]["mass_kg"],
                       layout["battery_tray"]["x_cg_m"], r_eff, 0.005),
        solid_cylinder("esc",      layout["esc"]["mass_kg"],
                       layout["esc"]["x_cg_m"],      r_eff, layout["esc"]["length_m"]),
        solid_cylinder("motor_fan", layout["motor_fan"]["mass_kg"],
                       layout["motor_fan"]["x_cg_m"], fus["r_hub_m"], 0.060),
        thin_tube("duct", layout["duct"]["mass_kg"], layout["duct"]["x_cg_m"],
                  (fus["D_duct_inner_m"] + fus["D_duct_outer_m"]) / 4.0,
                  fus["duct_chord_m"]),
        shell_of_revolution("shell_struct", layout["shell_struct"]["mass_kg"],
                            layout["shell_struct"]["x_cg_m"],
                            D_fus, L_fus, fus["f_nose"], fus["f_tail"], fus["r_hub_m"]),
        flat_plate_wing("wing", layout["wing"]["mass_kg"], layout["wing"]["x_cg_m"],
                        b_wing_m, chord_wing_m, t_wing_m),
        radial_cluster("vanes (4x)",    m_vane_each,  n_vanes,
                       fus["x_vane_m"], vanes["R_mid_m"], "radial cluster"),
        radial_cluster("servos (4x)",   m_servo_each, n_vanes,
                       x_hinge, r_servo_cg, "radial cluster"),
        radial_cluster("linkages (4x)", m_link_each,  n_vanes,
                       x_hinge, vanes["R_hub_m"], "radial cluster"),
        radial_cluster("aileron_hw (2x)", m_aileron_servo + m_aileron_linkage,
                       n_ailerons, layout["aileron_hw"]["x_cg_m"], ail["y_arm_m"],
                       "radial cluster"),
        solid_cylinder("isolation_hw", m_isolation_hw,
                       layout["isolation_hw"]["x_cg_m"], 0.7 * r_eff, 0.01),
        solid_cylinder("joint_hw", m_joint_hw,
                       layout["joint_hw"]["x_cg_m"], 0.8 * r_eff, 0.012),
        # spar: thin tube spanning the wing carry-through, axis along y --
        # model as a rod for Ixx (its length is spanwise, like the wing)
        thin_spanwise_rod("spar_hw", m_spar_hw,
                          layout["spar_hw"]["x_cg_m"], fus["spar_length_m"]),
        solid_cylinder("misc (rem.)", layout["misc"]["mass_kg"],
                       layout["misc"]["x_cg_m"], R_int / 2.0, L_fus / 2.0),
    ]

    # ---- as-packaged assembly (nominal battery station) ------------------
    m_tot0, x_cg0, I_cg0 = assemble(components)

    SM_nb4  = fus["static_margin"]
    SM_ref0 = (fus["x_wing_AC_m"] - x_cg0) / chord_wing_m
    dx_cg0  = x_cg0 - fus["x_CG_m"]                # as-packaged - NB4 handoff

    # ---- battery-tray CG trim --------------------------------------------
    # Solve the fore-aft slide on the battery tray that restores the NB4
    # target static margin exactly, check it against the rail travel limit
    # from config/fuselage.yaml, then apply it to battery + tray stations.
    travel         = fus["battery_tray_travel_m"]
    m_battery      = layout["battery"]["mass_kg"]
    x_cg_target    = fus["x_wing_AC_m"] - SM_nb4 * chord_wing_m
    x_battery_trim = (x_cg_target - x_cg0) * m_tot0 / m_battery
    trim_ok        = abs(x_battery_trim) <= travel

    for c in components:
        if c.name in ("battery", "battery_tray"):
            c.x_s += x_battery_trim

    m_tot, x_cg_s, I_cg = assemble(components)     # as-trimmed
    r_cg_body = np.array([-x_cg_s, 0.0, 0.0])      # FRD, origin at nose tip
    k = np.sqrt(np.diag(I_cg) / m_tot)             # radii of gyration
    SM_ref = (fus["x_wing_AC_m"] - x_cg_s) / chord_wing_m

    # out/fuselage.yaml rounds each layout mass to 4 d.p. (0.1 g) before
    # writing YAML, while m_tot re-derives vanes/servos/linkages from the
    # exact (unrounded) per-vane formula -- allow 1 g slack.
    assert abs(m_tot - m_layout) < 1e-3, "mass bookkeeping broken"
    assert np.allclose(I_cg, np.diag(np.diag(I_cg))), "unexpected products of inertia"

    return MassProperties(
        components=components,
        b_wing_m=b_wing_m, chord_wing_m=chord_wing_m, t_wing_m=t_wing_m,
        D_fus=D_fus, L_fus=L_fus, R_int=R_int, r_eff=r_eff,
        m_vane_each=m_vane_each, m_servo_each=m_servo_each,
        m_link_each=m_link_each, m_ctl_total=m_ctl_total,
        m_aileron_servo=m_aileron_servo, m_aileron_linkage=m_aileron_linkage,
        m_aileron_hw_total=m_aileron_hw_total,
        x_vane_te=x_vane_te, x_hinge=x_hinge, r_servo_cg=r_servo_cg,
        m_tot0=m_tot0, x_cg0=x_cg0, I_cg0=I_cg0, SM_ref0=SM_ref0, dx_cg0=dx_cg0,
        SM_nb4=SM_nb4, travel=travel, x_battery_trim=x_battery_trim,
        trim_ok=trim_ok,
        m_tot=m_tot, x_cg=x_cg_s, I_cg=I_cg, r_cg_body=r_cg_body, k=k,
        SM_ref=SM_ref,
    )


def inertia_contributions(mp: MassProperties):
    """Per-component inertia build-up about the vehicle CG.

    Returns (names, own, transport): own-CG terms and parallel-axis
    transport terms, each an (n, 3) array of (Ixx, Iyy, Izz).
    """
    names  = [c.name for c in mp.components]
    own    = np.array([c.I_cg for c in mp.components])
    transp = np.array([[0.0,
                        c.mass_kg * (c.x_s - mp.x_cg)**2,
                        c.mass_kg * (c.x_s - mp.x_cg)**2]
                       for c in mp.components])
    return names, own, transp


# ---------------------------------------------
#  Bill of materials
# ---------------------------------------------

def build_bom(
    mp: MassProperties,
    fus: dict, vanes: dict, ail: dict, vib: dict, af: dict,
    fus_cfg: dict, fus_cfg_mod: dict,
    D_rotor_m: float,
    servo_torque_avail_gcm: float,   # candidate vane-servo class torque
) -> pd.DataFrame:
    """Assemble the BOM DataFrame from the trimmed component stations.

    Raises AssertionError if the BOM mass does not close against MTOW.
    """
    layout  = {c["name"]: c for c in fus["layout"]}
    comp_x  = {c.name: c.x_s for c in mp.components}   # as-trimmed stations
    n_vanes    = vanes["n_vanes"]
    n_ailerons = ail["n_ailerons"]
    tau_req    = vanes["servo_torque_req_gcm"]

    rows = [
     # part_no        item                          qty  material / spec        unit_kg  x_cg_mm
     ("VBT-STR-001", "Fuselage semi-monocoque",      1,
      f"{fus['construction_method']} skin + longeron frame + rings (ADR-0010)",
      layout["shell_struct"]["mass_kg"], layout["shell_struct"]["x_cg_m"]*1e3),
     ("VBT-STR-002", "Wing, constant chord",         1, f"CFRP skin/foam core, {af['designation']}",
      layout["wing"]["mass_kg"], layout["wing"]["x_cg_m"]*1e3),
     ("VBT-PRP-001", "EDF motor + fan rotor",        1, f"D = {D_rotor_m*1e3:.0f} mm EDF, BLDC outrunner",
      layout["motor_fan"]["mass_kg"], layout["motor_fan"]["x_cg_m"]*1e3),
     ("VBT-PRP-002", "Duct / shroud annulus",        1, "CFRP wound annulus, 8 mm wall",
      layout["duct"]["mass_kg"], layout["duct"]["x_cg_m"]*1e3),
     ("VBT-PWR-001", "Battery pack",                 1, "LiPo, 140 Wh/kg pack-level",
      layout["battery"]["mass_kg"], comp_x["battery"]*1e3),
     ("VBT-PWR-002", "ESC",                          1, "sized to hover power; incl. inflow-washed cold-plate (NB7)",
      layout["esc"]["mass_kg"], layout["esc"]["x_cg_m"]*1e3),
     ("VBT-AVI-001", "Avionics stack",               1, "FC + GPS + RX + telemetry",
      layout["avionics"]["mass_kg"], layout["avionics"]["x_cg_m"]*1e3),
     ("VBT-PLD-001", "Payload (EO sensor + mount)",  1, "requirement, 0.5 kg",
      layout["payload"]["mass_kg"], layout["payload"]["x_cg_m"]*1e3),
     # -- control hardware, placed by NB4 at the true aft hinge station -----
     ("VBT-CTL-001", "Jet control vane", n_vanes,
      f"CFRP flat plate {fus_cfg['t_vane_plate_m']*1e3:.0f} mm, c={vanes['c_vane_m']*1e3:.1f} mm, "
      f"h={vanes['h_vane_m']*1e3:.1f} mm",
      mp.m_vane_each, fus["x_vane_m"]*1e3),
     ("VBT-CTL-002", "Vane servo, 9g-class", n_vanes,
      f"~{servo_torque_avail_gcm:.0f} g-cm >= req {tau_req:.0f} g-cm, "
      f"recessed in hub, shaft on hinge line",
      mp.m_servo_each, mp.x_hinge*1e3),
     ("VBT-CTL-003", "Hinge + pushrod set", n_vanes,
      "pin hinge + wire pushrod per vane",
      mp.m_link_each, mp.x_hinge*1e3),
     # -- aileron hardware, cruise roll backup (NB4 aileron_design) ----------
     ("VBT-CTL-004", "Aileron servo, 9g-class", n_ailerons,
      f">= req {ail['servo_torque_req_gcm']:.0f} g-cm, on wing at y_arm="
      f"{ail['y_arm_m']*1e3:.0f} mm",
      mp.m_aileron_servo, comp_x["aileron_hw (2x)"]*1e3),
     ("VBT-CTL-005", "Aileron hinge + pushrod set", n_ailerons,
      "pin hinge + wire pushrod per aileron",
      mp.m_aileron_linkage, comp_x["aileron_hw (2x)"]*1e3),
     # -- vibration isolators, FC/IMU + payload soft mounts (NB5) ------------
     ("VBT-STR-092", "Vibration isolators", vib["modules"]["fc_imu"]["n_isolators"]
                     + vib["modules"]["payload"]["n_isolators"],
      f"FC/IMU + payload soft mounts, f_n={vib['f_n_hz']:.0f} Hz, "
      f"{vib['modules']['fc_imu']['attenuation_pct']:.0f}% 1/rev cut, {vib['sway_mm']:.1f} mm sway",
      vib["modules"]["fc_imu"]["hw_mass_kg"] / vib["modules"]["fc_imu"]["n_isolators"],
      comp_x["isolation_hw"]*1e3),
     ("VBT-STR-093", "Lid hinge strip", 1,
      f"piano hinge along one longeron, lid [0, {fus['x_clam_aft_m']*1e3:.0f}] mm",
      fus_cfg_mod["lid_hinge_mass_kg"], fus["x_clam_aft_m"]/2*1e3),
     ("VBT-STR-094", "Lid latches", 1,
      "2x quarter-turn on the opposite longeron + reinforcement",
      fus_cfg_mod["lid_latch_mass_kg"], fus["x_clam_aft_m"]/2*1e3),
     ("VBT-STR-095", "Wing carry-through spar", 1,
      f"CFRP tube d{fus['spar_od_m']*1e3:.0f} x {fus['spar_length_m']*1e3:.0f} mm (computed geometry)",
      # layout-consistent remainder of spar_hw after the 2 root fittings
      layout["spar_hw"]["mass_kg"] - 2*fus_cfg_mod["wing_root_fitting_mass_kg"],
      layout["spar_hw"]["x_cg_m"]*1e3),
     ("VBT-STR-096", "Wing root fitting", 2,
      "printed socket + retention screw per side",
      fus_cfg_mod["wing_root_fitting_mass_kg"], layout["spar_hw"]["x_cg_m"]*1e3),
     ("VBT-STR-091", "Battery tray, adjustable rail", 1,
      f"sliding rail, +/-{mp.travel*1e3:.0f} mm travel, trimmed {mp.x_battery_trim*1e3:+.1f} mm",
      layout["battery_tray"]["mass_kg"], comp_x["battery_tray"]*1e3),
     ("VBT-STR-090", "Misc: harness, fasteners, bonding, margin", 1, "allocation (remainder)",
      layout["misc"]["mass_kg"], layout["misc"]["x_cg_m"]*1e3),
     # -- geometry carried inside the structure fraction --------------------
     ("VBT-STR-010", "Duct support strut", 4, "flat plate, min 2 mm (in structure)",
      None, layout["duct"]["x_cg_m"]*1e3),
     ("VBT-GEA-001", "Landing leg / skid", 4, "on duct, tail-sitter stance (in structure)",
      None, fus["x_vane_m"]*1e3),
    ]

    bom = pd.DataFrame(rows, columns=["part_no", "item", "qty",
                                      "material_spec", "unit_mass_kg", "x_cg_mm"])
    bom["total_mass_kg"] = bom["unit_mass_kg"] * bom["qty"]
    bom["x_cg_mm"] = bom["x_cg_mm"].round(1)

    m_bom = bom["total_mass_kg"].sum()
    assert abs(m_bom - mp.m_tot) < 1e-6, "BOM does not close against MTOW"
    return bom


# ---------------------------------------------
#  Handoff writer
# ---------------------------------------------

def write_mass_properties_yaml(mp: MassProperties, fus: dict, path) -> None:
    """Write out/mass_properties.yaml -- the rigid-body handoff consumed
    by the 6-DoF simulation and the design summary."""
    handoff = {
        "axis_convention": fus["axis_convention"],
        "m_total_kg":  float(round(mp.m_tot, 5)),
        "x_CG_m":      float(round(mp.x_cg, 5)),       # as-trimmed station from nose, +aft
        "x_CG_nb4_m":  float(fus["x_CG_m"]),           # NB4 estimate, for reference
        "x_CG_as_packaged_m": float(round(mp.x_cg0, 5)),  # before battery-tray trim
        "x_battery_trim_m":      float(round(mp.x_battery_trim, 5)),
        "battery_tray_travel_m": float(mp.travel),
        "battery_trim_within_travel": bool(mp.trim_ok),
        "static_margin_refined":    float(round(mp.SM_ref, 4)),   # as-trimmed
        "static_margin_as_packaged": float(round(mp.SM_ref0, 4)),
        "static_margin_nb4":     float(mp.SM_nb4),
        "r_CG_body_m": [float(round(v, 5)) for v in mp.r_cg_body],
        "inertia_about_CG_body_FRD": {
            "Ixx_kgm2": float(round(mp.Ixx, 6)),
            "Iyy_kgm2": float(round(mp.Iyy, 6)),
            "Izz_kgm2": float(round(mp.Izz, 6)),
            "Ixy_kgm2": 0.0, "Ixz_kgm2": 0.0, "Iyz_kgm2": 0.0,
        },
        "radii_of_gyration_m": {
            "k_x": float(round(mp.k[0], 5)),
            "k_y": float(round(mp.k[1], 5)),
            "k_z": float(round(mp.k[2], 5)),
        },
        "components": [
            {"name": c.name, "primitive": c.primitive,
             "mass_kg": float(round(c.mass_kg, 5)),
             "x_cg_m":  float(round(c.x_s, 5)),
             "Ixx_kgm2": float(round(c.I_cg[0], 7)),
             "Iyy_kgm2": float(round(c.I_cg[1], 7)),
             "Izz_kgm2": float(round(c.I_cg[2], 7))}
            for c in mp.components
        ],
    }

    header = (
        "# AUTO-GENERATED -- do not edit by hand.\n"
        "# Source : notebooks/mass_properties.ipynb\n"
        "# Input  : out/fuselage.yaml layout + mass closure + config/\n"
        "# Regen  : re-run notebooks/mass_properties.ipynb\n"
    )
    with open(path, "w") as f:
        f.write(header)
        yaml.safe_dump(handoff, f, sort_keys=False)
