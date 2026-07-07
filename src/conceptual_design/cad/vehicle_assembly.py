"""
vehicle_assembly.py  --  Full tail-sitter solid model (CadQuery / OCCT)

Assembles the sized components into one parametric vehicle:

    fuselage      body of revolution        (out/fuselage.yaml)
    wing          NACA section, constant chord  (mass closure + out/airfoil.yaml)
    duct          annular EDF shroud        (out/fuselage.yaml)
    duct struts   4x flat plates fuselage->duct
    vanes         4x jet control vanes      (out/control_vanes.yaml)
    servos        4x 9g-class vane servos, recessed in the centerbody
                  hub, output shaft on the vane hinge line
    legs          4x landing skids on the duct (tail-sitter stands on them)

AXIS CONVENTION (Aetherion):  body FRD -- x forward (out the nose),
y right, z down.  Origin at the nose tip; stations x_s map to x = -x_s.
Vane placement follows NB3's aft-view labels: T=+z, B=-z, L=+y, R=-y,
with struts and legs rotated 45 deg between the vanes.

Units: geometry is built in MILLIMETRES (STEP/STL convention); all
public arguments are METRES, matching the out/*.yaml handoff files.
"""

from __future__ import annotations

from typing import Dict, Tuple

import cadquery as cq

from .fuselage_body import fuselage_solid, MM
from .wing_profile import naca4_points
from conceptual_design.fuselage_design import fuselage_radius

# minimum manufacturable plate thickness for flat parts [mm]
T_PLATE_MIN_MM = 2.0

VANE_ANGLES = {"T": 0.0, "B": 180.0, "L": 270.0, "R": 90.0}  # about +x from +z
LEG_ANGLES  = (45.0, 135.0, 225.0, 315.0)


# ---------------------------------------------
#  Components
# ---------------------------------------------

def wing_solid(
    designation:  str,
    chord_m:      float,
    span_m:       float,
    x_wing_LE_m:  float,
    n:            int = 120,
) -> cq.Workplane:
    """
    Constant-chord wing through the fuselage, spanning y in [-b/2, +b/2].

    Section plane is body XZ: chord runs aft from the LE station
    (x = -x_LE at the leading edge), suction side toward -z (up in
    cruise).  Airfoil (xc, yc) on unit chord maps to:

        x = -(x_LE + xc * c),      z = -yc * c
    """
    c = chord_m * MM
    x_le = x_wing_LE_m * MM
    pts = [(-(x_le + xc * c), -yc * c) for xc, yc in naca4_points(designation, n=n)]

    # Workplane("XZ") local (u, v) -> global (u, 0, v); extrude is +/- y.
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.extrude(span_m / 2.0 * MM, both=True)


def duct_solid(
    D_inner_m:   float,
    D_outer_m:   float,
    chord_m:     float,
    x_center_m:  float,
) -> cq.Workplane:
    """Annular duct centred (axially) at station x_center_m."""
    ri = D_inner_m / 2.0 * MM
    ro = D_outer_m / 2.0 * MM
    ch = chord_m * MM
    x0 = -(x_center_m * MM + ch / 2.0)   # body x of the aft (exit) face

    ring = (
        cq.Workplane("YZ", origin=(x0, 0, 0))
        .circle(ro).circle(ri)
        .extrude(ch)          # +x normal: extrudes forward (toward inlet)
    )
    try:
        # soften the inlet/exit lips; cosmetic -- skip if OCCT refuses
        return ring.edges("%CIRCLE").chamfer(0.15 * (ro - ri))
    except Exception:
        return ring


def _servo_box(
    x_hinge_m:  float,   # station of the vane hinge line [m, +aft]
    r_hub_m:    float,   # exhaust centerbody (hub) radius [m]
    angle_deg:  float,   # rotation about +x, 0 = +z (same convention as vanes)
    body_l_mm:  float = 23.0,   # 9g-class servo case: L x W x H
    body_w_mm:  float = 12.4,
    body_h_mm:  float = 27.0,
    shaft_d_mm: float = 5.0,
    recess_mm:  float = 22.0,   # how deep the case sits inside the hub
) -> cq.Workplane:
    """
    9g-class vane servo, recessed into the exhaust centerbody with its
    output shaft on the vane hinge line (radial axis).

    Case long axis runs fore-aft (body x); the shaft protrudes radially
    to meet the vane root.  Built along +z, then rotated about +x by the
    same aft-view angle used for the vane it drives.
    """
    x0 = -(x_hinge_m * MM)                     # body x of the hinge line
    r_top = r_hub_m * MM - recess_mm + body_h_mm   # outer face of the case

    case = (
        cq.Workplane("XY", origin=(x0, 0, 0))
        # servo shaft is offset toward one end of the case: put the shaft
        # on the hinge line and let the case extend forward (+x)
        .box(body_l_mm, body_w_mm, body_h_mm,
             centered=(False, True, False))
        .translate((-0.25 * body_l_mm, 0, r_hub_m * MM - recess_mm))
    )
    shaft = (
        cq.Workplane("XY", origin=(x0, 0, r_top))
        .circle(shaft_d_mm / 2.0)
        .extrude(r_hub_m * MM - r_top + 1.0)   # reach the hub surface (+1 mm)
    )
    return case.union(shaft).rotate((0, 0, 0), (1, 0, 0), angle_deg)


def _radial_plate(
    x_aft_m:   float,   # station of the plate's aft edge [m, +aft]
    chord_m:   float,   # axial extent [m]
    r_in_m:    float,   # inner radius [m]
    r_out_m:   float,   # outer radius [m]
    t_mm:      float,   # tangential thickness [mm]
    angle_deg: float,   # rotation about +x, 0 = +z
) -> cq.Workplane:
    """Flat radial plate (vane / strut primitive), oriented along +z, rotated."""
    ch = chord_m * MM
    plate = (
        cq.Workplane("XY", origin=(-(x_aft_m * MM) , 0, 0))
        .box(ch, t_mm, (r_out_m - r_in_m) * MM, centered=(False, True, False))
        .translate((0, 0, r_in_m * MM))
    )
    return plate.rotate((0, 0, 0), (1, 0, 0), angle_deg)


def build_vehicle(
    fus:    Dict,            # out/fuselage.yaml
    vanes:  Dict,            # out/control_vanes.yaml
    b_wing_m:     float,     # wing span (mass closure)
    chord_wing_m: float,     # wing chord (mass closure)
    wing_designation: str,   # e.g. "NACA 2412" (out/airfoil.yaml)
) -> Tuple[cq.Assembly, cq.Workplane]:
    """
    Build the complete vehicle.

    Returns (assembly, fused):
      assembly -- cq.Assembly with named, coloured parts (STEP export)
      fused    -- single boolean-united solid (STL export / mass props)
    """
    L   = fus["L_fus_m"]
    D   = fus["D_fus_m"]

    # -- fuselage ------------------------------------------------------
    fuselage = fuselage_solid(D, L, fus["f_nose"], fus["f_tail"], fus["r_hub_m"])

    # -- wing ----------------------------------------------------------
    wing = wing_solid(wing_designation, chord_wing_m, b_wing_m, fus["x_wing_LE_m"])

    # -- duct ----------------------------------------------------------
    x_duct_c = next(it["x_cg_m"] for it in fus["layout"] if it["name"] == "duct")
    duct = duct_solid(fus["D_duct_inner_m"], fus["D_duct_outer_m"],
                      fus["duct_chord_m"], x_duct_c)
    duct_le_station = x_duct_c - fus["duct_chord_m"] / 2.0
    duct_te_station = x_duct_c + fus["duct_chord_m"] / 2.0

    # -- duct support struts (between the vane axes, at 45 deg) ---------
    strut_ch    = 0.35 * fus["duct_chord_m"]
    x_strut_aft = duct_le_station + 0.5 * fus["duct_chord_m"]
    r_body      = fuselage_radius(x_strut_aft - 0.5 * strut_ch, D, L,
                                  fus["f_nose"], fus["f_tail"], fus["r_hub_m"])
    struts = [
        _radial_plate(x_strut_aft, strut_ch,
                      r_body - 0.004,                       # embed into body
                      fus["D_duct_inner_m"] / 2.0 + 0.002,  # embed into duct
                      3.0, ang)
        for ang in LEG_ANGLES
    ]

    # -- control vanes (T=+z, B=-z, L=+y, R=-y) -------------------------
    t_vane = max(T_PLATE_MIN_MM, vanes["tc_ratio"] * vanes["c_vane_m"] * MM)
    x_vane_te = fus["x_vane_m"] + 0.5 * vanes["c_vane_m"]
    vane_parts = {
        # root embedded 4 mm into the centerbody so the union is watertight
        name: _radial_plate(x_vane_te, vanes["c_vane_m"],
                            vanes["R_hub_m"] - 0.004, vanes["R_tip_m"], t_vane, ang)
        for name, ang in VANE_ANGLES.items()
    }

    # -- vane servos: one per vane, recessed in the centerbody hub with
    #    the output shaft on the hinge line (hinge_xc of the vane chord)
    x_hinge = x_vane_te - (1.0 - vanes.get("hinge_xc", 0.25)) * vanes["c_vane_m"]
    servo_parts = {
        name: _servo_box(x_hinge, vanes["R_hub_m"], ang)
        for name, ang in VANE_ANGLES.items()
    }

    # -- exhaust centerbody: hub cylinder carrying the vane hinges, plus
    #    an exhaust cone.  Without it the vanes would float in the jet.
    r_hub_mm = vanes["R_hub_m"] * MM
    cyl_len  = x_vane_te - L
    cone_len_m = 0.018
    centerbody = (
        cq.Workplane("YZ", origin=(-(x_vane_te * MM), 0, 0))
        .circle(r_hub_mm)
        .extrude(cyl_len * MM)          # +x: forward to the fan face
        .union(cq.Workplane(obj=cq.Solid.makeCone(
            r_hub_mm, 2.0, cone_len_m * MM,
            cq.Vector(-(x_vane_te * MM), 0, 0), cq.Vector(-1, 0, 0))))
    )

    # -- landing legs: skids on the duct, feet aft of the vanes ---------
    clearance_m = 0.025
    x_foot      = x_vane_te + clearance_m
    leg_len_m   = x_foot - (duct_te_station - 0.4 * fus["duct_chord_m"])
    r_duct_o    = fus["D_duct_outer_m"] / 2.0
    legs = [
        _radial_plate(x_foot, leg_len_m, r_duct_o - 0.006, r_duct_o + 0.006,
                      8.0, ang)
        for ang in LEG_ANGLES
    ]

    # -- assembly --------------------------------------------------------
    grey   = cq.Color(0.75, 0.78, 0.82)
    dark   = cq.Color(0.30, 0.32, 0.36)
    accent = cq.Color(0.84, 0.15, 0.16)
    green  = cq.Color(0.10, 0.59, 0.25)

    asm = cq.Assembly(name="vbat_tailsitter")
    asm.add(fuselage,   name="fuselage",   color=grey)
    asm.add(wing,       name="wing",       color=green)
    asm.add(duct,       name="duct",       color=dark)
    asm.add(centerbody, name="centerbody", color=grey)
    for i, s in enumerate(struts):
        asm.add(s, name=f"strut_{i+1}", color=dark)
    blue   = cq.Color(0.17, 0.35, 0.78)
    for name, v in vane_parts.items():
        asm.add(v, name=f"vane_{name}", color=accent)
    for name, sv in servo_parts.items():
        asm.add(sv, name=f"servo_{name}", color=blue)
    for i, leg in enumerate(legs):
        asm.add(leg, name=f"leg_{i+1}", color=dark)

    # -- fused single solid ----------------------------------------------
    fused = fuselage.union(wing).union(duct).union(centerbody)
    for s in struts:
        fused = fused.union(s)
    for v in vane_parts.values():
        fused = fused.union(v)
    for sv in servo_parts.values():
        fused = fused.union(sv)
    for leg in legs:
        fused = fused.union(leg)

    return asm, fused


# ---------------------------------------------
#  Export helpers
# ---------------------------------------------

def export_vehicle(asm: cq.Assembly, fused: cq.Workplane, out_dir) -> Dict[str, str]:
    """
    Write STEP (assembly with part names/colours), STEP (fused single
    solid), and STL (mesh) under out_dir/{step,stl}.  Returns the paths.
    """
    from pathlib import Path
    out_dir = Path(out_dir)
    step_dir = out_dir / "step"
    stl_dir  = out_dir / "stl"
    step_dir.mkdir(parents=True, exist_ok=True)
    stl_dir.mkdir(parents=True, exist_ok=True)

    paths = {
        "step_assembly": str(step_dir / "vbat_assembly.step"),
        "step_fused":    str(step_dir / "vbat_fused.step"),
        "stl":           str(stl_dir / "vbat_fused.stl"),
    }
    if hasattr(asm, "export"):
        asm.export(paths["step_assembly"])      # cadquery >= 2.7
    else:
        asm.save(paths["step_assembly"])
    cq.exporters.export(fused, paths["step_fused"])
    cq.exporters.export(fused, paths["stl"], tolerance=0.15, angularTolerance=0.15)
    return paths
