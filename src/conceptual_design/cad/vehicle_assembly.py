"""
vehicle_assembly.py  --  Full tail-sitter solid model (CadQuery / OCCT)

Assembles the sized components into one parametric vehicle:

    fuselage      body of revolution        (out/fuselage.yaml)
    wing          NACA section, constant chord  (mass closure + out/airfoil.yaml)
    ailerons      2x outboard TE control surfaces, split from the wing
                  at zero deflection (cruise trim)  (out/aileron.yaml)
    ail. servos   2x 9g-class servos as lower-surface pods on the fixed
                  wing forward of the aileron hinge (thin wing -> pod)
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

import math
from typing import Dict, Tuple

import cadquery as cq

from .fuselage_body import fuselage_solid, MM
from .wing_profile import naca4_points
from conceptual_design.fuselage_design import fuselage_radius
from conceptual_design.airfoil_selection import parse_naca4, naca4_coordinates

# minimum manufacturable plate thickness for flat parts [mm]
T_PLATE_MIN_MM = 2.0

VANE_ANGLES = {"T": 0.0, "B": 180.0, "L": 270.0, "R": 90.0}  # about +x from +z
LEG_ANGLES  = (45.0, 135.0, 225.0, 315.0)


# ---------------------------------------------
#  Components
# ---------------------------------------------

def fuselage_split_parts(
    fuselage:        cq.Workplane,   # full fuselage solid
    x_split_nose_m:  float,          # nose module split plane [m, +aft]
    hatch_x_start_m: float,          # battery hatch axial start [m, +aft]
    hatch_length_m:  float,          # battery hatch axial extent [m]
    hatch_arc_deg:   float,          # hatch angular extent [deg]
    D_fus_m:         float,          # max fuselage diameter [m]
    t_shell_m:       float,          # shell thickness [m]
) -> Dict[str, cq.Workplane]:
    """
    Split the fuselage along the modularity lines (ADR-0008):

      fuselage_nose : forward of the split plane (removable payload module)
      battery_hatch : surface panel over the battery bay, centered on -z
                      (up in cruise), `hatch_arc_deg` wide
      fuselage_main : everything else

    Same coplanar-cut discipline as the aileron split -- the union of the
    three parts reproduces the original solid exactly, so the fused
    external-aero STL is unchanged in shape.
    """
    bb = fuselage.val().BoundingBox()
    x_split = -(x_split_nose_m * MM)          # body x of the split plane
    margin = 5.0                              # mm past the body extents

    nose_box = (
        cq.Workplane("XY")
        .box(bb.xmax - x_split + margin,
             bb.ymax - bb.ymin + 2 * margin,
             bb.zmax - bb.zmin + 2 * margin,
             centered=(False, True, True))
        .translate((x_split, 0, 0))
    )
    nose = fuselage.intersect(nose_box)
    main = fuselage.cut(nose_box)

    # hatch: wedge sector (about +x, centered on -z) x outer annulus band
    panel_depth = max(T_PLATE_MIN_MM, t_shell_m * MM)
    r_outer_big = (D_fus_m * MM)              # comfortably outside the body
    r_inner     = D_fus_m / 2.0 * MM - panel_depth
    x_aft       = -((hatch_x_start_m + hatch_length_m) * MM)
    length      = hatch_length_m * MM

    half = math.radians(hatch_arc_deg / 2.0)
    # sector polygon in the YZ workplane: local (u, v) -> global (y, z);
    # -z (up in cruise) is local v = -1
    n_arc = 24
    pts = [(0.0, 0.0)] + [
        (r_outer_big * math.sin(-half + 2 * half * i / n_arc),
         -r_outer_big * math.cos(-half + 2 * half * i / n_arc))
        for i in range(n_arc + 1)
    ]
    wedge = (
        cq.Workplane("YZ", origin=(x_aft, 0, 0))
        .polyline(pts).close()
        .extrude(length)                      # +x: forward over the bay
    )
    band = (
        cq.Workplane("YZ", origin=(x_aft, 0, 0))
        .circle(r_outer_big).circle(r_inner)
        .extrude(length)
    )
    hatch_region = wedge.intersect(band)
    hatch = main.intersect(hatch_region)
    main  = main.cut(hatch_region)

    return {"fuselage_nose": nose, "fuselage_main": main,
            "battery_hatch": hatch}


def wing_split_parts(
    wing:         cq.Workplane,   # fixed-wing solid (post aileron split)
) -> Dict[str, cq.Workplane]:
    """Two-piece wing: cut at the centerline (y = 0) for the carry-through
    spar assembly.  wing_L is +y (left, FRD), wing_R is -y."""
    bb = wing.val().BoundingBox()
    margin = 5.0
    half_box = (
        cq.Workplane("XY")
        .box(bb.xmax - bb.xmin + 2 * margin,
             bb.ymax + margin,
             bb.zmax - bb.zmin + 2 * margin,
             centered=(False, False, True))
        .translate((bb.xmin - margin, 0, 0))
    )
    wing_L = wing.intersect(half_box)
    wing_R = wing.cut(half_box)
    return {"wing_L": wing_L, "wing_R": wing_R}


def spar_tube_solid(
    designation:     str,     # wing airfoil (for the camber line)
    chord_m:         float,   # wing MAC [m]
    x_wing_LE_m:     float,   # wing LE station [m, +aft]
    spar_od_m:       float,   # tube OD [m]
    spar_length_m:   float,   # tube length [m]
    spar_chord_frac: float,   # chordwise station / chord [-]
) -> cq.Workplane:
    """
    CFRP carry-through spar tube: axis along body y, at the spar chord
    fraction, on the local camber line (mid-thickness) -- the same
    station `wing_lighten --spar-hole` bores.  ASSEMBLY-ONLY part (like
    the prop rotor): it is internal structure, excluded from the fused
    external-aero STL.
    """
    c    = chord_m * MM
    x_le = x_wing_LE_m * MM

    # local mid-thickness (camber) z at the spar station, from the same
    # airfoil coordinates the wing solid is built from
    M, P, t = parse_naca4(designation)
    xu, yu, xl, yl = naca4_coordinates(M, P, t, n=200)
    iu = min(range(len(xu)), key=lambda i: abs(xu[i] - spar_chord_frac))
    il = min(range(len(xl)), key=lambda i: abs(xl[i] - spar_chord_frac))
    z_mid = -0.5 * (yu[iu] + yl[il]) * c

    x0 = -(x_le + spar_chord_frac * c)
    L  = spar_length_m * MM
    return (
        cq.Workplane("XZ", origin=(x0, -L / 2.0, z_mid))
        # Workplane("XZ") extrudes along -y ... use explicit cylinder:
        .circle(spar_od_m / 2.0 * MM)
        .extrude(-L)   # XZ plane normal is -y; extrude(-L) spans +y
    )


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


def aileron_parts(
    wing:            cq.Workplane,   # full wing solid, from wing_solid()
    chord_m:         float,          # wing MAC [m]
    span_m:          float,          # wing span [m]
    x_wing_LE_m:     float,          # wing LE station [m, +aft]
    span_frac_wing:  float,          # aileron span / wing half-span [-] (out/aileron.yaml)
    chord_frac:      float,          # aileron chord / wing MAC [-]      (out/aileron.yaml)
) -> Tuple[cq.Workplane, Dict[str, cq.Workplane]]:
    """
    Split the outboard trailing-edge aileron surfaces out of the wing solid.

    This is a cruise-trim (zero-deflection) solid model, not an
    articulated one: the wing is cut along the hinge line and the two
    span stations into three pieces (wing_cut, aileron_L, aileron_R)
    whose union reproduces the original wing exactly -- the aileron
    parameters (`span_frac_wing`, `chord_frac`) match aileron_design.py's
    own geometry exactly, so the CAD split sits precisely where the
    physics model assumes it does.

    Returns (wing_cut, {"aileron_L": ..., "aileron_R": ...}).
    """
    bb = wing.val().BoundingBox()
    c    = chord_m * MM
    x_le = x_wing_LE_m * MM
    x_te_body    = -(x_le + c)                          # trailing edge
    x_hinge_body = -(x_le + (1.0 - chord_frac) * c)      # aileron hinge line

    b_half  = span_m / 2.0 * MM
    y_inner = (1.0 - span_frac_wing) * b_half
    y_outer = b_half

    z_margin = 5.0   # mm, generous beyond the airfoil thickness
    z_lo, z_hi = bb.zmin - z_margin, bb.zmax + z_margin
    x_lo = min(x_te_body, x_hinge_body) - 1.0
    x_hi = max(x_te_body, x_hinge_body) + 1.0

    def _span_box(y_lo: float, y_hi: float) -> cq.Workplane:
        return (
            cq.Workplane("XY")
            .box(x_hi - x_lo, y_hi - y_lo, z_hi - z_lo,
                 centered=(False, False, False))
            .translate((x_lo, y_lo, z_lo))
        )

    box_L = _span_box(y_inner, y_outer)     # +y (left, FRD)
    box_R = _span_box(-y_outer, -y_inner)   # -y (right, FRD)

    aileron_L = wing.intersect(box_L)
    aileron_R = wing.intersect(box_R)
    wing_cut  = wing.cut(box_L).cut(box_R)

    return wing_cut, {"aileron_L": aileron_L, "aileron_R": aileron_R}


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


def _aileron_servo_pod(
    designation:  str,     # wing airfoil (out/airfoil.yaml)
    chord_m:      float,   # wing MAC [m]
    x_wing_LE_m:  float,   # wing LE station [m, +aft]
    y_center_m:   float,   # spanwise station of the aileron centroid [m]
    xc_frac:      float = 0.40,   # chord fraction of the servo (near max t/c)
    body_l_mm:    float = 22.8,   # 9g-class body: chord x span x thickness
    body_w_mm:    float = 22.5,
    body_h_mm:    float = 12.2,
    embed_mm:     float = 4.0,    # how far the pod top sits up into the wing
) -> cq.Workplane:
    """
    9g-class aileron servo as a lower-surface pod on the fixed wing,
    at the aileron centroid, forward of the hinge line.

    A 9g servo is ~12 mm thick; this wing is only ~6 mm thick at the
    aileron hinge (0.88c) and ~22 mm at max thickness (0.30c), so the
    servo CANNOT be buried inside the aileron the way the vane servos
    bury in the exhaust hub.  It is modeled the way thin-wing UAVs
    actually mount it: a shallow fairing on the lower (pressure) surface
    near max thickness, driving the aileron via a short pushrod (the
    pushrod itself is below conceptual-model fidelity).

    The lower-surface offset is read from the SAME airfoil coordinates
    the wing solid is built from, so the pod always seats on the skin.
    """
    c    = chord_m * MM
    x_le = x_wing_LE_m * MM

    # lower-surface z at xc_frac from the airfoil math (wing_solid uses
    # z = -yc * c, so the lower surface -- yc < 0 -- is at positive z/down)
    M, P, t = parse_naca4(designation)
    xu, yu, xl, yl = naca4_coordinates(M, P, t, n=200)
    il = min(range(len(xl)), key=lambda i: abs(xl[i] - xc_frac))
    z_lower = -yl[il] * c

    x0 = -(x_le + xc_frac * c)
    pod = (
        cq.Workplane("XY", origin=(x0, y_center_m * MM, z_lower - embed_mm))
        # extrude downward (+z is down in FRD): pod hangs below the skin
        .box(body_l_mm, body_w_mm, body_h_mm, centered=(True, True, False))
    )
    return pod


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


# ---------------------------------------------
#  Soft-mounted internal modules (vibration isolation)
# ---------------------------------------------
# These are INTERNAL parts (assembly / exploded-view only); they are NOT
# fused into the external-aero STL. Both hang off the shell on isolator
# standoffs with a visible rattle gap -- the conceptual picture of the
# soft mount sized in vibration_isolation.py.

def isolated_fc_tray(
    x_station_m: float,   # avionics bay station [m, +aft]
    r_int_m:     float,   # internal shell radius at the bay [m]
    n_iso:       int = 4,
    plate_l_mm:  float = 42.0,   # tray footprint (fore-aft x cross)
    plate_w_mm:  float = 32.0,
    plate_t_mm:  float = 3.0,
    iso_d_mm:    float = 6.0,     # isolator grommet diameter
    iso_h_mm:    float = 8.0,     # isolator standoff height
    rattle_mm:   float = 1.5,     # gap between tray and shell floor
) -> cq.Workplane:
    """FC/IMU tray on n grommet isolators standing off the lower shell wall."""
    x0 = -(x_station_m * MM)
    r_int = r_int_m * MM
    z_floor = r_int - rattle_mm                 # shell "floor" (down = +z)
    z_iso_top = z_floor - iso_h_mm
    z_plate_c = z_iso_top - plate_t_mm / 2.0

    tray = (
        cq.Workplane("XY", origin=(x0, 0, z_plate_c))
        .box(plate_l_mm, plate_w_mm, plate_t_mm, centered=(True, True, True))
    )
    ox = 0.35 * plate_l_mm
    oy = 0.35 * plate_w_mm
    for sx in (-1, 1):
        for sy in (-1, 1):
            tray = tray.union(
                cq.Workplane("XY", origin=(x0 + sx * ox, sy * oy, z_iso_top))
                .circle(iso_d_mm / 2.0).extrude(iso_h_mm)   # +z: down to the floor
            )
    return tray


def soft_mounted_nose(
    x_cg_m:    float,   # payload bay CG station [m, +aft]
    length_m:  float,   # payload bay length [m]
    r_int_m:   float,   # internal shell radius [m]
    n_iso:     int = 4,
    rattle_mm: float = 1.5,   # radial gap payload -> shell
    iso_d_mm:  float = 6.0,
) -> cq.Workplane:
    """Soft-mounted nose payload module: a block held off the shell by a
    ring of isolators, with a radial rattle gap."""
    x0 = -(x_cg_m * MM)
    r_int = r_int_m * MM
    r_pl = r_int - rattle_mm - iso_d_mm          # payload radius inside the gap
    L = length_m * MM

    body = (
        cq.Workplane("YZ", origin=(x0 - L / 2.0, 0, 0))
        .circle(r_pl).extrude(L)
    )
    # a ring of isolator studs on the aft face, extruding aft toward the
    # avionics bay; placed inside the OD so they union into one solid
    import math as _m
    x_aft = x0 - L / 2.0                 # aft face (toward +station)
    for i in range(n_iso):
        a = 2.0 * _m.pi * i / n_iso
        yc = 0.6 * r_pl * _m.cos(a)
        zc = 0.6 * r_pl * _m.sin(a)
        body = body.union(
            cq.Workplane("YZ", origin=(x_aft, yc, zc))
            .circle(iso_d_mm / 2.0).extrude(-0.3 * L)   # -x: aft
        )
    return body


def build_vehicle(
    fus:    Dict,            # out/fuselage.yaml
    vanes:  Dict,            # out/control_vanes.yaml
    ail:    Dict,            # out/aileron.yaml
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

    # -- fuselage, split along the modularity lines (ADR-0008): removable
    #    nose module + battery hatch panel + main body.  Union of the three
    #    reproduces the original body of revolution exactly.
    fuselage = fuselage_solid(D, L, fus["f_nose"], fus["f_tail"], fus["r_hub_m"])
    fus_parts = fuselage_split_parts(
        fuselage, fus["x_split_nose_m"],
        fus["hatch_x_start_m"], fus["hatch_length_m"], fus["hatch_arc_deg"],
        D, fus["t_shell_m"],
    )

    # -- wing (split into fixed wing + 2 ailerons, at zero deflection),
    #    then the fixed wing again at the centerline for the two-piece
    #    carry-through assembly (ADR-0008)
    wing_full = wing_solid(wing_designation, chord_wing_m, b_wing_m, fus["x_wing_LE_m"])
    wing, aileron_parts_dict = aileron_parts(
        wing_full, chord_wing_m, b_wing_m, fus["x_wing_LE_m"],
        ail["span_frac_wing"], ail["chord_frac"],
    )
    wing_parts = wing_split_parts(wing)

    # -- CFRP carry-through spar tube (assembly-only, like the prop rotor:
    #    internal structure, excluded from the fused external-aero STL)
    spar = spar_tube_solid(
        wing_designation, chord_wing_m, fus["x_wing_LE_m"],
        fus["spar_od_m"], fus["spar_length_m"], fus["spar_chord_frac"],
    )

    # -- aileron servos: one lower-surface pod per aileron, on the fixed
    #    wing at the aileron centroid (out/aileron.yaml y_arm), forward of
    #    the hinge where the wing is thick enough to carry it.
    y_arm_m = ail["y_arm_m"]
    aileron_servo_parts = {
        "servo_aileron_L": _aileron_servo_pod(
            wing_designation, chord_wing_m, fus["x_wing_LE_m"], +y_arm_m),
        "servo_aileron_R": _aileron_servo_pod(
            wing_designation, chord_wing_m, fus["x_wing_LE_m"], -y_arm_m),
    }

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

    aileron_color = cq.Color(0.95, 0.55, 0.10)   # amber: a distinct hue, not a darker green,
    #                                              so ailerons read clearly against the wing

    hatch_color = cq.Color(0.55, 0.35, 0.75)   # violet: access panel reads
    #                                            against the grey body

    asm = cq.Assembly(name="vbat_tailsitter")
    asm.add(fus_parts["fuselage_nose"], name="fuselage_nose", color=grey)
    asm.add(fus_parts["fuselage_main"], name="fuselage_main", color=grey)
    asm.add(fus_parts["battery_hatch"], name="battery_hatch", color=hatch_color)
    for name, wp_half in wing_parts.items():
        asm.add(wp_half, name=name, color=green)
    for name, ap in aileron_parts_dict.items():
        asm.add(ap, name=name, color=aileron_color)
    asm.add(spar,       name="spar_tube",  color=dark)   # assembly-only
    asm.add(duct,       name="duct",       color=dark)
    asm.add(centerbody, name="centerbody", color=grey)
    for i, s in enumerate(struts):
        asm.add(s, name=f"strut_{i+1}", color=dark)
    blue   = cq.Color(0.17, 0.35, 0.78)
    for name, v in vane_parts.items():
        asm.add(v, name=f"vane_{name}", color=accent)
    for name, sv in servo_parts.items():
        asm.add(sv, name=f"servo_{name}", color=blue)
    for name, asv in aileron_servo_parts.items():
        asm.add(asv, name=name, color=blue)
    for i, leg in enumerate(legs):
        asm.add(leg, name=f"leg_{i+1}", color=dark)

    # -- fused single solid ----------------------------------------------
    # Built from the UNSPLIT fuselage and wing (the splits union back to
    # exactly these), so the external-aero STL shape is unchanged by the
    # modularity lines.  The spar tube is internal structure and is
    # deliberately excluded (like the prop rotor).
    fused = fuselage.union(wing).union(duct).union(centerbody)
    for ap in aileron_parts_dict.values():
        fused = fused.union(ap)
    for asv in aileron_servo_parts.values():
        fused = fused.union(asv)
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
    solid), STL (mesh), and one STEP + STL PER PART under
    out_dir/{step,stl}/parts/.  Returns the paths.
    """
    from pathlib import Path
    out_dir = Path(out_dir)
    step_dir = out_dir / "step"
    stl_dir  = out_dir / "stl"
    step_parts_dir = step_dir / "parts"
    stl_parts_dir  = stl_dir / "parts"
    for d in (step_dir, stl_dir, step_parts_dir, stl_parts_dir):
        d.mkdir(parents=True, exist_ok=True)

    # Wipe the per-part directories before writing: the set of parts
    # changes when a component is renamed or split (e.g. fuselage ->
    # fuselage_nose + fuselage_main), and a bare re-export only WRITES the
    # current parts -- it never removes the old ones.  Left behind, the
    # orphaned STEP/STL (e.g. a monolithic fuselage) overlap the new split
    # pieces in any viewer that loads the folder (the Pages 3D viewer
    # globs stl/parts/*.stl).  Clean first so parts on disk == parts in
    # the current assembly, always.
    for d in (step_parts_dir, stl_parts_dir):
        for stale in list(d.glob("*.step")) + list(d.glob("*.stl")):
            stale.unlink()

    paths = {
        "step_assembly": str(step_dir / "vbat_assembly.step"),
        "step_fused":    str(step_dir / "vbat_fused.step"),
        "stl":           str(stl_dir / "vbat_fused.stl"),
        "parts":         {},
    }
    if hasattr(asm, "export"):
        asm.export(paths["step_assembly"])      # cadquery >= 2.7
    else:
        asm.save(paths["step_assembly"])
    cq.exporters.export(fused, paths["step_fused"])
    cq.exporters.export(fused, paths["stl"], tolerance=0.15, angularTolerance=0.15)

    # -- individual parts (already positioned in the body frame) ---------
    for child in asm.children:
        obj = child.obj
        wp = obj if isinstance(obj, cq.Workplane) else cq.Workplane(obj=obj)
        p_step = str(step_parts_dir / f"{child.name}.step")
        p_stl  = str(stl_parts_dir / f"{child.name}.stl")
        cq.exporters.export(wp, p_step)
        cq.exporters.export(wp, p_stl, tolerance=0.15, angularTolerance=0.15)
        paths["parts"][child.name] = {"step": p_step, "stl": p_stl}

    return paths
