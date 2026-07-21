"""
prop_rotor.py  --  Parametric COTS propeller rotor (CadQuery / OCCT)

Builds a watertight bladed rotor solid for visualisation AND for
rotating-zone (MRF / sliding mesh) CFD of the propulsion unit:

    hub             short adapter collar (optionally with an
                    ellipsoidal spinner nose; the prop-in-duct runs
                    without one -- the fuselage tailcone is upstream)
    blades          n_blades lofted Clark Y sections (ADR-0017,
                    thickness-scaled root->tip off the digitized
                    reference in config/airfoils/clarky.dat), peaked
                    chord planform with a rounded tip, exact
                    constant-geometric-pitch twist
                    beta(r) = atan(pitch / (2*pi*r))

All geometric ratios come from config/prop_geometry.yaml -- no magic
numbers in code. The rotor diameter is the COTS prop diameter from
config/rotor.yaml (built to the exact tip radius; the duct bore carries
the tip clearance).

LOCAL FRAME: rotor axis = +x (matches the body FRD x axis and the
cfd/prop propulsion frame); the disk plane is x = 0, collar front face
toward +x (into the inflow). Geometry is built in MILLIMETRES.
"""

from __future__ import annotations

import math

import cadquery as cq

from ..prop_geometry import ClarkYSection, PropGeometry, clark_y_points
from .fuselage_body import MM

__all__ = ["ClarkYSection", "PropGeometry", "build_prop_rotor", "export_prop_rotor"]

# OCCT cannot loft to a zero-chord section: the tip-rounding planform
# factor sqrt(1 - f^m) is 0 exactly at f = 1, so the last station is
# clamped to this chord [mm] -- a tessellation guard, not geometry.
MIN_TIP_CHORD_MM = 2.0


def _blade_section_wire(
    r_mm:    float,   # radial station (z in the blade frame) [mm]
    chord_mm: float,
    beta_rad: float,  # local pitch (twist) angle from the disk plane
    section: ClarkYSection,
    tc_target: float,
    n_pts:   int = 60,
) -> cq.Wire:
    """
    Airfoil wire at radius r_mm, twisted by beta about the radial axis.

    Blade frame: blade spans +z (radial); the section lies in a plane
    z = r. Unit-chord airfoil (xc, yc) is centred at quarter chord and
    rotated so the chord makes angle beta with the disk plane (y =
    tangential, x = axial):

        u = (xc - 0.25) * c,  v = yc * c
        x = u*sin(beta) + v*cos(beta)      (axial)
        y = u*cos(beta) - v*sin(beta)      (tangential)
    """
    sb, cb = math.sin(beta_rad), math.cos(beta_rad)
    pts = []
    for xc, yc in clark_y_points(section, tc_target, n=n_pts):
        u = (xc - 0.25) * chord_mm
        v = yc * chord_mm
        pts.append((u * sb + v * cb, u * cb - v * sb))
    wire = (
        cq.Workplane("XY", origin=(0, 0, r_mm))
        .polyline(pts).close()
    )
    return wire.wires().val()


def build_prop_rotor(
    D_rotor_m: float, geom: PropGeometry, section: ClarkYSection,
) -> cq.Workplane:
    """
    Build the prop rotor solid (adapter-collar hub + n_blades blades).

    Parameters
    ----------
    D_rotor_m : prop (blade tip) diameter from config/rotor.yaml [m]
    geom      : PropGeometry ratios from config/prop_geometry.yaml
    section   : ClarkYSection, config/airfoils/clarky.dat (ADR-0017)

    Returns a single watertight cq.Workplane solid in mm, rotor axis +x,
    disk plane at x = 0.
    """
    R_tip = D_rotor_m / 2.0 * MM
    r_hub = geom.hub_radius_ratio * R_tip
    L_hub = geom.hub_length_ratio * D_rotor_m * MM
    pitch = geom.pitch_ratio * D_rotor_m * MM

    # -- one blade: loft Clark Y sections from inside the hub to the tip -
    sections = []
    for i in range(geom.n_sections):
        f = i / (geom.n_sections - 1)
        # start slightly inside the hub so the union is watertight
        r = (0.85 * r_hub) + f * (R_tip - 0.85 * r_hub)
        chord = max(geom.chord_ratio(f) * R_tip, MIN_TIP_CHORD_MM)
        tc = geom.tc_at(f)
        beta = math.atan2(pitch, 2.0 * math.pi * r)
        sections.append(_blade_section_wire(r, chord, beta, section, tc))

    # newObject() fills the stack but not the pending-wire list that
    # loft() consumes -- toPending() bridges the two
    blade = cq.Workplane("XY").newObject(sections).toPending().loft(combine=True)

    # -- circular pattern about the rotor (+x) axis ----------------------
    blades = blade
    for k in range(1, geom.n_blades):
        blades = blades.union(
            blade.rotate((0, 0, 0), (1, 0, 0), 360.0 * k / geom.n_blades)
        )

    # -- hub collar (+ optional elliptical spinner): one revolved profile --
    # Profile in the XY plane, (x, r) points revolved about the +x axis:
    # tail face -> cylinder -> flat front face, or, with spinner_ratio > 0,
    # -> half-ellipse nose (length spinner_l) -> tip.
    spinner_l = geom.spinner_ratio * r_hub
    pts = [(-L_hub / 2.0, 0.0), (-L_hub / 2.0, r_hub)]
    if spinner_l > 0.0:
        n_arc = 24
        for i in range(n_arc + 1):
            th = (math.pi / 2.0) * i / n_arc
            pts.append((L_hub / 2.0 + spinner_l * math.sin(th),
                        r_hub * math.cos(th)))
    else:
        pts += [(L_hub / 2.0, r_hub), (L_hub / 2.0, 0.0)]
    hub = cq.Workplane("XY").polyline(pts).close().revolve(
        360.0, (0, 0, 0), (1, 0, 0))

    return hub.union(blades)


def export_prop_rotor(rotor: cq.Workplane, out_dir) -> dict:
    """Write STEP + STL of the rotor under out_dir/{step,stl}."""
    from pathlib import Path
    out_dir = Path(out_dir)
    step_dir = out_dir / "step"
    stl_dir = out_dir / "stl"
    step_dir.mkdir(parents=True, exist_ok=True)
    stl_dir.mkdir(parents=True, exist_ok=True)
    paths = {
        "step": str(step_dir / "prop_rotor.step"),
        "stl":  str(stl_dir / "prop_rotor.stl"),
    }
    cq.exporters.export(rotor, paths["step"])
    cq.exporters.export(rotor, paths["stl"],
                        tolerance=0.1, angularTolerance=0.1)
    return paths
