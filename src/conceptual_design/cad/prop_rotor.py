"""
prop_rotor.py  --  Parametric EDF fan rotor (CadQuery / OCCT)

Builds a watertight bladed rotor solid for visualisation AND for
rotating-zone (MRF / sliding mesh) CFD of the propulsion unit:

    hub + spinner   cylinder with an ellipsoidal nose cap
    blades          n_blades lofted NACA 4-digit sections, thickness
                    tapering root->tip, constant-geometric-pitch twist
                    beta(r) = atan(pitch / (2*pi*r))

All geometric ratios come from config/prop_geometry.yaml -- no magic
numbers in code. The fan diameter is the COTS EDF diameter from
config/rotor.yaml.

LOCAL FRAME: rotor axis = +x (matches the body FRD x axis and the
cfd/prop propulsion frame); the disk plane is x = 0, spinner nose
points +x (into the inflow). Geometry is built in MILLIMETRES.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cadquery as cq
import yaml

from .fuselage_body import MM
from .wing_profile import naca4_points


@dataclass
class PropGeometry:
    """Non-dimensional fan-rotor geometry (config/prop_geometry.yaml)."""
    n_blades:         int
    hub_radius_ratio: float
    hub_length_ratio: float
    spinner_ratio:    float
    pitch_ratio:      float
    c_root_ratio:     float
    c_tip_ratio:      float
    camber_M:         float
    camber_P:         float
    tc_root:          float
    tc_tip:           float
    n_sections:       int

    @classmethod
    def from_yaml(cls, path) -> "PropGeometry":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(
            n_blades         = int(data["n_blades"]),
            hub_radius_ratio = float(data["hub_radius_ratio"]),
            hub_length_ratio = float(data["hub_length_ratio"]),
            spinner_ratio    = float(data["spinner_ratio"]),
            pitch_ratio      = float(data["pitch_ratio"]),
            c_root_ratio     = float(data["c_root_ratio"]),
            c_tip_ratio      = float(data["c_tip_ratio"]),
            camber_M         = float(data["camber_M"]),
            camber_P         = float(data["camber_P"]),
            tc_root          = float(data["tc_root"]),
            tc_tip           = float(data["tc_tip"]),
            n_sections       = int(data["n_sections"]),
        )


def _naca_designation(M: float, P: float, tc: float) -> str:
    """NACA 4-digit designation string from camber/thickness fractions."""
    return f"{round(M * 100)}{round(P * 10)}{round(tc * 100):02d}"


def _blade_section_wire(
    r_mm:    float,   # radial station (z in the blade frame) [mm]
    chord_mm: float,
    beta_rad: float,  # local pitch (twist) angle from the disk plane
    designation: str,
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
    for xc, yc in naca4_points(designation, n=n_pts):
        u = (xc - 0.25) * chord_mm
        v = yc * chord_mm
        pts.append((u * sb + v * cb, u * cb - v * sb))
    wire = (
        cq.Workplane("XY", origin=(0, 0, r_mm))
        .polyline(pts).close()
    )
    return wire.wires().val()


def build_prop_rotor(D_rotor_m: float, geom: PropGeometry) -> cq.Workplane:
    """
    Build the fan rotor solid (hub + spinner + n_blades blades).

    Parameters
    ----------
    D_rotor_m : fan (blade tip) diameter from config/rotor.yaml [m]
    geom      : PropGeometry ratios from config/prop_geometry.yaml

    Returns a single watertight cq.Workplane solid in mm, rotor axis +x,
    disk plane at x = 0.
    """
    R_tip = D_rotor_m / 2.0 * MM
    r_hub = geom.hub_radius_ratio * R_tip
    L_hub = geom.hub_length_ratio * D_rotor_m * MM
    pitch = geom.pitch_ratio * D_rotor_m * MM

    # -- one blade: loft NACA sections from inside the hub to the tip ----
    sections = []
    for i in range(geom.n_sections):
        f = i / (geom.n_sections - 1)
        # start slightly inside the hub so the union is watertight
        r = (0.85 * r_hub) + f * (R_tip - 0.85 * r_hub)
        chord = (geom.c_root_ratio
                 + f * (geom.c_tip_ratio - geom.c_root_ratio)) * R_tip
        tc = geom.tc_root + f * (geom.tc_tip - geom.tc_root)
        beta = math.atan2(pitch, 2.0 * math.pi * r)
        desig = _naca_designation(geom.camber_M, geom.camber_P, tc)
        sections.append(_blade_section_wire(r, chord, beta, desig))

    blade = cq.Workplane("XY").newObject(sections).loft(combine=True)

    # -- circular pattern about the rotor (+x) axis ----------------------
    blades = blade
    for k in range(1, geom.n_blades):
        blades = blades.union(
            blade.rotate((0, 0, 0), (1, 0, 0), 360.0 * k / geom.n_blades)
        )

    # -- hub + elliptical spinner nose: one revolved profile ---------------
    # Profile in the XY plane, (x, r) points revolved about the +x axis:
    # tail face -> cylinder -> half-ellipse nose (length spinner_l) -> tip.
    spinner_l = geom.spinner_ratio * r_hub
    n_arc = 24
    pts = [(-L_hub / 2.0, 0.0), (-L_hub / 2.0, r_hub)]
    for i in range(n_arc + 1):
        th = (math.pi / 2.0) * i / n_arc
        pts.append((L_hub / 2.0 + spinner_l * math.sin(th),
                    r_hub * math.cos(th)))
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
