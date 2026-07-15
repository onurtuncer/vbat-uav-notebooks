"""
prop_rotor.py  --  Parametric COTS propeller rotor (CadQuery / OCCT)

Builds a watertight bladed rotor solid for visualisation AND for
rotating-zone (MRF / sliding mesh) CFD of the propulsion unit:

    hub             short adapter collar (optionally with an
                    ellipsoidal spinner nose; the prop-in-duct runs
                    without one -- the fuselage tailcone is upstream)
    blades          n_blades lofted NACA 4-digit sections, peaked
                    chord planform with a rounded tip, thickness
                    tapering root->tip, exact constant-geometric-pitch
                    twist beta(r) = atan(pitch / (2*pi*r))

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
from dataclasses import dataclass

import cadquery as cq
import yaml

from .fuselage_body import MM
from .wing_profile import naca4_points

# OCCT cannot loft to a zero-chord section: the tip-rounding planform
# factor sqrt(1 - f^m) is 0 exactly at f = 1, so the last station is
# clamped to this chord [mm] -- a tessellation guard, not geometry.
MIN_TIP_CHORD_MM = 2.0


@dataclass
class PropGeometry:
    """Non-dimensional prop-rotor geometry (config/prop_geometry.yaml)."""
    n_blades:         int
    hub_radius_ratio: float
    hub_length_ratio: float
    spinner_ratio:    float
    pitch_ratio:      float
    c_root_ratio:     float
    c_peak_ratio:     float
    f_peak:           float
    c_tip_ratio:      float
    tip_round_expo:   float
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
            c_peak_ratio     = float(data["c_peak_ratio"]),
            f_peak           = float(data["f_peak"]),
            c_tip_ratio      = float(data["c_tip_ratio"]),
            tip_round_expo   = float(data["tip_round_expo"]),
            camber_M         = float(data["camber_M"]),
            camber_P         = float(data["camber_P"]),
            tc_root          = float(data["tc_root"]),
            tc_tip           = float(data["tc_tip"]),
            n_sections       = int(data["n_sections"]),
        )

    def chord_ratio(self, f: float) -> float:
        """Blade chord / tip radius at span fraction f (0 root, 1 tip).

        Piecewise-linear root -> peak -> tip planform (real 8x6-class
        3-blades carry their maximum chord around 35% span), multiplied
        by the tip-rounding factor sqrt(1 - f^m)."""
        if f <= self.f_peak:
            base = self.c_root_ratio + (self.c_peak_ratio - self.c_root_ratio) * (
                f / self.f_peak)
        else:
            base = self.c_peak_ratio + (self.c_tip_ratio - self.c_peak_ratio) * (
                (f - self.f_peak) / (1.0 - self.f_peak))
        return base * math.sqrt(max(1.0 - f ** self.tip_round_expo, 0.0))


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
    Build the prop rotor solid (adapter-collar hub + n_blades blades).

    Parameters
    ----------
    D_rotor_m : prop (blade tip) diameter from config/rotor.yaml [m]
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
        chord = max(geom.chord_ratio(f) * R_tip, MIN_TIP_CHORD_MM)
        tc = geom.tc_root + f * (geom.tc_tip - geom.tc_root)
        beta = math.atan2(pitch, 2.0 * math.pi * r)
        desig = _naca_designation(geom.camber_M, geom.camber_P, tc)
        sections.append(_blade_section_wire(r, chord, beta, desig))

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
