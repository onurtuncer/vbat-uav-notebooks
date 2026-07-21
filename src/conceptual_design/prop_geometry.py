"""Non-dimensional COTS prop-rotor geometry (config/prop_geometry.yaml).

Single source of truth for the blade planform (peaked chord law with a
rounded tip), the exact constant-geometric-pitch twist law

    beta(r) = atan(pitch / (2*pi*r)),  pitch = pitch_ratio * D,

the blade section (a real, published low-Re propeller airfoil --
Clark Y, digitized coordinates in config/airfoils/clarky.dat --
thickness-scaled root->tip, replacing the earlier ad-hoc parametric
NACA-4-digit-like taper), and the rotation sense the twist law implies
(ROTATION_AXIS_BODY_FRD).

The CAD rotor (cad/prop_rotor.py, CadQuery) and the Aeolion BEMT
handoff (aeolion_handoff.py) both sample these laws — this module is
deliberately free of CadQuery so the handoff and its tests run in
environments without OCCT.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import numpy as np
import yaml

# Angular velocity vector direction (right-hand rule), body FRD, that the
# twist law above is built for. Derived (not assumed) from the blade-
# element relative-wind geometry: for the twist beta(r) defined here, a
# rotation vector along +x (numerically checked at r/R = 0.3/0.6/0.95
# against both a hover-like and a high-advance-ratio axial inflow) gives
# every blade station a small, physically sensible POSITIVE angle of
# attack and a thrust reaction on the vehicle in +x ("x forward, out the
# nose" -- see CLAUDE.md axis convention); the opposite sign gives an
# AoA near 180 deg at every station and radius, i.e. the blade flying
# backwards through its own relative wind. Equivalently: clockwise
# viewed from aft (the exhaust/-x end) looking forward toward the nose
# (+x), counter-clockwise viewed from the nose looking aft.
ROTATION_AXIS_BODY_FRD: Tuple[float, float, float] = (1.0, 0.0, 0.0)


@dataclass
class ClarkYSection:
    """Digitized Clark Y coordinates (config/airfoils/clarky.dat), plus
    the reference thickness/camber the raw data itself carries -- both
    DERIVED from the loaded points, never configured, so they can never
    drift from the data they describe."""
    xu: np.ndarray
    yu: np.ndarray
    xl: np.ndarray
    yl: np.ndarray
    tc_ref: float
    camber_max_ref: float

    @classmethod
    def from_dat(cls, path) -> "ClarkYSection":
        """Parse a Lednicer-format .dat file (title line; 'n_upper
        n_lower' counts; upper block x=0->1; blank line; lower block
        x=0->1) -- the format config/airfoils/clarky.dat is stored in,
        verbatim from the UIUC Airfoil Coordinates Database (see the
        .dat.source sidecar for provenance)."""
        lines = [ln.strip() for ln in Path(path).read_text(encoding="utf-8").splitlines()]
        n_u, n_l = (int(float(v)) for v in lines[1].split())
        rows = [ln for ln in lines[2:] if ln]
        upper = [tuple(map(float, ln.split())) for ln in rows[:n_u]]
        lower = [tuple(map(float, ln.split())) for ln in rows[n_u:n_u + n_l]]
        xu, yu = (np.array(v) for v in zip(*upper))
        xl, yl = (np.array(v) for v in zip(*lower))
        if not np.allclose(xu, xl):
            raise ValueError(
                "Lednicer .dat upper/lower x-stations must match pointwise")
        tc = yu - yl
        camber = 0.5 * (yu + yl)
        return cls(xu=xu, yu=yu, xl=xl, yl=yl,
                  tc_ref=float(tc.max()), camber_max_ref=float(camber.max()))


def _scaled_clarky_surfaces(
    section: ClarkYSection, tc_target: float, n: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Clark Y camber line + thickness distribution, thickness-scaled to
    tc_target and resampled to n cosine-spaced chordwise stations.

    Decomposes the reference data into a camber line yc = (yu+yl)/2 and
    a half-thickness distribution yt = (yu-yl)/2, then rebuilds the
    surfaces at the target local thickness ratio: yu' = yc + yt*(k),
    yl' = yc - yt*k, k = tc_target / tc_ref. This is the standard way a
    published section is adapted to a locally different thickness along
    a propeller blade span while preserving its camber-line identity --
    the SAME decomposition the wing's NACA formula already keeps
    separate (naca4_camber/naca4_thickness in airfoil_selection.py),
    just applied to a real digitized dataset instead of an analytic one.
    """
    if tc_target <= 0.0:
        raise ValueError("tc_target must be positive")
    x = 0.5 * (1.0 - np.cos(np.pi * np.arange(n) / (n - 1)))   # cosine spacing
    yc = np.interp(x, section.xu, 0.5 * (section.yu + section.yl))
    yt = np.interp(x, section.xu, 0.5 * (section.yu - section.yl))
    k = tc_target / section.tc_ref
    return x, yc + yt * k, x, yc - yt * k


def clark_y_surfaces(
    section: ClarkYSection, tc_target: float, n: int = 61,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """(xu, yu, xl, yl) at a target local thickness ratio, for CST
    fitting (aeolion_handoff.py) -- same signature convention as
    airfoil_selection.naca4_coordinates()."""
    return _scaled_clarky_surfaces(section, tc_target, n)


def clark_y_points(
    section: ClarkYSection, tc_target: float, n: int = 120,
) -> List[Tuple[float, float]]:
    """CLOSED loop of (x, y) points on the unit chord at a target local
    thickness ratio: upper surface LE->TE, then lower surface TE->LE,
    without duplicating LE/TE points -- same convention as
    cad.wing_profile.naca4_points(), so it drops into
    cad.prop_rotor._blade_section_wire() unchanged."""
    xu, yu, xl, yl = _scaled_clarky_surfaces(section, tc_target, n)
    upper = list(zip(xu.tolist(), yu.tolist()))
    lower = list(zip(xl.tolist(), yl.tolist()))[1:-1][::-1]
    return upper + lower


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
            tc_root          = float(data["tc_root"]),
            tc_tip           = float(data["tc_tip"]),
            n_sections       = int(data["n_sections"]),
        )

    def tc_at(self, f: float) -> float:
        """Local target thickness/chord at blade-loft span fraction f
        (0 root, 1 tip): linear taper between the configured root/tip
        ratios, applied to the Clark Y reference section."""
        return self.tc_root + f * (self.tc_tip - self.tc_root)

    def chord_ratio(self, f: float) -> float:
        """Blade chord / tip radius at span fraction f (0 root, 1 tip).

        Piecewise-linear root -> peak -> tip planform (real 8x6-class
        3-blades carry their maximum chord around 35% span), multiplied
        by the tip-rounding factor sqrt(1 - f^m). f is the fraction of
        the lofted blade span (0.85 * hub radius -> tip), matching the
        CAD loft stations."""
        if f <= self.f_peak:
            base = self.c_root_ratio + (self.c_peak_ratio - self.c_root_ratio) * (
                f / self.f_peak)
        else:
            base = self.c_peak_ratio + (self.c_tip_ratio - self.c_peak_ratio) * (
                (f - self.f_peak) / (1.0 - self.f_peak))
        return base * math.sqrt(max(1.0 - f ** self.tip_round_expo, 0.0))

    def loft_fraction(self, r_over_R: float) -> float:
        """Map a radius fraction r/R to the blade-loft span fraction f.

        The lofted blade (and thus the chord law's domain) starts at
        0.85 * hub_radius_ratio, slightly inside the hub collar so the
        CAD union is watertight."""
        r0 = 0.85 * self.hub_radius_ratio
        return (r_over_R - r0) / (1.0 - r0)

    def twist_deg(self, r_over_R: float) -> float:
        """Exact constant-geometric-pitch twist at r/R, in degrees.

        beta(r) = atan(pitch / (2*pi*r)) with pitch = pitch_ratio * D
        reduces to the dimensionless atan(pitch_ratio / (pi * r/R))."""
        return math.degrees(math.atan2(self.pitch_ratio, math.pi * r_over_R))
