"""Non-dimensional COTS prop-rotor geometry (config/prop_geometry.yaml).

Single source of truth for the blade planform (peaked chord law with a
rounded tip) and the exact constant-geometric-pitch twist law

    beta(r) = atan(pitch / (2*pi*r)),  pitch = pitch_ratio * D.

The CAD rotor (cad/prop_rotor.py, CadQuery) and the Aeolion BEMT
handoff (aeolion_handoff.py) both sample these laws — this module is
deliberately free of CadQuery so the handoff and its tests run in
environments without OCCT.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import yaml


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
