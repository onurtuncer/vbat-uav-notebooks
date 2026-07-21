"""VBAT -> Aeolion parametric geometry handoff (ADR-0016).

Emits `out/cad/aeolion_geometry.json` conforming to
schemas/vbat-aeolion-geometry-handoff.schema.json (schema 1.0.0), the
executable geometry contract for the Aeolion VLM/BEMT adjoint loop:

- planform stations + CST airfoil sections, aligned 1:1 by eta, with a
  FIXED count across the whole design space (config/aeolion.yaml) so
  the optimizer sees a constant-length design vector;
- control surfaces: the aileron AND the four jet vanes. Vane entries
  are all-moving flat plates in the duct jet: eta is measured on the
  duct-exit radius (hub collar -> duct wall), the hinge axes are the
  radial vane directions (CAD angle convention: about +x, 0 deg = +z),
  and chord_fraction 1.0 marks the whole chord rotating about the
  hinge_xc line. The vane DAVE-ML path still characterizes the
  jet-wash aerodynamics; the geometry rides here so Aeolion can model
  the jet directly. Every entry carries deflection_limits_deg -- the
  symmetric mechanical range about hinge_axis (+-delta_max_deg from
  out/aileron.yaml / out/control_vanes.yaml) -- so a CFD/VLM sweep
  never asks for a deflection the physical actuator/plate cannot
  reach; vanes additionally carry deflection_soft_limit_deg (the
  flat-plate stall onset), informational, not a hard bound;
- BEMT blade count and blade stations sampled from the SAME chord/twist
  law that builds the CAD rotor (prop_geometry.PropGeometry) -- solidity
  and thrust need the count, not just one blade's planform;
- the fuselage as a body of revolution (schema 1.2.0): radius stations
  sampled from the SAME 3-segment meridian the CAD revolve uses
  (fuselage_design.fuselage_radius), cosine-spaced nose -> tail, body
  x = -station (0 at the nose tip, negative aft). Not bound to any
  lifting lattice -- for slender-body/Munk trim corrections and
  duct-jet context;
- planform.placement (schema 1.5.0): the wing root LE anchored in the
  SAME body-frame convention as `body`, so a consumer can actually
  place the two against each other instead of assuming the root sits
  at the reference-frame origin (which puts the wing at the fuselage
  nose -- see ADR-0016);
- moment_reference_point (schema 1.6.0): the vehicle CG, SAME source
  and convention as cfd/vehicle/Allrun.case's CofR, so a consumer has
  a physically meaningful centre for Cm/Cl/Cn instead of defaulting
  to the coordinate-system origin;
- static mesh-topology directives (config/aeolion.yaml, not design
  variables).

The schema forbids additional properties, so the document carries
exactly the contract fields -- STEP/STL traceability stays in ADR-0016
prose, not in the JSON. `design_id` is the SHA-256 of the canonical
JSON of every other field: byte-stable across reruns of the same
design point, guaranteed to move when any parameter moves.

Airfoil sections are Kulfan CST fits (class function x^0.5 (1-x)^1.0).
The schema carries no trailing-edge thickness term, so each surface is
fitted after removing the linear TE ramp x*y_te -- the handoff section
is the sharp-TE aerodynamic equivalent of the NACA section, which is
what a VLM camber surface consumes anyway.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from .airfoil_selection import naca4_coordinates, parse_naca4
from .fuselage_design import fuselage_radius
from .prop_geometry import PropGeometry

SCHEMA_VERSION = "1.6.0"
REFERENCE_FRAME = "aetherion_body_frd"

# Kulfan class-function exponents: round nose (N1 = 0.5), sharp
# trailing edge (N2 = 1.0) -- the standard airfoil class shape.
CST_N1 = 0.5
CST_N2 = 1.0


@dataclass
class AeolionMeshConfig:
    """Static discretization directives (config/aeolion.yaml).

    NOT design variables: the schema requires these to stay constant
    across the whole optimization to preserve gradient continuity."""
    n_planform_stations: int
    cst_order: int
    chordwise_panels: int
    spanwise_panels_per_section: int
    wake_model: str
    bemt_stations: int
    n_body_stations: int

    @classmethod
    def from_yaml(cls, path) -> "AeolionMeshConfig":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(
            n_planform_stations         = int(data["n_planform_stations"]),
            cst_order                   = int(data["cst_order"]),
            chordwise_panels            = int(data["chordwise_panels"]),
            spanwise_panels_per_section = int(data["spanwise_panels_per_section"]),
            wake_model                  = str(data["wake_model"]),
            bemt_stations               = int(data["bemt_stations"]),
            n_body_stations             = int(data["n_body_stations"]),
        )

    def validate(self) -> None:
        """Fail fast on values the schema would reject downstream."""
        if self.n_planform_stations < 2:
            raise ValueError("n_planform_stations must be at least 2")
        if not 4 <= self.cst_order <= 8:
            raise ValueError("cst_order must be within the schema's 4..8")
        if self.chordwise_panels < 4:
            raise ValueError("chordwise_panels must be at least 4")
        if self.spanwise_panels_per_section < 1:
            raise ValueError("spanwise_panels_per_section must be at least 1")
        if self.wake_model not in ("frozen", "relaxed"):
            raise ValueError("wake_model must be 'frozen' or 'relaxed'")
        if self.bemt_stations < 2:
            raise ValueError("bemt_stations must be at least 2")
        if self.n_body_stations < 2:
            raise ValueError("n_body_stations must be at least 2")


def _bernstein(x: np.ndarray, order: int) -> np.ndarray:
    """Bernstein basis matrix, shape (len(x), order)."""
    n = order - 1
    return np.stack(
        [math.comb(n, i) * x**i * (1.0 - x) ** (n - i) for i in range(order)],
        axis=1,
    )


def fit_cst(x, y, order: int) -> list[float]:
    """Least-squares Kulfan CST fit of one airfoil surface.

    x, y: surface coordinates on the unit chord, LE -> TE. The linear
    trailing-edge ramp x*y(1) is removed before fitting (sharp-TE
    equivalent -- the schema has no TE-thickness term). Returns `order`
    coefficients."""
    x = np.clip(np.asarray(x, dtype=float), 0.0, 1.0)
    y = np.asarray(y, dtype=float)
    y_sharp = y - x * y[-1]
    C = x**CST_N1 * (1.0 - x) ** CST_N2
    A = C[:, None] * _bernstein(x, order)
    coef, *_ = np.linalg.lstsq(A, y_sharp, rcond=None)
    return [float(c) for c in coef]


def eval_cst(x, coefficients) -> np.ndarray:
    """Evaluate a sharp-TE CST surface at x (for fit verification)."""
    x = np.clip(np.asarray(x, dtype=float), 0.0, 1.0)
    C = x**CST_N1 * (1.0 - x) ** CST_N2
    return C * (_bernstein(x, len(coefficients)) @ np.asarray(coefficients))


def cst_sections_from_naca4(designation: str, order: int) -> dict[str, list[float]]:
    """CST coefficients for the upper/lower surfaces of a NACA 4-digit."""
    M, P, t = parse_naca4(designation)
    xu, yu, xl, yl = naca4_coordinates(M, P, t)
    return {
        "coefficients_upper": fit_cst(xu, yu, order),
        "coefficients_lower": fit_cst(xl, yl, order),
    }


def _vane_hinge_axes(n_vanes: int) -> list[list[float]]:
    """Radial unit vectors of the vane hinge lines (FRD body axes).

    Same convention as the CAD `_radial_plate`: angle about +x with
    0 deg = +z, so four vanes land on +z, -y, -z, +y (T, R, B, L)."""
    axes = []
    for k in range(n_vanes):
        ang = 2.0 * math.pi * k / n_vanes
        axes.append([0.0,
                     round(-math.sin(ang), 12) + 0.0,
                     round(math.cos(ang), 12) + 0.0])
    return axes


def build_aeolion_geometry(
    *,
    span_m: float,
    chord_m: float,
    airfoil: str,
    ail: Mapping[str, Any],
    vanes: Mapping[str, Any],
    fus: Mapping[str, Any],
    rotor_D_m: float,
    prop: PropGeometry,
    f_shaft_hz: float,
    mesh: AeolionMeshConfig,
) -> dict[str, Any]:
    """Build the schema-1.0.0 Aeolion geometry document.

    span_m/chord_m come from the converged sizing result, `ail` from
    out/aileron.yaml, `vanes` from out/control_vanes.yaml, `fus` from
    out/fuselage.yaml, `f_shaft_hz` (hover 1/rev) from
    out/vibration.yaml. eta is the semispan fraction (0 root, 1 tip);
    the wing is currently rectangular and untwisted, so every station
    carries the same chord and the same CST section -- the fixed
    station count is the contract for future twist/taper variables.
    Vane entries reuse eta as the duct-exit radius fraction (see
    module docstring). The body block (schema 1.2.0) samples the same
    3-segment meridian the CAD revolve is built from
    (fuselage_design.fuselage_radius), cosine-spaced nose -> tail and
    mapped to body x = -station.
    """
    if span_m <= 0 or chord_m <= 0:
        raise ValueError("wing span and chord must be positive")
    if rotor_D_m <= 0:
        raise ValueError("rotor diameter must be positive")
    if f_shaft_hz <= 0:
        raise ValueError("shaft frequency must be positive")
    if not 0.0 < float(vanes["R_hub_m"]) < float(vanes["R_tip_m"]):
        raise ValueError("vane radii must satisfy 0 < R_hub < R_tip")
    if float(fus["D_fus_m"]) <= 0 or float(fus["L_fus_m"]) <= 0:
        raise ValueError("fuselage diameter and length must be positive")
    if float(ail["delta_max_deg"]) <= 0 or float(vanes["delta_max_deg"]) <= 0:
        raise ValueError("control-surface delta_max_deg must be positive")
    if not 0.0 < float(fus["x_wing_LE_m"]) < float(fus["L_fus_m"]):
        raise ValueError(
            "wing LE station must sit strictly between the nose tip and "
            "the tail base (0 < x_wing_LE_m < L_fus_m)"
        )
    if not 0.0 < float(fus["x_CG_m"]) < float(fus["L_fus_m"]):
        raise ValueError(
            "CG station must sit strictly between the nose tip and the "
            "tail base (0 < x_CG_m < L_fus_m)"
        )
    mesh.validate()

    n = mesh.n_planform_stations
    etas = [i / (n - 1) for i in range(n)]
    section = cst_sections_from_naca4(airfoil, mesh.cst_order)

    # BEMT stations span the exposed blade, hub collar -> tip, sampling
    # the same planform/twist laws the CAD rotor is lofted from.
    R = rotor_D_m / 2.0
    r0 = prop.hub_radius_ratio
    nb = mesh.bemt_stations
    blade_stations = []
    for i in range(nb):
        r_over_R = r0 + (1.0 - r0) * i / (nb - 1)
        blade_stations.append({
            "r_over_R": r_over_R,
            "chord": prop.chord_ratio(prop.loft_fraction(r_over_R)) * R,
            "twist": prop.twist_deg(r_over_R),
        })

    # Body of revolution: sample the 3-segment meridian (the SAME law
    # the CAD revolve uses) cosine-spaced from nose tip to tail base so
    # the elliptical nose and the tail cone are resolved where the
    # curvature lives. Station x_s (from nose, +aft) -> body x = -x_s.
    L_fus = float(fus["L_fus_m"])
    nbdy = mesh.n_body_stations
    body_stations = []
    for i in range(nbdy):
        x_s = 0.5 * L_fus * (1.0 - math.cos(math.pi * i / (nbdy - 1)))
        r = fuselage_radius(x_s, float(fus["D_fus_m"]), L_fus,
                            float(fus["f_nose"]), float(fus["f_tail"]),
                            float(fus["r_hub_m"]))
        body_stations.append({"x": round(-x_s, 12) + 0.0, "radius": r})

    payload: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "units": {"length": "m", "angle": "deg"},
        "reference_frame": REFERENCE_FRAME,
        # Vehicle CG (1.6.0), SAME source and sign convention as
        # cfd/vehicle/Allrun.case's CofR (out/fuselage.yaml x_CG_m): the
        # moment reference for any Cm/Cl/Cn a sweep of this geometry
        # produces, so VLM/BEMT moments stay directly comparable to the
        # project's CFD force/moment coefficients.
        "moment_reference_point": {
            "x": -float(fus["x_CG_m"]),
            "y": 0.0,
            "z": 0.0,
        },
        "planform": {
            # Root LE anchor (1.5.0): the SAME station and body-frame sign
            # convention as `body` (x = -station, 0 at the nose tip). The
            # wing chord datum sits at y=0/z=0 -- a through-fuselage wing
            # on the ADR-0008 carry-through spar, not an incidental
            # default -- so a consumer can place every planform station
            # in the same frame `body` is already in and detect overlap.
            "placement": {
                "root_leading_edge": {
                    "x": -float(fus["x_wing_LE_m"]),
                    "y": 0.0,
                    "z": 0.0,
                },
            },
            "span": float(span_m),
            "stations": [
                {
                    "eta": eta,
                    "chord": float(chord_m),
                    "twist": 0.0,
                    "sweep_qc": 0.0,
                    "dihedral": 0.0,
                }
                for eta in etas
            ],
        },
        "airfoil_sections": [
            {"eta": eta, "parameterization": "CST", **section} for eta in etas
        ],
        "control_surfaces": [
            {
                # outboard TE aileron (out/aileron.yaml); eta on the
                # semispan, hinge line spanwise (+y, FRD)
                "name": "aileron",
                "surface": "wing",
                "eta_start": 1.0 - float(ail["span_frac_wing"]),
                "eta_end": 1.0,
                "chord_fraction": float(ail["chord_frac"]),
                "hinge_axis": [0.0, 1.0, 0.0],
                "deflection_limits_deg": {
                    "min": -float(ail["delta_max_deg"]),
                    "max": float(ail["delta_max_deg"]),
                },
            },
        ] + [
            {
                # jet vanes (out/control_vanes.yaml): all-moving flat
                # plates in the duct jet, eta on the duct-exit radius
                # (hub collar -> duct wall), radial hinge axes
                "name": "vane",
                "surface": "duct_jet",
                "eta_start": float(vanes["R_hub_m"]) / float(vanes["R_tip_m"]),
                "eta_end": 1.0,
                "chord_fraction": 1.0,
                "hinge_axis": axis,
                "deflection_limits_deg": {
                    "min": -float(vanes["delta_max_deg"]),
                    "max": float(vanes["delta_max_deg"]),
                },
                "deflection_soft_limit_deg": float(vanes["delta_stall_deg"]),
            }
            for axis in _vane_hinge_axes(int(vanes["n_vanes"]))
        ],
        "body": {
            "length": L_fus,
            "stations": body_stations,
        },
        "propulsion_bemt": {
            "disk_radius": R,
            "reference_rpm": float(f_shaft_hz) * 60.0,
            "n_blades": int(prop.n_blades),
            "blade_stations": blade_stations,
        },
        "mesh_topology": {
            "chordwise_panels": mesh.chordwise_panels,
            "spanwise_panels_per_section": mesh.spanwise_panels_per_section,
            "wake_model": mesh.wake_model,
        },
    }

    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    digest = hashlib.sha256(canonical.encode("utf-8")).hexdigest()
    return {"design_id": f"sha256:{digest}", **payload}


def export_aeolion_geometry(path: str | Path, geometry: Mapping[str, Any]) -> str:
    """Write deterministic UTF-8 JSON and return its path."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(geometry, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return str(destination)
