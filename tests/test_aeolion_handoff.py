"""Aeolion geometry handoff: schema conformance and geometry fidelity.

Validates the emitted document against the pinned contract
(schemas/vbat-aeolion-geometry-handoff.schema.json) and checks the
parts jsonschema cannot see: CST fit fidelity, station alignment,
blade-law consistency with the CAD rotor, and design_id traceability.
"""

import json
import math
from pathlib import Path

import jsonschema
import numpy as np
import pytest

from conceptual_design.aeolion_handoff import (
    AeolionMeshConfig,
    build_aeolion_geometry,
    cst_sections_from_naca4,
    eval_cst,
    export_aeolion_geometry,
)
from conceptual_design.airfoil_selection import naca4_coordinates, parse_naca4
from conceptual_design.prop_geometry import PropGeometry

REPO = Path(__file__).resolve().parents[1]
SCHEMA = json.loads(
    (REPO / "schemas" / "vbat-aeolion-geometry-handoff.schema.json").read_text(
        encoding="utf-8"
    )
)

AIL = {"span_frac_wing": 0.12, "chord_frac": 0.25}
VANES = {"n_vanes": 4, "R_hub_m": 0.0406, "R_tip_m": 0.1015}
FUS = {"D_fus_m": 0.0982, "L_fus_m": 0.49099, "f_nose": 0.22,
       "f_tail": 0.3, "r_hub_m": 0.0406}


@pytest.fixture(scope="module")
def mesh():
    return AeolionMeshConfig.from_yaml(REPO / "config" / "aeolion.yaml")


@pytest.fixture(scope="module")
def prop():
    return PropGeometry.from_yaml(REPO / "config" / "prop_geometry.yaml")


@pytest.fixture(scope="module")
def geometry(mesh, prop):
    return build_aeolion_geometry(
        span_m=1.016, chord_m=0.1695, airfoil="NACA 4412", ail=AIL, vanes=VANES, fus=FUS,
        rotor_D_m=0.203, prop=prop, f_shaft_hz=203.5, mesh=mesh,
    )


class TestSchemaConformance:
    def test_validates_against_pinned_schema(self, geometry):
        jsonschema.validate(geometry, SCHEMA)

    def test_no_extra_top_level_keys(self, geometry):
        # additionalProperties: false -- e.g. no "$schema" marker, no
        # "cad" traceability block; the schema is the whole contract
        assert set(geometry) <= set(SCHEMA["properties"])

    def test_roundtrips_through_export(self, geometry, tmp_path):
        out = export_aeolion_geometry(tmp_path / "geom.json", geometry)
        loaded = json.loads(Path(out).read_text(encoding="utf-8"))
        jsonschema.validate(loaded, SCHEMA)
        assert loaded == geometry


class TestStations:
    def test_sections_align_with_stations_by_eta(self, geometry):
        st = [s["eta"] for s in geometry["planform"]["stations"]]
        se = [s["eta"] for s in geometry["airfoil_sections"]]
        assert st == se
        assert st[0] == 0.0 and st[-1] == 1.0

    def test_station_count_comes_from_config(self, geometry, mesh):
        # the count is the fixed design-vector length -- must track the
        # config, never the design point
        assert len(geometry["planform"]["stations"]) == mesh.n_planform_stations

    def test_rectangular_untwisted_wing(self, geometry):
        for s in geometry["planform"]["stations"]:
            assert s["chord"] == pytest.approx(0.1695)
            assert s["twist"] == 0.0
            assert s["sweep_qc"] == 0.0
            assert s["dihedral"] == 0.0


class TestCstFit:
    def test_reconstructs_naca4412_surfaces(self, mesh):
        section = cst_sections_from_naca4("NACA 4412", mesh.cst_order)
        M, P, t = parse_naca4("NACA 4412")
        xu, yu, xl, yl = naca4_coordinates(M, P, t)
        for x, y, coeffs in (
            (xu, yu, section["coefficients_upper"]),
            (xl, yl, section["coefficients_lower"]),
        ):
            x = np.clip(np.asarray(x), 0.0, 1.0)
            y = np.asarray(y)
            y_sharp = y - x * y[-1]          # sharp-TE equivalent
            err = eval_cst(x, coeffs) - y_sharp
            assert float(np.sqrt(np.mean(err**2))) < 1e-3

    def test_coefficient_count_within_schema_bounds(self, geometry):
        for sec in geometry["airfoil_sections"]:
            assert 4 <= len(sec["coefficients_upper"]) <= 8
            assert 4 <= len(sec["coefficients_lower"]) <= 8

    def test_camber_sign(self, geometry):
        # NACA 4412: upper surface lifts above the chord line, lower
        # stays shallow -- leading CST coefficients must reflect that
        sec = geometry["airfoil_sections"][0]
        assert sec["coefficients_upper"][0] > 0
        assert sec["coefficients_upper"][0] > sec["coefficients_lower"][0]


class TestControlsAndBemt:
    def test_aileron_mapping(self, geometry):
        aileron, *rest = geometry["control_surfaces"]
        assert aileron["name"] == "aileron"
        assert aileron["surface"] == "wing"
        assert aileron["eta_start"] == pytest.approx(0.88)
        assert aileron["eta_end"] == 1.0
        assert aileron["chord_fraction"] == pytest.approx(0.25)
        assert aileron["hinge_axis"] == [0.0, 1.0, 0.0]

    def test_vane_mapping(self, geometry):
        # four all-moving jet vanes: eta on the duct-exit radius (hub
        # collar -> duct wall), radial hinge axes on the CAD's +z/-y/
        # -z/+y pattern (angle about +x, 0 deg = +z), whole chord
        # rotating about the hinge_xc line.
        vanes = [c for c in geometry["control_surfaces"] if c["name"] == "vane"]
        assert len(vanes) == VANES["n_vanes"]
        for v in vanes:
            assert v["surface"] == "duct_jet"    # never the wing lattice
            assert v["eta_start"] == pytest.approx(
                VANES["R_hub_m"] / VANES["R_tip_m"])
            assert v["eta_end"] == 1.0
            assert v["chord_fraction"] == 1.0
        axes = {tuple(v["hinge_axis"]) for v in vanes}
        assert axes == {(0.0, 0.0, 1.0), (0.0, -1.0, 0.0),
                        (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)}

    def test_bemt_matches_cad_blade_law(self, geometry, prop):
        bemt = geometry["propulsion_bemt"]
        assert bemt["disk_radius"] == pytest.approx(0.1015)
        assert bemt["reference_rpm"] == pytest.approx(203.5 * 60.0)
        stations = bemt["blade_stations"]
        assert stations[0]["r_over_R"] == pytest.approx(prop.hub_radius_ratio)
        assert stations[-1]["r_over_R"] == pytest.approx(1.0)
        mid = stations[len(stations) // 2]
        f = prop.loft_fraction(mid["r_over_R"])
        assert mid["chord"] == pytest.approx(prop.chord_ratio(f) * 0.1015)
        assert mid["twist"] == pytest.approx(
            math.degrees(math.atan2(prop.pitch_ratio, math.pi * mid["r_over_R"]))
        )
        # exact 8x6 pitch law at the tip: atan(0.75/pi)
        assert stations[-1]["twist"] == pytest.approx(13.43, abs=0.01)


class TestBody:
    def test_body_of_revolution_samples_the_cad_meridian(self, geometry):
        # body x = -station: 0 at the nose tip, -L at the tail base;
        # radius 0 at the tip, D/2 on the cylindrical mid, hub radius
        # at the tail base -- the same 3-segment law the CAD revolves.
        body = geometry["body"]
        assert body["length"] == pytest.approx(FUS["L_fus_m"])
        st = body["stations"]
        assert st[0]["x"] == 0.0
        assert st[0]["radius"] == 0.0
        assert st[-1]["x"] == pytest.approx(-FUS["L_fus_m"])
        assert st[-1]["radius"] == pytest.approx(FUS["r_hub_m"])
        xs = [s["x"] for s in st]
        assert xs == sorted(xs, reverse=True)          # nose -> tail
        assert max(s["radius"] for s in st) == pytest.approx(
            FUS["D_fus_m"] / 2.0)

    def test_station_count_comes_from_config(self, geometry, mesh):
        assert len(geometry["body"]["stations"]) == mesh.n_body_stations


class TestDesignId:
    def test_deterministic_across_reruns(self, geometry, mesh, prop):
        again = build_aeolion_geometry(
            span_m=1.016, chord_m=0.1695, airfoil="NACA 4412", ail=AIL, vanes=VANES, fus=FUS,
            rotor_D_m=0.203, prop=prop, f_shaft_hz=203.5, mesh=mesh,
        )
        assert again == geometry

    def test_moves_with_the_design_point(self, geometry, mesh, prop):
        perturbed = build_aeolion_geometry(
            span_m=1.017, chord_m=0.1695, airfoil="NACA 4412", ail=AIL, vanes=VANES, fus=FUS,
            rotor_D_m=0.203, prop=prop, f_shaft_hz=203.5, mesh=mesh,
        )
        assert perturbed["design_id"] != geometry["design_id"]


class TestGuards:
    def test_rejects_nonpositive_dimensions(self, mesh, prop):
        with pytest.raises(ValueError, match="positive"):
            build_aeolion_geometry(
                span_m=0.0, chord_m=0.2, airfoil="NACA 4412", ail=AIL, vanes=VANES, fus=FUS,
                rotor_D_m=0.203, prop=prop, f_shaft_hz=203.5, mesh=mesh,
            )

    def test_rejects_cst_order_outside_schema(self, mesh, prop):
        bad = AeolionMeshConfig(
            n_planform_stations=mesh.n_planform_stations, cst_order=3,
            chordwise_panels=mesh.chordwise_panels,
            spanwise_panels_per_section=mesh.spanwise_panels_per_section,
            wake_model=mesh.wake_model, bemt_stations=mesh.bemt_stations,
            n_body_stations=mesh.n_body_stations,
        )
        with pytest.raises(ValueError, match="4..8"):
            build_aeolion_geometry(
                span_m=1.0, chord_m=0.2, airfoil="NACA 4412", ail=AIL, vanes=VANES, fus=FUS,
                rotor_D_m=0.203, prop=prop, f_shaft_hz=203.5, mesh=bad,
            )
