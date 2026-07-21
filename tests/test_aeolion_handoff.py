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
from conceptual_design.prop_geometry import (
    ROTATION_AXIS_BODY_FRD, ClarkYSection, PropGeometry, clark_y_surfaces,
)

REPO = Path(__file__).resolve().parents[1]
SCHEMA = json.loads(
    (REPO / "schemas" / "vbat-aeolion-geometry-handoff.schema.json").read_text(
        encoding="utf-8"
    )
)

AIL = {"span_frac_wing": 0.12, "chord_frac": 0.25, "delta_max_deg": 20.0}
VANES = {"n_vanes": 4, "R_hub_m": 0.0406, "R_tip_m": 0.1015,
         "delta_max_deg": 20.0, "delta_stall_deg": 15.0}
FUS = {"D_fus_m": 0.0982, "L_fus_m": 0.49099, "f_nose": 0.22,
       "f_tail": 0.3, "r_hub_m": 0.0406, "x_wing_LE_m": 0.20687,
       "x_CG_m": 0.24229}


@pytest.fixture(scope="module")
def mesh():
    return AeolionMeshConfig.from_yaml(REPO / "config" / "aeolion.yaml")


@pytest.fixture(scope="module")
def prop():
    return PropGeometry.from_yaml(REPO / "config" / "prop_geometry.yaml")


@pytest.fixture(scope="module")
def clarky():
    return ClarkYSection.from_dat(REPO / "config" / "airfoils" / "clarky.dat")


@pytest.fixture(scope="module")
def geometry(mesh, prop, clarky):
    return build_aeolion_geometry(
        span_m=1.016, chord_m=0.1695, airfoil="NACA 4412", ail=AIL, vanes=VANES, fus=FUS,
        rotor_D_m=0.203, prop=prop, clarky=clarky, f_shaft_hz=203.5, mesh=mesh,
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
        assert aileron["deflection_limits_deg"] == {"min": -20.0, "max": 20.0}
        assert "deflection_soft_limit_deg" not in aileron

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
            assert v["deflection_limits_deg"] == {"min": -20.0, "max": 20.0}
            assert v["deflection_soft_limit_deg"] == pytest.approx(15.0)
        axes = {tuple(v["hinge_axis"]) for v in vanes}
        assert axes == {(0.0, 0.0, 1.0), (0.0, -1.0, 0.0),
                        (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)}

    def test_bemt_matches_cad_blade_law(self, geometry, prop):
        bemt = geometry["propulsion_bemt"]
        assert bemt["disk_radius"] == pytest.approx(0.1015)
        assert bemt["reference_rpm"] == pytest.approx(203.5 * 60.0)
        assert bemt["n_blades"] == prop.n_blades
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

    def test_rotation_axis_is_the_derived_constant(self, geometry):
        # ADR-0017: numerically derived, not an assumed/guessed sign
        assert geometry["propulsion_bemt"]["rotation_axis"] == list(
            ROTATION_AXIS_BODY_FRD)

    def test_bemt_airfoil_sections_align_with_blade_stations(self, geometry):
        stations = geometry["propulsion_bemt"]["blade_stations"]
        sections = geometry["propulsion_bemt"]["airfoil_sections"]
        assert [s["r_over_R"] for s in stations] == [
            a["r_over_R"] for a in sections]
        for sec in sections:
            assert sec["parameterization"] == "CST"
            assert 4 <= len(sec["coefficients_upper"]) <= 8
            assert 4 <= len(sec["coefficients_lower"]) <= 8

    def test_bemt_sections_thickness_scale_root_to_tip(
        self, geometry, prop, clarky,
    ):
        # root section must be measurably thicker than the tip section
        # (tc_root=0.12 > tc_tip=0.06) -- reconstruct max thickness from
        # the fitted CST coefficients via eval_cst and compare.
        sections = geometry["propulsion_bemt"]["airfoil_sections"]
        root, tip = sections[0], sections[-1]
        x = np.linspace(0.0, 1.0, 200)
        for sec, tc_expected in ((root, prop.tc_root), (tip, prop.tc_tip)):
            yu = eval_cst(x, sec["coefficients_upper"])
            yl = eval_cst(x, sec["coefficients_lower"])
            assert float((yu - yl).max()) == pytest.approx(tc_expected, rel=0.1)

    def test_reconstructs_clarky_surfaces(self, clarky):
        # same fit-fidelity check as the wing's NACA CST fit, applied
        # to the Clark Y reference at its own (unscaled) thickness
        from conceptual_design.aeolion_handoff import cst_sections_from_clarky
        order = 6
        sec = cst_sections_from_clarky(clarky, clarky.tc_ref, order)
        xu, yu, xl, yl = clark_y_surfaces(clarky, clarky.tc_ref, n=81)
        for x, y, coeffs in (
            (xu, yu, sec["coefficients_upper"]),
            (xl, yl, sec["coefficients_lower"]),
        ):
            y_sharp = y - x * y[-1]
            err = eval_cst(x, coeffs) - y_sharp
            assert float(np.sqrt(np.mean(err**2))) < 1e-3


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


class TestPlacement:
    def test_root_leading_edge_matches_the_fuselage_station(self, geometry):
        # SAME body-frame sign convention as `body`: x = -station, 0 at
        # the nose tip. Root sits on the centerline (y = z = 0) --
        # a through-fuselage wing on the carry-through spar, not an
        # incidental default.
        rle = geometry["planform"]["placement"]["root_leading_edge"]
        assert rle["x"] == pytest.approx(-FUS["x_wing_LE_m"])
        assert rle["y"] == 0.0
        assert rle["z"] == 0.0

    def test_root_leading_edge_sits_inside_the_body(self, geometry):
        # the exact bug this field fixes: an anchor-less consumer that
        # assumes the root sits at the reference-frame origin places
        # the wing at the fuselage nose tip. Pin that x is strictly
        # between the nose (0) and the tail base (-L), i.e. inside the
        # body's station range, not coincident with either end.
        rle_x = geometry["planform"]["placement"]["root_leading_edge"]["x"]
        body = geometry["body"]
        assert -body["length"] < rle_x < 0.0

    def test_rejects_wing_le_outside_the_fuselage(self, mesh, prop, clarky):
        bad_fus = {**FUS, "x_wing_LE_m": 0.0}
        with pytest.raises(ValueError, match="nose tip and the tail base"):
            build_aeolion_geometry(
                span_m=1.016, chord_m=0.1695, airfoil="NACA 4412", ail=AIL,
                vanes=VANES, fus=bad_fus, rotor_D_m=0.203, prop=prop, clarky=clarky,
                f_shaft_hz=203.5, mesh=mesh,
            )
        bad_fus = {**FUS, "x_wing_LE_m": FUS["L_fus_m"]}
        with pytest.raises(ValueError, match="nose tip and the tail base"):
            build_aeolion_geometry(
                span_m=1.016, chord_m=0.1695, airfoil="NACA 4412", ail=AIL,
                vanes=VANES, fus=bad_fus, rotor_D_m=0.203, prop=prop, clarky=clarky,
                f_shaft_hz=203.5, mesh=mesh,
            )


class TestMomentReferencePoint:
    def test_matches_the_fuselage_cg_station(self, geometry):
        # SAME body-frame sign convention as `body`/`placement`, and
        # the SAME source Allrun.case's CofR uses (out/fuselage.yaml
        # x_CG_m) -- VLM/BEMT moments stay comparable to project CFD.
        mrp = geometry["moment_reference_point"]
        assert mrp["x"] == pytest.approx(-FUS["x_CG_m"])
        assert mrp["y"] == 0.0
        assert mrp["z"] == 0.0

    def test_cg_sits_inside_the_body(self, geometry):
        mrp_x = geometry["moment_reference_point"]["x"]
        body = geometry["body"]
        assert -body["length"] < mrp_x < 0.0

    def test_rejects_cg_outside_the_fuselage(self, mesh, prop, clarky):
        bad_fus = {**FUS, "x_CG_m": 0.0}
        with pytest.raises(ValueError, match="nose tip and the tail base"):
            build_aeolion_geometry(
                span_m=1.016, chord_m=0.1695, airfoil="NACA 4412", ail=AIL,
                vanes=VANES, fus=bad_fus, rotor_D_m=0.203, prop=prop, clarky=clarky,
                f_shaft_hz=203.5, mesh=mesh,
            )
        bad_fus = {**FUS, "x_CG_m": FUS["L_fus_m"]}
        with pytest.raises(ValueError, match="nose tip and the tail base"):
            build_aeolion_geometry(
                span_m=1.016, chord_m=0.1695, airfoil="NACA 4412", ail=AIL,
                vanes=VANES, fus=bad_fus, rotor_D_m=0.203, prop=prop, clarky=clarky,
                f_shaft_hz=203.5, mesh=mesh,
            )


def _minimal_doc(schema_version: str, *, with_placement: bool,
                 with_mrp: bool, propulsion_bemt: str | None = None) -> dict:
    planform = {
        "span": 1.0,
        "stations": [
            {"eta": 0.0, "chord": 0.2, "twist": 0.0,
             "sweep_qc": 0.0, "dihedral": 0.0},
            {"eta": 1.0, "chord": 0.2, "twist": 0.0,
             "sweep_qc": 0.0, "dihedral": 0.0},
        ],
    }
    if with_placement:
        planform["placement"] = {
            "root_leading_edge": {"x": -0.2, "y": 0.0, "z": 0.0}
        }
    doc = {
        "schema_version": schema_version,
        "design_id": "sha256:0" * 8,
        "units": {"length": "m", "angle": "deg"},
        "reference_frame": "aetherion_body_frd",
        "planform": planform,
        "airfoil_sections": [
            {"eta": 0.0, "parameterization": "CST",
             "coefficients_upper": [0.1, 0.1, 0.1, 0.1],
             "coefficients_lower": [-0.1, -0.1, -0.1, -0.1]},
            {"eta": 1.0, "parameterization": "CST",
             "coefficients_upper": [0.1, 0.1, 0.1, 0.1],
             "coefficients_lower": [-0.1, -0.1, -0.1, -0.1]},
        ],
        "mesh_topology": {"chordwise_panels": 4,
                          "spanwise_panels_per_section": 1},
    }
    if with_mrp:
        doc["moment_reference_point"] = {"x": -0.24, "y": 0.0, "z": 0.0}
    if propulsion_bemt in ("incomplete", "complete"):
        bemt = {
            "disk_radius": 0.1,
            "reference_rpm": 12000.0,
            "n_blades": 3,
            "blade_stations": [
                {"r_over_R": 0.4, "chord": 0.02, "twist": 30.0},
                {"r_over_R": 1.0, "chord": 0.01, "twist": 10.0},
            ],
        }
        if propulsion_bemt == "complete":
            bemt["airfoil_sections"] = [
                {"r_over_R": 0.4, "parameterization": "CST",
                 "coefficients_upper": [0.1, 0.1, 0.1, 0.1],
                 "coefficients_lower": [-0.1, -0.1, -0.1, -0.1]},
                {"r_over_R": 1.0, "parameterization": "CST",
                 "coefficients_upper": [0.1, 0.1, 0.1, 0.1],
                 "coefficients_lower": [-0.1, -0.1, -0.1, -0.1]},
            ]
            bemt["rotation_axis"] = [1.0, 0.0, 0.0]
        doc["propulsion_bemt"] = bemt
    return doc


class TestVersionConditionalPlacement:
    """The schema's own if/then: placement is required from 1.5.0
    onward but must not retroactively invalidate 1.0.0-1.4.0
    documents that never carried it."""

    def test_pre_1_5_0_documents_stay_valid_without_placement(self):
        for v in ("1.0.0", "1.1.0", "1.2.0", "1.3.0", "1.4.0"):
            jsonschema.validate(
                _minimal_doc(v, with_placement=False, with_mrp=False), SCHEMA)

    def test_1_5_0_document_requires_placement(self):
        jsonschema.validate(
            _minimal_doc("1.5.0", with_placement=True, with_mrp=False), SCHEMA)
        with pytest.raises(jsonschema.ValidationError):
            jsonschema.validate(
                _minimal_doc("1.5.0", with_placement=False, with_mrp=False), SCHEMA)


class TestVersionConditionalMomentReference:
    """Same pattern, one version later: moment_reference_point is
    required from 1.6.0 onward but must not retroactively invalidate
    1.0.0-1.5.0 documents that never carried it."""

    def test_pre_1_6_0_documents_stay_valid_without_mrp(self):
        # placement is required from 1.5.0 (a separate, earlier
        # constraint) -- supply it for that one version so this test
        # isolates the moment_reference_point constraint specifically.
        for v in ("1.0.0", "1.1.0", "1.2.0", "1.3.0", "1.4.0", "1.5.0"):
            jsonschema.validate(
                _minimal_doc(v, with_placement=(v == "1.5.0"), with_mrp=False),
                SCHEMA,
            )

    def test_1_6_0_document_requires_mrp(self):
        jsonschema.validate(
            _minimal_doc("1.6.0", with_placement=True, with_mrp=True), SCHEMA)
        with pytest.raises(jsonschema.ValidationError):
            jsonschema.validate(
                _minimal_doc("1.6.0", with_placement=True, with_mrp=False), SCHEMA)


class TestVersionConditionalPropulsionBemt:
    """Same pattern again: propulsion_bemt.airfoil_sections/rotation_axis
    are required from 1.7.0 onward, but ONLY if propulsion_bemt itself
    is present -- the block stays entirely optional at every version."""

    def _kw(self, schema_version: str) -> dict:
        return dict(schema_version=schema_version, with_placement=True,
                   with_mrp=True)

    def test_propulsion_bemt_remains_fully_optional_at_1_7_0(self):
        # omitting propulsion_bemt altogether must still validate, even
        # at 1.7.0 -- the constraint only bites when the block exists
        jsonschema.validate(
            _minimal_doc(**self._kw("1.7.0"), propulsion_bemt=None), SCHEMA)

    def test_pre_1_7_0_documents_stay_valid_with_incomplete_bemt(self):
        for v in ("1.0.0", "1.1.0", "1.2.0", "1.3.0", "1.4.0", "1.5.0", "1.6.0"):
            jsonschema.validate(
                _minimal_doc(**self._kw(v), propulsion_bemt="incomplete"), SCHEMA)

    def test_1_7_0_document_requires_complete_bemt_if_present(self):
        jsonschema.validate(
            _minimal_doc(**self._kw("1.7.0"), propulsion_bemt="complete"), SCHEMA)
        with pytest.raises(jsonschema.ValidationError):
            jsonschema.validate(
                _minimal_doc(**self._kw("1.7.0"), propulsion_bemt="incomplete"),
                SCHEMA,
            )


class TestDesignId:
    def test_deterministic_across_reruns(self, geometry, mesh, prop, clarky):
        again = build_aeolion_geometry(
            span_m=1.016, chord_m=0.1695, airfoil="NACA 4412", ail=AIL, vanes=VANES, fus=FUS,
            rotor_D_m=0.203, prop=prop, clarky=clarky, f_shaft_hz=203.5, mesh=mesh,
        )
        assert again == geometry

    def test_moves_with_the_design_point(self, geometry, mesh, prop, clarky):
        perturbed = build_aeolion_geometry(
            span_m=1.017, chord_m=0.1695, airfoil="NACA 4412", ail=AIL, vanes=VANES, fus=FUS,
            rotor_D_m=0.203, prop=prop, clarky=clarky, f_shaft_hz=203.5, mesh=mesh,
        )
        assert perturbed["design_id"] != geometry["design_id"]


class TestGuards:
    def test_rejects_nonpositive_dimensions(self, mesh, prop, clarky):
        with pytest.raises(ValueError, match="positive"):
            build_aeolion_geometry(
                span_m=0.0, chord_m=0.2, airfoil="NACA 4412", ail=AIL, vanes=VANES, fus=FUS,
                rotor_D_m=0.203, prop=prop, clarky=clarky, f_shaft_hz=203.5, mesh=mesh,
            )

    def test_rejects_nonpositive_deflection_limit(self, mesh, prop, clarky):
        bad_ail = {**AIL, "delta_max_deg": 0.0}
        with pytest.raises(ValueError, match="delta_max_deg must be positive"):
            build_aeolion_geometry(
                span_m=1.0, chord_m=0.2, airfoil="NACA 4412", ail=bad_ail, vanes=VANES, fus=FUS,
                rotor_D_m=0.203, prop=prop, clarky=clarky, f_shaft_hz=203.5, mesh=mesh,
            )

    def test_rejects_cst_order_outside_schema(self, mesh, prop, clarky):
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
                rotor_D_m=0.203, prop=prop, clarky=clarky, f_shaft_hz=203.5, mesh=bad,
            )
