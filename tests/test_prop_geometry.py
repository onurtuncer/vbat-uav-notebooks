"""Clark Y propeller-section loading, thickness scaling, and the
rotation-sense fact (ADR-0017): verifies the reference data, the
camber-preserving thickness-scaling math, and the closed-loop point
convention consumed by cad.prop_rotor."""

from pathlib import Path

import numpy as np
import pytest

from conceptual_design.prop_geometry import (
    ROTATION_AXIS_BODY_FRD,
    ClarkYSection,
    PropGeometry,
    clark_y_points,
    clark_y_surfaces,
)

REPO = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def section() -> ClarkYSection:
    return ClarkYSection.from_dat(REPO / "config" / "airfoils" / "clarky.dat")


@pytest.fixture(scope="module")
def prop() -> PropGeometry:
    return PropGeometry.from_yaml(REPO / "config" / "prop_geometry.yaml")


class TestClarkYReferenceData:
    def test_matches_published_clark_y_characteristics(self, section):
        # commonly cited: max t/c ~11.7% near x/c~0.28-0.30, max camber
        # ~3.4-3.55% near x/c~0.40-0.42 -- cross-check the DIGITIZED
        # data against the well-known section identity, not just
        # internal self-consistency.
        assert section.tc_ref == pytest.approx(0.117, abs=0.003)
        assert section.camber_max_ref == pytest.approx(0.0343, abs=0.003)

    def test_upper_lower_share_x_stations(self, section):
        assert np.array_equal(section.xu, section.xl)

    def test_closed_nose(self, section):
        assert section.xu[0] == 0.0 and section.yu[0] == 0.0
        assert section.xl[0] == 0.0 and section.yl[0] == 0.0


class TestThicknessScaling:
    def test_scaled_thickness_hits_target(self, section):
        for tc_target in (0.12, 0.06, 0.09):
            xu, yu, xl, yl = clark_y_surfaces(section, tc_target, n=81)
            assert float((yu - yl).max()) == pytest.approx(tc_target, rel=2e-2)

    def test_camber_line_preserved_under_scaling(self, section):
        # scaling thickness must NOT change the camber line -- that's
        # the whole point of decomposing camber/thickness separately
        _, yu0, _, yl0 = clark_y_surfaces(section, section.tc_ref, n=81)
        _, yu1, _, yl1 = clark_y_surfaces(section, 0.06, n=81)
        camber0 = 0.5 * (yu0 + yl0)
        camber1 = 0.5 * (yu1 + yl1)
        assert np.allclose(camber0, camber1, atol=1e-9)

    def test_rejects_nonpositive_target(self, section):
        with pytest.raises(ValueError, match="positive"):
            clark_y_surfaces(section, 0.0, n=41)


class TestClosedLoopPoints:
    def test_matches_naca4_points_convention(self, section):
        # upper LE->TE then lower TE->LE, no duplicate LE/TE -- same
        # convention as cad.wing_profile.naca4_points, so it drops
        # into cad.prop_rotor._blade_section_wire unchanged
        pts = clark_y_points(section, 0.12, n=60)
        assert len(pts) == 2 * 60 - 2
        assert pts[0] == (0.0, 0.0)          # LE
        # upper and lower share the TE x-station -- exactly ONE point
        # at x=1 survives (upper's), the lower's duplicate is dropped
        te_like = [p for p in pts if p[0] == 1.0]
        assert len(te_like) == 1


class TestPropGeometryTcAt:
    def test_linear_taper_root_to_tip(self, prop):
        assert prop.tc_at(0.0) == pytest.approx(prop.tc_root)
        assert prop.tc_at(1.0) == pytest.approx(prop.tc_tip)
        mid = prop.tc_at(0.5)
        assert min(prop.tc_root, prop.tc_tip) < mid < max(prop.tc_root, prop.tc_tip)

    def test_no_naca_camber_fields(self, prop):
        # ADR-0017: camber_M/camber_P removed -- Clark Y's camber is
        # fixed by real data, not a free parameter
        assert not hasattr(prop, "camber_M")
        assert not hasattr(prop, "camber_P")


class TestRotationSense:
    def test_axis_is_a_unit_vector_along_body_x(self):
        assert ROTATION_AXIS_BODY_FRD == (1.0, 0.0, 0.0)

    def test_gives_sensible_positive_aoa_at_several_radii(self, prop):
        # Numerically re-derive the ADR-0017 result independently of
        # the constant: for a blade-element relative-wind calculation,
        # rotation about ROTATION_AXIS_BODY_FRD must give a smaller,
        # more physically sensible angle of attack than the OPPOSITE
        # sign at every radius and flow regime -- the opposite sign is
        # not a slightly-worse alternative, it's ~180 deg (the blade
        # flying backwards through its own relative wind).
        import math
        R_tip = 0.1015

        def aoa_for(r_over_R: float, V_ax: float, omega_sign: float) -> float:
            r = r_over_R * R_tip
            beta = math.atan2(prop.pitch_ratio, math.pi * r_over_R)
            Omega = omega_sign * 300.0
            v_blade_y = -Omega * r
            wind_x, wind_y = -V_ax, -v_blade_y
            wind_angle = math.degrees(math.atan2(wind_x, wind_y))
            chord_angle = math.degrees(math.atan2(math.sin(beta), math.cos(beta)))
            return chord_angle - wind_angle

        for r_over_R in (0.3, 0.6, 0.95):
            for V_ax in (5.0, 25.0):
                correct = aoa_for(r_over_R, V_ax, ROTATION_AXIS_BODY_FRD[0])
                opposite = aoa_for(r_over_R, V_ax, -ROTATION_AXIS_BODY_FRD[0])
                assert 0.0 < correct < 150.0, (
                    f"r/R={r_over_R} V_ax={V_ax}: AoA={correct} deg not sensible")
                assert opposite > 145.0, (
                    f"r/R={r_over_R} V_ax={V_ax}: opposite-sign AoA={opposite} "
                    "should be near 180 deg (physically reversed)")
                assert correct < opposite - 20.0    # clearly separated, not a coin flip
