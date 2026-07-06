"""Tests for wing_sizing.py -- wing geometry and empirical structural mass."""

import math

import pytest

from conceptual_design.models import Aerodynamics, Environment
from conceptual_design.wing_sizing import (
    WingStructureParams,
    size_wing,
    wing_mass_raymer_kg,
)


@pytest.fixture
def aero() -> Aerodynamics:
    return Aerodynamics(LD=8.0, CD0=0.025, AR=6.0, e=0.80, CL_max=1.4, V_stall=12.0, V_max=35.0)


class TestWingGeometry:
    def test_geometry_relations(self, aero):
        env = Environment()
        MTOW, WS = 5.0, 200.0
        wing = size_wing(MTOW, WS, 0.35, aero, mission_V=20.0, env=env)

        S_expected = MTOW * env.g / WS
        assert wing.S_wing == pytest.approx(S_expected, rel=1e-12)
        assert wing.b_wing == pytest.approx(math.sqrt(aero.AR * S_expected), rel=1e-12)
        assert wing.chord_mean == pytest.approx(wing.S_wing / wing.b_wing, rel=1e-12)
        # Span/chord must reproduce the aspect ratio
        assert wing.b_wing / wing.chord_mean == pytest.approx(aero.AR, rel=1e-12)
        assert wing.wing_loading == WS
        assert wing.T_W_design == 0.35

    @pytest.mark.parametrize("mtow,ws", [(0.0, 200.0), (5.0, 0.0), (-1.0, 200.0), (5.0, -50.0)])
    def test_rejects_nonpositive_inputs(self, aero, mtow, ws):
        with pytest.raises(ValueError):
            size_wing(mtow, ws, 0.35, aero, mission_V=20.0)


class TestWingMass:
    def test_raymer_mass_positive_and_increases_with_mtow(self):
        m_light = wing_mass_raymer_kg(0.25, 6.0, MTOW_kg=3.0, V_cruise_ms=20.0, rho=1.225)
        m_heavy = wing_mass_raymer_kg(0.25, 6.0, MTOW_kg=6.0, V_cruise_ms=20.0, rho=1.225)
        assert 0.0 < m_light < m_heavy

    def test_material_factor_scales_linearly(self):
        cfrp = WingStructureParams(k_material=0.70)
        alu = WingStructureParams(k_material=1.00)
        m_cfrp = wing_mass_raymer_kg(0.25, 6.0, 5.0, 20.0, 1.225, ws=cfrp)
        m_alu = wing_mass_raymer_kg(0.25, 6.0, 5.0, 20.0, 1.225, ws=alu)
        assert m_cfrp == pytest.approx(0.70 * m_alu, rel=1e-12)

    def test_wing_mass_is_small_fraction_of_mtow(self, aero):
        # Sanity: a 5 kg UAV wing should weigh grams-to-fraction-of-kg,
        # not more than the aircraft itself.
        wing = size_wing(5.0, 200.0, 0.35, aero, mission_V=20.0)
        assert 0.0 < wing.mass_wing_kg < 5.0

    def test_nicolai_method_selectable(self, aero):
        ws = WingStructureParams(method="nicolai")
        wing = size_wing(5.0, 200.0, 0.35, aero, mission_V=20.0, ws=ws)
        assert wing.mass_wing_kg > 0.0

    def test_unknown_method_raises(self, aero):
        ws = WingStructureParams(method="torenbeek")
        with pytest.raises(ValueError):
            size_wing(5.0, 200.0, 0.35, aero, mission_V=20.0, ws=ws)
