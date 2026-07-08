"""Tests for vtol_power.py -- hover/climb power-to-weight (momentum theory)."""

from math import sqrt

import pytest

from conceptual_design.vtol_power import (
    VTOLParams,
    rpm_from_diameter,
    vtip_from_rpm_and_diameter,
    vtol_climb_power_to_weight,
    vtol_hover_power_to_weight,
    vtol_power_requirements,
)


class TestHoverPowerToWeight:
    def test_hand_checked_value(self):
        # P/W = sqrt(DL/(2*rho))/FoM = sqrt(150/2.45)/0.70 = 11.178 W/N
        p = VTOLParams(rho0=1.225, FoM=0.70)
        assert vtol_hover_power_to_weight(150.0, p) == pytest.approx(11.178, rel=1e-3)

    def test_scales_with_sqrt_of_disk_loading(self):
        p = VTOLParams()
        ratio = vtol_hover_power_to_weight(400.0, p) / vtol_hover_power_to_weight(100.0, p)
        assert ratio == pytest.approx(2.0, rel=1e-12)

    def test_perfect_rotor_gives_ideal_momentum_power(self):
        p = VTOLParams(rho0=1.225, FoM=1.0)
        DL = 150.0
        assert vtol_hover_power_to_weight(DL, p) == pytest.approx(sqrt(DL / (2 * 1.225)), rel=1e-12)

    @pytest.mark.parametrize("bad_dl", [0.0, -10.0])
    def test_rejects_nonpositive_disk_loading(self, bad_dl):
        with pytest.raises(ValueError):
            vtol_hover_power_to_weight(bad_dl)


class TestRpmAndTipSpeed:
    def test_rpm_decreases_with_diameter(self):
        p = VTOLParams()  # rpm_exp < 0
        assert rpm_from_diameter(0.20, p) > rpm_from_diameter(0.40, p)

    def test_rpm_rejects_nonpositive_diameter(self):
        with pytest.raises(ValueError):
            rpm_from_diameter(0.0, VTOLParams())

    def test_tip_speed_in_physical_range(self):
        # Small rotors typically run 100-250 m/s tip speed
        p = VTOLParams()
        for D in (0.2, 0.28, 0.5):
            vtip = vtip_from_rpm_and_diameter(rpm_from_diameter(D, p), D)
            assert 50.0 < vtip < 300.0


class TestClimbPowerToWeight:
    def test_exceeds_ideal_hover_at_zero_climb_rate(self):
        # At RoC=0 the induced term collapses to ideal hover power;
        # blade profile drag makes the total strictly larger.
        p = VTOLParams()
        DL = 150.0
        pw = vtol_climb_power_to_weight(0.0, DL, 200.0, 0.28, p)
        assert pw > sqrt(DL / (2 * p.rho0))

    def test_monotonically_increasing_with_climb_rate(self):
        p = VTOLParams()
        values = [
            vtol_climb_power_to_weight(roc, 150.0, 200.0, 0.28, p)
            for roc in (0.0, 1.0, 2.5, 5.0)
        ]
        assert values == sorted(values)
        assert values[0] < values[-1]

    @pytest.mark.parametrize("dl,wl", [(0.0, 200.0), (150.0, 0.0), (-1.0, 200.0)])
    def test_rejects_nonpositive_loadings(self, dl, wl):
        with pytest.raises(ValueError):
            vtol_climb_power_to_weight(2.5, dl, wl, 0.28)


class TestPowerRequirements:
    def test_absolute_powers_scale_linearly_with_weight(self):
        res1 = vtol_power_requirements(50.0, 2.5, 150.0, 200.0, 0.28)
        res2 = vtol_power_requirements(100.0, 2.5, 150.0, 200.0, 0.28)
        assert res2["P_hover_W"] == pytest.approx(2 * res1["P_hover_W"], rel=1e-12)
        assert res2["P_climb_W"] == pytest.approx(2 * res1["P_climb_W"], rel=1e-12)

    def test_design_power_is_max_of_hover_and_climb(self):
        res = vtol_power_requirements(50.0, 2.5, 150.0, 200.0, 0.28)
        assert res["P_design_W"] == pytest.approx(max(res["P_hover_W"], res["P_climb_W"]))

    def test_rejects_nonpositive_weight(self):
        with pytest.raises(ValueError):
            vtol_power_requirements(0.0, 2.5, 150.0, 200.0, 0.28)
