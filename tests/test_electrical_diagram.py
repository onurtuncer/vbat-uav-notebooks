"""Tests for electrical_diagram.py -- wiring diagram generation."""

import xml.etree.ElementTree as ET

import pytest

from conceptual_design.electrical_diagram import (
    ElectricalParams,
    compute_operating_point,
    render_wiring_svg,
    write_wiring_diagram,
)
from conceptual_design.mass_closure import run_sizing_loop
from conceptual_design.models import (
    Aerodynamics,
    Battery,
    Environment,
    Mission,
    PropulsiveSystemParameters,
    RotorParams,
    WeightFraction,
)
from conceptual_design.forward_flight_power import ForwardFlightParams
from conceptual_design.wing_sizing import WingStructureParams


@pytest.fixture
def elec() -> ElectricalParams:
    return ElectricalParams(
        battery_series_cells=6, cell_nominal_v=3.7,
        esc_current_margin=1.6, bec1_budget_a=3.0, bec2_budget_a=6.0,
    )


@pytest.fixture
def batt() -> Battery:
    return Battery(specific_energy=140.0, usable_fraction=0.85, eta_bat=0.97, c_rate_max=25.0)


@pytest.fixture
def rotor() -> RotorParams:
    return RotorParams(D_rotor_m=0.195)


@pytest.fixture
def sizing_result(batt):
    mission = Mission(t_hover=120, t_cruise=900, V_cruise=20.0, rate_of_climb=2.5,
                      reserve_factor=1.20, payload_kg=0.5, t_transition=40.0)
    aero = Aerodynamics(LD=8.0, CD0=0.025, AR=6.0, e=0.80, CL_max=1.4, V_stall=12.0, V_max=35.0)
    wf = WeightFraction(fs=0.25, fp=0.15, fa=0.08, fm=0.07)
    prop = PropulsiveSystemParameters(eta_prop=0.80, eta_motor=0.90, eta_esc=0.95, fom=0.70,
                                      Cd_blade=0.01, sigma_rotor=0.077, s_ratio=1.3,
                                      k_rpm=2762.786, exp_rpm=-0.932)
    ff = ForwardFlightParams(RoC_climb=2.5, h_ceiling=3000, RoC_ceiling=0.5)
    ws = WingStructureParams(tc_ratio=0.12, n_ult=3.75, k_material=0.70)
    return run_sizing_loop(
        m_payload_kg=0.5, mission=mission, aero=aero, batt=batt, wf=wf,
        prop_params=prop, ff_params=ff, ws_params=ws, env=Environment(),
        D_rotor_m=0.195, P_hotel_W=15.0,
    )


class TestOperatingPoint:
    def test_hand_checked_values(self, sizing_result, batt, elec):
        op = compute_operating_point(sizing_result, batt, elec)
        # V = 6 * 3.7 = 22.2 V
        assert op.pack_voltage_v == pytest.approx(22.2, rel=1e-6)
        assert op.pack_capacity_ah == pytest.approx(
            sizing_result.m_battery_kg * batt.specific_energy / 22.2, rel=1e-9)
        assert op.hover_current_a == pytest.approx(sizing_result.P_hover_W / 22.2, rel=1e-9)
        assert op.esc_rating_a == pytest.approx(op.hover_current_a * 1.6, rel=1e-9)

    def test_higher_cell_count_lowers_current(self, sizing_result, batt, elec):
        elec_12s = ElectricalParams(battery_series_cells=12, cell_nominal_v=3.7,
                                    esc_current_margin=1.6, bec1_budget_a=3.0, bec2_budget_a=6.0)
        op6 = compute_operating_point(sizing_result, batt, elec)
        op12 = compute_operating_point(sizing_result, batt, elec_12s)
        assert op12.hover_current_a < op6.hover_current_a
        assert op12.pack_voltage_v > op6.pack_voltage_v

    def test_gauge_and_connector_scale_with_current(self, sizing_result, batt, elec):
        # A much smaller pack (implausibly high voltage) should still
        # resolve to a valid, lighter gauge/connector selection
        elec_hv = ElectricalParams(battery_series_cells=24, cell_nominal_v=3.7,
                                   esc_current_margin=1.6, bec1_budget_a=3.0, bec2_budget_a=6.0)
        op_hv = compute_operating_point(sizing_result, batt, elec_hv)
        op_lv = compute_operating_point(sizing_result, batt, elec)
        assert op_hv.main_gauge_awg >= op_lv.main_gauge_awg  # smaller current -> thinner or equal gauge


class TestSvgRendering:
    def test_produces_valid_xml(self, sizing_result, batt, elec, rotor):
        op = compute_operating_point(sizing_result, batt, elec)
        svg = render_wiring_svg(op, elec, rotor, batt, servo_torque_gcm=461.0,
                                aileron_servo_torque_gcm=86.0,
                                package_version="0.0.2")
        root = ET.fromstring(svg)  # raises if malformed / undefined entities
        assert root.tag.endswith("}svg")

    def test_no_html_named_entities(self, sizing_result, batt, elec, rotor):
        # Named HTML entities (&middot; &mdash; ...) are invalid in strict
        # XML/SVG and previously broke the standalone file -- guard the regression.
        op = compute_operating_point(sizing_result, batt, elec)
        svg = render_wiring_svg(op, elec, rotor, batt, servo_torque_gcm=461.0,
                                aileron_servo_torque_gcm=86.0,
                                package_version="0.0.2")
        for entity in ("&middot;", "&mdash;", "&hellip;", "&ndash;"):
            assert entity not in svg

    def test_key_values_appear_in_output(self, sizing_result, batt, elec, rotor):
        op = compute_operating_point(sizing_result, batt, elec)
        svg = render_wiring_svg(op, elec, rotor, batt, servo_torque_gcm=461.0,
                                aileron_servo_torque_gcm=86.0,
                                package_version="0.0.2")
        assert op.main_connector in svg
        assert "195 mm" in svg
        assert "6S" in svg


class TestWriteOutputs:
    def test_writes_svg_and_yaml(self, tmp_path, sizing_result, batt, elec, rotor):
        op = compute_operating_point(sizing_result, batt, elec)
        svg = render_wiring_svg(op, elec, rotor, batt, servo_torque_gcm=461.0,
                                aileron_servo_torque_gcm=86.0,
                                package_version="0.0.2")
        paths = write_wiring_diagram(svg, op, elec, tmp_path)

        svg_path = tmp_path / "wiring_diagram.svg"
        yaml_path = tmp_path / "electrical.yaml"
        assert svg_path.exists() and svg_path.read_text(encoding="utf-8") == svg
        assert yaml_path.exists()
        assert paths == {"svg": str(svg_path), "yaml": str(yaml_path)}

        import yaml as _yaml
        data = _yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        assert data["battery_series_cells"] == 6
        assert data["main_connector"] == op.main_connector
