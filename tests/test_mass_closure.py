"""Tests for mass_closure.py -- battery sizing and the iterative MTOW loop."""

import pytest

from conceptual_design.forward_flight_power import ForwardFlightParams
from conceptual_design.mass_closure import (
    battery_mass_from_energy,
    mtow_from_components,
    run_sizing_loop,
)
from conceptual_design.models import (
    Aerodynamics,
    Battery,
    Environment,
    Mission,
    PropulsiveSystemParameters,
    WeightFraction,
)
from conceptual_design.wing_sizing import WingStructureParams


class TestBatteryMass:
    def test_hand_checked_value(self):
        # m = E / (e_spec * eta * f_usable) = 100 / (140*0.97*0.85) = 0.8663 kg
        batt = Battery(specific_energy=140.0, usable_fraction=0.85)
        assert battery_mass_from_energy(100.0, batt, eta_bat=0.97) == pytest.approx(0.86633, rel=1e-4)

    def test_zero_capacity_raises(self):
        batt = Battery(specific_energy=0.0, usable_fraction=0.85)
        with pytest.raises(ValueError):
            battery_mass_from_energy(100.0, batt)


class TestMtowFromComponents:
    def test_hand_checked_value(self):
        # (0.5 + 1.0) / (1 - 0.7) = 5.0 kg
        wf = WeightFraction(fs=0.30, fp=0.20, fa=0.10, fm=0.10)
        assert mtow_from_components(0.5, 1.0, wf) == pytest.approx(5.0, rel=1e-12)

    def test_fractions_summing_to_one_raise(self):
        wf = WeightFraction(fs=0.5, fp=0.3, fa=0.1, fm=0.1)
        with pytest.raises(ValueError):
            mtow_from_components(0.5, 1.0, wf)


@pytest.fixture
def sizing_inputs():
    """Representative V-BAT-scale inputs (mirrors the repo config values)."""
    return dict(
        m_payload_kg=0.5,
        mission=Mission(t_hover=120, t_cruise=1680, V_cruise=20.0,
                        rate_of_climb=2.5, reserve_factor=1.20),
        aero=Aerodynamics(LD=8.0, CD0=0.025, AR=6.0, e=0.80,
                          CL_max=1.4, V_stall=12.0, V_max=35.0),
        batt=Battery(specific_energy=140.0, usable_fraction=0.85),
        wf=WeightFraction(fs=0.25, fp=0.15, fa=0.08, fm=0.07),
        prop_params=PropulsiveSystemParameters(
            eta_prop=0.80, eta_motor=0.90, eta_esc=0.95,
            fom=0.70, Cd_blade=0.01, sigma_rotor=0.077,
            s_ratio=1.3, k_rpm=2762.786, exp_rpm=-0.932),
        ff_params=ForwardFlightParams(RoC_climb=2.5, h_ceiling=3000, RoC_ceiling=0.5),
        ws_params=WingStructureParams(tc_ratio=0.12, n_ult=3.75, k_material=0.70),
        env=Environment(),
        D_rotor_m=0.28,
        disk_loading=150.0,
    )


class TestSizingLoop:
    def test_converges(self, sizing_inputs):
        result = run_sizing_loop(**sizing_inputs)
        assert result.converged
        assert result.n_iterations < 100

    def test_mass_breakdown_sums_to_mtow(self, sizing_inputs):
        result = run_sizing_loop(**sizing_inputs)
        total = (result.m_battery_kg + result.m_payload_kg + result.m_structure_kg
                 + result.m_propulsion_kg + result.m_avionics_kg + result.m_misc_kg)
        assert total == pytest.approx(result.m_total_kg, rel=1e-6)

    def test_converged_battery_mass_is_fixed_point(self, sizing_inputs):
        result = run_sizing_loop(**sizing_inputs)
        # Re-deriving battery mass from the converged energy must return
        # (almost exactly) the converged battery mass.
        m_batt_recomputed = battery_mass_from_energy(
            result.E_total_Wh, sizing_inputs["batt"], eta_bat=0.97
        )
        assert m_batt_recomputed == pytest.approx(result.m_battery_kg, abs=1e-4)

    def test_result_is_physically_plausible(self, sizing_inputs):
        result = run_sizing_loop(**sizing_inputs)
        assert 1.0 < result.m_total_kg < 20.0
        assert 0.0 < result.batt_fraction < 0.6
        assert result.P_hover_W > result.P_cruise_W  # hover dominates for VTOL
        assert result.E_total_Wh > result.E_hover_Wh + result.E_cruise_Wh  # reserve applied

    def test_longer_cruise_needs_heavier_aircraft(self, sizing_inputs):
        short = run_sizing_loop(**sizing_inputs)

        long_inputs = dict(sizing_inputs)
        long_inputs["mission"] = Mission(t_hover=120, t_cruise=2400, V_cruise=20.0,
                                         rate_of_climb=2.5, reserve_factor=1.20)
        long = run_sizing_loop(**long_inputs)

        assert long.m_battery_kg > short.m_battery_kg
        assert long.m_total_kg > short.m_total_kg
