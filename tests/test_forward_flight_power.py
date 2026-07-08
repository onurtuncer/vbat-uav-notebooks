"""Tests for forward_flight_power.py -- T/W constraint curves and Size Matching Diagram."""

import math

import numpy as np
import pytest

from conceptual_design.forward_flight_power import (
    ForwardFlightParams,
    compute_size_matching_diagram,
    cruise_power_W,
    tw_cruise,
    ws_stall,
)
from conceptual_design.models import Aerodynamics, Environment, Mission


@pytest.fixture
def aero() -> Aerodynamics:
    return Aerodynamics(LD=8.0, CD0=0.025, AR=6.0, e=0.80, CL_max=1.4, V_stall=12.0, V_max=35.0)


@pytest.fixture
def mission() -> Mission:
    return Mission(t_hover=120, t_cruise=1680, V_cruise=20.0, rate_of_climb=2.5, reserve_factor=1.2)


@pytest.fixture
def ff() -> ForwardFlightParams:
    return ForwardFlightParams(RoC_climb=2.5, h_ceiling=3000, RoC_ceiling=0.5)


class TestStallBoundary:
    def test_hand_checked_value(self, aero):
        # (W/S)_stall = 0.5 * 1.225 * 12^2 * 1.4 = 123.48 N/m^2
        assert ws_stall(aero, Environment()) == pytest.approx(123.48, rel=1e-6)


class TestCruiseCurve:
    def test_minimum_matches_analytic_optimum(self, aero, mission):
        # T/W_cruise = q*CD0/WS + k*WS/q has analytic minimum 2*sqrt(k*CD0)
        # at WS* = q*sqrt(CD0/k).
        env = Environment()
        WS = np.linspace(30.0, 500.0, 5000)
        tw = tw_cruise(WS, aero, mission, env)

        tw_min_analytic = 2.0 * math.sqrt(aero.k * aero.CD0)
        assert tw.min() == pytest.approx(tw_min_analytic, rel=1e-4)

        q = 0.5 * env.rho * mission.V_cruise**2
        WS_opt_analytic = q * math.sqrt(aero.CD0 / aero.k)
        assert WS[np.argmin(tw)] == pytest.approx(WS_opt_analytic, rel=1e-2)


class TestSizeMatchingDiagram:
    def test_design_point_is_feasible_and_on_envelope(self, aero, mission, ff):
        res = compute_size_matching_diagram(aero, mission, ff)

        assert res.WS_design <= res.WS_stall
        assert res.TW_design > 0.0

        # Envelope must dominate every individual constraint curve
        for curve in (res.tw_takeoff, res.tw_climb, res.tw_cruise, res.tw_ceiling, res.tw_vmax):
            assert np.all(res.tw_envelope >= curve - 1e-12)

        # The design T/W is the minimum of the envelope over the feasible region
        feasible = res.WS_array <= res.WS_stall
        assert res.TW_design == pytest.approx(res.tw_envelope[feasible].min())

    def test_power_to_weight_conversion(self, aero, mission, ff):
        eta = 0.684
        res = compute_size_matching_diagram(aero, mission, ff, eta_propulsive=eta)
        assert res.P_W_design == pytest.approx(res.TW_design * mission.V_cruise / eta)

    def test_infeasible_stall_limit_raises(self, aero, mission, ff):
        # Stall limit below the sweep range -> no feasible wing loading
        slow = Aerodynamics(LD=8.0, CD0=0.025, AR=6.0, e=0.80, CL_max=1.4, V_stall=1.0, V_max=35.0)
        with pytest.raises(ValueError):
            compute_size_matching_diagram(slow, mission, ff)


class TestCruisePower:
    def test_hand_checked_values(self, aero, mission):
        # W = 5 kg * 9.80665 = 49.033 N
        # P_aero = W*V/(L/D) = 49.033*20/8 = 122.58 W ; P_elec = P_aero/0.75
        res = cruise_power_W(5.0, aero, mission, eta_propulsive=0.75)
        assert res["P_aero_W"] == pytest.approx(122.583, rel=1e-4)
        assert res["P_elec_W"] == pytest.approx(163.444, rel=1e-4)
        assert res["drag_N"] == pytest.approx(6.1292, rel=1e-4)
        assert res["thrust_N"] == pytest.approx(res["drag_N"])
