"""Round-trip tests: every from_yaml loader against the real config/ files.

These catch drift between the YAML schemas in config/ and the loaders in
src/conceptual_design/models.py (key renames, typos, missing fields).
"""

import pytest

from conceptual_design.forward_flight_power import ForwardFlightParams
from conceptual_design.models import (
    Aerodynamics,
    Avionics,
    Battery,
    Mission,
    PropulsiveSystemParameters,
    RotorParams,
    WeightFraction,
)
from conceptual_design.wing_sizing import WingStructureParams


def test_mission(config_dir):
    m = Mission.from_yaml(config_dir / "mission.yaml")
    assert m.V_cruise == 20.0
    # COTS 195 mm EDF design review: mission in the 15-20 min band
    total_min = (m.t_hover + m.t_transition + m.t_cruise) / 60.0
    assert 15.0 <= total_min <= 20.0
    assert m.t_transition > 0.0
    assert m.reserve_factor > 1.0
    assert m.payload_kg == pytest.approx(0.5)


def test_battery(config_dir):
    b = Battery.from_yaml(config_dir / "battery.yaml")
    assert 100.0 <= b.specific_energy <= 300.0
    assert 0.0 < b.usable_fraction <= 1.0
    assert 0.9 < b.eta_bat <= 1.0
    assert b.c_rate_max >= 10.0


def test_avionics(config_dir):
    av = Avionics.from_yaml(config_dir / "avionics.yaml")
    assert 0.0 < av.P_hotel_W < 100.0


def test_aerodynamics(config_dir):
    a = Aerodynamics.from_yaml(config_dir / "aerodynamics.yaml")
    assert a.LD > 0
    assert a.V_stall < a.V_max
    assert a.k > 0  # induced drag factor derivable


def test_weight_fractions(config_dir):
    wf = WeightFraction.from_yaml(config_dir / "initial_weight_fraction_estimation.yaml")
    f_fixed = wf.fs + wf.fp + wf.fa + wf.fm
    # Must leave room for battery + payload
    assert 0.0 < f_fixed < 1.0


def test_propulsive_parameters(config_dir):
    p = PropulsiveSystemParameters.from_yaml(config_dir / "propulsive_system_parameters.yaml")
    assert 0.0 < p.eta_total < 1.0
    assert p.eta_total == pytest.approx(p.eta_prop * p.eta_motor * p.eta_esc)
    assert 0.5 < p.fom <= 1.0


def test_rotor(config_dir):
    r = RotorParams.from_yaml(config_dir / "rotor.yaml")
    # COTS EDF class decision: 195 mm
    assert r.D_rotor_m == pytest.approx(0.195)


def test_forward_flight_params(config_dir):
    ff = ForwardFlightParams.from_yaml(config_dir / "forward_flight_params.yaml")
    assert ff.WS_min < ff.WS_max
    assert ff.WS_n > 1
    assert ff.RoC_ceiling < ff.RoC_climb


def test_wing_structure_params(config_dir):
    ws = WingStructureParams.from_yaml(config_dir / "wing_structure_params.yaml")
    assert ws.method in ("raymer", "nicolai")
    assert 0.0 < ws.tc_ratio < 0.3
    assert ws.n_ult >= 1.0
