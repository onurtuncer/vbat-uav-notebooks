# conceptual_design/design_point.py
"""Load every sizing config and re-run the mass-closure design point.

NB2 onwards re-run ``run_sizing_loop`` from ``config/`` to reconstruct the
same converged design point.  This module is the single call site for that
pattern (it used to be an identical ~35-line block in every notebook): if
the sizing API changes, only ``DesignInputs.solve`` needs updating.
"""

from dataclasses import dataclass
from pathlib import Path

import yaml

from .forward_flight_power import ForwardFlightParams
from .mass_closure import SizingResult, run_sizing_loop
from .models import (
    Aerodynamics,
    Avionics,
    Battery,
    Environment,
    Mission,
    PropulsiveSystemParameters,
    RotorParams,
    WeightFraction,
)
from .wing_sizing import WingStructureParams


@dataclass(frozen=True)
class DesignInputs:
    """All sizing-loop inputs, loaded from ``config/*.yaml``."""

    env:      Environment
    mission:  Mission
    aero:     Aerodynamics
    batt:     Battery
    wf:       WeightFraction
    prop:     PropulsiveSystemParameters
    ff:       ForwardFlightParams
    ws:       WingStructureParams
    rotor:    RotorParams
    avionics: Avionics

    @classmethod
    def from_config(cls, config_dir: Path | str) -> "DesignInputs":
        config_dir = Path(config_dir)
        return cls(
            env      = Environment(),
            mission  = Mission.from_yaml(config_dir / "mission.yaml"),
            aero     = Aerodynamics.from_yaml(config_dir / "aerodynamics.yaml"),
            batt     = Battery.from_yaml(config_dir / "battery.yaml"),
            wf       = WeightFraction.from_yaml(
                config_dir / "initial_weight_fraction_estimation.yaml"),
            prop     = PropulsiveSystemParameters.from_yaml(
                config_dir / "propulsive_system_parameters.yaml"),
            ff       = ForwardFlightParams.from_yaml(
                config_dir / "forward_flight_params.yaml"),
            ws       = WingStructureParams.from_yaml(
                config_dir / "wing_structure_params.yaml"),
            rotor    = RotorParams.from_yaml(config_dir / "rotor.yaml"),
            avionics = Avionics.from_yaml(config_dir / "avionics.yaml"),
        )

    def sizing_kwargs(self) -> dict:
        """The ``run_sizing_loop`` keyword set for this input bundle.

        Exposed for parameter sweeps that substitute one input at a time.
        """
        return dict(
            m_payload_kg = self.mission.payload_kg,
            mission      = self.mission,
            aero         = self.aero,
            batt         = self.batt,
            wf           = self.wf,
            prop_params  = self.prop,
            ff_params    = self.ff,
            ws_params    = self.ws,
            env          = self.env,
            D_rotor_m    = self.rotor.D_rotor_m,
            P_hotel_W    = self.avionics.P_hotel_W,
        )

    def solve(self, **overrides) -> SizingResult:
        """Run the mass-closure loop; ``overrides`` replace any kwarg."""
        kwargs = self.sizing_kwargs()
        kwargs.update(overrides)
        return run_sizing_loop(**kwargs)


def solve_design_point(config_dir: Path | str) -> tuple[DesignInputs, SizingResult]:
    """Load ``config/`` and re-run the converged design point."""
    inputs = DesignInputs.from_config(config_dir)
    return inputs, inputs.solve()


def load_handoff(out_dir: Path | str, name: str) -> dict:
    """Read one ``out/<name>.yaml`` handoff written by an upstream notebook."""
    path = Path(out_dir) / f"{name}.yaml"
    return yaml.safe_load(path.read_text(encoding="utf-8"))
