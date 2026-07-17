# Agent driver for the V-BAT conceptual-design pipeline.
#
# The "app" here is a one-way design pipeline (config/ -> src/ ->
# notebooks/ -> out/), so driving it means: run the sizing physics
# directly (smoke), execute the notebooks in dependency order
# (pipeline), and regression-check the resulting design point (test).
#
# Runs with the repo venv's python from the repo root:
#   .venv/Scripts/python.exe .claude/skills/run-vbat-uav-notebooks/driver.py smoke
#
# Works on Windows (local 3.14 venv) and Linux (CI 3.12). On 3.14,
# cadquery cannot import (no OCP wheel), so `pipeline` auto-skips
# vehicle_solid_model unless --include-cad forces it.

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]

# CI dependency order (design-pipeline.yml); each notebook reads the
# out/*.yaml handoffs written by the ones before it.
NOTEBOOKS = [
    "vbat_conceptual_design",
    "wing_design",
    "control_vane_design",
    "aileron_design",
    "vibration_isolation",
    "fuselage_design",
    "thermal_design",
    "vehicle_solid_model",   # CadQuery -- skipped when OCP is missing
    "mass_properties",
    "wiring_diagram",
    "cots_selection",
    "aileron_design_cots",        # post-freeze as-selected re-solves (ADR-0012)
    "vibration_isolation_cots",
    "fuselage_design_cots",
    "design_summary",             # final rollup, reads out/ only
]

CAD_NOTEBOOK = "vehicle_solid_model"


def cadquery_available() -> bool:
    r = subprocess.run(
        [sys.executable, "-c", "import cadquery"],
        capture_output=True, cwd=REPO,
    )
    return r.returncode == 0


def cmd_smoke(_args) -> int:
    """Run the mass-closure sizing loop straight from config/ (no notebooks)."""
    sys.path.insert(0, str(REPO / "src"))
    os.chdir(REPO)
    from conceptual_design import (
        run_sizing_loop, Environment, Mission, Aerodynamics, Battery,
        WeightFraction, PropulsiveSystemParameters,
    )
    from conceptual_design.forward_flight_power import ForwardFlightParams
    from conceptual_design.wing_sizing import WingStructureParams
    from conceptual_design.models import RotorParams, Avionics

    result = run_sizing_loop(
        m_payload_kg=Mission.from_yaml("config/mission.yaml").payload_kg,
        mission=Mission.from_yaml("config/mission.yaml"),
        aero=Aerodynamics.from_yaml("config/aerodynamics.yaml"),
        batt=Battery.from_yaml("config/battery.yaml"),
        wf=WeightFraction.from_yaml("config/initial_weight_fraction_estimation.yaml"),
        prop_params=PropulsiveSystemParameters.from_yaml(
            "config/propulsive_system_parameters.yaml"),
        ff_params=ForwardFlightParams.from_yaml("config/forward_flight_params.yaml"),
        ws_params=WingStructureParams.from_yaml("config/wing_structure_params.yaml"),
        env=Environment(),
        D_rotor_m=RotorParams.from_yaml("config/rotor.yaml").D_rotor_m,
        P_hotel_W=Avionics.from_yaml("config/avionics.yaml").P_hotel_W,
    )
    print(f"converged   : {result.converged} ({result.n_iterations} iterations)")
    print(f"MTOW        : {result.m_total_kg:.3f} kg")
    print(f"P_hover     : {result.P_hover_W:.0f} W")
    print(f"P_cruise    : {result.P_cruise_W:.0f} W")
    print(f"C_rate_peak : {result.C_rate_peak:.1f}")
    print(f"wing        : S={result.wing.S_wing:.4f} m^2  b={result.wing.b_wing:.3f} m")
    if not result.converged:
        print("SMOKE FAIL: sizing loop did not converge", file=sys.stderr)
        return 1
    return 0


def cmd_pipeline(args) -> int:
    """Execute the design notebooks in dependency order via nbconvert."""
    names = args.nb or NOTEBOOKS
    skip_cad = not args.include_cad and not cadquery_available()
    env = {**os.environ, "MPLBACKEND": "Agg"}
    outdir = REPO / "executed"

    for nb in names:
        if nb == CAD_NOTEBOOK and skip_cad:
            print(f"-- SKIP {nb}: cadquery not importable in this env "
                  "(no OCP wheel for this Python; run in CI / a 3.12 env)")
            continue
        path = REPO / "notebooks" / f"{nb}.ipynb"
        if not path.exists():
            print(f"FAIL: no such notebook {path}", file=sys.stderr)
            return 1
        print(f"-- RUN  {nb}", flush=True)
        t0 = time.time()
        r = subprocess.run(
            [sys.executable, "-m", "jupyter", "nbconvert",
             "--to", "notebook", "--execute",
             "--ExecutePreprocessor.timeout=1200",
             "--output-dir", str(outdir), str(path)],
            cwd=REPO, env=env,
        )
        print(f"        {nb}: {'ok' if r.returncode == 0 else 'FAILED'} "
              f"({time.time() - t0:.0f}s)")
        if r.returncode != 0:
            return r.returncode
    print(f"executed notebooks -> {outdir}; design outputs -> {REPO / 'out'}")
    return 0


def cmd_test(args) -> int:
    """Design-regression + geometry checks (what CI gates on), or --all."""
    targets = ["tests"] if args.all else [
        "tests/test_design_outputs.py", "tests/test_geometry.py"]
    return subprocess.run(
        [sys.executable, "-m", "pytest", *targets, "-v"], cwd=REPO,
    ).returncode


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("smoke", help="sizing loop from config/, no notebooks (~1 s)")
    pl = sub.add_parser("pipeline", help="execute notebooks in dependency order")
    pl.add_argument("--nb", action="append",
                    help="run only this notebook (repeatable, no .ipynb)")
    pl.add_argument("--include-cad", action="store_true",
                    help="run vehicle_solid_model even if cadquery import check fails")
    t = sub.add_parser("test", help="design-regression + geometry pytest")
    t.add_argument("--all", action="store_true", help="full test suite")
    args = p.parse_args()
    return {"smoke": cmd_smoke, "pipeline": cmd_pipeline, "test": cmd_test}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
