---
name: run-vbat-uav-notebooks
description: Run, build, and test the V-BAT tail-sitter design pipeline. Use when asked to run the sizing loop, execute the design notebooks, regenerate out/ artifacts, or run the design-regression tests.
---

This repo is a one-way design pipeline (`config/*.yaml` → `src/` physics →
`notebooks/` → `out/` handoffs), not a server or GUI. "Running the app" means
running that pipeline. Drive it via
`.claude/skills/run-vbat-uav-notebooks/driver.py` with the repo venv's Python.

All paths are relative to the repo root. All commands below were verified on
Windows (Git Bash) with the committed `.venv` (Python 3.14).

## Prerequisites

The repo venv at `.venv/` already has everything except CadQuery (see
Gotchas). If starting from a bare clone:

```bash
./.venv/Scripts/python.exe -m pip install -e ".[dev,notebooks]"
```

(On Linux/CI the interpreter is `.venv/bin/python`; CI additionally installs
`.[cad]` on Python 3.12.)

## Run (agent path)

The driver has three subcommands. `smoke` is the fast check (~1 s, no
notebooks) — it runs the mass-closure sizing loop directly from `config/`:

```bash
./.venv/Scripts/python.exe .claude/skills/run-vbat-uav-notebooks/driver.py smoke
```

Expected output (current design point, re-baselined per ADR-0010):

```
converged   : True (24 iterations)
MTOW        : 2.376 kg
P_hover     : 710 W
P_cruise    : 100 W
C_rate_peak : 8.6
wing        : S=0.1892 m^2  b=1.066 m
```

`pipeline` executes the eleven notebooks in dependency order (same order and
mechanism as CI: `jupyter nbconvert --execute`, `MPLBACKEND=Agg`). Executed
copies land in `executed/`, design outputs in `out/`, figures in
`notebooks/figures/`. Takes ~4 min locally; `vehicle_solid_model` is
auto-skipped when CadQuery can't import (always, on the local 3.14 venv):

```bash
./.venv/Scripts/python.exe .claude/skills/run-vbat-uav-notebooks/driver.py pipeline
```

Run a single notebook (repeatable, dependency order is on you):

```bash
./.venv/Scripts/python.exe .claude/skills/run-vbat-uav-notebooks/driver.py pipeline --nb wiring_diagram
```

| subcommand | what it does |
|---|---|
| `smoke` | sizing loop from `config/`, prints design point, exits 1 if not converged |
| `pipeline [--nb NAME] [--include-cad]` | nbconvert-execute notebooks in CI order into `executed/` |
| `test [--all]` | design-regression + geometry pytest (what CI gates on); `--all` = full suite |

## Direct invocation

Most PRs touch one physics module. Import and call it without any notebook —
the package is installed editable, configs load via `from_yaml`:

```bash
./.venv/Scripts/python.exe -c "
from conceptual_design.models import RotorParams
print(RotorParams.from_yaml('config/rotor.yaml'))"
```

There is also a hardcoded-parameter demo in `mass_closure.py` (prints a
2.502 kg closure from built-in params, NOT the config design point; the
runpy `found in sys.modules` RuntimeWarning is harmless):

```bash
./.venv/Scripts/python.exe -m conceptual_design.mass_closure
```

## Test

```bash
./.venv/Scripts/python.exe .claude/skills/run-vbat-uav-notebooks/driver.py test
```

Runs `tests/test_design_outputs.py` + `tests/test_geometry.py` — the same
gate as `design-pipeline.yml`. Expected locally: everything passes **except**
`test_geometry.py::TestFusedStl::test_span_matches_design_outputs` whenever
`out/cad/` lags a design change — the STL can only be regenerated in
CI/3.12 (see Gotchas), so after any change that moves the wing span this
one failure is stale CAD, not a bug. A `test_design_outputs` pin failure
after an intentional design change means the pins (and
`cfd/vehicle/Allrun.case`) must be updated in the same commit.

Full suite + lint:

```bash
./.venv/Scripts/python.exe .claude/skills/run-vbat-uav-notebooks/driver.py test --all
./.venv/Scripts/python.exe -m ruff check src tests scripts printprep cfd
```

(Verified 2026-07: `--all` gives 87 passed + the 1 stale-CAD span failure
above; ruff clean.)

## Run (human path)

Open any notebook under `notebooks/` in Jupyter (`.venv` has jupyterlab) and
run cells top-to-bottom, respecting the eleven-notebook order in `CLAUDE.md`.
Agents should use the driver instead.

## Gotchas

- **CadQuery pip-installs on Python 3.14 but does not import** —
  `pip list` shows `cadquery 2.3.0`, yet `import cadquery` fails with
  `ModuleNotFoundError: No module named 'OCP'` (no OCP wheel for 3.14).
  Don't trust `pip list`; the driver probes the actual import. NB8
  (`vehicle_solid_model`) and fresh `out/cad/` therefore only regenerate in
  CI or a separate 3.12 env. `mass_properties` still runs locally against
  the committed `out/cad/`.
- **Trust `config/` + the test pins over prose design-point numbers.**
  CLAUDE.md may quote a design point from an earlier baseline (e.g.
  ≈3.06 kg vs the ADR-0010 re-baselined 2.376 kg in
  `config/initial_weight_fraction_estimation.yaml`). `smoke` prints
  whatever `config/` implies; `tests/test_design_outputs.py` is the
  authoritative pin set.
- **`executed/` is generated** by the pipeline at repo root (same as CI).
  Don't commit it.
- **PowerShell 5.1 chokes on heredocs and `&&`** — run the commands above
  from Git Bash, or use the driver (plain `argv`, no shell tricks).
- **zmq `Proactor event loop` RuntimeWarning** on every notebook under
  Windows — harmless nbconvert/tornado noise, not a failure.

## Troubleshooting

- **`ModuleNotFoundError: No module named 'OCP'`**: you tried to import
  cadquery on the 3.14 venv. Use CI or a 3.12 env for CAD regeneration.
- **`test_geometry` span failure**: `out/cad/` is stale (NB8 not re-run
  after a design change), usually not a real geometry bug — see CLAUDE.md.
