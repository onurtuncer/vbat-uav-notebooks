# CLAUDE.md

Conceptual design study for a small electric tail-sitter VTOL UAV
(V-BAT-like, single EDF, jet vanes). Python physics package + Jupyter
notebooks + CadQuery CAD + OpenFOAM CFD cases, all wired into CI.

## Architecture: the design pipeline

Everything flows one way, from configuration to CFD:

```
config/*.yaml  ->  src/conceptual_design/  ->  notebooks/  ->  out/  ->  cfd/
(inputs)           (ALL physics)               (orchestrate)   (handoffs)  (consumers)
```

Notebook execution order matters — each writes YAML handoffs to `out/`
that the next one reads:

1. `vbat_conceptual_design` — mission sizing, mass closure (MTOW, wing, power)
2. `wing_design` — airfoil selection → `out/airfoil.yaml`
3. `control_vane_design` — jet vanes → `out/control_vanes.yaml`
4. `fuselage_design` — layout, CG, drag → `out/fuselage.yaml`
5. `vehicle_solid_model` — CadQuery CAD → `out/cad/` (STEP/STL, per-part + fused + prop rotor)
6. `mass_properties` — inertia tensor, BOM → `out/mass_properties.yaml`, `out/bom.csv`

NB2–NB6 re-run `run_sizing_loop` from `config/` to reconstruct the same
design point — if you change the sizing API, update **all six** call sites.

## Hard rules

- **No magic numbers in Python.** Every physical parameter lives in
  `config/*.yaml` with a commented rationale. Numerical tolerances and
  documented physical guard limits (as named module constants) are the
  only exceptions.
- **All physics lives in `src/`; notebooks only set parameters, call
  modules, and interpret results.**
- **Never run full/converged CFD in CI.** CI validates case *setup*
  only: the coarse-mesh, 60-iteration smoke run in the design pipeline
  is the ceiling. Production CFD runs on the user's own hardware.
- **Axis convention (body FRD):** x forward (out the nose), y right,
  z DOWN. Origin at the nose tip; stations from the nose map to
  `x_body = -station`. Keep this consistent everywhere (CAD, CFD, docs).
- **Units:** YAML handoffs and module APIs are SI (metres); CAD solids
  are built in **millimetres** (STEP/STL convention). `Allrun.mesh`
  scales mm→m — the geometry tests guard this.

## Design decisions (2026-07 review)

- **COTS 195 mm EDF** (Schübeler DS-215 class) — no custom fan
  development. Disk loading is therefore **derived** (`DL = MTOW·g/A`),
  never configured.
- **15–20 min mission** (120 s hover + 40 s transitions + 900 s cruise);
  hover is expensive at this disk loading, so the mission is
  deliberately short-hover.
- Design point ≈ 2.5 kg MTOW, ~770 W hover electrical, ~9C peak.
- The external-aero STL (`vbat_fused.stl`) deliberately excludes the
  fan blades: vehicle CFD uses an actuator disk. The parametric rotor
  (`out/cad/step/prop_rotor.step`) is for visualisation and
  rotating-zone propulsion CFD.

## CI (GitHub Actions)

- `ci.yml` — ruff, pytest (3.10/3.12/3.14), ShellCheck on `cfd/**/Allrun*`.
- `design-pipeline.yml` — executes all six notebooks on every PR, runs
  design-regression + geometry tests, uploads `out/` artifacts; on main
  additionally: coarse OpenFOAM smoke run and GitHub Pages deploy
  (rendered notebooks, 3D viewer with exploded view, BOM page).
- `release.yml` — pushing a `v*` tag re-runs the pipeline and attaches
  the frozen design snapshot to a GitHub Release. Version comes from
  git tags via setuptools-scm; never edit a version number by hand.

## Tests and regression pins

`tests/test_design_outputs.py` pins the converged design point
(fuselage dims, CG, MAC, vane torque…) and cross-checks the values
hardcoded in `cfd/vehicle/Allrun.case` (`lRef`, `Aref`, `CofR`).
**An intentional design change must update these pins — and
Allrun.case — in the same commit.** That diff documents the new design
point; a failing pin you didn't expect means the design drifted.

`tests/test_geometry.py` validates the exported STL (main body
watertight — one stray degenerate triangle is a known CadQuery
tessellation artifact — mm units, span consistent with the design
YAMLs). A span failure usually means `out/cad/` is stale, not wrong.

## Local environment gotchas

- The local venv is Python 3.14: **CadQuery does not install there**,
  so NB5/NB6 (and thus `out/cad/`, `out/bom.csv`, `out/mass_properties.yaml`)
  can only be regenerated in CI (Python 3.12) or a separate 3.12 env.
  After a design change, the committed copies of those outputs are
  stale until regenerated — CI validates against fresh ones.
- Run tests with `pip install -e ".[dev]"` then `pytest`; lint with
  `ruff check src tests scripts printprep cfd`.
- Notebook figures save to `notebooks/figures/` (`FIG_DIR`), design
  outputs to `out/`.
- OpenFOAM scripts assume `OpenFOAM.com v2306+`, run on Linux
  (`.gitattributes` forces LF under `cfd/`). `Allrun.case` needs
  `python3` on PATH.
