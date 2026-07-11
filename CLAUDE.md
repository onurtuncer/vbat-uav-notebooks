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
4. `aileron_design` — cruise-phase roll backup → `out/aileron.yaml`
5. `vibration_isolation` — FC/IMU + payload soft mounts → `out/vibration.yaml`
6. `fuselage_design` — layout, CG, drag → `out/fuselage.yaml`
7. `thermal_design` — ESC cold-plate + vented battery bay → `out/thermal.yaml`
8. `vehicle_solid_model` — CadQuery CAD → `out/cad/` (STEP/STL, per-part + fused + prop rotor)
9. `mass_properties` — inertia tensor, BOM → `out/mass_properties.yaml`, `out/bom.csv`
10. `wiring_diagram` — electrical block diagram → `out/wiring_diagram.svg`, `out/electrical.yaml`

NB2–NB10 re-run `run_sizing_loop` from `config/` to reconstruct the same
design point — if you change the sizing API, update **all ten** call sites.

`aileron_design` exists because jet-vane control authority is sized from
**hover** thrust and collapses in cruise (`q_jet` scales linearly with
thrust — cruise thrust is only ~L/D⁻¹ of hover thrust). Ailerons use wing
dynamic pressure instead, so they stay effective exactly where jet vanes
are weakest. Jet vanes remain primary for all three axes in hover/transition
and stay available as a roll backup in cruise. `ddot_min_deg_s2`
(`config/aerodynamics.yaml`) is the one requirement both notebooks check
against — keep it in sync if either notebook's authority margin changes.

`vibration_isolation` soft-mounts the FC/IMU and payload against the EDF
1/rev imbalance (~211 Hz forcing, derived from rotor RPM). It hands
`fuselage_design` the isolator hardware mass (carved from the
avionics/structural fractions) and the sway/rattle space added to the bay
stack. Because the forcing is far above any practical isolator corner
frequency, the sway cost is a few mm — small, but real on a
packaging-constrained fuselage.

`thermal_design` sizes the two hover heat paths as structure (ADR-0009):
an ESC cold-plate on the inflow-washed inner wall (forced convection) and
a vented battery bay. Heat loads are derived (`P_hover·(1−eta_esc)` and
`P_hover·(1/eta_bat−1)`), not configured. It does **not** change the mass
model — the cold-plate is absorbed into the ESC/propulsion "mounts"
allocation and its two CAD parts are assembly-only (excluded from the
fused STL). It reports margins honestly: at the current 3.06 kg / ~1 kW-
hover point the battery bay vents comfortably, but the ESC cold-plate is
**marginal** (its ~50 W load needs a plate heavier than the ESC allocation
with only a few °C margin) — a standing finding, not a hard failure.

`wiring_diagram` is generated, not hand-drawn: box positions/wiring
topology are a fixed layout in `electrical_diagram.py`, but every label
(pack voltage/capacity, operating current, wire gauge, connector,
servo torque) is computed from `config/electrical.yaml` +
`config/battery.yaml` + `config/rotor.yaml` + the sizing result. Only
`battery_series_cells` (cell count) is a free electrical variable — the
mass-closure loop is voltage-agnostic. Doesn't need CadQuery, so it
also runs in the local 3.14 venv.

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
- **Architectural design records** Keep ADR records for architectural design decisions in a folder named adr under project root folder.
- **Change Logs** When a version is tagged write a changelog.

## Design decisions (2026-07 review)

- **COTS 195 mm EDF** (Schübeler DS-215 class) — no custom fan
  development. Disk loading is therefore **derived** (`DL = MTOW·g/A`),
  never configured. `T_max_N` (`config/rotor.yaml`) guards the hover
  thrust requirement against the class capability — the sizing fails
  loudly rather than sizing past the COTS fan.
- **15–20 min mission** (120 s hover + 40 s transitions + 900 s cruise);
  hover is expensive at this disk loading, so the mission is
  deliberately short-hover.
- **Segmented-FDM construction is the baseline** (ADR-0008): 3D-printed
  airframe on an 8 mm CFRP wing spar. `construction_method`/
  `k_construction` in `config/initial_weight_fraction_estimation.yaml`
  scale the structural fraction INSIDE the mass closure — k is the most
  sensitive mass parameter in the project (its config comment carries
  the sweep table; MTOW nearly doubles between k=1.1 and 1.2). NB1 shows
  the monocoque-vs-FDM trade every run.
- **Semi-monocoque clamshell fuselage** (ADR-0010, external review):
  structural lower half + full-length hinged upper lid; a rectangular
  profile around the joint line works as the longerons, with crossbeam
  equipment mounts and half-rings tying the lower-centerline battery
  rail to the frame. `config/modularity.yaml` holds the clamshell/frame/
  skin geometry; structure mass is an explicit member model (skin +
  longerons + crossbeams + rings — no k_struct knock-up), evaluated for
  the configured method AND both skin options (FDM ~352 g now,
  2-ply CFRP ~242 g later) on every run. `t_shell_m` remains ONLY the
  packaging wall — decoupled from the structural skin.
- Design point ≈ 3.06 kg MTOW, ~1.03 kW hover electrical, ~9.2C peak
  (segmented-FDM, k=1.1; the monocoque equivalent is ≈ 2.5 kg / 770 W).
- The external-aero STL (`vbat_fused.stl`) deliberately excludes the
  fan blades: vehicle CFD uses an actuator disk. The parametric rotor
  (`out/cad/step/prop_rotor.step`) is for visualisation and
  rotating-zone propulsion CFD.

## CI (GitHub Actions)

- `ci.yml` — ruff, pytest (3.10/3.12/3.14), ShellCheck on `cfd/**/Allrun*`.
- `design-pipeline.yml` — executes all ten notebooks on every PR, runs
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
  so NB8 (`vehicle_solid_model`, and thus `out/cad/`) can only be
  regenerated in CI (Python 3.12) or a separate 3.12 env. NB9
  (`mass_properties`) has no CadQuery dependency of its own and *can*
  run locally, but its BOM/CAD cross-check still lags until NB8 has
  regenerated in CI. After a design change, the committed copies of
  CadQuery-dependent outputs are stale until regenerated — CI validates
  against fresh ones.
- Run tests with `pip install -e ".[dev]"` then `pytest`; lint with
  `ruff check src tests scripts printprep cfd`.
- Notebook figures save to `notebooks/figures/` (`FIG_DIR`), design
  outputs to `out/`.
- OpenFOAM scripts assume `OpenFOAM.com v2306+`, run on Linux
  (`.gitattributes` forces LF under `cfd/`). `Allrun.case` needs
  `python3` on PATH.
- Generated SVGs (`electrical_diagram.py`) must be strict XML: only
  numeric character references (`&#183;`, `&#8212;`), never named HTML
  entities (`&middot;`, `&mdash;`) — the latter aren't defined in bare
  XML and break parsers (GitHub's SVG viewer, `xml.etree`, Illustrator).
