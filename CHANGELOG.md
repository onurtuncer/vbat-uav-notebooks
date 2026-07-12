# Changelog

All notable changes to this conceptual design study are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Versions are derived from git tags via `setuptools-scm`; never edit a version
by hand. Per project policy, a changelog entry is written whenever a version
is tagged.

## [Unreleased]

### Added

- **README hero render** (`assets/vbat_render.png`): shaded 3D view of the
  vehicle standing tail-down, rasterised straight from the exported part
  STLs by `scripts/render_readme_cad.py` (self-contained numpy/Pillow
  z-buffer renderer — no OpenGL needed); linked to the interactive Pages
  viewer from the top of the README.

## [0.4.0] — 2026-07-11

Fourth design snapshot: the semi-monocoque clamshell architecture from
Çağlar Uçler's external design review, and the budget re-baseline it
justified — **MTOW 3.06 → 2.376 kg**, lighter than the original 2.5 kg
monocoque baseline. Also lays the PX4 flight-validation groundwork
(ADR-0011): fly on a COTS Pixhawk before designing the custom FC card.

### Added

- **Semi-monocoque clamshell fuselage** (ADR-0010, from Çağlar Uçler's
  external design review): the fuselage becomes a longitudinal clamshell —
  structural lower half + full-length hinged upper lid — with a rectangular
  profile around the joint line working as the longerons, crossbeam
  equipment mounts, and 2 half-rings tying the lower-centerline battery
  rail to the frame. An explicit member model (skin + longerons +
  crossbeams + rings, all computed from geometry) replaces the monocoque
  area-density estimate (`k_struct` removed); the per-method skin table
  keeps the print-first (FDM ~321 g) vs 2-ply-carbon trade visible on
  every run.
- **PX4 flight-validation plan before the custom flight controller**
  (ADR-0011): new `px4/` pipeline consumer (downstream of `out/`, like
  `cfd/`) holding a draft custom tailsitter airframe/actuator
  configuration — single EDF + four jet vanes via dynamic control
  allocation, ailerons as the cruise roll effector — plus a Gazebo SITL
  model built from the `out/` handoffs (inertia, vane geometry, motor
  constants). Vane jet-wash aerodynamics in SITL are explicitly
  placeholder pending a custom gz plugin; param names await validation
  against a pinned PX4 release. The custom FC card will be specified
  from flight logs, not estimates.
- CAD: `fuselage_lid`/`fuselage_lower` clamshell split plus assembly-only
  frame display parts (longerons, crossbeams, half-rings, battery rail).

### Changed

- **Design point re-baselined to the semi-monocoque budget** (ADR-0010
  follow-through): `structual_weight_fraction` fitted 0.25 → 0.22 by
  bisection so the structural budget lands at ~110% of the explicit FDM
  member-model estimate after full re-convergence. **MTOW 3.06 →
  2.376 kg** (hover 710 W / 30.3 N, 8.6C peak) — lighter than the
  original 2.5 kg monocoque baseline: the semi-monocoque architecture
  more than pays back the FDM print penalty. All regression pins,
  `cfd/vehicle/Allrun.case` references, and committed `out/` artifacts
  updated. The feasible floor (~0.19) is set by the ADR-0005 guards
  (fixed servo/isolator hardware vs. the shrinking avionics budget).
- ADR-0008's transverse splits (removable nose module, 90° battery hatch)
  are superseded by the clamshell; the two-piece wing + spar and the
  construction-method mechanism remain.
- The ESC cold-plate thermal finding survives at the lighter point
  (~35 W load, plate still over the ESC allocation) — reduced, not
  resolved.

## [0.3.0] — 2026-07-10

Third design snapshot. Adds the first thermal model — the two hover
heat-rejection paths (ESC cold-plate, vented battery bay) sized as
structure — completing the mechanical-design layer (vibration, modularity,
construction, thermal) on top of the 3.06 kg segmented-FDM airframe. The
ESC cold-plate comes out marginal at this power level and is reported as a
standing finding, not tuned away.

### Added

- **Thermal paths as structure** (new pipeline stage `thermal_design`,
  NB7; ADR-0009): first-order sizing of the two hover heat-rejection paths
  as structure — an ESC cold-plate on the inflow-washed inner wall (flat-
  plate forced convection) and a vented battery bay (through-flow). Heat
  loads are derived from the propulsion/battery efficiencies and hover
  power; `config/thermal.yaml` holds only the limits and cooling-airflow
  fractions. Writes `out/thermal.yaml`; two assembly-only CAD parts
  (`esc_cold_plate`, `battery_vents`) excluded from the fused external-aero
  STL. **No mass re-baseline** — the cold-plate is absorbed into the ESC/
  propulsion allocation and no geometry pins move.
- **Finding:** the battery bay vents comfortably (~40x area headroom,
  ~20 C margin), but the ESC cold-plate is **marginal** at the 3.06 kg /
  ~1 kW-hover point — its ~50 W load needs a plate heavier than the ESC
  allocation with only a few C margin. Surfaced honestly (`ok: false` in
  `out/thermal.yaml`, printed in NB7) rather than tuned to a false pass.

### Changed

- Pipeline is now ten notebooks (was nine); `thermal_design` inserts as
  NB7, shifting `vehicle_solid_model` →NB8, `mass_properties` →NB9,
  `wiring_diagram` →NB10.

## [0.2.1] — 2026-07-10

### Fixed

- **Stale CAD part files no longer accumulate.** `export_vehicle` only
  wrote the current per-part STEP/STL and never removed old ones, so
  splitting `fuselage` → `fuselage_nose` + `fuselage_main` (and `wing` →
  `wing_L` + `wing_R`) orphaned the old monolithic `fuselage.*`/`wing.*`
  files in `out/cad/*/parts/`. The Pages 3D viewer globs
  `stl/parts/*.stl`, so the old whole fuselage rendered on top of the two
  split pieces. `export_vehicle` now wipes the per-part directories before
  writing (on-disk parts always equal the current assembly); the 4 orphan
  files are removed. No effect on the mass model — CAD is not an input to
  it.

## [0.2.0] — 2026-07-10

Second design snapshot. Adds the mechanical-design layer the v0.1.0
airframe was hand-waving: vibration isolation, segmented-FDM
construction as the baseline (re-converging the whole design point to
~3.06 kg), and the modularity split lines (removable nose, battery
hatch, two-piece wing) needed to actually build and iterate it.

### Added

- **Segmented-FDM construction as the baseline airframe** (ADR-0008):
  `construction_method` + per-method `k_construction` in
  `config/initial_weight_fraction_estimation.yaml` scale the structural
  fraction inside the mass closure, re-converging the whole design.
  Baseline k=1.1 moves the design point from ~2.5 kg to **~3.06 kg MTOW**
  (~39 N hover thrust, ~1.03 kW hover electrical). NB1 gains a
  side-by-side construction trade study; a new `T_max_N` guard
  (`config/rotor.yaml`) fails the sizing loudly if hover thrust exceeds
  the COTS DS-215-class EDF capability.
- **Modularity split lines** (`config/modularity.yaml`): removable nose
  module (payload access, split just aft of the payload bay), 90-degree
  battery hatch over the battery bay, and a two-piece wing on an 8 mm
  CFRP carry-through spar (matching `wing_lighten --spar-hole`). Joint
  hardware (nose ring, hatch frame, spar tube + root fittings) is carved
  from the structural fraction as named mass; spar tube mass is computed
  from geometry. New `joint_hw`/`spar_hw` layout items, BOM rows, and CAD
  parts (`fuselage_nose`/`fuselage_main`/`battery_hatch`,
  `wing_L`/`wing_R`, assembly-only `spar_tube`) whose unions reproduce
  the OML exactly.
- **Vibration isolation** (new pipeline stage `vibration_isolation`, NB5):
  soft-mounts the FC/IMU cluster and the payload against the EDF 1-per-rev
  imbalance (~211 Hz forcing, derived from rotor RPM). Sizes each as a 1-DOF
  base-excitation isolator — corner frequency, transmissibility, static sag,
  sway space, stiffness, and hardware mass — with a valid-window check
  (above the control bandwidth, below the isolation threshold). Achieves
  ~90% attenuation of the 1/rev at f_n ≈ 58 Hz with a few mm of sway.
  See ADR-0007.
- Isolator hardware is carved from the avionics/structural fractions and the
  sway/rattle space is fed into the fuselage bay stack; the fuselage,
  mass-properties, and CAD notebooks consume `out/vibration.yaml`.
- CAD: soft-mounted FC/IMU tray and payload nose module added to the assembly
  (isolator standoffs + rattle gap), assembly-only — the fused external-aero
  STL is unchanged.

### Changed

- Design point re-baselined: all regression pins, `cfd/vehicle/Allrun.case`
  references (`lRef`, `Aref`, `CofR`), and committed `out/` artifacts
  updated for the 3.06 kg segmented-FDM vehicle.
- Pipeline is now nine notebooks (was eight); `fuselage_design` →NB6,
  `vehicle_solid_model` →NB7, `mass_properties` →NB8, `wiring_diagram` →NB9.
- CFD smoke-run mesh gate now checks mesh *usability* (single region,
  closed boundary, positive cell volumes, solver-safe non-orthogonality,
  skewness ≤ 8) rather than demanding checkMesh's perfect "Mesh OK"
  verdict — the coarsened smoke mesh can trip the skewness heuristic on a
  couple of faces without being unusable.

## [0.1.0] — 2026-07-09

First tagged design snapshot of the electric tail-sitter VTOL UAV
(V-BAT-like, single EDF, jet vanes). Establishes the full one-way design
pipeline — `config/*.yaml` → `src/conceptual_design/` (all physics) →
`notebooks/` (orchestrate) → `out/` (handoffs) → `cfd/` (consumers) — and a
converged conceptual design point.

### Design point

- ≈ 2.5 kg MTOW, COTS 195 mm EDF (Schübeler DS-215 class), ~770 W hover
  electrical, ~9C peak battery discharge.
- 15–20 min mission: 120 s hover + 40 s transitions + 900 s cruise
  (deliberately short-hover — hover is expensive at this disk loading).
- NACA 2412 wing, AR 6, ~1.09 m span; +0.05 MAC cruise static margin.

### Pipeline (8 notebooks, executed in dependency order)

1. `vbat_conceptual_design` — mission sizing, mass closure.
2. `wing_design` — airfoil selection → `out/airfoil.yaml`.
3. `control_vane_design` — jet vanes → `out/control_vanes.yaml`.
4. `aileron_design` — cruise-phase roll backup → `out/aileron.yaml`.
5. `fuselage_design` — packaging, CG, drag → `out/fuselage.yaml`.
6. `vehicle_solid_model` — CadQuery CAD → `out/cad/` (STEP/STL, per-part +
   fused + parametric prop rotor).
7. `mass_properties` — inertia tensor, BOM → `out/mass_properties.yaml`,
   `out/bom.csv`.
8. `wiring_diagram` — generated electrical block diagram →
   `out/wiring_diagram.svg`, `out/electrical.yaml`.

### Control

- **Jet vanes** (4×, X-configuration) provide pitch/yaw/roll in
  hover/transition, sized from hover thrust.
- **Ailerons** (2×) added as the primary roll actuator in wing-borne cruise:
  jet-vane authority scales linearly with thrust and collapses to a thin
  margin in cruise (cruise thrust ≈ L/D⁻¹ of hover thrust), so ailerons —
  driven by wing dynamic pressure — cover roll exactly where the vanes are
  weakest. `ddot_min_deg_s2` (`config/aerodynamics.yaml`) is the single
  authority requirement both notebooks check against.

### Mechanical

- **Adjustable battery-tray CG trim**: rail-mounted battery (±25 mm travel)
  as a bounded, checked CG-trim actuator; `mass_properties` solves the trim
  to hit the target static margin and flags if the requirement exceeds the
  rail travel.
- **Mass-fraction carve-out discipline**: named hardware (vane/aileron
  servos, linkages, battery tray) is carved from the top-down mass-closure
  fraction that documents it (avionics / structural / misc), keeping both
  the original budget and the carved amount on the handoff, with a guard
  that fails loudly rather than silently net down.
- **CAD**: full parametric solid model incl. wing-split ailerons at zero
  deflection and 9g-class aileron servos as lower-surface pods (the wing is
  too thin at the hinge to bury them).

### CFD & geometry

- `cfd/vehicle` OpenFOAM case driven by the exported STL; vehicle CFD models
  the fan as an actuator disk (fused STL excludes blades). Coarse-mesh,
  60-iteration smoke run is the CI ceiling — production CFD runs off-CI.

### CI / tooling

- `ci.yml` — ruff, pytest (3.10/3.12/3.14), ShellCheck on `cfd/**/Allrun*`.
- `design-pipeline.yml` — executes all eight notebooks, runs
  design-regression + geometry tests, uploads `out/`; on main also runs the
  coarse OpenFOAM smoke run and deploys the GitHub Pages design report.
- `release.yml` — pushing a `v*` tag re-runs the pipeline and attaches the
  frozen design snapshot to a GitHub Release.
- Regression pins in `tests/test_design_outputs.py` lock the converged
  design point and cross-check `cfd/vehicle/Allrun.case`; `tests/test_geometry.py`
  guards the exported STL (watertight, mm units, span).

[0.4.0]: https://github.com/onurtuncer/vbat-uav-notebooks/releases/tag/v0.4.0
[0.3.0]: https://github.com/onurtuncer/vbat-uav-notebooks/releases/tag/v0.3.0
[0.2.1]: https://github.com/onurtuncer/vbat-uav-notebooks/releases/tag/v0.2.1
[0.2.0]: https://github.com/onurtuncer/vbat-uav-notebooks/releases/tag/v0.2.0
[0.1.0]: https://github.com/onurtuncer/vbat-uav-notebooks/releases/tag/v0.1.0
