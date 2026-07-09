# Changelog

All notable changes to this conceptual design study are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Versions are derived from git tags via `setuptools-scm`; never edit a version
by hand. Per project policy, a changelog entry is written whenever a version
is tagged.

## [Unreleased]

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

[0.1.0]: https://github.com/onurtuncer/vbat-uav-notebooks/releases/tag/v0.1.0
