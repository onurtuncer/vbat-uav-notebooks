# Changelog

All notable changes to this conceptual design study are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Versions are derived from git tags via `setuptools-scm`; never edit a version
by hand. Per project policy, a changelog entry is written whenever a version
is tagged.

## [Unreleased]

## [0.5.1] — 2026-07-19

Patch release: the NACA 4412 wing re-selection, the ADR-0014 amendment
folding the pack's internal resistance into the closure honestly
(design point moves to **2.518 kg MTOW / 743 W hover**), and the
versioned Aeolion geometry handoff (ADR-0016) — the parametric contract
for the external VLM/BEMT adjoint loop, jet vanes included.

### Added

- **Aeolion geometry handoff** (ADR-0016): NB8 exports
  `out/cad/aeolion_geometry.json` beside the STEP files, conforming to
  the Aeolion-side pinned 1.0.0 JSON Schema
  (`schemas/vbat-aeolion-geometry-handoff.schema.json`, 2020-12,
  `additionalProperties: false`, stored verbatim and validated in
  tests): planform stations + Kulfan CST airfoil sections aligned 1:1
  by eta at fixed counts (`config/aeolion.yaml` — structural constants
  of the optimization, not design variables), control surfaces (the
  aileron plus the four jet vanes as all-moving plates on the duct-exit
  radius with radial hinge axes), BEMT blade stations, static mesh topology,
  and a `design_id` sha256 of the canonical payload (byte-stable per
  design point). JSON is the differentiable-analysis contract; STEP
  stays the exact-CAD traceability artifact, off the derivative path.
  New CadQuery-free `prop_geometry.py` is the single source of truth
  for the blade chord/twist laws, shared by the CAD rotor and the BEMT
  handoff.

### Changed

- **Battery internal-resistance accounting made consistent (ADR-0014
  amendment)**: `eta_bat` 0.97 → 0.916, the mission-averaged I²R
  efficiency of the nominal 90 mΩ pack iterated to the closure fixed
  point (the old 0.97 implied an unrealistic ~23 mΩ pack), and the
  battery-bay vent load now uses the same I²R law as the mission
  transient. The honest accounting costs **+215 g: MTOW 2.303 →
  2.518 kg** (hover 743 W, 8.1C peak; wing S 0.1883 m², b 1.063 m).
  Standing findings move with it: the nominal-pack mission transient
  worsens to ~65 °C vs the 60 °C limit, ESC cold-plate load rises to
  ~37 W (still marginal), the margined ESC current requirement rises to
  ~54 A (still within the APD 80F3[x]), the frozen 450 g Molicel pack
  now undercuts the sized pack by ~203 g (as-selected all-up 2.336 kg =
  closure −182 g) — and the ADR-0012 structure-over-budget finding is
  **resolved** (the larger structural pool covers the packaging-floored
  shell with ~9 g margin). Measured pack DCIR at procurement remains the
  lever: a 60–70 mΩ build claws most of the mass back. Pins,
  `Allrun.case`, and all `out/` artifacts updated in the same change.
- **NACA 4412 wing** (ADR-0015): airfoil re-selected 2412 → 4412 after
  the v0.5.0 stall-consistency fix made section lift buy wing area
  directly. W/S 107.2 → 131.2 N/m², S 0.2107 → 0.1722 m², b 1.124 →
  1.016 m (smaller than the pre-fix wing), MAC 0.187 → 0.169 m, cruise
  CL 0.535 near the polar optimum, L/D 12.76 → 13.35, V_MD ≈ 18.5 m/s
  so the 20 m/s cruise is within ~2% of max L/D. A V_cruise change was
  studied (16–22 m/s through the closure) and rejected: the fixed-range
  mission energy is nearly V-independent under the configured constant
  L/D. MTOW / hover power / battery point unchanged. Watch items
  (ADR-0015): un-modelled 4412 nose-down Cm0 (cruise trim) and the
  empirical Cl_max at Re ≈ 1.4e5 near stall. Pins, `Allrun.case`, and
  CAD updated in the same change.

## [0.5.0] — 2026-07-17

Minor release: the wing is re-sized to the selected airfoil's real
CL_max — a design-point change (wing geometry, MAC, CFD references),
hence the minor bump. Ships on top of the v0.4.5 content (Li-ion
freeze, thin notebooks, pack-transient check), now merged to main
(PR #18).

### Fixed

- **Stall-speed consistency** (from the 2026-07 external aero review by
  C. Ucler): the wing card's 11.98 m/s stall line was computed from the
  pre-selection placeholder `CL_max: 1.4` while every other number on
  the card came from the selected NACA 2412 (`CL_max_3D = 1.2186`) —
  the real wing stalled at ~12.8 m/s and the V_stall ≤ 12 m/s
  requirement was silently unmet. `CL_max` is now the selected wing's
  1.2186 (and must track the NB2 selection). The stall-limited wing
  loading drops 123.2 → 107.2 N/m² (S 0.1834 → 0.2107 m², b 1.049 →
  1.124 m, MAC 0.175 → 0.187 m, cruise L/D 13.22 → 12.76); MTOW,
  hover power, and the battery point are unchanged. Regression pins,
  `Allrun.case` (lRef/Aref/CofR), and the CAD exports updated in the
  same change.

## [0.4.5] — 2026-07-17

Patch release: Li-ion battery freeze, thin-notebook refactor, and the
battery pack mission-transient thermal check from the external design
review.

### Added

- **Battery pack mission-transient thermal check** (ADR-0014, from the
  external battery-thermal review by C. Ucler): NB7 now integrates the
  pack's own `I²R_pack` Joule heat through the two vertical legs
  (adiabatic) with laminar flat-plate cruise cooling between, at the
  wiring-law currents and configured 60/90/120 mΩ pack-resistance
  cases. New `battery_transient` block in `out/thermal.yaml`; margins
  row and standing finding in `design_summary`; regression pins. New
  standing finding: at the 40 °C hot-day ambient the nominal 90 mΩ
  pack ends the mission at 62.4 °C average vs the 60 °C limit — the
  optimistic 60 mΩ build passes (54.9 °C), making measured pack DCIR
  the procurement acceptance criterion. `eta_bat: 0.97` is unchanged
  in the closure but documented as implying an unrealistic ~23 mΩ
  pack.
- **Molicel P50B 6S1P 5000 mAh Li-ion (21700)** added to the battery
  catalog and won the freeze at 450 g — ~105 g under the sized LiPo
  (pack-level specific energy beats the configured LiPo density). The
  battery budget line flips positive and the as-selected all-up drops
  to 2.264 kg (−38 g **under** the closure MTOW, from +125 g over in
  v0.4.4). Note its 3.6 V/cell nominal vs the configured 3.7.

### Changed

- **Thin-notebook refactor** (ADR-0013): all physics, figure, and
  report code moved from the notebooks into `src/conceptual_design/`
  (`design_point.py` as the single sizing-loop call site, `plots/`,
  `reports.py`, `design_summary.py`, `notebook.py` shared prelude).
  Notebooks are now parameters + module calls + interpretation only;
  the `out/` handoffs were verified byte-identical across the move.
  NB8 (`vehicle_solid_model`) keeps its inline sizing block (CI-only).

## [0.4.4] — 2026-07-17

Patch release: post-freeze as-selected re-solve stage (ADR-0012) — the
pipeline is now fifteen notebooks.

### Added

- **NB12–NB14 as-selected re-solves** (`aileron_design_cots`,
  `vibration_isolation_cots`, `fuselage_design_cots`): NB4–NB6 re-solved
  downstream of the COTS freeze with the frozen hardware — real servo/FC
  masses, pack density from the battery's own envelope, bay lengths
  floored at each rigid part's best-orientation axial length, actual
  motor/prop/ESC layout masses, and a physical-fit report. Parallel
  `out/*_cots.yaml` handoffs; the conceptual solutions and the CAD/CFD
  geometry are untouched (no back-edge). New standing findings, pinned:
  as-selected all-up 2.427 kg (+125 g over the 2.303 kg closure),
  as-selected hull ⌀106 × 531 mm, structure model ~33 g over budget.
- **NB15 `design_summary`** (last notebook): pure reader — design point,
  margins table, frozen COTS hardware, and all standing findings
  collected programmatically from the `out/` handoffs.
- `src/conceptual_design/cots_integration.py` and backward-compatible
  `fuselage_design.py` extensions (`min_axial_length_m`,
  `part_envelopes`, `prop_item_masses`); `part_clearance_m` in
  `config/fuselage.yaml`; regression pins for the new handoffs;
  ADR-0012.

## [0.4.3] — 2026-07-15

Patch release: frame-longeron display parts no longer protrude through
the nose taper.

### Fixed

- **Frame longerons trimmed to the hull**: the L/R clamshell-joint
  longerons were straight bars at y = ±0.35 D running the full
  clamshell length to the nose tip, so forward of ~station 22 mm they
  protruded through the nose taper. `frame_parts` now intersects them
  with the fuselage solid — they end where the ogive becomes narrower
  than the joint line (display part only; the member mass model is
  unchanged).

## [0.4.2] — 2026-07-15

Patch release: the v0.4.1 snapshot shipped with the prop rotor CAD still
carrying EDF-cartridge proportions at the EDF-era station; this release
rebuilds it as the actual COTS 3-blade prop-in-duct.

### Fixed

- **Prop rotor CAD rebuilt to the actual COTS 3-blade prop** — the v0.4.1
  snapshot still carried EDF-cartridge proportions and the EDF-era
  station: a 114 mm hub barrel + spinner (hub_length_ratio 0.35,
  spinner_ratio 1.0) placed at the duct CG, which buried the hub inside
  the fuselage tailcone, pushed the spinner out through its skin, and
  had the blade roots emerging through the tailcone (the tailcone is
  still ~83 mm across at that station). The rotor is now a short
  adapter collar (~24 mm, no spinner) with the disk plane at the
  fuselage-tailcone / exhaust-centerbody junction (`L_fus_m`), where
  the body has shrunk to the 40.6 mm centerbody radius and the blades
  clear the airframe. Blade planform replaced by a peaked chord law
  (`c_peak_ratio`/`f_peak`, max chord at 35% span) with a rounded tip
  (`tip_round_expo`), matching the COTS 8x6 look while keeping the
  ~14-15 mm mean chord of the sigma = 0.077 solidity model. Diameter
  stays exactly 203 mm and the twist stays the exact
  constant-geometric-pitch law beta(r) = atan(p/2*pi*r) with the true
  6 in pitch. NB8's rotor-diameter printout fixed for 3 blades (bbox
  z-max, not z-extent).

## [0.4.1] — 2026-07-15

Point release on the ADR-0010 baseline: freezes the open COTS hardware
(NB11), amends ADR-0003 to the 3-blade 203 mm prop-in-duct the selection
forced — re-converging the design to **MTOW 2.302 kg / 652 W hover** —
and drift-proofs the propulsion/vane CFD cases against future design
moves.

### Added

- **README hero render** (`assets/vbat_render.png`): shaded 3D view of the
  vehicle standing tail-down, rasterised straight from the exported part
  STLs by `scripts/render_readme_cad.py` (self-contained numpy/Pillow
  z-buffer renderer — no OpenGL needed); linked to the interactive Pages
  viewer from the top of the README.
- **COTS component selection & freeze** (new pipeline stage
  `cots_selection`, NB11): selects the open COTS hardware — flight
  controller, ESC, EDF drive motor, propeller/fan unit, vane/aileron
  servo — from a per-category candidate database
  (`config/components/*.yaml`) against requirements **derived** from the
  converged design point (ESC current via the wiring-module law, motor/fan
  power from `P_design`, rotor diameter from `config/rotor.yaml`, servo
  torque from the NB3/NB4 hinge moments; only the margins are configured).
  Lightest feasible candidate wins deterministically; a `selection.frozen`
  id pins a procured part and is re-validated every run, failing loudly if
  a design change outgrows it. Freezes ids, masses, and own-CG inertia
  tensors to `out/components.yaml`, pinned by new design-regression tests.
  Selected stack: Pixhawk 6C, APD 80F3[X] (ADR-0011's telemetry
  requirement eliminates the Hobbywing/T-Motor field), SunnySky
  X4120-class motor, DS-215-DIA HST fan, KST X08 servos.
- **Finding (dominant): the ADR-0003 "COTS 195 mm EDF" class has no
  light implementation.** The only COTS fan at the frozen diameter is the
  Schübeler DS-215 heavy-lift family; fan + lightest plausible motor land
  ~6× over the propulsion allocation (−1069 g). A COTS 3-blade prop in
  the airframe duct (V-BAT-like) is ~40× lighter but exists at
  178/203 mm, not 195 mm — kept visible as a diameter-only rejection
  because adopting it means moving `D_rotor_m`, re-converging, and
  amending ADR-0003. Second finding: the lightest PX4 stack overruns the
  avionics bay by ~37 g. Both pinned as standing findings (ADR-0009
  discipline).

- **Battery pack joins the COTS selection** (`batteries.yaml`): fully
  derived requirements (6S exact, capacity ≥ the sized mission Ah,
  continuous discharge ≥ the ESC's margined hover current — no
  configurable margin), compared against the mass-closure battery mass
  where COTS capacity quantisation makes a small overshoot expected
  (Gens Ace 6S 4000 30C: ~58 g over the sized pack). Candidates across
  all categories now carry procurement URLs preferring Turkish
  retailers; the forced imports are the telemetry ESC (no TR retailer
  stocks a live-telemetry 50 A+ single ESC) and the KST X08 servo
  (TR fallback: MG90S, +3 g each).

### Changed

- **ADR-0003 amended: the COTS rotor is a 3-blade 8×6 propeller
  (203 mm) in the airframe duct, replacing the 195 mm COTS EDF
  cartridge** — resolving NB11's dominant finding (the only purchasable
  195 mm fan is 10 kW-class heavy-lift hardware). The sizing physics
  already modelled a low-solidity prop (σ = 0.077 matches a 3-blade 8×6
  over the annulus outside the 0.4 R hub); only the procurement
  assumption changed. `D_rotor_m` 0.195 → 0.203 and the whole design
  re-converged: **MTOW 2.376 → 2.302 kg, hover 710 → 652 W** (+8% disk
  area, lower disk loading), 1/rev forcing 211 → 204 Hz, pack 3.71 →
  3.50 Ah. `T_max_N` guard re-derived for the prop-in-duct power
  ceiling (50 → 35 N — note hover now sits at ~84% of the guard;
  thrust-stand verification is ADR-0011 stage a). The CAD rotor is now
  a 3-blade prop (`config/prop_geometry.yaml`). All regression pins,
  `cfd/vehicle/Allrun.case`, and the `px4/` provenance values
  (airframe params, Gazebo model, README) updated in the same commit;
  `out/cad/` regenerates in CI. Propulsion mass now closes to a ~100 g
  finding (all motor, EST) instead of ~1.1 kg of unbuyable hardware;
  NB11 pins the propeller via `selection.frozen` (the TR-stocked
  2-blade Gemfan is lighter but re-opens the solidity/FoM revisit).
- **Trusted SITL plant redirected to Aetherion + CFD-derived DAVE-ML**
  (amends the v0.4.0 PX4 plan): the trusted plant is **Aetherion**
  (external 6-DoF repo) consuming **DAVE-ML models generated from CFD
  results** (`px4/sitl/daveml_spec.md`: coefficient-form airframe +
  dimensional prop/vane models to survive the hover `q_∞ → 0`
  singularity, S-119 check-cases, validation gates); the Gazebo model is
  demoted to PX4-side integration smoke checks.

### Fixed

- **Prop/vane CFD geometry sourced from the design handoffs, not
  hardcodes** (`cfd/prop/make_geom.py`, `Allrun.prop`, `Allrun.vanes`,
  `foam2dml_prop.py`): the actuator-disk propulsion case still built the
  ancient 286/302 mm duct (56 mm hub) and the vane sweep still meshed
  the pre-resize vane (span 0.0714 m, chord 0.02856 m, V_jet
  23.38 m/s) — both had silently drifted from the converged design
  point. The duct/hub dims now come from `out/fuselage.yaml` and the
  vane dims/default jet speed from `out/control_vanes.yaml` at run
  time; `Allrun.prop` refuses to reuse a base mesh built for a
  different design point (`base/.geom_dims` stamp), and the DAVE-ML
  exporter builds its provenance text from `run_meta.yaml`, exiting
  loudly on sweeps that predate the change. `Allrun.vanes` also gains
  the previously missing `constant/transportProperties` and
  `turbulenceProperties`, without which simpleFoam could not run at
  all.

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

[0.5.1]: https://github.com/onurtuncer/vbat-uav-notebooks/releases/tag/v0.5.1
[0.5.0]: https://github.com/onurtuncer/vbat-uav-notebooks/releases/tag/v0.5.0
[0.4.5]: https://github.com/onurtuncer/vbat-uav-notebooks/releases/tag/v0.4.5
[0.4.4]: https://github.com/onurtuncer/vbat-uav-notebooks/releases/tag/v0.4.4
[0.4.3]: https://github.com/onurtuncer/vbat-uav-notebooks/releases/tag/v0.4.3
[0.4.2]: https://github.com/onurtuncer/vbat-uav-notebooks/releases/tag/v0.4.2
[0.4.1]: https://github.com/onurtuncer/vbat-uav-notebooks/releases/tag/v0.4.1
[0.4.0]: https://github.com/onurtuncer/vbat-uav-notebooks/releases/tag/v0.4.0
[0.3.0]: https://github.com/onurtuncer/vbat-uav-notebooks/releases/tag/v0.3.0
[0.2.1]: https://github.com/onurtuncer/vbat-uav-notebooks/releases/tag/v0.2.1
[0.2.0]: https://github.com/onurtuncer/vbat-uav-notebooks/releases/tag/v0.2.0
[0.1.0]: https://github.com/onurtuncer/vbat-uav-notebooks/releases/tag/v0.1.0
