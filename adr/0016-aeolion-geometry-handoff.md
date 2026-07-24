# ADR-0016: Schema-pinned parametric JSON for the Aeolion geometry handoff

Status: Accepted (2026-07-19; supersedes the earlier ad-hoc JSON draft)

## Context

Aeolion (companion VLM/BEMT repo) closes an adjoint optimization loop
over the VBAT geometry. It needs a differentiable, fixed-topology
parametric description — not the watertight STEP solid, whose booleans
and tessellation are nondifferentiable. The Aeolion side pinned the
contract as a JSON Schema (2020-12, `additionalProperties: false`).

## Decision

- The contract is `schemas/vbat-aeolion-geometry-handoff.schema.json`
  (schema_version **1.0.0**), stored verbatim in this repo and
  validated by `tests/test_aeolion_handoff.py` (jsonschema, dev extra).
- NB8 (`vehicle_solid_model`) exports `out/cad/aeolion_geometry.json`
  beside the STEP files (same execution ⇒ same design point), built by
  `src/conceptual_design/aeolion_handoff.py`.
- **Frame/units**: `aetherion_body_frd`, metres, degrees — identical to
  the repo axis convention, so there is no transform seam to
  differentiate through.
- **Fixed design-vector structure**: station count, CST order, BEMT
  station count, and mesh topology come from `config/aeolion.yaml` and
  are structural constants of the optimization, not design variables.
  Changing them is an outer-loop remeshing event (deliberate commit).
- **Airfoil sections are Kulfan CST fits** (class function
  x^0.5·(1−x)^1.0, least squares) of the NB2-selected section. The
  schema has no trailing-edge-thickness term, so each surface is fitted
  after removing the linear TE ramp — the handoff carries the sharp-TE
  aerodynamic equivalent (what a VLM camber surface consumes). Fit RMS
  is test-guarded at < 1e-3 c.
- **BEMT blade stations sample the same chord/twist law as the CAD
  rotor** (`prop_geometry.PropGeometry`, moved out of `cad/` so it
  imports without CadQuery): r/R from the hub collar to the tip, exact
  constant-geometric-pitch twist, `reference_rpm` from the NB5 hover
  1/rev. One law, three consumers (CAD, BEMT, vibration-forcing).
- **Controls**: the aileron and the four jet vanes are emitted. Vane
  entries reuse the schema's control-surface shape for the duct jet:
  eta is the duct-exit radius fraction (hub collar → duct wall), the
  hinge axes are the radial vane directions (CAD angle convention:
  about +x, 0° = +z), and `chord_fraction: 1.0` marks the all-moving
  plate rotating about its `hinge_xc` line. The vane CFD → DAVE-ML
  path still characterizes the jet-wash aerodynamics; carrying the
  geometry here lets Aeolion model the jet directly.
- **Duct block (schema 1.8.0, 2026-07-24)**: the EDF shroud rides in
  the contract explicitly — a new top-level `duct` block with
  `inner_diameter` / `outer_diameter` / `chord` and
  `placement.center`, the SAME parameterisation the CAD duct solid is
  built from (`out/fuselage.yaml` `D_duct_inner_m` / `D_duct_outer_m`
  / `duct_chord_m` and the layout duct station): an annulus of
  revolution about the body x-axis, mid-chord centre at x = −station
  (same convention as `body`), inlet face forward at
  `center.x + chord/2`. Before this the duct existed in the contract
  only by implication — the vane entries' eta is measured on the
  duct-EXIT radius and BEMT models a prop-in-duct, yet no duct
  geometry or position was stated anywhere, so a duct-aware consumer
  had nothing to place against `body` (and the duct legitimately
  overhangs the tail base, so guessing from the body extent gets it
  wrong). The bore carries the rotor tip clearance — new exporter
  guard `rotor_D < D_duct_inner < D_duct_outer`, plus exactly one
  `duct` item required in the fuselage layout. **Required from 1.8.0
  onward**, same version-conditional `if`/`then` discipline as
  `planform.placement` and `moment_reference_point`; 1.0.0–1.7.0
  documents remain structurally valid without it.
- **Moment reference point (schema 1.6.0, 2026-07-21, from an Aeolion
  solver report)**: a new top-level `moment_reference_point` {x, y,
  z} carries the vehicle CG, in the SAME body-frame convention and
  from the SAME source (`out/fuselage.yaml` `x_CG_m`) as
  `cfd/vehicle/Allrun.case`'s `CofR` — so VLM/BEMT moments computed
  from this contract stay directly comparable to the project's CFD
  force/moment coefficients, not an arbitrary second reference.
  Without it, Cm/Cl/Cn have no stated centre and default to the
  coordinate-system origin — not physically meaningful for a body
  whose CG sits well aft of the nose. **Required from 1.6.0 onward**,
  same discipline as `planform.placement`: a version-conditional
  `if`/`then` (`schema_version` not in the pre-1.6.0 list ⇒
  `moment_reference_point` required at the document root), so
  1.0.0–1.5.0 documents remain structurally valid without it. New
  exporter guard mirrors the placement guard: `0 < x_CG_m < L_fus_m`.
- **Wing/body placement (schema 1.5.0, 2026-07-20, from an Aeolion
  solver-side bug report)**: `planform` gained a `placement.
  root_leading_edge` anchor {x, y, z} in the same body-frame
  convention `body` already uses (x = -station, 0 at the nose tip).
  Before this the planform and the body profile each implicitly
  started at the reference-frame origin, so a consumer with no other
  convention to fall back on placed the wing root at the fuselage
  nose — 22 of 384 wing control points landed inside the body of
  revolution in Aeolion's report, a boundary condition with no
  physical meaning. The anchor is `[-x_wing_LE_m, 0, 0]`: the wing
  chord datum sits on the fuselage centerline by construction (a
  through-fuselage wing on the ADR-0008 carry-through spar), not an
  incidental default. **Required from 1.5.0 onward** — unlike every
  earlier addition, this field is NOT optional for its own version,
  because the unsafe fallback (assume the origin) is exactly the bug
  being fixed; the schema enforces this with a version-conditional
  `if`/`then` (`schema_version` not in the pre-1.5.0 list ⇒
  `planform.placement` required) rather than a prose promise, so
  1.0.0–1.4.0 documents that never carried it remain structurally
  valid while every 1.5.0+ document is held to the anchor. The
  exporter also guards `0 < x_wing_LE_m < L_fus_m` so a future design
  change can't silently regress the wing outside the fuselage again.
  The natural follow-on — trimming the wing planform/lattice at the
  fuselage surface so the root load transfers to the body — is left
  to Aeolion's side now that placement exists; it needs the `body`
  radius law (already in the contract since 1.2.0) evaluated at the
  root LE station, not a new VBAT field.
- **Deflection range (schema 1.4.0, 2026-07-20)**: every
  `control_surfaces` entry carries `deflection_limits_deg` — the
  symmetric mechanical range about `hinge_axis` (`±delta_max_deg` from
  `out/aileron.yaml` / `out/control_vanes.yaml`), so a CFD/VLM
  deflection sweep never asks for an angle the physical actuator/plate
  cannot reach. Vane entries additionally carry
  `deflection_soft_limit_deg` (the flat-plate stall onset,
  `delta_stall_deg`) — informational, not a hard bound, since sweeping
  past it is a legitimate (if off-design) query.
- **Blade count (schema 1.3.0, 2026-07-20, missing field caught in
  review)**: `propulsion_bemt.n_blades` (from the same
  `prop_geometry.PropGeometry` the blade stations sample). BEMT
  solidity and thrust are per-blade quantities scaled by blade count —
  `blade_stations` alone describes one blade's chord/twist law, not
  the rotor, so the count was a load-bearing gap, not an
  optional refinement.
- **Body block (schema 1.2.0, 2026-07-19)**: the fuselage rides in the
  contract as a body of revolution — `body.stations` of {x, radius}
  sampled from the same 3-segment meridian the CAD revolve is built
  from (`fuselage_design.fuselage_radius`: elliptical nose, cylinder,
  tail cone to the hub radius), cosine-spaced nose → tail at a fixed
  count (`n_body_stations`, config/aeolion.yaml), body x = −station
  (0 at the nose tip, negative aft, consistent with
  `aetherion_body_frd`). Optional in the schema (1.0.0/1.1.0 documents
  stay valid); never bound to a lifting lattice — intended for
  slender-body/Munk trim corrections and duct-jet context. One
  meridian law, two consumers (CAD revolve, handoff).
- **Surface binding (schema 1.1.0, 2026-07-19, from the Aeolion-side
  review)**: control-surface entries carry an explicit `surface` field
  — `wing` (eta = semispan fraction of the planform) or `duct_jet` (an
  all-moving plate in the jet, eta = duct-exit radius fraction, never
  bound to a lifting-surface lattice). The field is optional so 1.0.0
  documents stay valid; when absent the binding convention is by
  `name`: `aileron`/`rudder` → `wing`, `vane` → `duct_jet`. The
  exporter always emits it. Frame conversion remains the consumer's
  explicit job at ingest (the contract states `aetherion_body_frd`
  once; a z-up VLM flips x and z — a proper rotation about y).
- **`design_id`** is `sha256:<hex>` over the canonical JSON of every
  other field: byte-stable when the design point is unchanged (clean
  regression diffs), guaranteed to move when any parameter moves —
  the adjoint-loop traceability key.

## Rationale

VLM needs a mean lifting lattice; passing span, per-station chord /
twist / sweep / dihedral, and CST sections preserves design intent and
keeps every quantity on a differentiable path. A CppAD implementation
can template geometry → residual → solve → objective on the scalar
type; JSON parsing stays outside the tape and supplies values, while
the fixed array lengths guarantee a constant tape structure.

STEP remains the exact CAD/traceability artifact, but because the
schema forbids additional properties, the STEP linkage is by
construction (same NB8 run, same `out/cad/` directory) rather than by
a field inside the JSON.

## Consequences

- Any change of field meaning requires a schema-version bump on the
  Aeolion side first; this repo follows the pinned schema file.
- The document intentionally repeats identical chord/CST data at every
  station today (rectangular untwisted wing) — that redundancy IS the
  contract for future twist/taper design variables.
- Design changes show up as a new `design_id`; CI's byte-diff of
  `out/cad/aeolion_geometry.json` doubles as a design-regression pin.
