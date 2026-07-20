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
