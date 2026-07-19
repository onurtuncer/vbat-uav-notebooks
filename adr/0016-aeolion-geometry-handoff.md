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
- **Controls**: only the aileron is emitted. The jet vanes act on the
  duct jet, not the lifting surface — they are characterized by the
  vane CFD → DAVE-ML path; the schema's `vane` enum value is reserved
  for when Aeolion models the jet directly.
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
