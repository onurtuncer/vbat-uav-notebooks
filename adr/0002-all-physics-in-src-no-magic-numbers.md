# 2. All physics in `src/`; no magic numbers in Python

Status: Accepted

## Context

Notebooks are convenient but poor homes for physics: their code is hard to
unit-test, hard to reuse across the eight notebooks, and invites embedding
tuning constants inline where they cannot be reviewed or version-controlled
with rationale. We want the physics reviewable, testable, and reused from a
single source, and every physical assumption visible.

## Decision

- **All physics lives in `src/conceptual_design/`** as importable modules
  with docstrings and references. Notebooks only set parameters, call those
  modules, and interpret/plot results.
- **No magic numbers in Python.** Every physical parameter lives in
  `config/*.yaml` with a commented rationale. The only exceptions are
  numerical tolerances and documented physical guard limits, expressed as
  named module constants (e.g. `MAX_HARDWARE_CARVE_FRACTION`,
  `T_PLATE_MIN_MM`).

## Consequences

- The same sizing code runs in every notebook and in the regression tests —
  one source of truth, so a model change propagates everywhere at once.
- Reviewing a design change means reading a `config/` diff with rationale,
  not hunting for constants scattered across notebook cells.
- CAD/geometry construction constants (placements, embed depths) live as
  named constants in the CAD modules; these are geometry-construction
  details, not physics, and are exempt from the config rule by the
  "documented physical guard limit" carve-out.
- Slightly more ceremony to add a parameter (edit YAML + a dataclass +
  `from_yaml`), accepted as the price of traceability.
