# 5. Mass-fraction carve-out with traceability and a guard

Status: Accepted

## Context

Early mass closure uses top-down weight fractions
(`config/initial_weight_fraction_estimation.yaml`): structural, propulsive,
avionics, misc, each a fraction of MTOW. As real hardware is identified
(vane servos, aileron servos, linkages, battery tray), its mass must move
from an anonymous fraction to a named, positioned line item — without
double-counting and without silently discarding the original top-down
assumption that justified the fraction.

An early attempt netted named hardware straight out of the `misc` fraction
with only a `> 0` check. This hid two problems: the fraction's documented
scope was ignored (servos belong to avionics, not misc), and a carve-out
could quietly exceed what the fraction was ever meant to hold.

## Decision

- Carve each piece of named hardware from the **fraction that documents it**:
  servos → avionics, vanes/linkages → structural, battery tray → misc — as
  each fraction's own comment in the config describes its scope.
- **Keep both numbers on the handoff**: the original fraction budget and the
  carved amount (and the net remainder), so the history of the top-down
  assumption is never lost — only the *net* line item shrinks.
- **Guard the carve-out**: a named module constant
  (`MAX_HARDWARE_CARVE_FRACTION`) asserts a carve-out cannot exceed a
  documented share of its budget; exceeding it fails loudly, signalling the
  fraction assumption itself needs revisiting rather than a silent net-down.

## Consequences

- Mass closes exactly, and every named item is traceable to the fraction it
  came from, with the original budget still visible for review.
- A design change that overloads a fraction (e.g. control hardware growing
  past its avionics share) surfaces as a hard failure, not a quiet drift.
- Adds bookkeeping fields to the fuselage handoff and a small amount of
  carve-out logic, accepted as the cost of not losing the sizing history.
