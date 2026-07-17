# 12. Post-freeze as-selected re-solve as new downstream notebooks

Status: Accepted

## Context

NB4 (`aileron_design`), NB5 (`vibration_isolation`), and NB6
(`fuselage_design`) size against configured estimates — a 12 g servo
class, a 60 g FC/IMU cluster, a 1500 kg/m³ effective pack density —
because nothing better exists before hardware is chosen. NB11
(`cots_selection`, ADR-0011) then freezes real parts with catalog masses
and envelopes, and immediately exposes the gaps: the Gens Ace pack is a
rigid 137 mm brick 58 g over the sized battery mass, the motor is ~99 g
over its allocation, the avionics stack is 42 g over the net bay budget.

The obvious move — feed the frozen hardware back and re-run NB4–NB6 — has
already been rejected in principle: ADR-0001 makes the pipeline one-way
precisely so that a change's provenance is always visible. Re-running the
old notebooks with new inputs would overwrite the conceptual solution
(the record of what the estimates gave), silently move the geometry the
CAD/CFD chain and the regression pins are built on, and create a
NB11 → NB4 back-edge that turns the pipeline into a loop.

## Decision

Re-solve the estimate-based designs **after** the freeze, as **new
notebooks appended to the pipeline**, followed by a read-only summary:

- NB12 `aileron_design_cots` — NB4's physics with the frozen servo
  (catalog mass; torque check against real stall) → `out/aileron_cots.yaml`
- NB13 `vibration_isolation_cots` — NB5's physics with the frozen FC mass
  (blade count consistency-checked against the frozen prop)
  → `out/vibration_cots.yaml`
- NB14 `fuselage_design_cots` — NB6's physics with as-selected masses,
  the pack density derived from the frozen battery's own envelope, each
  bay's length **floored at the best-orientation axial length of the
  rigid part it holds**, actual motor/prop/ESC masses in the layout, and
  a physical-fit report for every frozen part → `out/fuselage_cots.yaml`
- NB15 `design_summary` — the last notebook: reads only `out/` handoffs,
  writes nothing; rolls up the design point, the margins, the frozen
  hardware list, and **every standing finding collected programmatically
  from the handoffs**.

Supporting physics lives in `src/conceptual_design/cots_integration.py`
plus backward-compatible extensions to `fuselage_design.py`
(`part_envelopes`, `prop_item_masses`, `min_axial_length_m`); the
`*_cots.yaml` handoffs reuse the NB4–NB6 writers and schemas so consumers
read either solution identically.

Rules of the stage:

- **No back-edges.** The conceptual `out/{aileron,vibration,fuselage}.yaml`
  are untouched; CAD (NB8), CFD, and the `Allrun.case` reference values
  stay on the conceptual geometry.
- **Deltas are findings, not feedback.** The re-solve reports them
  (`delta_vs_conceptual`, `bay_fit`, `as_selected` blocks, pinned by the
  regression tests) exactly like the ADR-0009 thermal finding and the
  NB11 allocation findings. Folding a delta into `config/` is a
  deliberate, reviewed design iteration — after procurement weighs the
  real parts — from which the whole pipeline re-converges.
- **Fit is reported, never filtered on** — same discipline as NB11's
  mass-allocation report; a misfit indicts a packaging assumption, not
  the part.

## Consequences

- The pipeline is fifteen notebooks; NB2–NB15 all re-run
  `run_sizing_loop`, so a sizing API change now touches fifteen call
  sites (was eleven).
- At the current design point the re-solve is materially informative:
  the as-selected all-up mass is **2.427 kg (+125 g over the 2.303 kg
  closure)**, and the hull that actually holds the rigid battery grows
  to **⌀106 × 531 mm** (from ⌀96 × 478 mm), pushing the structure model
  ~33 g over the structural fraction — three new pinned standing
  findings on top of the freeze's allocation findings.
- Two designs coexist by construction (conceptual vs as-selected). The
  regression tests pin both and the relation between them (the
  as-selected hull may never be smaller); the summary notebook makes the
  gap impossible to overlook in every CI run and release snapshot.
- `config/fuselage.yaml` gains `part_clearance_m` (axial clearance per
  rigid part in a bay) — the one new configured physical parameter.
