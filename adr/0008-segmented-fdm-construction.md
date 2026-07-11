# 8. Segmented-FDM construction as the baseline airframe

Status: Accepted (split-line architecture amended by ADR-0010:
the transverse splits -- removable nose, battery hatch -- are
superseded by the longitudinal clamshell; the construction-method
mechanism and the two-piece wing/spar remain in force)

## Context

The airframe was an idealized CFRP monocoque with no split lines: no payload
access, no battery hatch, a one-piece wing. A buildable vehicle needs
modularity — and for a university project, composite tooling is the
bottleneck: every OML iteration means a new mold. The repository already
carries a print-preparation pipeline (`printprep/step2print` segments
oversize parts with alignment pins; `wing_lighten` shells, ribs, and bores a
spar hole) that makes **segmented 3D-printed construction on a CFRP spar
backbone** a realistic alternative: cheaper iteration, worse mass.

The mass cost cannot be bolted on after sizing: the structural fraction sits
inside the mass-closure equation `m_total = (m_payload + m_battery) /
(1 − f_fixed)` and compounds through the battery (heavier → more hover power
→ more battery → heavier). It must re-converge the whole design.

## Decision

1. **Construction method is a first-class config choice**
   (`construction_method` in `config/initial_weight_fraction_estimation.yaml`),
   with a per-method structural-fraction multiplier `k_construction`.
   `WeightFraction.from_yaml` applies `fs = fs_base × k_construction[method]`
   and keeps `fs_base`, `k_construction`, and the method name as fields —
   every notebook re-converges automatically, and the top-down assumption's
   history is never lost (ADR-0005 discipline).

2. **`segmented_fdm` is the baseline** (not a side study). The design point
   re-converges from ~2.5 kg to **~3.06 kg MTOW**.

3. **k = 1.1, deliberately at the low end** of the published 15–30% FDM
   penalty range, paired with a commitment to printprep's mass-discipline
   tooling (shelling, rib lightening, web removal). The closure is
   hypersensitive near its singularity — the sweep that drove this choice:

   | k | MTOW | hover thrust | hover power |
   |---|------|-------------|-------------|
   | 1.00 | 2.50 kg | 32 N | 0.77 kW |
   | 1.10 | 3.06 kg | 39 N | 1.03 kW |
   | 1.15 | 3.53 kg | 45 N | 1.27 kW |
   | 1.20 | 4.57 kg | 58 N | 1.87 kW |

   At the mid-range k=1.2 the vehicle nearly doubles and the hover thrust
   requirement exceeds what a DS-215-class EDF comfortably delivers,
   breaking ADR-0003's COTS constraint. A `T_max_N` guard
   (`config/rotor.yaml`) now fails the sizing loudly if any future change
   pushes the hover requirement past the COTS fan class.

4. **The wing keeps its CFRP-spar Raymer estimate in both methods** — the
   carry-through spar carries wing bending either way; the FDM penalty
   applies to the fuselage/fairing-dominated structural fraction. This is a
   documented approximation, not an oversight.

5. **Three split lines**, geometry and hardware in `config/modularity.yaml`:
   removable nose (payload access, joint ring just aft of the payload bay),
   battery hatch (90° panel over the battery bay), two-piece wing on an
   8 mm CFRP spar tube (matching `wing_lighten --spar-hole 8.0`). Joint
   hardware is discrete named mass carved from the structural pool
   (ADR-0005); spar tube mass is computed from geometry, never configured.
   The CAD splits the fuselage/wing along these exact lines — unions
   reproduce the OML, so the external-aero STL is unchanged in shape and
   the spar tube stays assembly-only (internal, like the prop rotor).

## Consequences

- The whole design point re-baselines (+22% MTOW, +34% hover power):
  every regression pin, `cfd/vehicle/Allrun.case` references, and all
  committed `out/` artifacts change in the same commit — the documented
  "intentional design change" flow.
- Iteration cost drops from mold-rework to overnight reprints; any OML
  change flows `config → CAD → printprep` untouched by hand.
- `k_construction` is the single most sensitive mass parameter in the
  project. Its config comment carries the sweep table; revisit it the
  moment a printed prototype part is weighed.
- Switching back to `monocoque` is one config line; NB1 computes both
  methods side-by-side every run so the trade stays visible.
