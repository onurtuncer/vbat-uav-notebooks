# 10. Semi-monocoque clamshell fuselage

Status: Accepted (amends ADR-0008's split-line architecture)

## Context

Design review (Çağlar Üçler, 2026-07-09) of the ADR-0008
airframe — a load-bearing monocoque cylinder with transverse splits
(removable nose, 90° battery hatch) — proposed a fundamentally better
structural architecture for manufacturability, access, and mass:

- Build the fuselage as **two half-moons** (longitudinal clamshell):
  far easier to mold or print than a closed cylinder.
- Run a **rectangular Alu/composite profile around the joint line**: it
  works as the **longerons** (a "picture frame" in plan view). Add
  **crossbeams** between the longerons and anything can be mounted
  anywhere — the camera/payload bolts to the frame, so no separate nose
  module is needed.
- Put the **battery rail on the lower centerline (180°)** and tie it to
  the frame with **2 half-rings**; the rings double as the longeron↔
  frame connection and a structural vibration path.
- Make the **upper half a full-length lid** — hinged on one longeron,
  latched on the other — for one-motion interior access.
- Result: **semi-monocoque** — the frame carries the loads, the skin
  thins to a covering — *lighter than the load-bearing cylinder*,
  finishing at **2-ply carbon** ("2 kat karbonla biter").

Follow-up review point (Onur Tuncer): the same architecture is FDM-printable —
there is no harm printing what could be laid up in carbon, especially in
first iterations; later a mold can itself be printed. So the
architecture is **construction-agnostic**; only skin/frame material
properties switch.

## Decision

1. **Longitudinal clamshell replaces the transverse splits.** The lid
   runs from the nose tip to `clam_aft_frac·L` (the tail-cone start —
   the boattail is the fan centerbody and stays one piece with the
   lower half). ADR-0008's removable nose and battery hatch are
   superseded; the two-piece wing on the CFRP carry-through spar is
   unchanged.

2. **Explicit semi-monocoque member model** replaces the monocoque
   area-density estimate (`m_shell = k_struct·ρ·t·S_wet`, k_struct=1.30):

       m_struct = k_skin·ρ_skin·t_skin·S_wet        (skin, k_skin=1.05)
                + λ_frame·(2·L_clam + 2·D)          (longeron frame)
                + λ_frame·n_cross·0.8·D             (crossbeams)
                + λ_frame·n_rings·π·D/2             (half-rings)

   λ_frame is computed from the box-profile section × material density —
   never configured as mass. The old k_struct=1.30 hid the frames inside
   a knock-up factor; they are explicit members now, so only the
   bonding/overlap allowance (k_skin=1.05) remains on the skin.

3. **Per-method skin table** (`config/modularity.yaml`): the model is
   evaluated for the configured `construction_method` AND for both skin
   options on every run, keeping the print-first/carbon-later trade
   visible:

   | structure | mass @ 3.06 kg design point |
   |---|---|
   | old monocoque estimate (0.8 mm × 1.30) | 256 g |
   | semi-mono, FDM skin (1.2 mm) + Alu frame | **352 g** |
   | semi-mono, 2-ply CFRP skin (0.5 mm) + Alu frame | **242 g** |

   The FDM clamshell is *heavier* than the old estimate but buildable
   immediately with full interior access; the carbon skin realizes the
   predicted mass gain — exactly the review's sequencing.

4. **Packaging wall decoupled from structural skin.** `t_shell_m`
   (config/fuselage.yaml) remains the conservative packaging allowance
   for the internal bay radius, so bay geometry (and every geometry pin)
   is stable across skin material choices at conceptual stage.

5. **Joint hardware** carve (ADR-0005 discipline) is now the lid's
   hinge strip + latches; the battery rail keeps its existing tray
   mass/travel model (its physical description moves to the lower
   centerline, tied to the frame by the half-rings).

## Consequences

- Structural budget utilization drops to ~61% (352 g FDM against the
  574 g budget) — grounds for a follow-up **re-baseline decision**
  (lower `fs_base` and/or `k_construction` with the explicit model as
  justification), which would shrink MTOW. Deliberately deferred to its
  own decision with these numbers on the table.
  **Executed (2026-07-11):** `fs_base` fitted 0.25 → 0.22 so the budget
  lands at ~110% of the FDM member estimate after full re-convergence;
  MTOW **3.06 → 2.376 kg** — lighter than the original 2.5 kg monocoque
  baseline, i.e. the semi-monocoque architecture more than pays back the
  FDM print penalty at the whole-vehicle level. The feasible floor
  (~0.19) is set by the ADR-0005 guards: the shrunk avionics budget must
  still carry the fixed servo/isolator hardware.
- CAD: `fuselage_lid`/`fuselage_lower` replace nose/main/hatch; the
  frame (longerons, crossbeams, half-rings, battery rail) is modeled as
  assembly-only display parts; the fused external-aero STL is unchanged
  in shape.
- The vibration-isolation model (ADR-0007) is unchanged; the half-rings'
  structural damping role is a modeling refinement left for a future
  pass.
- Equipment mounting generalizes: anything bolts to the frame/crossbeams
  — the packaging model keeps its bay abstraction, which now has a
  physical carrier.
