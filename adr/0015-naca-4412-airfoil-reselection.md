# ADR-0015: Airfoil re-selection — NACA 2412 → NACA 4412

Date: 2026-07-18
Status: Accepted

## Context

The v0.5.0 stall-consistency fix (external aero review, C. Ucler) made
`config/aerodynamics.yaml` `CL_max` track the selected airfoil's real
`CL_max_3D`. Because the wing sizing is stall-limited
(`W/S ≤ 0.5·ρ·V_stall²·CL_max` binds at the design point), section
lift now buys wing area directly — and the fix left two open items:

1. the NACA 2412 wing grew to 0.2107 m² / b 1.124 m, and its cruise
   CL (0.437) fell well below the polar optimum (~0.62), dropping
   cruise L/D to 12.76 with best-range speed ~16.7 m/s, below the
   20 m/s cruise;
2. the section ran with zero 2D→3D Cl_max headroom.

A trade study on this branch swept both levers:

**V_cruise** (16–22 m/s, through the mass closure): in the fixed-range
framing the closure is nearly V-independent — cruise energy is
`W·R/(L/D·η)` with the configured constant L/D — and MTOW moves only a
few tens of grams across the sweep. Changing V_cruise would ripple the
mission, CFD reference velocity, and every handoff for no modelled
gain. **Rejected.**

**Airfoil** (NACA 0012 / 2412 / 4412 / 2415 at the current design
point, MTOW invariant — the closure computes cruise power from the
conservative config `LD: 8.0`, so the airfoil affects wing geometry
and realized L/D, not the closure):

| section | CL_max_3D | W/S [N/m²] | S [m²] | b [m] | CL_cruise | L/D | V_MD [m/s] |
|---|---|---|---|---|---|---|---|
| 0012 | 0.949 | 83.6 | 0.2700 | 1.273 | 0.341 | 11.49 | 14.8 |
| 2412 | 1.219 | 107.2 | 0.2107 | 1.124 | 0.437 | 12.76 | 16.7 |
| **4412** | **1.489** | **131.2** | **0.1722** | **1.016** | **0.535** | **13.35** | **18.5** |
| 2415 | 1.287 | 113.3 | 0.1994 | 1.094 | 0.462 | 11.83 | 16.6 |

## Decision

Select the **NACA 4412** (`config/airfoil_selection.yaml`), with
`CL_max: 1.4886` in `config/aerodynamics.yaml` per the tracking rule.
**V_cruise stays 20 m/s** — with the 4412 wing, V_MD ≈ 18.5 m/s and
L/D at 20 m/s is within ~2% of the maximum, the same "near-optimum"
posture the original design card claimed.

The 4412 dominates the trade for this cruise-dominated mission
(config's own guidance: "good if endurance is the priority"): highest
cruise L/D, the smallest wing of any candidate — smaller than even the
pre-fix 2412 wing (b 1.016 vs 1.049 m), which un-does the v0.5.0 span
growth in CAD/structure — and the stall requirement met honestly at
the section's real lift.

## Consequences

- Wing: S 0.2107 → 0.1722 m², b 1.124 → 1.016 m, MAC 0.187 → 0.169 m;
  regression pins, `Allrun.case` (lRef/Aref/CofR), and the CAD exports
  move in the same commit. MTOW (2.303 kg), hover power (652 W), and
  the battery point are unchanged.
- Aileron re-sizes with the wing; the frozen KST servo margin is
  re-checked by the pipeline (requirement scales with the smaller
  chord/arm).
- **Watch — pitching moment**: the 4412's nose-down Cm0 (≈ −0.09 vs
  ≈ −0.05 for the 2412) is not modelled at this stage. Cruise pitch
  trim (vane deflection / CG placement at the 0.05 static margin) must
  be checked in the preliminary phase; this is the price of the camber.
- **Watch — low-Re Cl_max**: the empirical section Cl_max (1.654) is
  least certain exactly where it is now load-bearing, at
  Re ≈ 1.4×10⁵ near stall on a highly cambered section. The zero
  2D→3D headroom MARGIN warning in NB2 remains, by construction, for
  whichever section defines CL_max. Validate the wing's real CL_max
  (bench/flight or a XFOIL/CFD pass at stall Re) before freezing wing
  tooling; if it erodes, the honest response is re-running this
  selection with the measured value, not padding the config.
