# ADR-0014: Battery pack mission-transient thermal check

Date: 2026-07-17
Status: Accepted

## Context

ADR-0009 gave the airframe its two structural heat paths: the ESC
cold-plate and the vented battery bay. The battery-side model is a
steady-state **air-side capacity check** — a through-flow enthalpy
balance (`Q_batt = mdot·cp·dT_air`) that sizes the vents to carry the
heat away. It answers "can the bay reject the load" (yes, with ~40x
area headroom) but says nothing about how hot the **pack itself** gets,
because it has no thermal mass and no mission time axis.

An external battery-thermal review (C. Ucler, JETTAIL battery thermal
analysis, 2026-07) modelled exactly that: a lumped-capacitance pack
integrating its Joule heat `I²·R_pack` through two hover legs with
cruise cooling in between. The review's arithmetic was verified
line-by-line and is correct; its inputs predate the current design
point (2.60 kg / 900 W / the pre-freeze 554.6 g sized-LiPo mass), but
re-running the model at the frozen point changes the conclusion very
little — the halved heat rate is almost exactly offset by the lighter
pack's smaller thermal capacity plus the transition legs the review
omitted.

Two structural gaps in our model surfaced:

1. **`eta_bat = 0.97` implies a ~23 mOhm pack** (20 W loss at ~29 A
   hover). A real 6S1P 21700 pack is ~60–120 mOhm at the terminals
   (P50B DCIR ~10–13 mOhm/cell, plus welds, leads, connector). The
   nominal 90 mOhm case puts the hover battery loss at ~78 W — 4x the
   vent-model load.
2. **No transient pack model existed.** The pack's thermal time
   constant (~30 min against cruise cooling) is longer than the cruise
   leg, so the second vertical leg starts from an elevated temperature.
   At the ADR-0009 design ambient of 40 °C (hot day), the nominal-
   resistance case ends the mission at ~62 °C average — above the
   60 °C pack limit — before the review's 5–10 °C hot-spot allowance.
   The review itself used a 16 °C ambient, which masked this.

## Decision

Add the review's lumped-capacitance mission transient to
`thermal_design.py` (`battery_pack_transient`) and run it in NB7 as a
**reported, never filtered-on** check — the same discipline as the
ESC cold-plate and the COTS budget lines:

- Heat loads are derived: `I²·R_pack` at the wiring-law currents
  (`electrical_diagram.compute_operating_point`, the single call site
  for the current law). Only the resistance cases, pack specific heat,
  envelope estimate, and exposed-surface fraction are configured
  (`config/thermal.yaml`), each with a rationale.
- Mission structure comes from `config/mission.yaml`: two vertical
  legs of `t_hover/2 + t_transition/2` each (transitions billed at
  hover power, the mass-closure convention), adiabatic (hover bay
  airflow conservatively neglected), separated by `t_cruise` of
  laminar flat-plate cooling over `pack_exposed_frac` of the envelope
  at `V_cruise`.
- Three pack-resistance cases (optimistic 60 / nominal 90 /
  conservative 120 mOhm) until the built pack's DCIR is measured at
  procurement, after which the cases collapse to the measured value.
- Results land in `out/thermal.yaml` under `battery_transient:`;
  `design_summary` picks the nominal case up as a margins row and,
  while it exceeds the limit, as a standing finding. The regression
  tests pin the state (nominal FAILS, optimistic passes).

The steady-state vent model, its `Q_batt`, and the vent geometry
handed to CAD are **unchanged** — no back-edge into ADR-0009 outputs.

`eta_bat` in the mass closure is also unchanged for now: mission
energy is cruise-dominated, so the closure impact of the optimistic
hover efficiency is a few percent, and folding a measured value in is
a deliberate next-iteration change (it moves the design point and the
regression pins). The gap is documented in `config/battery.yaml`.

## Consequences

- New standing finding at the current design point: **the nominal
  90 mOhm pack ends the mission at ~62 °C average vs the 60 °C limit
  at 40 °C ambient** (optimistic 60 mOhm: ~55 °C, passes;
  conservative 120 mOhm: ~70 °C, fails clearly). Levers, in order of
  leverage: build/buy a low-resistance pack (measure DCIR; thick
  nickel+copper busbars on the in-country spot-welded build), duct
  part of the hover vent flow directly over the pack (the legs are
  modelled adiabatic — any hover-side convection is pure margin), or
  restrict operations at high ambient.
- The ground test of the review's §13 becomes the procurement
  acceptance step: run the mission current profile, measure pack DCIR
  and cell temperatures, then collapse `R_pack_mohm` to the measured
  value and re-pin.
- The model is a spatial average; cell cores and busbars run 5–10 °C
  hotter until validated by test.
