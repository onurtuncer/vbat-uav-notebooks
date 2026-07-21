# ADR-0017: Propeller blade section (Clark Y) and rotation sense

Date: 2026-07-21
Status: Accepted

## Context

Two related gaps surfaced from Aeolion-side BEMT integration work on the
ADR-0016 geometry handoff:

1. `propulsion_bemt.blade_stations` carried only `r_over_R`/`chord`/`twist`
   — no 2D section shape at all. The CAD rotor (`cad/prop_rotor.py`) built
   its blade from an ad-hoc parametric taper (`camber_M`/`camber_P`/
   `tc_root`/`tc_tip` in `config/prop_geometry.yaml`, described only as
   "NACA 4-digit, thickness tapering root -> tip") — a proprietary shape
   invented for this project, with no published 2D polar data a BEMT
   solver could use. As the user put it: "Most researchers start with a
   well-documented low-Re airfoil rather than a proprietary propeller
   section."
2. `propulsion_bemt` carried no rotation sense at all. Thrust direction,
   torque reaction, and swirl sign in a BEMT/VLM solve all depend on it,
   and there is no safe default (the two options are ~180° apart in
   blade-element angle of attack, not a small ambiguity).

## Decision

### Blade section: Clark Y

Three well-documented low-Re candidates were on the table, each with a
real trade-off:

| section | character | fit for this rotor |
|---|---|---|
| Selig S1223 | extreme camber, very high lift, optimized for one design point (max static thrust) | poor: this prop drives BOTH hover and forward flight as the same blade, not a single hover operating point |
| Eppler E387 | general-purpose low-Re section, extensively wind-tunnel validated (UIUC) | fine, but a wing-proven section more than a propeller-proven one |
| **Clark Y** | robust, moderate camber, the classical propeller section with the deepest track record in propeller BEMT/momentum-theory validation literature | **best fit**: well-behaved across the wide AoA/advance-ratio range a multi-regime (hover + cruise) rotor actually sees, and maximizes comparability with existing validated propeller theory |

**Clark Y wins.** Coordinates are the digitized UIUC Airfoil Coordinates
Database file (`config/airfoils/clarky.dat`, verbatim, Lednicer format,
61+61 points; provenance in the `.dat.source` sidecar — fetched twice
independently, byte-identical, and sanity-checked against the published
characteristics: max t/c 11.71% at x/c=0.28 vs the commonly cited 11.7%,
max camber 3.43% at x/c=0.42 vs the commonly cited ~3.4–3.55%).

The reference section is decomposed into a camber line
`yc = (yu+yl)/2` and a half-thickness distribution `yt = (yu-yl)/2`
(`prop_geometry.ClarkYSection`/`_scaled_clarky_surfaces`), then
thickness-scaled per blade station by `tc_target / tc_ref` — the
standard way a published section is adapted to a locally different
thickness along a span while preserving its camber-line identity (the
same decomposition the wing's own NACA formula already keeps separate).
`config/prop_geometry.yaml`'s `tc_root`/`tc_tip` keys are repurposed as
these scaling targets; `camber_M`/`camber_P` are removed (Clark Y's
camber is now fixed by real data, not a free parameter). One section
law, two consumers: `cad/prop_rotor.py`'s loft and the Aeolion
`propulsion_bemt.airfoil_sections` CST fit (schema 1.7.0).

### Rotation sense

No config or code anywhere in the project stated the prop's rotation
sense — a genuine gap, not an overridden decision. Because the vehicle
carries a single, non-counter-rotating prop, the choice has no
aerodynamic/mass consequence in isolation, EXCEPT that it must be
*consistent with the existing blade twist geometry* — an arbitrary
label would silently contradict the twist law's own implicit handedness.

Verified numerically (not by symbolic sign-chasing, which is too easy
to get wrong here): for the twist law `beta(r) = atan(pitch/(2*pi*r))`,
a rotation vector along **body +x** (right-hand rule) gives every blade
station a small, physically sensible **positive** angle of attack and a
thrust reaction on the vehicle in +x ("forward, out the nose" per the
project's FRD convention), checked at r/R = 0.3/0.6/0.95 against both a
hover-like and a high-advance-ratio axial inflow. The opposite sign
gives an angle of attack near 180° at every station and every flow
regime tested — the blade flying backwards through its own relative
wind, not a subtle error. Equivalently: **clockwise viewed from aft (the
exhaust/−x end) looking forward toward the nose**; counter-clockwise
viewed from the nose looking aft.

This is recorded as `prop_geometry.ROTATION_AXIS_BODY_FRD = (1, 0, 0)`
and exported as `propulsion_bemt.rotation_axis` — the same unit-vector,
right-hand-rule idiom the schema already uses for `hinge_axis`, chosen
specifically to avoid relying on a verbal CW/CCW convention that
requires an assumed viewpoint (the same class of ambiguity ADR-0016's
placement and moment-reference-point fixes eliminated elsewhere in this
contract).

## Consequences

- `cad/prop_rotor.py` blade cross-sections change shape (Clark Y instead
  of the ad-hoc NACA-like taper); `build_prop_rotor` and
  `_blade_section_wire` now take a `ClarkYSection` parameter. `out/cad/`
  regenerates with a visibly different (but still watertight) blade
  profile; geometry tests re-verified against the new export.
- `propulsion_bemt.airfoil_sections` and `.rotation_axis` are **required
  within `propulsion_bemt`** from schema 1.7.0 onward (a
  version-conditional constraint, not merely documented as
  always-present) — the same discipline as the 1.5.0 placement and
  1.6.0 moment-reference-point fixes. `propulsion_bemt` itself remains
  entirely optional at every version.
- `config/prop_geometry.yaml`'s `camber_M`/`camber_P` keys are removed;
  `tc_root`/`tc_tip` are reinterpreted as thickness-scaling targets
  against the Clark Y reference, not NACA thickness-formula inputs.
- No design-point (MTOW/hover power/wing) change — this is a blade
  section/BEMT-fidelity and CAD-consistency fix, not a sizing change.
- Open item for later: this analysis stream (propeller section, BEMT
  contract, rotation sense) has grown enough in its own right that a
  dedicated notebook stage (parallel to `wing_design`/
  `control_vane_design`) may be worth splitting out of NB8's inline
  Aeolion-export block if it keeps growing — noted, not acted on here.
