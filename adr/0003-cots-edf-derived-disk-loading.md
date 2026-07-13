# 3. COTS 195 mm EDF; disk loading derived, not configured

Status: Accepted — **amended 2026-07-12**: the COTS rotor is a 3-blade
propeller (203 mm) in the airframe duct, not an EDF cartridge; the
derived-disk-loading principle is unchanged.

## Context

The propulsor is the single most design-driving component of an EDF
tail-sitter. Two paths exist: develop a custom fan sized to a chosen disk
loading, or adopt a commercial off-the-shelf (COTS) EDF and let the vehicle
size around it. Custom-fan development is out of scope for a conceptual
university study and would make disk loading a free variable that the sizing
loop could game.

## Decision

- Adopt a **COTS 195 mm EDF** (Schübeler DS-215 class). Rotor diameter is a
  fixed input (`config/rotor.yaml`), not a sizing degree of freedom.
- **Disk loading is derived** (`DL = MTOW·g / A`), never configured — it
  falls out of the converged MTOW and the fixed disk area.

## Consequences

- Hover power and thrust margin follow from a real, purchasable fan, so the
  design point is buildable rather than notional.
- Because disk loading is high for this class, hover is expensive; this
  drives the deliberately short-hover mission profile (see the mission
  design decision) rather than the other way around.
- The parametric prop rotor in CAD is for visualisation and rotating-zone
  CFD only; vehicle aero CFD models the fan as an actuator disk, so the
  external-aero STL deliberately excludes the blades.
- Swapping EDF class is a single-config change that re-converges the whole
  design through the pipeline.

## Amendment (2026-07-12): COTS 3-blade prop in the airframe duct

### Context

The COTS component selection stage (NB11, `config/components/`) put real
hardware against this decision and found that **the 195 mm COTS EDF class
has no light implementation**. The only purchasable unit at that diameter
is the Schübeler DS-215-DIA heavy-lift family — the full HST power unit is
3.4 kg, heavier than the whole aircraft, and even the fan alone (EST
~1 kg) plus the lightest plausible motor lands ~6× over the propulsion
mass allocation (−1069 g at the 2.376 kg design point). The original
decision was buyable only as hardware sized for 20 kg-class jets.

Meanwhile the vehicle never needed an EDF's high disk loading: the design
runs deliberately low disk loading, and the configured rotor solidity
(0.077) always corresponded to a ~3-blade propeller over the exposed
annulus outside the 0.4 R hub — the sizing physics was modelling a prop
all along; only the procurement assumption said "EDF cartridge". A
prop-in-duct is also what the actual V-BAT flies. COTS 3-blade sizes
bracket 195 mm (7" = 178 mm, 8" = 203 mm), so keeping 195 mm would mean a
custom rotor — exactly what this ADR exists to avoid.

### Decision

- The COTS rotor is a **3-blade 8×6-class propeller (203 mm, e.g. Master
  Airscrew 3-blade 8×6, ~23 g)** running in the airframe's own duct (the
  duct was always airframe structure; the exhaust centerbody keeps
  housing the vane servos, the prop mounts to the motor via an adapter).
- `D_rotor_m` moves 0.195 → **0.203** and the whole design re-converges
  through the pipeline (this amendment's commit carries the new pins).
- **Disk loading stays derived** — the principle of this ADR is
  unchanged; only the rotor hardware class moved.
- The `T_max_N` guard is re-derived for the prop-in-duct: momentum-theory
  static thrust at the selected motor's power ceiling
  (`T = (P_shaft · FM · sqrt(2 ρ A))^(2/3)` ≈ 35 N), replacing the
  DS-215-class 50 N.
- Rotor solidity (0.077) and figure of merit (0.70) are retained — both
  are consistent with a 3-blade prop in a duct — but now carry an
  explicit verify-at-procurement note; the empirical RPM law was always a
  propeller law.

### Consequences

- Propulsion mass closes: motor + prop ≈ 306 g against a ~210 g
  allocation is a finding of ~100 g, versus ~1.3 kg of unbuyable
  hardware before. NB11 selects the prop; the DS-215 fan remains in the
  database as the diameter-rejected documentation of this amendment.
- Slightly larger disk (+8% area) lowers hover disk loading and induced
  power; all regression pins, `cfd/vehicle/Allrun.case`, and the `px4/`
  provenance values move in this commit (ADR-0011 discipline).
- The parametric CAD rotor becomes a 3-blade prop
  (`config/prop_geometry.yaml`); `out/cad/` regenerates in CI.
- Duct aero (tip clearance at 203 mm, augmentation) and the prop's
  power limit (~900 W class vs the EDF's kilowatts) are the new risks to
  retire on the thrust stand (ADR-0011 stage a).
