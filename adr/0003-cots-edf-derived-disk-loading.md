# 3. COTS 195 mm EDF; disk loading derived, not configured

Status: Accepted

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
