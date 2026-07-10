# 9. Thermal paths as structure

Status: Accepted

## Context

The airframe carries two continuous heat sources in hover — the ESC
(~50 W switching/conduction loss) and the battery (~30 W pack IR loss) —
and had no thermal model at all (`docs/assumptions.md` listed "no thermal
limits"). The ESC was a lumped mass with a bare "cooling air is available"
comment; the battery bay was a sealed packed cylinder.

Heat-rejection paths are cheap to plan into the current geometry and
expensive to retrofit once the shell is frozen: the ESC already sits
mid-body in the fan's inflow field, and the battery-bay wall is free to
vent. The decision is to size both **now**, as structure that also does
thermal management.

## Decision

1. **Model both paths as structure, first-order** (new `thermal_design`
   stage, NB7). Heat loads are **derived**, not configured:
   `Q_esc = P_hover·(1 − eta_esc)`, `Q_batt = P_hover·(1/eta_bat − 1)`.
   Cooling airflow is a configured fraction of the fan induced velocity
   `v_h = sqrt(W / (2 ρ A_disk))`.
   - **ESC cold-plate**: flat-plate forced convection off a plate on the
     inflow-washed inner wall. Square-plate Nusselt closes in one step:
     `A_req = (Q_esc / (C_h · ΔT))^(4/3)`.
   - **Vented battery bay**: through-flow energy balance
     `Q_batt = ρ V_vent A_vent cp ΔT_air` → required vent area.

2. **No mass re-baseline.** The ESC cold-plate is absorbed into the
   existing ESC/propulsion "mounts" allocation (the propulsion fraction
   already covers "motor + ESC + EDF fan + mounts"); the vents remove a
   little skin. `thermal_design` guards that the plate fits inside that
   allocation rather than carving new mass. The design point stays 3.06 kg.

3. **CAD parts are assembly-only** (excluded from the fused external-aero
   STL, like the spar tube and soft-mount trays): an `esc_cold_plate`
   wall segment and a `battery_vents` louvre bank. The external-aero model
   treats the skin as closed — consistent with the actuator-disk fan
   convention — so the fused STL, CFD case, and geometry pins are unchanged.

4. **Report margins, don't mask them.** Where a path is tight, surface it
   as a finding rather than forcing a pass.

## Consequences

- **Battery bay: comfortable** — ~2 cm² of vent needed against ~90 cm² of
  bay wall (~40× headroom), ~20 °C margin at 40 °C ambient. Genuinely free
  in the current geometry.
- **ESC cold-plate: marginal, and it breaks assumption 2's premise** — at
  the 3.06 kg / ~1 kW-hover point the ~50 W ESC loss needs a ~350 cm²
  (~186 mm) plate that masses ~190 g, **exceeding the ~115 g ESC
  allocation**, with only a few °C margin cooling mid-body at half the
  induced velocity. `out/thermal.yaml` records `ok: false` for the ESC
  path; the notebook prints the finding and the levers (a higher-efficiency
  ESC drops the load to ~20 W; the ESC nearer the inlet cools better at a
  CG cost; a finned spreader; or a larger propulsion fraction). This is a
  standing design finding for the next pass, deliberately not resolved by
  tuning config to a false pass.
- Zero churn elsewhere: no mass move, no CG shift, no CFD/geometry-pin
  change. `TestThermal` pins the (marginal) ESC state and the (healthy)
  battery state so a future drift in either is caught.
