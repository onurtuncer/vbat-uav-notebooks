# 4. Dual roll control: jet vanes (hover) + ailerons (cruise)

Status: Accepted

## Context

Jet control vanes in the EDF exhaust provide pitch/yaw/roll for the
tail-sitter and are sized from **hover** thrust. Vane authority is driven by
jet dynamic pressure, `q_jet = T / A_disk` (actuator-disk momentum theory),
which scales linearly with thrust. In wing-borne cruise the EDF only needs to
overcome drag, so cruise thrust is roughly `L/D⁻¹` of hover thrust (~6% at
this design point). Vane roll authority therefore collapses from a large
hover margin to a thin cruise margin over the long (~900 s) cruise leg — a
previously unexamined risk.

## Decision

- Keep **jet vanes primary for all three axes in hover and transition**,
  where aerodynamic surfaces are ineffective.
- Add **ailerons as the primary roll actuator in cruise**. Aileron authority
  is driven by *wing* dynamic pressure, independent of EDF throttle, so it is
  strong exactly where vane authority is weak.
- Vanes remain available as a cruise roll backup; nothing is removed.
- Both `control_vane_design` and `aileron_design` check against a single
  shared requirement, `ddot_min_deg_s2` in `config/aerodynamics.yaml`.

## Consequences

- Adds a fourth pipeline stage (`aileron_design`, NB4) and its own
  config/handoff; fuselage/CAD/mass/wiring shift down one number.
- Ailerons are sized deliberately small (12% span / 12% chord): textbook GA
  proportions would give a >100× authority margin and an unrealistic servo
  torque at this vehicle's cruise q and wing arm.
- Introduces two more servos into the mass budget, CAD, and wiring diagram;
  aileron servos mount as lower-surface pods because the thin wing cannot
  bury them at the hinge.
- Roll is now a two-actuator system across the flight envelope; any future
  control-allocation/mixing work must account for the hover→cruise handoff.
