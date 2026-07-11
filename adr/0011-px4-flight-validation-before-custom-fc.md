# 11. Fly on PX4 + COTS Pixhawk before a custom flight controller

Status: Accepted

## Context

The long-term goal includes a custom flight-control card. Designing that
card now would mean guessing its requirements: sensor grades, loop rates,
filtering needs, output channel count, power rails. Every one of those is
measurable by first flying the vehicle on a proven COTS stack — and the
vehicle itself (transition behaviour, vane authority in gusts, hover
power) is unproven, so early flights should not also carry avionics risk.

PX4 is the natural stack: it has a supported **VTOL Tailsitter** type and,
since v1.14, **dynamic control allocation** that accepts a custom actuator
geometry. That matters because this vehicle is non-standard for PX4: a
single EDF with **all hover attitude authority from four jet vanes** — no
differential thrust — plus ailerons as the cruise-phase roll effector
(ADR-0004). No stock airframe matches; a custom actuator configuration is
required either way.

Two vehicle-specific risks shape the plan:

- **Vane effectiveness scales with jet dynamic pressure** (`q_jet` ∝
  thrust). PX4's allocator treats control-surface effectiveness as
  constant, so through transition the vanes' true authority collapses
  while the allocator's model doesn't — exactly the regime where the
  ailerons take over (ADR-0004). This is a tuning risk, not a blocker,
  but it must be characterised in SITL before hardware flies.
- **Standard Gazebo aero plugins model surfaces in the freestream**, not
  in a jet. A custom (or carefully abused) plugin is needed for the vanes
  in SITL; this is the main SITL work item.

## Decision

1. **First flight is on PX4 running on a COTS Pixhawk-class FC** (Pixhawk
   6C / Cube class), with COTS GPS/compass, digital airspeed sensor,
   telemetry, power module, and an ESC with RPM/current telemetry. The FC
   and sensors live inside the existing avionics budget and on the
   vibration-isolated mount (ADR-0007); no mass-model change.

2. **`px4/` is a new pipeline consumer, downstream of `out/` — exactly
   like `cfd/`** (ADR-0001). It holds the custom airframe/actuator
   configuration and the Gazebo SITL model. Every number in it is copied
   from `config/` or `out/` handoffs and carries a provenance comment
   naming its source, mirroring how `Allrun.case` hardcodes `lRef`/
   `Aref`/`CofR`. Consequently the same discipline applies: **an
   intentional design change must update the `px4/` values in the same
   commit** (a generator script from `out/*.yaml` is the acknowledged
   follow-up once the file set stabilises).

3. **SITL gates hardware.** Order of proof in simulation: vane-only hover
   attitude control → hover position hold → back-transition →
   front-transition → cruise with aileron roll. No hardware flight before
   transitions work in SITL.

4. **Staged flight-test campaign**, each stage feeding measured data back
   against notebook predictions: (a) bench — servo signs against the vane
   mixing matrix, thrust stand, IMU vibration survey against the ~211 Hz
   1/rev forcing (`out/vibration.yaml`); (b) tethered/low hover — rate
   tuning, hover power vs the predicted ~710 W; (c) free hover envelope;
   (d) transitions at altitude; (e) cruise — aileron authority vs
   `ddot_min_deg_s2`, cruise current vs prediction.

5. **The custom FC card is specified from flight logs, not estimates.**
   ulog data (sensor PSDs, loop timing, actuator usage, power) becomes
   the requirements document. The custom card's bring-up target is
   **running PX4 itself** (it is designed to be ported) before any custom
   firmware is considered — the card is a hardware project first, not a
   firmware project.

## Consequences

- New top-level `px4/` directory (README, hardware airframe file, SITL
  airframe file, Gazebo model). Like `cfd/`, its scripts run on
  Linux/NuttX — `.gitattributes` forces LF.
- A second set of hardcoded design-point values now exists outside
  `out/`. Until a generator exists, drift is guarded only by the
  provenance comments and commit discipline; adding `px4/` cross-checks
  to `tests/test_design_outputs.py` (as done for `Allrun.case`) is the
  right follow-up once param names are validated against a pinned PX4
  release.
- The vane jet-wash SITL plugin is accepted as real work; until it
  exists, SITL transition results carry an asterisk.
- BEC current budgets and connector choices in `config/electrical.yaml`
  stop being placeholders once the COTS hardware list is frozen —
  expect a config update (and possibly a wiring-diagram regen) at that
  point.
- Flight data may move pinned design values (hover power, vane margins).
  That is the point: measured corrections flow back into `config/` and
  re-converge the pipeline, not into `px4/` locally.
