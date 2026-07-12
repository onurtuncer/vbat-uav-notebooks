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
- **The built-in simulator cannot be trusted for this vehicle.** Stock
  Gazebo aero plugins are point lift-drag models in the freestream: no
  jet wash on the vanes, no post-stall wing behaviour, no power-on
  duct/airframe interaction — precisely the physics that decide whether
  a tailsitter transition works. Patching a vane plugin into that would
  fix one hole in a model that stays untrustworthy everywhere SITL is
  supposed to de-risk.

The assets for a trustworthy plant already exist: the OpenFOAM vehicle
case (`cfd/`) for the aerodynamics; **Aetherion**
(github.com/onurtuncer/Aetherion), a NASA-checkcase-validated 6-DoF
Featherstone-SVA flight-dynamics library in C++ with a DAVE-ML 2.0
(AIAA S-119) model loader (Janus-derived, CppAD-friendly); and DAVE-ML
itself as the tool-neutral exchange format between the two.

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

3. **The SITL physics plant is Aetherion driven by CFD-derived DAVE-ML
   models — not Gazebo aero.**
   - The pipeline gains an aero-database leg: parametric OpenFOAM sweeps
     (`cfd/`, production runs on the user's hardware per the CI rule) →
     coefficient/force tables → generated `out/daveml/*.dml`
     (spec: `px4/sitl/daveml_spec.md`), with the same provenance and
     regeneration discipline as every other `out/` artifact, including
     S-119 embedded check-cases so any compliant reader can verify the
     tables round-trip.
   - Aetherion loads them through its `Serialization/DAVEML` readers and
     exposes the plant to PX4 over the simulator-MAVLink **lockstep**
     interface (TCP 4560: `HIL_ACTUATOR_CONTROLS` in;
     `HIL_SENSOR`/`HIL_GPS`/… out). The bridge, sensor models, V-BAT
     plant assembly, and control-law design live in a dedicated
     flight-control repository (planned, unnamed yet) that consumes
     Aetherion as a C++ library — Aetherion itself stays
     vehicle-agnostic, and none of it lives here.
   - The Gazebo model (`px4/sitl/models/`) is **demoted to PX4-side
     integration smoke checks and visualisation**; no flight-dynamics
     conclusion may be drawn from it.

4. **SITL gates hardware.** Order of proof on the Aetherion plant:
   vane-only hover attitude control → hover position hold →
   back-transition → front-transition → cruise with aileron roll. No
   hardware flight before transitions work in SITL.

5. **Staged flight-test campaign**, each stage feeding measured data back
   against notebook predictions: (a) bench — servo signs against the vane
   mixing matrix, thrust stand, IMU vibration survey against the ~211 Hz
   1/rev forcing (`out/vibration.yaml`); (b) tethered/low hover — rate
   tuning, hover power vs the predicted ~710 W; (c) free hover envelope;
   (d) transitions at altitude; (e) cruise — aileron authority vs
   `ddot_min_deg_s2`, cruise current vs prediction.

6. **The custom FC card is specified from flight logs, not estimates.**
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
- The trusted-plant decision creates four named work items:
  (a) a parametric CFD sweep runner + post-processing to force/coefficient
  tables (this repo, `cfd/`); (b) a DAVE-ML generator writing
  `out/daveml/*.dml` per `px4/sitl/daveml_spec.md` (this repo, new
  pipeline stage); (c) the dedicated V-BAT flight-control repo — plant
  assembly on Aetherion, PX4 lockstep bridge + sensor models
  (IMU/mag/baro/GPS with noise, optionally the 211 Hz 1/rev line),
  trim/linearization, control-law design; (d) V-BAT trim/response
  check-cases in Aetherion's validation style. Until (a)–(c) exist,
  SITL results carry an asterisk. The flight-control repo consumes
  tagged release snapshots of this repo (never a working tree) and
  seeds the `px4/` gain parameters here, citing its own release tag.
- Aetherion's typed `DAVEMLAeroModel::Inputs` is F-16-shaped (el/ail/rdr);
  the V-BAT model (4 vane deflections + jet state) is driven through the
  generic `evaluateRaw` map interface, or a typed V-BAT policy is added
  Aetherion-side — either way, no change to this repo's handoffs.
- In hover `q_∞ → 0`, so classic coefficient normalisation blows up:
  the airframe table is coefficient-form (valid above a few m/s) while
  the prop/duct and vane models are **dimensional** (N, N·m) functions of
  throttle/jet state — the split is part of the DAVE-ML spec, not an
  implementation detail.
- BEC current budgets and connector choices in `config/electrical.yaml`
  stop being placeholders once the COTS hardware list is frozen —
  expect a config update (and possibly a wiring-diagram regen) at that
  point.
- Flight data may move pinned design values (hover power, vane margins).
  That is the point: measured corrections flow back into `config/` and
  re-converge the pipeline, not into `px4/` locally.
