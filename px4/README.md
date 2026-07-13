# px4/ — PX4 flight-stack configuration (ADR-0011)

Pipeline consumer, downstream of `out/` — the same role `cfd/` plays:

```
config/*.yaml -> src/ -> notebooks/ -> out/ -> px4/   (this directory)
                                        |
                                        v
                              cfd/ sweep results
                                        |
                                        v
                     out/daveml/*.dml  (generated, AIAA S-119)
                                        |
                                        v
                    Aetherion 6-DoF plant (separate repo)
                                        |
                          simulator-MAVLink lockstep (TCP 4560)
                                        |
                                        v
                                   PX4 SITL
```

The SITL physics plant is **Aetherion** (github.com/onurtuncer/Aetherion)
consuming DAVE-ML models generated from this repo's CFD results — the
built-in Gazebo aero is not trusted for this vehicle (no jet wash, no
post-stall, no power-on effects; see ADR-0011). The DAVE-ML contract
lives in [sitl/daveml_spec.md](sitl/daveml_spec.md).

Everything numeric in here is **copied from `config/` or `out/`
handoffs** and carries a provenance comment naming the source (the
`Allrun.case` discipline). An intentional design change must update
these values in the same commit. A generator script from `out/*.yaml`
is the planned follow-up once the file set stabilises.

Design point (ADR-0003 prop-in-duct amendment, 2026-07-12): MTOW
2.302 kg, hover ~652 W / 29.4 N (1.3 T/W), 6S 3.50 Ah, cruise 20 m/s.

## Layout

| Path | What it is |
|---|---|
| `airframes/22100_vbat_tailsitter` | Hardware airframe file (custom start script for the PX4 ROMFS, `init.d/airframes` style). Source of truth for all tunable params. |
| `sitl/daveml_spec.md` | Contract for the CFD-derived DAVE-ML model set (`out/daveml/*.dml`) consumed by Aetherion — grids, frames, check-cases, validation gates. |
| `sitl/airframes/4801_gz_vbat_tailsitter` | SITL (Gazebo `gz`) airframe file, `init.d-posix` style. Param values are kept in sync with the hardware file by hand. |
| `sitl/models/vbat_tailsitter/` | Gazebo model (`model.sdf` + `model.config`) — **integration smoke checks and visualisation only**, not a trusted plant. Inertia from `out/mass_properties.yaml`, geometry from `out/fuselage.yaml` / `out/control_vanes.yaml`. |

The PX4 lockstep bridge and sensor models live in the Aetherion
repository; this repo only produces the `.dml` inputs and the PX4
parameter set.

## Vehicle → PX4 mapping

- **Airframe type:** VTOL Tailsitter (`CA_AIRFRAME = 4`, `VT_TYPE = 0`),
  dynamic control allocation (PX4 ≥ v1.14).
- **Motor:** single ducted 3-blade prop, `CA_ROTOR_COUNT = 1`, thrust along body +x
  (out the nose, FRD).
- **Jet vanes:** four custom control surfaces (CS0–CS3 = T/B/L/R). Their
  roll/pitch/yaw torque entries transcribe the mixing matrix in
  `out/control_vanes.yaml` (rows T,B,L,R × cols pitch,yaw,roll).
- **Ailerons:** CS4/CS5 as left/right aileron — the cruise-phase roll
  effector (ADR-0004); jet vanes remain available as roll backup.
- **Outputs:** 1 ESC (DShot preferred — RPM telemetry feeds the dynamic
  notch filter) + 6 servos.

## Known limitations / open work

1. **Param names are drafted against PX4 v1.15 docs and not yet
   validated against a pinned release.** Validate before first SITL run;
   then add cross-checks to `tests/test_design_outputs.py`.
2. **The trusted SITL plant does not exist yet.** Its pieces are named
   work items (ADR-0011): the parametric CFD sweep runner and the
   DAVE-ML generator in this repo, the PX4 lockstep bridge + sensor
   models in Aetherion. The Gazebo model remains usable for PX4-side
   plumbing checks (actuator mapping, mode logic, parameter sanity) but
   its vane aero blocks are physically meaningless placeholders — the
   plugins see the freestream, not the rotor jet — so no flight-dynamics
   conclusion may be drawn from it.
3. **Vane effectiveness ∝ thrust** is not modelled by PX4's allocator —
   expect apparent gain variation through transition; tune with the
   ailerons carrying cruise roll as designed.
4. Servo directions/signs must be verified on the bench against the
   FRD axis convention before flight — the matrix here fixes relative
   signs only.

## Usage

**SITL against Aetherion (the trusted path, once the bridge exists):**
build the Aetherion PX4 bridge pointing at `out/daveml/`, start PX4 with
no built-in simulator (`make px4_sitl none_vbat_tailsitter` — external
sim connects on TCP 4560 in lockstep), then launch the bridge. Exact
invocation is defined by the bridge in the Aetherion repo.

**Gazebo smoke run** (PX4-side plumbing checks only — not flight
dynamics; Linux, PX4-Autopilot checkout, Gazebo Harmonic):

```sh
# make the model and airframe visible to the PX4 build
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/path/to/vbat-uav-notebooks/px4/sitl/models
cp px4/sitl/airframes/4801_gz_vbat_tailsitter \
   PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
# add it to that directory's CMakeLists.txt, then:
make px4_sitl gz_vbat_tailsitter
```

**Hardware:** copy `airframes/22100_vbat_tailsitter` into
`ROMFS/px4fmu_common/init.d/airframes/` of a PX4 source build (register
it in the CMakeLists), or replay the same `param set-default` lines as a
QGroundControl parameter import after selecting a generic VTOL
tailsitter airframe.
