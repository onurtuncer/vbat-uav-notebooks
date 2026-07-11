# px4/ — PX4 flight-stack configuration (ADR-0011)

Pipeline consumer, downstream of `out/` — the same role `cfd/` plays:

```
config/*.yaml -> src/ -> notebooks/ -> out/ -> px4/   (this directory)
```

Everything numeric in here is **copied from `config/` or `out/`
handoffs** and carries a provenance comment naming the source (the
`Allrun.case` discipline). An intentional design change must update
these values in the same commit. A generator script from `out/*.yaml`
is the planned follow-up once the file set stabilises.

Design point (2026-07 re-baseline, ADR-0010): MTOW 2.376 kg, hover
~710 W / 30.3 N, 6S 3.71 Ah, cruise 20 m/s.

## Layout

| Path | What it is |
|---|---|
| `airframes/22100_vbat_tailsitter` | Hardware airframe file (custom start script for the PX4 ROMFS, `init.d/airframes` style). Source of truth for all tunable params. |
| `sitl/airframes/4801_gz_vbat_tailsitter` | SITL (Gazebo `gz`) airframe file, `init.d-posix` style. Param values are kept in sync with the hardware file by hand. |
| `sitl/models/vbat_tailsitter/` | Gazebo model (`model.sdf` + `model.config`): inertia from `out/mass_properties.yaml`, geometry from `out/fuselage.yaml` / `out/control_vanes.yaml`. |

## Vehicle → PX4 mapping

- **Airframe type:** VTOL Tailsitter (`CA_AIRFRAME = 4`, `VT_TYPE = 0`),
  dynamic control allocation (PX4 ≥ v1.14).
- **Motor:** single EDF, `CA_ROTOR_COUNT = 1`, thrust along body +x
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
2. **Vane aerodynamics in SITL:** standard Gazebo lift-drag plugins see
   the freestream, not the EDF jet (`V_jet ≈ 35.7 m/s` at hover per
   `out/control_vanes.yaml` while the vehicle hovers at ~0 airspeed).
   The placeholder plugin blocks in `model.sdf` are marked; a custom gz
   system plugin (or an equivalent hack) is the main SITL work item.
   Until then, SITL hover results are not trustworthy.
3. **Vane effectiveness ∝ thrust** is not modelled by PX4's allocator —
   expect apparent gain variation through transition; tune with the
   ailerons carrying cruise roll as designed.
4. Servo directions/signs must be verified on the bench against the
   FRD axis convention before flight — the matrix here fixes relative
   signs only.

## Usage

**SITL** (Linux, PX4-Autopilot checkout, Gazebo Harmonic):

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
