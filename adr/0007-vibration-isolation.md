# 7. Soft-mount the FC/IMU and payload against EDF imbalance

Status: Accepted

## Context

The single EDF is a high-RPM rotating machine bolted to a stiff CFRP
monocoque. Its dominant forcing is the 1-per-rev shaft imbalance at the shaft
frequency (~211 Hz at the hover design point, derived from rotor RPM via
`rpm_from_diameter`); blade-pass (×11) is far higher. That 1/rev line
corrupts the IMU attitude estimate and blurs the EO payload. Until now the
FC/IMU and payload were rigid internal bays with no isolation modeled.

The isolators cost mass and, because they need rattle space, packing volume —
and the fuselage is packaging-constrained at 100% bay-stack utilization, so
any volume demand shows up in the geometry. This is a conceptual-stage
decision precisely because that volume is cheap to reserve now and painful to
retrofit.

## Decision

- Soft-mount the **FC/IMU cluster** and the **payload** each as a 1-DOF
  base-excitation isolator, sized in `vibration_isolation.py` (new NB5).
- Size the isolator corner frequency `f_n` as high (stiffest, least sway) as
  still meets a target transmissibility at the 1/rev forcing, and require it
  to sit in a valid window: above the flight-control bandwidth (a mount
  softer than the control loop would let the FC chase its own sway) and below
  `f_forcing/√2` (the isolation threshold).
- Feed the sway/rattle space into the fuselage bay stack (`sway_pad_m`) and
  carve the isolator hardware from the fractions that document it — FC-tray
  isolators from avionics, payload-mount isolators from structure — using the
  same carve-out/guard machinery as the vane and aileron hardware
  (ADR-0005).
- Model both soft-mounted modules in the CAD assembly (isolator standoffs +
  rattle gap) as **assembly-only** internal parts — not fused into the
  external-aero STL, following the prop-rotor precedent.

## Consequences

- Because the forcing is far above any practical corner frequency, the sized
  mount (f_n ≈ 58 Hz, ~90% attenuation of the 1/rev) needs only ~0.1 mm
  static deflection and a few mm of sway — the packing cost is small, and in
  fact the fuselage shrank marginally (the isolator mass carved out of
  avionics reduces packed volume more than the sway pad adds length).
- The FC/IMU isolated mass (`m_fc_imu_kg`) is a config parameter: only the
  vibration-sensitive electronics are soft-mounted, not the whole avionics
  fraction.
- `out/vibration.yaml` becomes a pipeline handoff; `fuselage_design`,
  `mass_properties`, and `vehicle_solid_model` all consume it.
- The transmissibility model is 1-DOF and linear; real multi-axis coupling,
  resonance dwell on spin-up, and isolator nonlinearity are out of scope at
  the conceptual stage.
