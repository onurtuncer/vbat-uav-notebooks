"""
vibration_isolation.py  --  Soft-Mount Sizing for the FC/IMU and Payload
=========================================================================

Sizes the vibration isolators that soft-mount the flight-controller/IMU
cluster and the EO payload against the single EDF's rotating imbalance.

MOTIVATION
----------
The EDF is a high-RPM rotating machine on a stiff CFRP monocoque. Its
dominant forcing is the 1-per-rev shaft imbalance at the shaft frequency
(f_shaft = RPM/60; ~211 Hz at the hover design point); blade-pass
(n_blades x f_shaft) is far higher. That 1/rev line corrupts the IMU
attitude estimate and blurs the EO payload. Each sensitive module is soft-
mounted so its transmissibility at the forcing frequency is small.

THEORY  (1-DOF base-excitation isolator)
----------------------------------------
A rigid module of mass m on isolators of total stiffness k (damping ratio
zeta) has undamped natural (corner) frequency and frequency ratio

    f_n = (1 / 2 pi) sqrt(k / m),      r = f_forcing / f_n

Its steady-state transmissibility (transmitted / input, force or motion) is

    T(r, zeta) = sqrt( (1 + (2 zeta r)^2) / ((1 - r^2)^2 + (2 zeta r)^2) )

T = 1 at r = sqrt(2) (isolation threshold) and falls monotonically for
r > sqrt(2). We size the corner frequency as high (stiffest, least sway)
as still meets a target transmissibility at the forcing frequency:

    r_req = min r > sqrt(2) with T(r, zeta) <= T_target
    f_n   = f_forcing / r_req

The static deflection (1 g preload sag) and the sway/rattle space to stroke
a maneuver/landing shock without bottoming follow from f_n alone:

    omega_n = 2 pi f_n
    delta_static = g / omega_n^2
    sway         = shock_g * delta_static        (linear spring)

f_n must sit ABOVE the flight-control bandwidth (a mount softer than the
control loop would let the FC chase its own sway) and BELOW f_forcing /
sqrt(2) (the isolation threshold) -- reported as a valid-window check.

Because f_n depends only on the forcing frequency, target, and damping
(not on mass), both modules share f_n, delta_static and sway; only the
isolator stiffness (proportional to mass) differs.

References
----------
  Rao, "Mechanical Vibrations", ch. 9 (base excitation, transmissibility).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import yaml


G0 = 9.80665   # standard gravity [m/s^2]


def transmissibility(r: float, zeta: float) -> float:
    """1-DOF base-excitation transmissibility at frequency ratio r."""
    num = 1.0 + (2.0 * zeta * r) ** 2
    den = (1.0 - r * r) ** 2 + (2.0 * zeta * r) ** 2
    return math.sqrt(num / den)


def _required_ratio(t_target: float, zeta: float) -> float:
    """
    Smallest frequency ratio r > sqrt(2) whose transmissibility meets
    t_target.  T = 1 at r = sqrt(2) and decreases monotonically beyond,
    so a simple bisection on r brackets the target.
    """
    if not 0.0 < t_target < 1.0:
        raise ValueError("target_transmissibility must be in (0, 1)")
    lo = math.sqrt(2.0) + 1e-6
    hi = lo
    # expand hi until transmissibility drops below the target
    for _ in range(200):
        if transmissibility(hi, zeta) <= t_target:
            break
        hi *= 1.5
    else:
        raise ValueError("could not bracket the target transmissibility")
    for _ in range(100):
        mid = 0.5 * (lo + hi)
        if transmissibility(mid, zeta) > t_target:
            lo = mid
        else:
            hi = mid
    return hi


# ---------------------------------------------
#  Input parameters (config/vibration.yaml)
# ---------------------------------------------
@dataclass
class VibrationParams:
    target_transmissibility: float   # [-]
    damping_ratio:           float   # zeta [-]
    fc_bandwidth_hz:         float   # lower f_n bound [Hz]
    shock_load_g:            float   # sway sizing [g]
    m_fc_imu_kg:             float   # isolated FC/IMU electronics [kg]
    n_isolators_fc:          int     # [-]
    n_isolators_payload:     int     # [-]
    isolator_mass_kg_each:   float   # [kg]

    @classmethod
    def from_yaml(cls, path) -> "VibrationParams":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(
            target_transmissibility = float(data["target_transmissibility"]),
            damping_ratio           = float(data["damping_ratio"]),
            fc_bandwidth_hz         = float(data["fc_bandwidth_hz"]),
            shock_load_g            = float(data["shock_load_g"]),
            m_fc_imu_kg             = float(data["m_fc_imu_kg"]),
            n_isolators_fc          = int(data["n_isolators_fc"]),
            n_isolators_payload     = int(data["n_isolators_payload"]),
            isolator_mass_kg_each   = float(data["isolator_mass_kg_each"]),
        )


# ---------------------------------------------
#  Per-module sizing output
# ---------------------------------------------
@dataclass
class IsolationResult:
    name:            str
    m_isolated_kg:   float
    n_isolators:     int
    f_n_hz:          float   # isolator corner frequency
    r_ratio:         float   # f_forcing / f_n
    transmissibility: float  # achieved at the forcing frequency
    attenuation_pct: float   # 100 * (1 - T)
    delta_static_mm: float   # 1 g preload sag
    sway_mm:         float   # rattle space for shock_load_g
    k_isolator_N_m:  float   # per-isolator stiffness
    hw_mass_kg:      float   # total isolator hardware for this module


# ---------------------------------------------
#  Main sizing routine
# ---------------------------------------------

def size_isolation(
    rpm:          float,   # EDF shaft speed [rev/min] (rpm_from_diameter)
    n_blades:     int,     # fan blade count (config/prop_geometry.yaml)
    m_payload_kg: float,   # mission payload, soft-mounted whole [kg]
    p:            VibrationParams = None,
    g:            float = G0,
) -> dict:
    """
    Size both isolated modules against the 1/rev EDF forcing.

    Returns a dict with the forcing frequencies, the shared corner-frequency
    solution, and one IsolationResult per module (FC/IMU, payload).
    """
    if p is None:
        raise ValueError("VibrationParams required (config/vibration.yaml)")

    f_shaft = rpm / 60.0                 # 1/rev forcing [Hz] -- design driver
    f_blade = f_shaft * n_blades         # blade-pass [Hz] -- easier to isolate

    # corner frequency: as stiff as still meets the target at f_shaft
    r_req = _required_ratio(p.target_transmissibility, p.damping_ratio)
    f_n = f_shaft / r_req
    omega_n = 2.0 * math.pi * f_n

    delta_static = g / (omega_n ** 2)          # [m]
    sway = p.shock_load_g * delta_static        # [m]

    # valid window: above the control bandwidth, below the isolation threshold
    f_n_upper = f_shaft / math.sqrt(2.0)
    window_ok = p.fc_bandwidth_hz < f_n < f_n_upper

    def _module(name, m, n_iso, hw_mass) -> IsolationResult:
        k_total = (omega_n ** 2) * m
        return IsolationResult(
            name=name, m_isolated_kg=m, n_isolators=n_iso,
            f_n_hz=f_n, r_ratio=r_req,
            transmissibility=transmissibility(r_req, p.damping_ratio),
            attenuation_pct=100.0 * (1.0 - transmissibility(r_req, p.damping_ratio)),
            delta_static_mm=delta_static * 1e3, sway_mm=sway * 1e3,
            k_isolator_N_m=k_total / n_iso,
            hw_mass_kg=n_iso * p.isolator_mass_kg_each,
        )

    fc = _module("fc_imu", p.m_fc_imu_kg, p.n_isolators_fc,
                 p.n_isolators_fc * p.isolator_mass_kg_each)
    pl = _module("payload", m_payload_kg, p.n_isolators_payload,
                 p.n_isolators_payload * p.isolator_mass_kg_each)

    return {
        "f_shaft_hz": f_shaft,
        "f_blade_hz": f_blade,
        "n_blades": n_blades,
        "f_n_hz": f_n,
        "f_n_window_hz": (p.fc_bandwidth_hz, f_n_upper),
        "window_ok": window_ok,
        "sway_mm": sway * 1e3,
        "modules": {"fc_imu": fc, "payload": pl},
    }


# ---------------------------------------------
#  Handoff writer
# ---------------------------------------------

def write_vibration_yaml(res: dict, p: VibrationParams, path) -> None:
    """Write out/vibration.yaml -- consumed by fuselage_design (sway pad +
    isolator hardware mass) and read for reference downstream."""
    def _mod(m: IsolationResult) -> dict:
        return {
            "m_isolated_kg":     round(m.m_isolated_kg, 5),
            "n_isolators":       m.n_isolators,
            "f_n_hz":            round(m.f_n_hz, 3),
            "r_ratio":           round(m.r_ratio, 4),
            "transmissibility":  round(m.transmissibility, 5),
            "attenuation_pct":   round(m.attenuation_pct, 2),
            "delta_static_mm":   round(m.delta_static_mm, 4),
            "sway_mm":           round(m.sway_mm, 4),
            "k_isolator_N_m":    round(m.k_isolator_N_m, 2),
            "hw_mass_kg":        round(m.hw_mass_kg, 5),
        }

    fc = res["modules"]["fc_imu"]
    pl = res["modules"]["payload"]
    data = {
        "target_transmissibility": p.target_transmissibility,
        "damping_ratio":           p.damping_ratio,
        "f_shaft_hz":              round(res["f_shaft_hz"], 3),
        "f_blade_hz":              round(res["f_blade_hz"], 3),
        "n_blades":                res["n_blades"],
        "f_n_hz":                  round(res["f_n_hz"], 3),
        "f_n_window_hz":           [round(v, 3) for v in res["f_n_window_hz"]],
        "window_ok":               bool(res["window_ok"]),
        "sway_mm":                 round(res["sway_mm"], 4),
        # sway padding fed to the fuselage bay stack: one clearance per
        # isolated bay (FC/IMU bay + payload bay)
        "sway_pad_total_m":        round(2.0 * res["sway_mm"] / 1e3, 5),
        # isolator hardware, carved from the mass budget by fuselage_design:
        # FC-tray isolators from avionics, payload isolators from structure
        "m_isolation_avionics_kg": round(fc.hw_mass_kg, 5),
        "m_isolation_struct_kg":   round(pl.hw_mass_kg, 5),
        "modules": {"fc_imu": _mod(fc), "payload": _mod(pl)},
    }
    with open(path, "w", encoding="utf-8") as f:
        f.write("# AUTO-GENERATED -- do not edit by hand.\n")
        f.write("# Source : src/conceptual_design/vibration_isolation.py\n")
        f.write("# Input  : config/vibration.yaml + rotor RPM + payload mass\n")
        f.write("# Regen  : re-run notebooks/vibration_isolation.ipynb\n\n")
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
