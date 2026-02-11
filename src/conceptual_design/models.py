from dataclasses import dataclass

# -----------------------------
# Environment
# -----------------------------
@dataclass
class Environment:
    rho: float = 1.225        # air density [kg/m^3]
    g: float = 9.80665        # gravity [m/s^2]


# -----------------------------
# Mission definition
# -----------------------------
@dataclass
class Mission:
    t_hover: float = 120.0    # hover time [s]
    t_cruise: float = 480.0   # cruise time [s]
    V_cruise: float = 20.0    # cruise speed [m/s]
    reserve_factor: float = 1.20


# -----------------------------
# Propulsor (EDF)
# -----------------------------
@dataclass
class Propulsor:
    fan_diameter: float = 0.12    # duct diameter [m]
    thrust_to_weight: float = 1.4
    eta_hover: float = 0.55
    eta_cruise: float = 0.60


# -----------------------------
# Aerodynamics
# -----------------------------
@dataclass
class Aerodynamics:
    LD: float = 10.0              # lift-to-drag ratio


# -----------------------------
# Mass breakdown
# -----------------------------
@dataclass
class MassBreakdown:
    payload: float = 0.50
    avionics: float = 0.20
    structure: float = 0.60
    motor_esc: float = 0.35
    misc: float = 0.15


# -----------------------------
# Battery
# -----------------------------
@dataclass
class Battery:
    specific_energy: float = 200.0  # Wh/kg (pack-level usable)
    usable_fraction: float = 0.85
