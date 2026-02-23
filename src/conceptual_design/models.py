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
    t_hover: float    # hover time [s]
    t_cruise: float   # cruise time [s]
    V_cruise: float   # cruise speed [m/s]
    reserve_factor: float 


# -----------------------------
# Propulsor (EDF)
# -----------------------------
@dataclass
class Propulsor:
    fan_diameter: float = 0.12    # duct diameter [m]
    thrust_to_weight: float 
    eta_hover: float 
    eta_cruise: float 


# -----------------------------
# Aerodynamics
# -----------------------------
@dataclass
class Aerodynamics:
    LD: float             # lift-to-drag ratio


# -----------------------------
# Mass breakdown
# -----------------------------
@dataclass
class MassBreakdown:
    payload: float 
    avionics: float 
    structure: float 
    motor_esc: float 
    misc: float 


# -----------------------------
# Battery
# -----------------------------
@dataclass
class Battery:
    specific_energy: float   # Wh/kg 
    usable_fraction: float 
