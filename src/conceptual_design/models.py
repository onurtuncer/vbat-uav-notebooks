from dataclasses import dataclass
import yaml

#------------------------------
# Environment
#------------------------------
@dataclass
class Environment:
    rho: float = 1.225        # air density [kg/m^3]
    g: float = 9.80665        # gravity [m/s^2]


#-------------------------------
# Propulsive System Parameters
#-------------------------------
@dataclass
class PropulsiveSystemParameters:
    eta_prop: float
    eta_motor: float
    eta_esc: float
    fom: float
    Cd_blade : float
    sigma_rotor: float
    s_ratio: float
    k_rpm: float
    exp_rpm: float

    @classmethod
    def from_yaml(cls, path: str) -> "Mission":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(
            eta_prop  = float(data["t_hover"]),
            eta_motor = float(data["t_cruise"]),
            eta_esc   = float(data["V_cruise"]),
            reserve_factor = float(data["reserve_factor"])
        )




propeller_efficiency: 0.8
motor_efficiency: 0.9
esc_efficiency: 0.85
figure_of_merit: 0.7
blade_drag_coefficient: 0.01
rotor_solidity: 0.077
S_ratio: 1.3
rpm_coefficient: 2762.786
rpm_exponent: -0.932

#------------------------------
# Weight fraction
#------------------------------
@dataclass
class WeightFraction:
    fs: float   #structural weight fraction



# -----------------------------
# Mission definition
# -----------------------------
@dataclass
class Mission:
    t_hover: float    # hover time [s]
    t_cruise: float   # cruise time [s]
    V_cruise: float   # cruise speed [m/s]
    reserve_factor: float 

    @classmethod
    def from_yaml(cls, path: str) -> "Mission":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(
            t_hover = float(data["t_hover"]),
            t_cruise= float(data["t_cruise"]),
            V_cruise= float(data["V_cruise"]),
            reserve_factor = float(data["reserve_factor"])
        )


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

    @classmethod
    def from_yaml(cls, path: str) -> "Battery":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(
            specific_energy=float(data["specific_energy"]),
            usable_fraction=float(data["usable_fraction"]),
        )
