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
    def from_yaml(cls, path: str) -> "PropulsiveSystemParameters":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(
            eta_prop  = float(data["propeller_efficiency"]),
            eta_motor = float(data["motor_efficiency"]),
            eta_esc   = float(data["esc_efficiency"]),
            fom = float(data["figure_of_merit"]),
            Cd_blade = float(data["blade_drag_coefficient"]),
            sigma_rotor = float(data["rotor_solidity"]),
            s_ratio = float(data["S_ratio"]),
            k_rpm = float(data["rpm_coefficient"]),
            exp_rpm = float(data["rmp_exponent"])
        )


#------------------------------
# Weight fraction
#------------------------------
@dataclass
class WeightFraction:
    fs: float   # structural weight fraction
    fp: float   # propulsive weight fraction
    fa: float   # avionics weight fraction
    fm: float   # miscellaneous weight fraction

    @classmethod
    def from_yaml(cls, path: str) -> "WeightFraction":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(
            fs = float(["structual_weight_fraction"]),
            fp = float(["propulsive_weight_fraction"]),
            fa = float("avionics_weight_fraction"),
            fm = float("miscellaneous_weight_fraction")
        )


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
