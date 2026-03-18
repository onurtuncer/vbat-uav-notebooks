"""
models.py  --  Data-container dataclasses for the V-BAT conceptual sizing toolchain.

Every physical quantity lives here.  No physics equations, just structure.

Bugs fixed vs. original:
  - WeightFraction.from_yaml: float(["key"]) -> float(data["key"])  (was TypeError)
  - WeightFraction.from_yaml: "avionics_weight_fraction" string lookup fixed
  - PropulsiveSystemParameters.from_yaml: typo "rmp_exponent" -> "rpm_exponent"
  - Mission.from_yaml: added rate_of_climb field (needed by power modules)
  - Propulsor: dataclass field ordering fixed (fields with defaults after fields without)
  - Added Aerodynamics fields needed by forward-flight module (CD0, k, AR, e)
  - Added WingSizing dataclass (output container for wing module)
  - Battery.from_yaml: guard against empty YAML (NoneType -> ValueError)
"""

from __future__ import annotations
from dataclasses import dataclass, field
import yaml


# ---------------------------------------------
#  Environment  (ISA sea-level by default)
# ---------------------------------------------
@dataclass
class Environment:
    rho: float = 1.225      # air density        [kg/m^3]
    g:   float = 9.80665    # gravitational accel [m/s^2]


# ---------------------------------------------
#  Mission profile
# ---------------------------------------------
@dataclass
class Mission:
    t_hover:       float   # hover time          [s]
    t_cruise:      float   # cruise time         [s]
    V_cruise:      float   # cruise speed        [m/s]
    rate_of_climb: float   # VTOL vertical RoC   [m/s]
    reserve_factor: float  # energy reserve (e.g. 1.20 = +20%)

    @classmethod
    def from_yaml(cls, path: str) -> "Mission":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(
            t_hover        = float(data["t_hover"]),
            t_cruise       = float(data["t_cruise"]),
            V_cruise       = float(data["V_cruise"]),
            rate_of_climb  = float(data["rate_of_climb"]),
            reserve_factor = float(data["reserve_factor"]),
        )


# ---------------------------------------------
#  Propulsive system parameters
#  (loaded from config/propulsive_system_parameters.yaml)
# ---------------------------------------------
@dataclass
class PropulsiveSystemParameters:
    eta_prop:    float   # propeller efficiency          [-]
    eta_motor:   float   # motor efficiency              [-]
    eta_esc:     float   # ESC efficiency                [-]
    fom:         float   # Figure of Merit (VTOL rotor)  [-]
    Cd_blade:    float   # blade profile drag coeff      [-]
    sigma_rotor: float   # rotor solidity                [-]
    s_ratio:     float   # S_total / S_wing              [-]
    k_rpm:       float   # RPM empirical coefficient     [rpm*m^|exp_rpm|]
    exp_rpm:     float   # RPM empirical exponent        [-]

    # Convenience: overall electrical efficiency chain (motor x ESC x prop)
    @property
    def eta_total(self) -> float:
        return self.eta_prop * self.eta_motor * self.eta_esc

    @classmethod
    def from_yaml(cls, path: str) -> "PropulsiveSystemParameters":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(
            eta_prop    = float(data["propeller_efficiency"]),
            eta_motor   = float(data["motor_efficiency"]),
            eta_esc     = float(data["esc_efficiency"]),
            fom         = float(data["figure_of_merit"]),
            Cd_blade    = float(data["blade_drag_coefficient"]),
            sigma_rotor = float(data["rotor_solidity"]),
            s_ratio     = float(data["S_ratio"]),
            k_rpm       = float(data["rpm_coefficient"]),
            exp_rpm     = float(data["rpm_exponent"]),   # BUG FIX: was "rmp_exponent"
        )


# ---------------------------------------------
#  Weight fractions  (non-battery fixed mass fractions)
# ---------------------------------------------
@dataclass
class WeightFraction:
    fs: float   # structural fraction   [-]
    fp: float   # propulsive fraction   [-]
    fa: float   # avionics fraction     [-]
    fm: float   # miscellaneous fraction[-]

    @classmethod
    def from_yaml(cls, path: str) -> "WeightFraction":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        # BUG FIX: original had float(["key"]) instead of float(data["key"])
        return cls(
            fs = float(data["structual_weight_fraction"]),   # note: original YAML has typo "structual"
            fp = float(data["propulsive_weight_fraction"]),
            fa = float(data["avionics_weight_fraction"]),    # BUG FIX: was float("avionics_weight_fraction")
            fm = float(data["miscellaneous_weight_fraction"]),
        )


# ---------------------------------------------
#  Aerodynamics parameters
# ---------------------------------------------
@dataclass
class Aerodynamics:
    """
    Aerodynamic parameters needed for the forward-flight power module.

    For a tail-sitter EDF in cruise the wing operates in a conventional
    sense.  At conceptual stage we assume:
        CD_total = CD0 + k * CL^2        (parabolic polar)
        k = 1 / (pi * AR * e)
    """
    LD:        float          # cruise lift-to-drag ratio  [-]  (used by legacy sizing_old)
    CD0:       float = 0.025  # zero-lift drag coefficient [-]
    AR:        float = 6.0    # wing aspect ratio          [-]
    e:         float = 0.80   # Oswald efficiency          [-]
    CL_max:    float = 1.4    # maximum lift coefficient   [-]
    V_stall:   float = 12.0   # desired stall speed        [m/s]
    V_max:     float = 35.0   # maximum speed              [m/s]

    @property
    def k(self) -> float:
        """Induced drag factor  k = 1/(pi*AR*e)."""
        import math
        return 1.0 / (math.pi * self.AR * self.e)

    @classmethod
    def from_yaml(cls, path: str) -> "Aerodynamics":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(
            LD      = float(data["LD"]),
            CD0     = float(data["CD0"]),
            AR      = float(data["AR"]),
            e       = float(data["e"]),
            CL_max  = float(data["CL_max"]),
            V_stall = float(data["V_stall"]),
            V_max   = float(data["V_max"]),
        )


# ---------------------------------------------
#  Propulsor  (EDF - single ducted fan)
# ---------------------------------------------
@dataclass
class Propulsor:
    """
    BUG FIX: In Python dataclasses all fields WITHOUT a default value must
    come before fields WITH a default value.  Original had fan_diameter with
    a default BEFORE fields without defaults, which is illegal.
    """
    thrust_to_weight: float          # T/W ratio required (>=1.3 typical) [-]
    eta_hover:        float          # hover propulsive efficiency        [-]
    eta_cruise:       float          # cruise propulsive efficiency       [-]
    fan_diameter:     float = 0.12   # EDF duct diameter                  [m]


# ---------------------------------------------
#  Mass breakdown  (fixed / non-iterated masses)
# ---------------------------------------------
@dataclass
class MassBreakdown:
    payload:   float   # [kg]
    avionics:  float   # [kg]
    structure: float   # [kg]
    motor_esc: float   # [kg]
    misc:      float   # [kg]

    @property
    def fixed_mass(self) -> float:
        """Sum of all non-battery masses."""
        return self.payload + self.avionics + self.structure + self.motor_esc + self.misc


# ---------------------------------------------
#  Battery
# ---------------------------------------------
@dataclass
class Battery:
    specific_energy: float   # pack-level specific energy  [Wh/kg]
    usable_fraction: float   # usable depth-of-discharge   [-]

    @classmethod
    def from_yaml(cls, path: str) -> "Battery":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        se = data.get("specific_energy")
        uf = data.get("usable_fraction")
        if se is None or uf is None:
            raise ValueError(
                f"battery.yaml is missing values.  "
                f"Set 'specific_energy' (Wh/kg) and 'usable_fraction' (0-1)."
            )
        return cls(
            specific_energy = float(se),
            usable_fraction = float(uf),
        )
    

@dataclass
class RotorParams:
    D_rotor_m:    float   # rotor / EDF diameter [m]
    disk_loading: float   # disk loading [N/m2]

    @classmethod
    def from_yaml(cls, path: str) -> "RotorParams":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(
            D_rotor_m    = float(data["D_rotor_m"]),
            disk_loading = float(data["disk_loading"]),
        )

# ---------------------------------------------
#  Wing sizing  (output container)
# ---------------------------------------------
@dataclass
class WingSizing:
    """Output of the wing_sizing module."""
    S_wing:        float   # wing planform area              [m^2]
    b_wing:        float   # wing span                       [m]
    chord_mean:    float   # mean aerodynamic chord          [m]
    wing_loading:  float   # W/S at design point             [N/m^2]
    T_W_design:    float   # T/W ratio at design point       [-]
    mass_wing_kg:  float   # estimated wing structural mass  [kg]