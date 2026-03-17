"""
wing_sizing.py  --  Wing Sizing and Weight Module
=================================================

Determines wing geometry and structural mass from:
  1. The design wing loading  (W/S)_design  from the Size Matching Diagram
  2. The estimated MTOW  from the initial mass estimation loop
  3. Aerodynamic and structural parameters

THEORY
------

WING GEOMETRY FROM WING LOADING
    Once the design point (W/S)_design is known:

        S_wing = W_total / (W/S)_design        [m^2]   wing planform area
        b_wing = sqrt( AR * S_wing )            [m]    span
        chord  = S_wing / b_wing  = sqrt(S_wing/AR)    [m]    MAC

    For a simple unswept, untapered wing (lambda=1) the MAC equals the chord.

WING STRUCTURAL MASS -- RAYMER GENERAL AVIATION METHOD  (eq. 2-11)
    Raymer's empirical regression for GA aircraft wing weight (in lb):

        W_wing = 0.036 * S_wing^0.758
                        * (AR / cos^2Lambda)^0.6
                        * q^0.006
                        * lambda^0.04
                        * (100*t/c / cosLambda)^-0.3
                        * (n_ult * W_To)^0.49

    where:
        S_wing  wing area              [ft^2]
        AR      aspect ratio           [-]
        Lambda       quarter-chord sweep    [rad]
        q       dynamic pressure at cruise  [lb/ft^2]
        lambda       taper ratio            [-]
        t/c     thickness-to-chord ratio    [-]
        n_ult   ultimate load factor   [-]
        W_To    take-off weight        [lb]

    This is converted to SI (kg) internally.

    Notes on applicability:
    - Validated for GA aircraft; broadly applicable to small UAVs
    - Underestimates composite-construction weight (multiply by ~0.7 for CFRP)
    - Does not include control surface mass

NICOLAI METHOD  (eq. 2-6, Imperial only)
    Also provided as an alternative; tends to give slightly heavier estimates
    for small aircraft.

References
----------
  Raymer (2006), Aircraft Design: A Conceptual Approach, eq. 15.25.
  DLR-IB-FT-BS-2024-106, 2.5.2.2, eq. (2-11).
  Nicolai (2010), Fundamentals of Aircraft & Airship Design, eq. (2-6).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .models import Aerodynamics, Environment, WingSizing


# ---------------------------------------------
#  Structural wing parameters
# ---------------------------------------------
@dataclass
class WingStructureParams:
    """
    Parameters needed for the empirical wing weight formulas.
    Defaults are appropriate for a small CFRP UAV wing.
    """
    sweep_rad:    float = 0.0     # quarter-chord sweep angle        [rad]
    taper:        float = 1.0     # taper ratio lambda = c_tip / c_root   [-]
    tc_ratio:     float = 0.12    # thickness-to-chord ratio t/c      [-]
    n_ult:        float = 3.75    # ultimate load factor (1.5xlimit)  [-]
    k_material:   float = 0.70    # material knockdown (0.70=CFRP,1.0=Al) [-]
    method:       str   = "raymer"  # "raymer" or "nicolai"

    @classmethod
    def from_yaml(cls, path) -> "WingStructureParams":
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(
            sweep_rad  = float(data["sweep_rad"]),
            taper      = float(data["taper"]),
            tc_ratio   = float(data["tc_ratio"]),
            n_ult      = float(data["n_ult"]),
            k_material = float(data["k_material"]),
            method     = str(data["method"]),
        )


# ---------------------------------------------
#  Unit conversion helpers
# ---------------------------------------------
def _kg_to_lb(kg: float) -> float:
    return kg * 2.20462

def _lb_to_kg(lb: float) -> float:
    return lb / 2.20462

def _m2_to_ft2(m2: float) -> float:
    return m2 * 10.7639

def _ms_to_kts(ms: float) -> float:
    return ms * 1.94384

def _pa_to_lbft2(pa: float) -> float:
    """Convert Pa (N/m^2) to lb/ft^2."""
    return pa * 0.020885


# ---------------------------------------------
#  Wing weight -- Raymer GA method
# ---------------------------------------------

def wing_mass_raymer_kg(
    S_wing_m2:   float,
    AR:          float,
    MTOW_kg:     float,
    V_cruise_ms: float,
    rho:         float,
    ws:          WingStructureParams = WingStructureParams(),
) -> float:
    """
    Raymer General Aviation wing weight formula (eq. 2-11).

    All intermediate calculations use Imperial units (Raymer's formula
    requires this), result is converted back to kg.

    Parameters
    ----------
    S_wing_m2   : wing planform area         [m^2]
    AR          : aspect ratio               [-]
    MTOW_kg     : maximum take-off mass      [kg]
    V_cruise_ms : cruise speed               [m/s]
    rho         : air density at cruise      [kg/m^3]
    ws          : WingStructureParams

    Returns
    -------
    float  -- estimated wing structural mass  [kg]
    """
    # Convert to Imperial
    S_ft2 = _m2_to_ft2(S_wing_m2)
    W_lb  = _kg_to_lb(MTOW_kg)
    q_Pa  = 0.5 * rho * V_cruise_ms**2
    q_lbft2 = _pa_to_lbft2(q_Pa)

    cos_sweep = math.cos(ws.sweep_rad)
    cos2      = cos_sweep**2

    # Raymer eq. (2-11) -- result in lb
    W_wing_lb = (
        0.036
        * (S_ft2**0.758)
        * ((AR / cos2)**0.6)
        * (q_lbft2**0.006)
        * (ws.taper**0.04)
        * ((100.0 * ws.tc_ratio / cos_sweep)**(-0.3))
        * ((ws.n_ult * W_lb)**0.49)
    )

    # Apply material factor and convert to kg
    return _lb_to_kg(W_wing_lb) * ws.k_material


def wing_mass_nicolai_kg(
    S_wing_m2:   float,
    AR:          float,
    MTOW_kg:     float,
    b_wing_m:    float,
    t_root_m:    float,
    ws:          WingStructureParams = WingStructureParams(),
) -> float:
    """
    Nicolai Light Utility Aircraft wing weight formula (eq. 2-6).

    Eq. (2-6) in Imperial:
        W_wing = 96.948 * [ (S/100)^0.61 * (AR/cosLambda)^0.57
                            * (1 + Ve/500)^0.5
                            * ((1+lambda)/(2*t/c))^0.36
                            * (n_ult*W_To/1e5)^0.65 ]^0.993

    Ve = cruise equivalent airspeed (knots); t_root is root thickness in ft.
    Note: b_wing_m is the span, t_root_m is the root thickness.
    """
    # Approximate equivalent airspeed (TAS at sea level = EAS at ISA SL)
    # For small UAVs flying low this is a fair approximation
    Ve_kts = 100.0   # representative -- update with actual cruise EAS

    # Convert
    S_ft2    = _m2_to_ft2(S_wing_m2)
    W_lb     = _kg_to_lb(MTOW_kg)
    cos_sw   = math.cos(ws.sweep_rad)
    lam      = ws.taper
    tc       = ws.tc_ratio

    inner = (
        (S_ft2 / 100.0)**0.61
        * ((AR / cos_sw)**0.57)
        * ((1.0 + Ve_kts / 500.0)**0.5)
        * (((1.0 + lam) / (2.0 * tc))**0.36)
        * ((ws.n_ult * W_lb / 1e5)**0.65)
    )
    W_wing_lb = 96.948 * (inner**0.993)

    return _lb_to_kg(W_wing_lb) * ws.k_material


# ---------------------------------------------
#  Main wing sizing function
# ---------------------------------------------

def size_wing(
    MTOW_kg:       float,
    WS_design:     float,         # design wing loading from SizeMatchingDiagram [N/m^2]
    TW_design:     float,         # design T/W from SizeMatchingDiagram          [-]
    aero:          Aerodynamics,
    mission_V:     float,         # cruise speed [m/s]
    ws:            WingStructureParams = WingStructureParams(),
    env:           Environment = Environment(),
) -> WingSizing:
    """
    Compute wing geometry and structural mass.

    Steps
    -----
    1. Wing area from design wing loading:
           S_wing = (MTOW_kg * g) / WS_design

    2. Span from aspect ratio:
           b_wing = sqrt( AR * S_wing )

    3. Mean chord:
           c_mean = S_wing / b_wing  =  sqrt( S_wing / AR )

    4. Wing structural mass via Raymer (default) or Nicolai.

    Parameters
    ----------
    MTOW_kg    : estimated MTOW                  [kg]
    WS_design  : design wing loading             [N/m^2]
    TW_design  : design T/W ratio                [-]
    aero       : Aerodynamics dataclass
    mission_V  : cruise speed                    [m/s]
    ws         : WingStructureParams
    env        : Environment

    Returns
    -------
    WingSizing dataclass
    """
    if WS_design <= 0:
        raise ValueError("WS_design must be positive")
    if MTOW_kg <= 0:
        raise ValueError("MTOW_kg must be positive")

    W_N    = MTOW_kg * env.g
    S_wing = W_N / WS_design                          # [m^2]
    b_wing = math.sqrt(aero.AR * S_wing)              # [m]
    c_mean = S_wing / b_wing                          # [m]  = sqrt(S/AR)

    # Wing structural mass
    if ws.method == "raymer":
        m_wing = wing_mass_raymer_kg(
            S_wing_m2=S_wing,
            AR=aero.AR,
            MTOW_kg=MTOW_kg,
            V_cruise_ms=mission_V,
            rho=env.rho,
            ws=ws,
        )
    elif ws.method == "nicolai":
        t_root = c_mean * ws.tc_ratio            # root thickness [m]
        m_wing = wing_mass_nicolai_kg(
            S_wing_m2=S_wing,
            AR=aero.AR,
            MTOW_kg=MTOW_kg,
            b_wing_m=b_wing,
            t_root_m=t_root,
            ws=ws,
        )
    else:
        raise ValueError(f"Unknown wing weight method: {ws.method!r}. Use 'raymer' or 'nicolai'.")

    return WingSizing(
        S_wing       = S_wing,
        b_wing       = b_wing,
        chord_mean   = c_mean,
        wing_loading = WS_design,
        T_W_design   = TW_design,
        mass_wing_kg = m_wing,
    )


if __name__ == "__main__":
    from .models import Aerodynamics, Environment

    aero = Aerodynamics(LD=8.0, CD0=0.025, AR=6.0, e=0.80,
                        CL_max=1.4, V_stall=12.0, V_max=35.0)
    ws_params = WingStructureParams(tc_ratio=0.12, n_ult=3.75, k_material=0.70)

    result = size_wing(
        MTOW_kg=5.0,
        WS_design=200.0,
        TW_design=0.35,
        aero=aero,
        mission_V=20.0,
        ws=ws_params,
    )

    print("Wing Sizing Results (5 kg UAV, W/S=200 N/m^2)")
    print("-" * 45)
    print(f"  Wing area      : {result.S_wing:.4f} m^2")
    print(f"  Span           : {result.b_wing:.4f} m")
    print(f"  Mean chord     : {result.chord_mean:.4f} m")
    print(f"  Wing mass      : {result.mass_wing_kg:.4f} kg")