import math
from .models import (
    Environment, Mission, Propulsor,
    Aerodynamics, MassBreakdown, Battery
)


# -----------------------------
# Hover power (momentum theory)
# -----------------------------
def hover_power(m_total, env: Environment, prop: Propulsor):
    A = math.pi * (prop.fan_diameter / 2.0) ** 2
    W = m_total * env.g
    T = prop.thrust_to_weight * W

    P_ideal = (T ** 1.5) / math.sqrt(2.0 * env.rho * A)
    P_elec = P_ideal / prop.eta_hover

    return P_elec, T


# -----------------------------
# Cruise power (L/D model)
# -----------------------------
def cruise_power(m_total, env: Environment,
                 aero: Aerodynamics, mission: Mission,
                 prop: Propulsor):

    W = m_total * env.g
    D = W / aero.LD
    P_aero = D * mission.V_cruise
    P_elec = P_aero / prop.eta_cruise

    return P_elec, D


# -----------------------------
# Battery sizing
# -----------------------------
def battery_mass(E_Wh, batt: Battery):
    E_pack = E_Wh / batt.usable_fraction
    m_batt = E_pack / batt.specific_energy
    return m_batt, E_pack


# -----------------------------
# Vehicle sizing loop
# -----------------------------
def size_vehicle(env: Environment,
                 mission: Mission,
                 prop: Propulsor,
                 aero: Aerodynamics,
                 mass: MassBreakdown,
                 batt: Battery,
                 max_iter=50,
                 tol=1e-4):

    m_batt = 0.8  # initial guess
    history = []

    for _ in range(max_iter):
        m_total = (
            mass.payload +
            mass.avionics +
            mass.structure +
            mass.motor_esc +
            mass.misc +
            m_batt
        )

        P_hover, T_req = hover_power(m_total, env, prop)
        P_cruise, D = cruise_power(m_total, env, aero, mission, prop)

        E_hover = P_hover * mission.t_hover / 3600.0
        E_cruise = P_cruise * mission.t_cruise / 3600.0
        E_total = mission.reserve_factor * (E_hover + E_cruise)

        m_batt_new, E_pack = battery_mass(E_total, batt)

        history.append(
            (m_total, m_batt, m_batt_new, P_hover, P_cruise, E_total)
        )

        if abs(m_batt_new - m_batt) < tol:
            m_batt = m_batt_new
            break

        m_batt = m_batt_new

    return {
        "m_total": m_total,
        "m_batt": m_batt,
        "P_hover": P_hover,
        "P_cruise": P_cruise,
        "T_required": T_req,
        "E_total": E_total,
        "E_pack": E_pack,
        "history": history
    }
