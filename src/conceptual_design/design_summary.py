# conceptual_design/design_summary.py
"""Rollup logic for the design-summary notebook (NB15).

NB15 is a pure reader: it re-runs the sizing loop for the design point
and collects everything else from the out/ handoffs.  The margin checks
and standing-finding collection are logic, so they live here; the
notebook only loads handoffs, calls these, and prints.
"""

from __future__ import annotations

import pandas as pd

#: Mass-budget groups reported by the COTS freeze (out/components.yaml).
BUDGET_GROUPS = ["avionics_bay", "esc", "motor_fan", "servo_each", "battery"]


def margins_table(vanes, ail, ail_cots, vib_cots, thermal, fus_cots,
                  af, aero) -> list[tuple]:
    """Margin/verdict rows collected programmatically from the handoffs.

    Each row: (check, owning notebook, value, requirement, ok).
    """
    # NB14 drag buildup on the as-selected hull (same allowance as NB6)
    CD0_total = af["Cd0_section"] * 1.10 + fus_cots["CD0_fus"] + 0.0025
    return [
        ("hover roll authority",      "NB3",  f"{vanes['ddot_roll_deg_s2']:.0f} deg/s^2",
         f">= {ail['ddot_min_deg_s2']:.0f}", vanes["ddot_roll_deg_s2"] >= ail["ddot_min_deg_s2"]),
        ("hover pitch authority",     "NB3",  f"{vanes['ddot_pitch_deg_s2']:.0f} deg/s^2",
         f">= {ail['ddot_min_deg_s2']:.0f}", vanes["ddot_pitch_deg_s2"] >= ail["ddot_min_deg_s2"]),
        ("cruise roll (ail + vane)",  "NB12", f"{ail_cots['ddot_roll_total_cruise_deg_s2']:.0f} deg/s^2",
         f">= {ail_cots['ddot_min_deg_s2']:.0f}", ail_cots["cruise_authority_ok"]),
        ("servo stall vs vane req",   "NB12", f"x{ail_cots['cots_servo']['margin_vs_vane_req']:.1f}",
         ">= x1.0", ail_cots["cots_servo"]["margin_vs_vane_req"] >= 1.0),
        ("1/rev isolation (FC/IMU)",  "NB13", f"{vib_cots['modules']['fc_imu']['attenuation_pct']:.0f}% att.",
         f">= {100*(1-vib_cots['target_transmissibility']):.0f}%", vib_cots["window_ok"]),
        ("battery bay venting",       "NB7",  f"{thermal['battery']['temp_margin_C']:.1f} C margin",
         "> 0 C", thermal["battery"]["ok"]),
        ("ESC cold-plate",            "NB7",  f"{thermal['esc']['temp_margin_C']:.1f} C margin",
         "mass in alloc", thermal["esc"]["ok"]),
        ("pack transient (nominal R)", "NB7",
         f"{thermal['battery_transient']['cases']['nominal']['T_final_C']:.0f} C end-of-mission",
         f"<= {thermal['battery_transient']['T_batt_max_C']:.0f} C",
         thermal["battery_transient"]["ok_nominal"]),
        ("cruise drag budget",        "NB14", f"CD0 {CD0_total:.4f}",
         f"<= {aero.CD0:.4f}", CD0_total <= aero.CD0),
        ("frozen parts fit the hull", "NB14", f"{sum(1 for e in fus_cots['bay_fit'] if e['ok'])}/"
         f"{len(fus_cots['bay_fit'])} fit", "all fit", all(e["ok"] for e in fus_cots["bay_fit"])),
        ("structure vs budget (as-sel.)", "NB14",
         f"{fus_cots['m_shell_kg']*1e3:.0f} g",
         f"<= {(fus_cots['m_struct_pool_kg'] - fus_cots['m_struct_carved_kg'])*1e3:.0f} g",
         fus_cots["m_shell_kg"] <= fus_cots["m_struct_pool_kg"] - fus_cots["m_struct_carved_kg"]),
    ]


def collect_findings(comp, thermal, fus, fus_cots) -> list[str]:
    """Every standing finding, collected programmatically from the handoffs."""
    findings = []

    for g in BUDGET_GROUPS:
        b = comp["budgets"][g]
        if not b["within"]:
            findings.append(f"{g}: {b['actual_g']:.0f} g vs {b['alloc_g']:.0f} g allocation "
                            f"({b['margin_g']:+.0f} g) -- {b.get('note', 'allocation finding')}")

    if not thermal["esc"]["ok"]:
        findings.append(f"ESC cold-plate (ADR-0009): needs {thermal['esc']['A_req_cm2']:.0f} cm^2 / "
                        f"{thermal['esc']['plate_mass_g']:.0f} g plate vs the ESC allocation; "
                        f"temp margin only {thermal['esc']['temp_margin_C']:.1f} C")

    bt = thermal["battery_transient"]
    if not bt["ok_nominal"]:
        n = bt["cases"]["nominal"]
        findings.append(f"battery pack transient (ADR-0014): nominal {n['R_pack_mohm']:.0f} mOhm pack "
                        f"ends the mission at {n['T_final_C']:.0f} C average vs the "
                        f"{bt['T_batt_max_C']:.0f} C limit at hot-day ambient ({n['temp_margin_C']:+.0f} C) "
                        f"-- measure pack DCIR at procurement; the optimistic "
                        f"{bt['cases']['optimistic']['R_pack_mohm']:.0f} mOhm build passes")

    for e in fus_cots["bay_fit"]:
        if not e["ok"]:
            findings.append(f"fit: {e['part']} does not fit {e['where']} "
                            f"(needs {e['need_mm']} mm, has {e['have_mm']} mm)")

    if fus_cots["as_selected"]["m_delta_kg"] > 0:
        findings.append(f"as-selected all-up mass {fus_cots['as_selected']['m_items_total_kg']:.3f} kg "
                        f"is {fus_cots['as_selected']['m_delta_kg']*1e3:+.0f} g over the closure MTOW "
                        f"-- the sum of the mass findings above")

    m_struct_budget_cots = fus_cots["m_struct_pool_kg"] - fus_cots["m_struct_carved_kg"]
    if fus_cots["m_shell_kg"] > m_struct_budget_cots:
        findings.append(f"as-selected structure model {fus_cots['m_shell_kg']*1e3:.0f} g exceeds the "
                        f"non-wing structural budget {m_struct_budget_cots*1e3:.0f} g -- the hull that "
                        f"holds the real hardware outgrows the structural fraction at the closure MTOW")

    d = fus_cots["delta_vs_conceptual"]
    if abs(d["D_fus_mm"]) > 0.02 * fus["D_fus_m"] * 1e3:
        findings.append(f"as-selected hull grows to {fus_cots['D_fus_m']*1e3:.0f} x "
                        f"{fus_cots['L_fus_m']*1e3:.0f} mm ({d['D_fus_mm']:+.1f} / {d['L_fus_mm']:+.1f} mm "
                        f"vs conceptual) -- rigid COTS envelopes drive the bay stack; CAD/CFD stay on "
                        f"the conceptual geometry until a config revision adopts this (ADR-0012)")

    return findings


def mission_profile_table(profile) -> pd.DataFrame:
    """Per-leg timeline: duration, power, altitude, range, energy, pack SoC (NB16)."""
    rows = []
    for leg in profile.legs:
        rows.append({
            "leg": leg.name,
            "phase": leg.phase,
            "dt_s": round(leg.duration_s, 0),
            "power_W": round(leg.power_W, 0),
            "alt_m": f"{leg.alt_start_m:.0f}->{leg.alt_end_m:.0f}",
            "range_km": round(leg.range_end_m / 1000.0, 2),
            "E_Wh": round(leg.E_leg_Wh, 1),
            "SoC_%": f"{leg.soc_start*100:.0f}->{leg.soc_end*100:.0f}",
        })
    return pd.DataFrame(rows).set_index("leg")


def selected_hardware_table(comp) -> pd.DataFrame:
    """Frozen-hardware overview table from out/components.yaml (NB15)."""
    rows = []
    for cat, s in comp["selected"].items():
        rows.append({
            "category": cat, "part": s["name"], "id": s["id"],
            "mass_g": s["mass_g"], "pinned": s["frozen"],
            "alternatives": len(s["feasible_alternatives"]),
        })
    rows.append({"category": "supporting avionics",
                 "part": "GPS/telemetry/RX/airspeed/PM/looms",
                 "id": "-",
                 "mass_g": sum(e["mass_g"] for e in comp["supporting_avionics"]),
                 "pinned": False, "alternatives": 0})
    return pd.DataFrame(rows).set_index("category")


# ---------------------------------------------
#  Console reports (NB15)
# ---------------------------------------------

def print_glance(result, mission, rotor, wf, af, elec, fus, fus_cots,
                 massprop, comp) -> None:
    """The vehicle-at-a-glance block."""
    as_sel = fus_cots["as_selected"]
    t_mission = mission.t_hover + mission.t_transition + mission.t_cruise

    print("VEHICLE AT A GLANCE")
    print("-" * 62)
    print(f"{'MTOW (mass closure)':<34}: {result.m_total_kg:.3f} kg")
    print(f"{'All-up, as-selected hardware':<34}: {as_sel['m_items_total_kg']:.3f} kg "
          f"({as_sel['m_delta_kg']*1e3:+.0f} g standing findings)")
    print(f"{'Hover / design power':<34}: {result.P_hover_W:.0f} / {result.P_design_W:.0f} W")
    print(f"{'Pack':<34}: {elec['battery_series_cells']}S {elec['pack_voltage_v']:.1f} V, "
          f"{elec['pack_capacity_ah']:.2f} Ah sized (hover {elec['hover_current_a']:.1f} A)")
    print(f"{'Mission':<34}: {mission.t_hover:.0f} s hover + {mission.t_transition:.0f} s "
          f"transition + {mission.t_cruise:.0f} s cruise = {t_mission/60:.1f} min")
    print(f"{'Wing':<34}: b {result.wing.b_wing*1e3:.0f} mm, S {result.wing.S_wing:.4f} m^2, "
          f"MAC {result.wing.chord_mean*1e3:.1f} mm, {af['designation']}")
    print(f"{'Rotor':<34}: {rotor.D_rotor_m*1e3:.0f} mm, {comp['selected']['propeller']['name']}")
    print(f"{'Fuselage (conceptual / as-sel.)':<34}: {fus['D_fus_m']*1e3:.0f} x {fus['L_fus_m']*1e3:.0f} mm  /  "
          f"{fus_cots['D_fus_m']*1e3:.0f} x {fus_cots['L_fus_m']*1e3:.0f} mm")
    print(f"{'CG / static margin':<34}: {fus['x_CG_m']*1e3:.1f} mm from nose, "
          f"{fus['static_margin']*100:.0f}% MAC")
    print(f"{'Inertia (body FRD, about CG)':<34}: Ixx {massprop['inertia_about_CG_body_FRD']['Ixx_kgm2']:.4f}, "
          f"Iyy {massprop['inertia_about_CG_body_FRD']['Iyy_kgm2']:.4f}, "
          f"Izz {massprop['inertia_about_CG_body_FRD']['Izz_kgm2']:.4f} kg m^2")
    print(f"{'Construction':<34}: {wf.construction_method} (ADR-0008/0010), "
          f"structure {fus['m_shell_kg']*1e3:.0f} g (as-sel. hull {fus_cots['m_shell_kg']*1e3:.0f} g)")


def print_mission_profile(profile) -> None:
    """Mission-level rollup: range, endurance, climb, and pack usage (NB16)."""
    print("MISSION PROFILE")
    print("-" * 62)
    print(f"{'Endurance (hover+transition+cruise)':<38}: {profile.endurance_min:.1f} min")
    print(f"{'Cruise range (ground track)':<38}: {profile.range_km:.1f} km")
    print(f"{'Cruise altitude AGL':<38}: {profile.h_cruise_m:.0f} m")
    print(f"{'Vertical climb':<38}: {profile.h_cruise_m:.0f} m at "
          f"{profile.v_climb_mps:.1f} m/s ({profile.t_vclimb_s:.0f} s)")
    print(f"{'Pack energy (sized, incl. reserve)':<38}: {profile.E_pack_Wh:.1f} Wh")
    print(f"{'Mission energy (nominal, no reserve)':<38}: {profile.E_mission_Wh:.1f} Wh")
    print(f"{'Pack SoC at landing':<38}: {profile.soc_end*100:.0f}% "
          f"({profile.reserve_frac*100:.0f}% reserve held)")


def print_budget_lines(budgets, fus_cots) -> None:
    """One line per mass-budget group plus the NB14 all-up rollup."""
    as_sel = fus_cots["as_selected"]
    for g in BUDGET_GROUPS:
        b = budgets[g]
        print(f"{'ok      ' if b['within'] else 'FINDING '}{g:14s}: "
              f"{b['actual_g']:7.1f} g vs {b['alloc_g']:7.1f} g ({b['margin_g']:+7.1f} g)")
    print(f"\nAll-up rollup (NB14): {as_sel['m_items_total_kg']*1e3:.0f} g as-selected vs "
          f"{as_sel['m_closure_mtow_kg']*1e3:.0f} g closure MTOW "
          f"({as_sel['m_delta_kg']*1e3:+.0f} g)")


def print_margins_table(rows) -> None:
    """The collected margins table."""
    print(f"{'check':<28}{'owner':<7}{'value':<22}{'requirement':<16}{'verdict'}")
    print("-" * 82)
    for name, owner, val, req, ok in rows:
        print(f"{name:<28}{owner:<7}{val:<22}{req:<16}{'ok' if ok else 'FINDING'}")


def print_findings(findings) -> None:
    """Numbered list of the standing findings."""
    print(f"{len(findings)} standing finding(s):\n")
    for i, f_ in enumerate(findings, 1):
        print(f"{i:2d}. {f_}")


def print_final_card(result, rotor, elec, fus, fus_cots, comp, findings) -> None:
    """The frozen-design-point closing card."""
    as_sel = fus_cots["as_selected"]
    bar = "=" * 66
    print(bar)
    print("  V-BAT-LIKE TAIL-SITTER -- FROZEN CONCEPTUAL DESIGN POINT".center(66))
    print(bar)
    print(f"  {'MTOW (closure) / as-selected':<36}: {result.m_total_kg:.3f} / "
          f"{as_sel['m_items_total_kg']:.3f} kg")
    print(f"  {'Hover power / pack':<36}: {result.P_hover_W:.0f} W / "
          f"{elec['battery_series_cells']}S {elec['pack_capacity_ah']:.2f} Ah")
    print(f"  {'Wing / rotor':<36}: b {result.wing.b_wing*1e3:.0f} mm / "
          f"D {rotor.D_rotor_m*1e3:.0f} mm ({comp['selected']['propeller']['id']})")
    print(f"  {'Fuselage (conceptual/as-sel.)':<36}: {fus['D_fus_m']*1e3:.0f}x{fus['L_fus_m']*1e3:.0f} / "
          f"{fus_cots['D_fus_m']*1e3:.0f}x{fus_cots['L_fus_m']*1e3:.0f} mm")
    print(f"  {'Standing findings':<36}: {len(findings)}")
    print(bar)
    print("  Next iteration: procure & weigh the frozen parts, fix the EST")
    print("  entries, then fold the as-selected deltas (battery envelope,")
    print("  motor mass, avionics stack) into config/ as a reviewed change --")
    print("  the pipeline re-converges from there (ADR-0012).")
    print(bar)
