# conceptual_design/reports.py
"""Console reports for the design notebooks.

Pure presentation: every function formats already-computed results as
the tables / summary cards the rendered notebooks show.  No physics
(ADR-0002) -- anything numeric here is arithmetic on results purely for
display (percentages, unit scaling).
"""

from __future__ import annotations

import math

from .control_vane_design import MU_AIR
from .sizing_studies import local_sensitivity


# =====================================================================
#  NB1 -- conceptual design
# =====================================================================

def print_mission_summary(mission) -> None:
    """Narrate the mission profile (config/mission.yaml)."""
    total_mission_min = (mission.t_hover + mission.t_transition + mission.t_cruise) / 60
    print(f"Total mission budget   : {total_mission_min:.1f} min")
    print(f"  Hover                : {mission.t_hover:.0f} s  ({mission.t_hover/60:.1f} min)")
    print(f"  Transitions          : {mission.t_transition:.0f} s  (billed at hover power)")
    print(f"  Cruise               : {mission.t_cruise:.0f} s  ({mission.t_cruise/60:.1f} min)")
    print(f"  Reserve factor       : {mission.reserve_factor:.2f}  (+{(mission.reserve_factor-1)*100:.0f}%)")
    print(f"  Payload              : {mission.payload_kg:.2f} kg")
    print(f"  Cruise distance      : {mission.V_cruise * mission.t_cruise / 1000:.1f} km")


def print_vehicle_input_summary(inp) -> None:
    """Narrate the aero / propulsion / battery / weight-fraction inputs."""
    aero = inp.aero
    print("Aerodynamics  (config/aerodynamics.yaml):")
    print(f"  CD0 = {aero.CD0}   AR = {aero.AR}   e = {aero.e}")
    print(f"  k   = {aero.k:.5f}   (induced drag factor = 1/pi/AR/e)")
    print(f"  L/D = {aero.LD}   CL_max = {aero.CL_max}")

    prop = inp.prop
    print("\nPropulsion  (config/propulsive_system_parameters.yaml):")
    print(f"  eta_prop={prop.eta_prop}  eta_motor={prop.eta_motor}  eta_esc={prop.eta_esc}")
    print(f"  eta_total = {prop.eta_total:.4f}  (motor x ESC x prop)")
    print(f"  FoM = {prop.fom}   sigma = {prop.sigma_rotor}   CD_blade = {prop.Cd_blade}")

    print("\nRotor  (config/rotor.yaml):")
    print(f"  D_rotor_m = {inp.rotor.D_rotor_m} m  (COTS EDF; disk loading derived in the loop)")

    print("\nAvionics  (config/avionics.yaml):")
    print(f"  P_hotel = {inp.avionics.P_hotel_W} W  (continuous, added to every phase)")

    batt = inp.batt
    print("\nBattery  (config/battery.yaml):")
    print(f"  Specific energy = {batt.specific_energy} Wh/kg")
    print(f"  Usable fraction = {batt.usable_fraction}  ({batt.usable_fraction*100:.0f}% DoD)")
    print(f"  eta_bat = {batt.eta_bat}   C-rate limit = {batt.c_rate_max} C")

    wf = inp.wf
    f_fixed = wf.fs + wf.fp + wf.fa + wf.fm
    print("\nWeight fractions  (config/initial_weight_fraction_estimation.yaml):")
    print(f"  Structure    fs = {wf.fs:.2f}  ({wf.fs*100:.0f}%)")
    print(f"  Propulsion   fp = {wf.fp:.2f}  ({wf.fp*100:.0f}%)")
    print(f"  Avionics     fa = {wf.fa:.2f}  ({wf.fa*100:.0f}%)")
    print(f"  Misc         fm = {wf.fm:.2f}  ({wf.fm*100:.0f}%)")
    print(f"  Fixed total     = {f_fixed:.2f}  ->  remaining {1-f_fixed:.2f} for battery + payload")


def print_pw_breakdown(terms: dict, DL_est, RoC, eta_motor, eta_esc) -> None:
    """Term-by-term climb P/W breakdown at the provisional disk loading."""
    A, B, C1, C2 = (terms["induced_climb"], terms["blade_profile"],
                    terms["fuselage_parasite"], terms["wing_parasite"])
    total_pw = A + B + C1 + C2
    print(f"P/W term breakdown at DL={DL_est:.0f} N/m^2, RoC={RoC} m/s (shaft):")
    print(f"  A. Induced + climb  : {A:.4f} W/N  ({A/total_pw*100:.1f}%)")
    print(f"  B. Profile drag     : {B:.4f} W/N  ({B/total_pw*100:.1f}%)")
    print(f"  C1. Fuselage paras  : {C1:.4f} W/N  ({C1/total_pw*100:.1f}%)")
    print(f"  C2. Wing parasite   : {C2:.4f} W/N  ({C2/total_pw*100:.1f}%)")
    print(f"  Total               : {total_pw:.4f} W/N")
    print(f"\nElectrical P/W = shaft / (eta_motor*eta_esc) "
          f"= {total_pw/(eta_motor*eta_esc):.4f} W/N")


def print_convergence_table(result) -> None:
    """The full iteration history of the mass-closure loop."""
    print(f"{'Iter':>4}  {'MTOW [kg]':>10}  {'m_batt [kg]':>12}  {'m_batt_new':>12}  "
          f"{'|delta| [kg]':>12}  {'E_total [Wh]':>13}")
    print("-" * 70)
    for s in result.history:
        d = abs(s.m_batt_new_kg - s.m_batt_kg)
        marker = " <-" if s.converged else ""
        print(f"{s.iteration:>4}  {s.m_total_kg:>10.4f}  {s.m_batt_kg:>12.5f}  "
              f"{s.m_batt_new_kg:>12.5f}  {d:>12.2e}  {s.E_total_Wh:>13.2f}{marker}")


def print_construction_trade(rows) -> None:
    """The monocoque-vs-segmented-FDM closure table (ADR-0008)."""
    print(f"{'method':<15}{'k':>5}{'MTOW kg':>9}{'batt kg':>9}"
          f"{'P_hov W':>9}{'C peak':>7}{'T_hov N':>9}{'span m':>8}")
    print("-" * 71)
    for r in rows:
        tag = "  <-- BASELINE" if r.baseline else ""
        print(f"{r.method:<15}{r.k:>5.2f}{r.m_total_kg:>9.3f}{r.m_batt_kg:>9.3f}"
              f"{r.P_hover_W:>9.0f}{r.C_rate_peak:>7.1f}{r.T_hover_N:>9.1f}"
              f"{r.b_wing_m:>8.3f}{tag}")

    d_mtow = rows[1].m_total_kg - rows[0].m_total_kg
    print(f"\nSegmented-FDM cost: {d_mtow:+.3f} kg MTOW "
          f"({d_mtow/rows[0].m_total_kg*100:+.1f}%) -- buys overnight-reprint iteration,")
    print("split-line modularity (removable nose, battery hatch, two-piece wing),")
    print("and no composite tooling. See config comments for the k sensitivity sweep.")


def print_elasticities(sweeps: dict, base_mtow: float) -> None:
    """Normalised local elasticities for the standard sensitivity sweeps."""
    print("Normalised elasticity  dMTOW/dX * X/MTOW  (% MTOW change per % param change)")
    labels = {
        "specific_energy": "Battery specific energy ",
        "D_rotor":         "EDF diameter            ",
        "LD":              "Aerodynamic L/D         ",
        "fs":              "Structural fraction fs  ",
    }
    for key, label in labels.items():
        print(f"  {label} : {local_sensitivity(sweeps[key], base_mtow):+.3f}")
    print()
    print("Interpretation: a value of -0.5 means a 10% improvement in that parameter")
    print("reduces MTOW by 5%.  A negative EDF-diameter elasticity quantifies how")
    print("much a larger fan relieves the hover power penalty.")


def print_mass_budget_checks(result, wf) -> None:
    """Assumption validation under the formatted mass budget."""
    print("Assumption validation:")
    print(f"  Wing (Raymer)  : {result.m_wing_kg:.4f} kg  "
          f"({result.m_wing_kg/result.m_total_kg*100:.1f}% MTOW)")
    print(f"  Structure frac : {result.m_structure_kg:.4f} kg  "
          f"({wf.fs*100:.0f}% MTOW)")
    wing_ok = result.m_wing_kg <= result.m_structure_kg
    print(f"  Wing fits within structural fraction: "
          f"{'OK YES' if wing_ok else 'FAIL NO -- increase fs'}")
    print()
    f_sum = (wf.fs + wf.fp + wf.fa + wf.fm
             + result.batt_fraction
             + result.m_payload_kg / result.m_total_kg)
    print(f"  Mass fraction sum  : {f_sum:.6f}  (must equal 1.000)")


def print_requirements_matrix(result, mission, batt) -> None:
    """Requirements compliance matrix for the converged design point."""
    mission_s = mission.t_hover + mission.t_transition + mission.t_cruise
    max_dim_m = result.wing.b_wing

    checks = [
        ("R-1  VTOL capable",             True,                            True),
        ("R-2  Mission >= 15 min",        mission_s / 60 >= 15,            mission_s / 60),
        ("R-3  Payload >= 0.5 kg",        result.m_payload_kg >= 0.5,      result.m_payload_kg),
        ("R-4  Cruise speed 20 m/s",      True,                            mission.V_cruise),
        ("R-5  MTOM < 25 kg",             result.m_total_kg < 25.0,        result.m_total_kg),
        ("R-6  Max dim <= 3 m",           max_dim_m <= 3.0,                max_dim_m),
        ("R-7  Electric, COTS EDF",       True,                            True),
        ("R-9  Peak C-rate within pack",  result.C_rate_peak <= batt.c_rate_max,
                                          result.C_rate_peak),
    ]

    print()
    print("=" * 60)
    print("  REQUIREMENTS COMPLIANCE MATRIX")
    print("=" * 60)
    all_pass = True
    for req, ok, val in checks:
        status = "OK PASS" if ok else "FAIL FAIL"
        if not ok:
            all_pass = False
        if isinstance(val, bool):
            v_str = str(val)
        elif isinstance(val, float):
            v_str = f"{val:.3g}"
        else:
            v_str = str(val)
        print(f"  {status}  {req:<35}  ->  {v_str}")
    print("=" * 60)
    if all_pass:
        print("  OK ALL REQUIREMENTS MET")
    else:
        print("  FAIL SOME REQUIREMENTS NOT MET -- revisit parameters")
    print("=" * 60)


def print_design_card(result, mission, batt, D_rotor_m, rpm, Vtip) -> None:
    """The final conceptual-design card."""
    mission_s = mission.t_hover + mission.t_transition + mission.t_cruise
    print()
    print("+" + "="*58 + "+")
    print("|  ELECTRIC V-BAT TAIL-SITTER -- CONCEPTUAL DESIGN CARD     |")
    print("+" + "="*58 + "+")
    print(f"|  MTOW                  {result.m_total_kg:>7.3f} kg                        |")
    print(f"|  Payload               {result.m_payload_kg:>7.3f} kg                        |")
    print(f"|  Battery               {result.m_battery_kg:>7.3f} kg  ({result.batt_fraction*100:.1f}% MTOW)          |")
    print(f"|  Battery energy        {result.E_total_Wh:>7.1f} Wh (with {(mission.reserve_factor-1)*100:.0f}% reserve)    |")
    print(f"|  Peak C-rate           {result.C_rate_peak:>7.1f} C  (limit {batt.c_rate_max:.0f} C)          |")
    print("+" + "="*58 + "+")
    print(f"|  Wing span             {result.wing.b_wing:>7.3f} m                         |")
    print(f"|  Wing area             {result.wing.S_wing:>7.4f} m^2                        |")
    print(f"|  Wing chord (MAC)      {result.wing.chord_mean:>7.4f} m                         |")
    print(f"|  Wing loading (design) {result.wing.wing_loading:>7.1f} N/m^2                     |")
    print(f"|  Wing mass (Raymer)    {result.m_wing_kg:>7.4f} kg                        |")
    print("+" + "="*58 + "+")
    print(f"|  Hover power (elec)    {result.P_hover_W:>7.1f} W                         |")
    print(f"|  Cruise power (elec)   {result.P_cruise_W:>7.1f} W                         |")
    print(f"|  Hotel load            {result.P_hotel_W:>7.1f} W                         |")
    print(f"|  Design (max) power    {result.P_design_W:>7.1f} W                         |")
    print(f"|  Cruise speed          {mission.V_cruise:>7.1f} m/s  ({mission.V_cruise*3.6:.0f} km/h)            |")
    print(f"|  Mission time          {mission_s/60:>7.1f} min                        |")
    print("+" + "="*58 + "+")
    print(f"|  EDF diameter (COTS)   {D_rotor_m*1000:>7.0f} mm                         |")
    print(f"|  EDF RPM (eq. 2-64)    {rpm:>7.0f} rpm                        |")
    print(f"|  Blade tip speed       {Vtip:>7.1f} m/s                         |")
    print(f"|  Disk loading          {result.disk_loading_N_m2:>7.0f} N/m^2  (derived)          |")
    print("+" + "="*58 + "+")


# =====================================================================
#  NB2 -- wing design
# =====================================================================

def print_planform_summary(wing, aero, ws) -> None:
    """Wing planform summary plus the R-6 span check."""
    AR_check = wing.b_wing**2 / wing.S_wing
    sweep_deg = math.degrees(ws.sweep_rad)
    c_mac = wing.chord_mean

    print("Wing planform summary")
    print(f"  Wing area   S   = {wing.S_wing:.4f} m^2")
    print(f"  Span        b   = {wing.b_wing:.4f} m")
    print(f"  MAC chord   c   = {c_mac:.4f} m")
    print(f"  Aspect ratio AR = {AR_check:.4f}  (target {aero.AR:.1f})")
    print(f"  Taper ratio lam = {ws.taper:.2f}")
    print(f"  Sweep (c/4) Λ   = {sweep_deg:.1f} deg")
    print(f"  t/c             = {ws.tc_ratio:.3f}")
    print(f"  Spar depth      = {c_mac * ws.tc_ratio * 100:.1f} mm  "
          f"({'OK' if ws.tc_ratio >= 0.09 else 'THIN -- check spar'})")

    print(f"\nMax dimension check (R-6 <= 3 m): {wing.b_wing:.3f} m  "
          f"{'OK' if wing.b_wing <= 3.0 else 'EXCEEDS LIMIT'}")


def print_reynolds_check(rho, V_cruise, V_stall, c_mac):
    """Cruise/stall Reynolds numbers vs the empirical calibration range.

    Returns (Re_cruise, Re_stall) for reuse in the design card.
    """
    Re_cruise = rho * V_cruise * c_mac / MU_AIR
    Re_stall  = rho * V_stall  * c_mac / MU_AIR

    print(f"Reynolds number at cruise : {Re_cruise:.3e}  "
          f"({'OK -- within calibration range' if 3e5 <= Re_cruise <= 1e6 else 'CHECK empirical limits'})")
    print(f"Reynolds number at stall  : {Re_stall:.3e}  "
          f"({'OK' if Re_stall >= 2e5 else 'WARNING: laminar-separation risk'})")
    print(f"Chord                     : {c_mac*100:.1f} cm")
    print(f"Cruise speed              : {V_cruise:.1f} m/s")
    print(f"Stall speed               : {V_stall:.2f} m/s")
    return Re_cruise, Re_stall


def print_wing_card(af, wing, aero, ws, M, P, WS_design, V_stall,
                    Re_cruise, Re_stall) -> None:
    """The wing design card."""
    print()
    print("+" + "="*56 + "+")
    print("|  WING DESIGN CARD — Electric V-BAT Tail-Sitter          |")
    print("+" + "="*56 + "+")
    print(f"|  Airfoil designation   {af.designation:<33}|")
    print(f"|  t/c                   {ws.tc_ratio:.3f}{'':<30}|")
    print(f"|  Camber (M)            {M:.4f}{'':<29}|")
    print(f"|  Max camber loc (P)    {P:.2f}{'':<31}|")
    print("+" + "="*56 + "+")
    print(f"|  Wing area  S          {wing.S_wing:.4f} m^2{'':<25}|")
    print(f"|  Span       b          {wing.b_wing:.4f} m{'':<27}|")
    print(f"|  MAC chord  c          {wing.chord_mean:.4f} m{'':<27}|")
    print(f"|  Aspect ratio AR       {aero.AR:.1f}{'':<32}|")
    print(f"|  Wing loading (design) {WS_design:.1f} N/m^2{'':<23}|")
    print(f"|  Wing mass (Raymer)    {wing.mass_wing_kg:.4f} kg{'':<25}|")
    print("+" + "="*56 + "+")
    print(f"|  Cl_alpha (2D)         {af.Cl_alpha_rad:.4f} /rad{'':<22}|")
    print(f"|  CL_alpha (3D)         {af.CL_alpha_rad:.4f} /rad{'':<22}|")
    print(f"|  CL_max (3D)           {af.CL_max_3D:.4f}{'':<29}|")
    print(f"|  CL cruise             {af.CL_cruise:.4f}{'':<29}|")
    print(f"|  L/D cruise            {af.LD_cruise:.2f}{'':<31}|")
    print(f"|  Oswald e              {af.e_oswald:.4f}{'':<29}|")
    print("+" + "="*56 + "+")
    print(f"|  Stall speed           {V_stall:.2f} m/s{'':<27}|")
    print(f"|  Re cruise             {Re_cruise:.2e}{'':<26}|")
    print(f"|  Re stall              {Re_stall:.2e}{'':<26}|")
    print("+" + "="*56 + "+")
    if af.warnings:
        print("  CONSTRAINT WARNINGS:")
        for w in af.warnings:
            print(f"    ! {w}")
    else:
        print("|  All design constraints: OK" + " "*28 + "|")
    print("+" + "="*56 + "+")


# =====================================================================
#  NB3 -- control vanes
# =====================================================================

def print_mixing_table(rows, delta_max_deg) -> None:
    """Per-vane deflections for the mixing test commands."""
    print(f"{'Command':<20}  {'T':>6} {'B':>6} {'L':>6} {'R':>6}  OK?")
    print("-" * 58)
    for label, d, ok in rows:
        verdict = "OK" if ok else "EXCEEDS"
        print(f"{label:<20}  {d[0]:>+6.1f} {d[1]:>+6.1f} {d[2]:>+6.1f} "
              f"{d[3]:>+6.1f}  {verdict}")


def print_vane_summary(d, wing_af_name) -> None:
    """The control-vane design summary card."""
    p = d.p
    bar = "=" * 60
    print("EDF CONTROL VANE DESIGN - SUMMARY")
    print(bar)
    print()
    print("  Function")
    print("    Jet-deflection control: Pitch, Yaw, Roll")
    print(f"    {p.n_vanes} x vanes at 90 deg spacing (T / B / L / R)")
    print()
    print("  Section choice")
    print(f"    {wing_af_name} (NB2 wing) : REJECTED")
    print("      Thick section, laminar bubble at vane Re, non-monotonic Cl(d).")
    print(f"    Flat plate t/c={p.tc_vane:.2f}  : SELECTED")
    print("      Linear Cl(d) at low Re. No LE bubble. Laser-cut CFRP.")
    print()
    print("  Geometry")
    print(f"    R_tip / R_hub  : {d.R_tip*1e3:.1f} / {d.R_hub*1e3:.1f} mm")
    print(f"    h_vane         : {d.h_vane*1e3:.2f} mm")
    print(f"    c_vane         : {d.c_vane*1e3:.2f} mm  (AR = {p.AR_vane:.1f})")
    print(f"    S_vane         : {d.S_vane*1e4:.3f} cm^2 per vane")
    print()
    print("  Flow at vane plane")
    print(f"    V_jet   : {d.V_jet:.3f} m/s")
    print(f"    q_jet   : {d.q_jet:.2f} Pa  ({d.q_jet/d.q_fs:.1f}x cruise q)")
    print(f"    Re_vane : {d.Re_vane:.0f}")
    print()
    print("  Deflection")
    print(f"    delta_stall : ~{p.delta_stall_deg:.0f} deg  (soft - flat plate)")
    print(f"    delta_max   :  {p.delta_max_deg:.0f} deg")
    print(f"    Thrust loss @ delta_max : {d.thrust_loss_pct(p.delta_max_deg):.2f}%")
    print()
    print(f"  Control authority at delta = {p.delta_design_deg:.0f} deg")
    print(f"    {'Axis':<8} {'Moment [N*mm]':>14}  {'ddot [d/s2]':>12}  {'Pass?':>6}")
    print(f"    {'-'*46}")
    for axis, M, a in [("Pitch", d.M_pitch_Nm, d.ddot_pitch),
                       ("Yaw",   d.M_yaw_Nm,   d.ddot_yaw),
                       ("Roll",  d.M_roll_Nm,  d.ddot_roll)]:
        ok = "OK" if a >= d.ddot_min else "FAIL"
        print(f"    {axis:<8} {M*1e3:>14.3f}  {a:>12.1f}  {ok:>6}")
    print()
    print("  Actuator")
    print(f"    M_hinge @ delta_max   : {d.M_hinge_max*1e3:.4f} N*mm")
    print(f"    Required servo torque : >= {d.servo_torque_req_gcm:.1f} g*cm  "
          f"(SF={p.SF_servo}, +20% margin)")
    print()
    print("  Outputs")
    print("    out/control_vanes.yaml")
    print("    out/vane_authority_curves.png")
    print("    out/vane_geometry.png")


# =====================================================================
#  NB4 / NB12 -- ailerons
# =====================================================================

def print_vane_cruise_authority(auth, ddot_min) -> None:
    """Residual jet-vane authority table, hover vs cruise."""
    print(f"T_hover              : {auth.T_hover_N:.2f} N   "
          f"(s_ratio={auth.s_ratio:.2f} x W)")
    print(f"T_cruise             : {auth.T_cruise_N:.3f} N  "
          f"(W / LD_cruise={auth.LD_cruise:.2f})")
    print(f"Thrust ratio          : {auth.thrust_ratio:.4f}  "
          f"({auth.thrust_ratio*100:.1f}% of hover)")
    print()
    print(f"{'Axis':<8}{'Hover [d/s2]':>14}{'Cruise [d/s2]':>16}{'Margin (cruise/min)':>22}")
    print("-" * 60)
    for axis, hover, cruise in [("Roll",  auth.ddot_roll_hover,  auth.ddot_roll_cruise),
                                ("Pitch", auth.ddot_pitch_hover, auth.ddot_pitch_cruise)]:
        print(f"{axis:<8}{hover:>14.1f}{cruise:>16.1f}{cruise/ddot_min:>21.2f}x")
    print()
    print("Jet-vane roll authority alone in cruise is thin (a few x over the")
    print("floor, down from ~30x in hover) -- this is the gap the ailerons close.")


def print_aileron_summary(ail, ail_p, auth, ddot_roll_total, ddot_min,
                          cruise_ok) -> None:
    """The aileron design summary card."""
    bar = "=" * 60
    print(bar)
    print("  AILERON DESIGN SUMMARY".center(60))
    print(bar)
    print(f"  {'Aileron chord / span':<34}: {ail.c_aileron_m*1e3:.1f} / {ail.b_aileron_m*1e3:.1f} mm")
    print(f"  {'Aileron area (1 side)':<34}: {ail.S_aileron_m2*1e4:.2f} cm^2")
    print(f"  {'Flap effectiveness tau':<34}: {ail.tau:.3f}")
    print()
    print(f"  {'Jet-vane ddot_roll, hover':<34}: {auth.ddot_roll_hover:8.1f} deg/s^2")
    print(f"  {'Jet-vane ddot_roll, cruise':<34}: {auth.ddot_roll_cruise:8.1f} deg/s^2")
    print(f"  {'Aileron ddot_roll, cruise':<34}: {ail.ddot_roll_deg_s2:8.1f} deg/s^2")
    print(f"  {'Combined ddot_roll, cruise':<34}: {ddot_roll_total:8.1f} deg/s^2")
    print(f"  {'Requirement':<34}: {ddot_min:8.1f} deg/s^2")
    print(f"  {'Cruise roll authority':<34}: {'OK' if cruise_ok else 'FAIL'}")
    print()
    print(f"  {'Required servo torque':<34}: >= {ail.servo_torque_req_gcm:.1f} g*cm")
    print(f"  {'Servo + linkage mass (2x)':<34}: "
          f"{2*(ail_p.servo_mass_kg + ail_p.linkage_mass_kg)*1e3:.1f} g")
    print(bar)


def print_aileron_cots_summary(ail, ail_est, servo, stall_gcm, hw_est, hw_cots,
                               ddot_roll_total, margin_vane, margin_ail) -> None:
    """The as-selected (COTS servo) aileron re-solve card (NB12)."""
    bar = "=" * 62
    print(bar)
    print("  AILERON AS-SELECTED RE-SOLVE SUMMARY".center(62))
    print(bar)
    print(f"  {'quantity':<30}{'NB4 (est.)':>12}{'this NB':>12}")
    print("  " + "-" * 58)
    print(f"  {'servo mass, each [g]':<30}{ail_est['servo_mass_kg_each']*1e3:>12.1f}{servo.mass_kg*1e3:>12.1f}")
    print(f"  {'aileron hw total (2x) [g]':<30}{hw_est:>12.1f}{hw_cots:>12.1f}")
    print(f"  {'servo torque req [g cm]':<30}{ail_est['servo_torque_req_gcm']:>12.1f}{ail.servo_torque_req_gcm:>12.1f}")
    print(f"  {'combined cruise ddot [d/s2]':<30}{ail_est['ddot_roll_total_cruise_deg_s2']:>12.1f}{ddot_roll_total:>12.1f}")
    print()
    print(f"  {'frozen servo':<30}: {servo.id}  ({servo.mass_kg*1e3:.1f} g, {stall_gcm:.0f} g cm)")
    print(f"  {'stall margin vane / aileron':<30}: {margin_vane:.1f}x / {margin_ail:.1f}x")
    print(f"  {'hardware delta vs NB4':<30}: {hw_cots - hw_est:+.1f} g  (into NB14 CG re-solve)")
    print(bar)


# =====================================================================
#  NB5 / NB13 -- vibration isolation
# =====================================================================

def print_isolation_table(res) -> None:
    """Per-module isolation sizing table (NB5)."""
    hdr = (f"{'module':<10}{'m [kg]':>8}{'f_n [Hz]':>10}{'r':>7}{'T':>8}"
           f"{'atten':>8}{'d_st [mm]':>11}{'k/iso [N/m]':>13}{'hw [g]':>8}")
    print(hdr)
    print('-' * len(hdr))
    for nm, m in res["modules"].items():
        print(f"{nm:<10}{m.m_isolated_kg:>8.3f}{m.f_n_hz:>10.1f}{m.r_ratio:>7.2f}"
              f"{m.transmissibility:>8.3f}{m.attenuation_pct:>7.1f}%{m.delta_static_mm:>11.3f}"
              f"{m.k_isolator_N_m:>13.0f}{m.hw_mass_kg*1e3:>8.0f}")


def print_vibration_summary(res, vib_p) -> None:
    """The vibration-isolation summary card (NB5)."""
    bar = "=" * 60
    fc = res["modules"]["fc_imu"]
    pl = res["modules"]["payload"]
    print(bar)
    print("  VIBRATION ISOLATION SUMMARY".center(60))
    print(bar)
    print(f"  {'EDF 1/rev forcing':<32}: {res['f_shaft_hz']:8.1f} Hz")
    print(f"  {'Blade-pass forcing':<32}: {res['f_blade_hz']:8.0f} Hz")
    print(f"  {'Isolator corner f_n':<32}: {res['f_n_hz']:8.1f} Hz")
    print(f"  {'Valid window':<32}: {res['f_n_window_hz'][0]:.0f} - {res['f_n_window_hz'][1]:.0f} Hz "
          f"({'OK' if res['window_ok'] else 'OUT'})")
    print(f"  {'1/rev attenuation':<32}: {fc.attenuation_pct:8.1f} %")
    print(f"  {'Static sag (1 g)':<32}: {fc.delta_static_mm:8.3f} mm")
    print(f"  {'Sway space per bay':<32}: {res['sway_mm']:8.2f} mm  ({vib_p.shock_load_g:.0f} g shock)")
    print()
    print(f"  {'FC/IMU isolators':<32}: {fc.n_isolators} x, k = {fc.k_isolator_N_m:.0f} N/m, {fc.hw_mass_kg*1e3:.0f} g")
    print(f"  {'Payload isolators':<32}: {pl.n_isolators} x, k = {pl.k_isolator_N_m:.0f} N/m, {pl.hw_mass_kg*1e3:.0f} g")
    print(f"  {'Total isolator hardware':<32}: {(fc.hw_mass_kg+pl.hw_mass_kg)*1e3:8.0f} g")
    print(bar)


def print_vibration_cots_table(res) -> None:
    """Per-module isolation table for the COTS re-solve (NB13)."""
    hdr = (f"{'module':<10}{'m [kg]':>8}{'f_n [Hz]':>10}{'T':>8}"
           f"{'d_st [mm]':>11}{'k/iso [N/m]':>13}{'hw [g]':>8}")
    print(hdr)
    print('-' * len(hdr))
    for nm, m in res["modules"].items():
        print(f"{nm:<10}{m.m_isolated_kg:>8.3f}{m.f_n_hz:>10.1f}"
              f"{m.transmissibility:>8.3f}{m.delta_static_mm:>11.3f}"
              f"{m.k_isolator_N_m:>13.0f}{m.hw_mass_kg*1e3:>8.0f}")


def print_vibration_cots_summary(res, vib_est, fc) -> None:
    """The as-selected (COTS FC) vibration re-solve card (NB13)."""
    fc_mod = res["modules"]["fc_imu"]
    pl_mod = res["modules"]["payload"]

    bar = "=" * 62
    print(bar)
    print("  VIBRATION AS-SELECTED RE-SOLVE SUMMARY".center(62))
    print(bar)
    print(f"  {'quantity':<32}{'NB5 (est.)':>12}{'this NB':>12}")
    print("  " + "-" * 58)
    print(f"  {'isolated FC/IMU mass [g]':<32}{vib_est['modules']['fc_imu']['m_isolated_kg']*1e3:>12.1f}{fc_mod.m_isolated_kg*1e3:>12.1f}")
    print(f"  {'FC isolator k [N/m]':<32}{vib_est['modules']['fc_imu']['k_isolator_N_m']:>12.0f}{fc_mod.k_isolator_N_m:>12.0f}")
    print(f"  {'corner frequency f_n [Hz]':<32}{vib_est['f_n_hz']:>12.1f}{res['f_n_hz']:>12.1f}")
    print(f"  {'sway pad total [mm]':<32}{vib_est['sway_pad_total_m']*1e3:>12.2f}{2*res['sway_mm']:>12.2f}")
    print()
    print(f"  {'frozen FC':<32}: {fc.id}  ({fc.mass_kg*1e3:.1f} g)")
    print(f"  {'1/rev attenuation':<32}: {fc_mod.attenuation_pct:.1f} %  (target met, window OK)")
    print(f"  {'payload isolators':<32}: {pl_mod.n_isolators} x, k = {pl_mod.k_isolator_N_m:.0f} N/m (unchanged)")
    print(bar)


# =====================================================================
#  NB6 / NB14 -- fuselage
# =====================================================================

def print_fuselage_sizing(fus) -> None:
    """Headline geometry of the sized fuselage (NB6)."""
    print(f"Active constraint : {fus.active_constraint.upper()}")
    print(f"D_fus             : {fus.D_fus*1e3:7.1f} mm")
    print(f"L_fus             : {fus.L_fus*1e3:7.1f} mm   (fineness {fus.fineness:.2f})")
    print(f"  nose / mid / tail : {fus.L_nose*1e3:.0f} / {fus.L_mid*1e3:.0f} / {fus.L_tail*1e3:.0f} mm")
    print(f"Enclosed volume   : {fus.V_total_m3*1e3:7.2f} L")
    print(f"Bay stack         : {fus.L_stack_m*1e3:7.1f} mm  of  {fus.L_avail_m*1e3:.1f} mm available "
          f"({fus.L_stack_m/fus.L_avail_m*100:.0f}% used)")
    print(f"  incl. sway pad  : {fus.sway_pad_m*1e3:.2f} mm rattle space (NB5 isolation)")
    print(f"Tail exit radius  : {fus.r_hub*1e3:7.1f} mm  (EDF hub)")
    print(f"Clamshell lid     : [0, {fus.x_clam_aft_m*1e3:.1f}] mm  (hinged upper half, ADR-0010)")
    print(f"Structure ({fus.construction_method}) : {fus.m_shell_kg*1e3:.1f} g  "
          f"(fdm {fus.m_semimono_fdm_kg*1e3:.0f} / cfrp {fus.m_semimono_cfrp_kg*1e3:.0f} g)")
    print(f"Wing spar         : d{fus.spar_od_m*1e3:.0f} x {fus.spar_length_m*1e3:.0f} mm "
          f"CFRP tube ({fus.m_spar_kg*1e3:.1f} g), joint hw {fus.m_joint_hw_kg*1e3:.1f} g")


def print_drag_budget(fus, af, aero, S_wing, CD0_misc) -> None:
    """CD0 build-up vs the sizing assumption (NB6)."""
    CD0_wing_est = af["Cd0_section"] * 1.10
    CD0_total    = CD0_wing_est + fus.CD0_fus + CD0_misc

    print(f"Re_L (cruise)        : {fus.Re_L:.3g}")
    print(f"Cf  (turbulent)      : {fus.Cf:.5f}")
    print(f"FF  (Raymer)         : {fus.FF:.3f}")
    print(f"S_wet fuselage       : {fus.S_wet_m2:.4f} m^2   (S_wet/S_wing = {fus.S_wet_m2/S_wing:.2f})")
    print()
    print(f"{'CD0 wing (NB2 section x1.10)':<32}: {CD0_wing_est:.5f}")
    print(f"{'CD0 fuselage (this notebook)':<32}: {fus.CD0_fus:.5f}")
    print(f"{'CD0 duct/vanes/misc allowance':<32}: {CD0_misc:.5f}")
    print("-" * 46)
    print(f"{'CD0 total (built up)':<32}: {CD0_total:.5f}")
    print(f"{'CD0 assumed in sizing (NB1)':<32}: {aero.CD0:.5f}")
    margin = (aero.CD0 - CD0_total) / aero.CD0 * 100
    verdict = "CLOSES" if CD0_total <= aero.CD0 else "EXCEEDED -- revisit aerodynamics.yaml"
    print(f"\nDrag budget {verdict}  (margin {margin:+.1f}%)")


def print_layout_table(fus, MTOW_kg, fus_p) -> None:
    """Layout item table with the CG and wing stations (NB6)."""
    print(f"{'item':<14}{'mass [kg]':>10}{'x_cg [mm]':>12}")
    for it in fus.items:
        print(f"{it.name:<14}{it.mass_kg:>10.3f}{it.x_cg*1e3:>12.1f}")
    print("-" * 36)
    print(f"{'CG':<14}{MTOW_kg:>10.3f}{fus.x_CG*1e3:>12.1f}")
    print(f"\nWing LE station : {fus.x_wing_LE*1e3:.1f} mm   (AC at {fus.x_wing_AC*1e3:.1f} mm, "
          f"SM = {fus_p.static_margin*100:.0f}% MAC)")


def print_vane_arm_check(fus, vanes) -> None:
    """Actual vane-to-CG arm vs the NB3 sizing assumption (NB6)."""
    L_assumed = vanes["L_CG_m"]
    L_actual  = fus.L_vane_arm
    factor    = L_actual / L_assumed

    print(f"Vane station              : {fus.x_vane*1e3:.1f} mm")
    print(f"CG station                : {fus.x_CG*1e3:.1f} mm")
    print(f"Actual vane-to-CG arm     : {L_actual*1e3:.1f} mm")
    print(f"NB3 assumed arm (L_CG_m)  : {L_assumed*1e3:.1f} mm")
    print(f"\nControl authority factor  : x{factor:.2f}")
    print(f"Pitch moment available    : {vanes['F_vane_design_N'] * L_actual:.3f} Nm "
          f"(NB3 designed for {vanes['M_pitch_design_Nm']:.3f} Nm)")
    if factor >= 1.0:
        print("NB3 vane sizing is CONSERVATIVE -- authority margin is positive.")
    else:
        print("WARNING: actual arm SHORTER than assumed -- re-run NB3 with L_CG_m updated.")


def print_fuselage_summary(fus, fus_p, vanes) -> None:
    """The fuselage design summary card (NB6)."""
    bar = "=" * 60
    print(bar)
    print("  FUSELAGE DESIGN SUMMARY".center(60))
    print(bar)
    print(f"  {'Max diameter D_fus':<34}: {fus.D_fus*1e3:8.1f} mm")
    print(f"  {'Length L_fus':<34}: {fus.L_fus*1e3:8.1f} mm")
    print(f"  {'Fineness ratio L/D':<34}: {fus.fineness:8.2f}")
    print(f"  {'Active diameter constraint':<34}: {fus.active_constraint:>8s}")
    print(f"  {'Nose / mid / tail':<34}: {fus.L_nose*1e3:.0f} / {fus.L_mid*1e3:.0f} / {fus.L_tail*1e3:.0f} mm")
    print(f"  {'Tail exit radius (EDF hub)':<34}: {fus.r_hub*1e3:8.1f} mm")
    print()
    print(f"  {'Enclosed volume':<34}: {fus.V_total_m3*1e3:8.2f} L")
    print(f"  {'Bay stack utilisation':<34}: {fus.L_stack_m/fus.L_avail_m*100:8.0f} %")
    print(f"  {'Wetted area':<34}: {fus.S_wet_m2:8.4f} m^2")
    print(f"  {'CD0 fuselage (on S_wing)':<34}: {fus.CD0_fus:8.5f}")
    print(f"  {'Shell + frames mass':<34}: {fus.m_shell_kg:8.3f} kg  "
          f"({fus.m_shell_kg/fus.m_struct_budget_kg*100:.0f}% of budget)")
    print()
    print(f"  {'CG station (from nose)':<34}: {fus.x_CG*1e3:8.1f} mm")
    print(f"  {'Wing LE station':<34}: {fus.x_wing_LE*1e3:8.1f} mm")
    print(f"  {'Static margin':<34}: {fus_p.static_margin*100:8.1f} % MAC")
    print(f"  {'Vane control arm (actual)':<34}: {fus.L_vane_arm*1e3:8.1f} mm")
    print(f"  {'Control authority vs NB3':<34}: x{fus.L_vane_arm/vanes['L_CG_m']:7.2f}")
    print()
    print(f"  {'Duct chord':<34}: {fus.duct_chord*1e3:8.1f} mm")
    print(f"  {'Duct inner / outer diameter':<34}: {fus.D_duct_inner*1e3:.0f} / {fus.D_duct_outer*1e3:.0f} mm")
    print(bar)
    print("  Axis convention: body FRD (x fwd, y right, z down),")
    print("  stations from nose +aft, x_body = -station  [Aetherion]")
    print(bar)


def print_fuselage_cots_summary(fus, fus_est, fus_p_cots, m_asselected,
                                m_delta, fit) -> None:
    """The as-selected fuselage re-solve card (NB14)."""
    bar = "=" * 62
    print(bar)
    print("  FUSELAGE AS-SELECTED RE-SOLVE SUMMARY".center(62))
    print(bar)
    print(f"  {'quantity':<30}{'NB6 (est.)':>13}{'this NB':>13}")
    print("  " + "-" * 58)
    print(f"  {'D_fus [mm]':<30}{fus_est['D_fus_m']*1e3:>13.1f}{fus.D_fus*1e3:>13.1f}")
    print(f"  {'L_fus [mm]':<30}{fus_est['L_fus_m']*1e3:>13.1f}{fus.L_fus*1e3:>13.1f}")
    print(f"  {'CG station [mm]':<30}{fus_est['x_CG_m']*1e3:>13.1f}{fus.x_CG*1e3:>13.1f}")
    print(f"  {'S_wet [m^2]':<30}{fus_est['S_wet_m2']:>13.4f}{fus.S_wet_m2:>13.4f}")
    print(f"  {'CD0 fuselage':<30}{fus_est['CD0_fus']:>13.5f}{fus.CD0_fus:>13.5f}")
    print(f"  {'vane arm [mm]':<30}{fus_est['L_vane_arm_m']*1e3:>13.1f}{fus.L_vane_arm*1e3:>13.1f}")
    print()
    print(f"  {'As-selected all-up mass':<30}: {m_asselected:.3f} kg  ({m_delta*1e3:+.0f} g vs closure)")
    print(f"  {'Bay fit':<30}: " +
          ("all frozen parts fit" if all(e['ok'] for e in fit)
           else f"{sum(1 for e in fit if not e['ok'])} misfit(s) -- see Section 4"))
    print(f"  {'Static margin':<30}: {fus_p_cots.static_margin*100:.1f} % MAC (by construction)")
    print(bar)
    print("  The conceptual geometry (out/fuselage.yaml) remains the CAD/CFD")
    print("  baseline; adopting these deltas is a config revision (ADR-0012).")
    print(bar)


def print_bay_fit_table(fit) -> None:
    """Frozen-part physical-fit report against the re-solved hull (NB14)."""
    print(f"{'part':<22}{'where':<28}{'need [mm]':>10}{'have [mm]':>10}{'fit':>6}")
    print("-" * 76)
    for e in fit:
        need = f"{e['need_mm']:.1f}" if e["need_mm"] is not None else "n/a"
        print(f"{e['part']:<22}{e['where']:<28}{need:>10}{e['have_mm']:>10.1f}"
              f"{'  ok' if e['ok'] else '  NO':>6}")

    n_misfit = sum(1 for e in fit if not e["ok"])
    print()
    print("All frozen parts fit the re-solved hull." if n_misfit == 0 else
          f"{n_misfit} part(s) do NOT fit -- standing finding(s) against the "
          f"packaging assumptions.")


# =====================================================================
#  NB7 -- thermal design
# =====================================================================

def print_thermal_sizing(res, tp, m_esc_alloc) -> None:
    """ESC cold-plate and vented battery-bay sizing report (NB7)."""
    e = res["esc"]
    b = res["battery"]
    print(f"Induced inflow v_h : {res['v_h_ms']:.1f} m/s   (Pr = {res['Pr']:.3f})")
    print()
    print("ESC COLD-PLATE")
    print(f"  heat load Q         : {e.Q_W:5.1f} W   (P_hover x (1 - eta_esc))")
    print(f"  cooling airflow     : {e.V_cool_ms:5.1f} m/s  ({tp.esc_inflow_frac:.2f} x v_h, mid-body)")
    print(f"  required plate area : {e.A_req_m2*1e4:5.0f} cm^2  ({e.plate_side_mm:.0f} mm square, "
          f"{tp.plate_thickness_m*1e3:.0f} mm {tp.plate_material})")
    print(f"  plate mass          : {e.plate_mass_kg*1e3:5.0f} g   vs ESC allocation {m_esc_alloc*1e3:.0f} g "
          f"-> {'WITHIN' if e.mass_within_alloc else 'EXCEEDS'}")
    print(f"  available wall area : {e.A_avail_m2*1e4:5.0f} cm^2  (headroom {e.area_headroom:.2f}x)")
    print(f"  ESC temp @ full wall: {e.T_at_avail_C:5.0f} C   (limit {tp.T_esc_max_C:.0f} C, margin {e.temp_margin_C:+.0f} C)")
    print(f"  verdict             : {'OK' if e.ok else 'MARGINAL / INFEASIBLE -- see finding below'}")
    print()
    print("VENTED BATTERY BAY")
    print(f"  heat load Q         : {b.Q_W:5.1f} W   (I_hover^2 x R_pack nominal)")
    print(f"  through-vent airflow: {b.V_vent_ms:5.1f} m/s  ({tp.vent_inflow_frac:.2f} x v_h)")
    print(f"  required vent area  : {b.A_req_m2*1e4:5.1f} cm^2")
    print(f"  available wall area : {b.A_avail_m2*1e4:5.0f} cm^2  (headroom {b.area_headroom:.1f}x)")
    print(f"  pack temp @ full vent: {b.T_at_avail_C:4.0f} C   (limit {tp.T_batt_max_C:.0f} C, margin {b.temp_margin_C:+.0f} C)")
    print(f"  verdict             : {'OK' if b.ok else 'INFEASIBLE'}")


def print_battery_transient(trans, tp) -> None:
    """Pack mission-transient card (NB7, ADR-0014)."""
    print(f"Pack thermal mass  : C_th = {trans.C_th_J_K:.0f} J/K, "
          f"cruise UA = {trans.UA_W_K:.3f} W/K  ->  tau = {trans.tau_s/60:.1f} min")
    print(f"Mission profile    : 2 x {trans.t_leg_s:.0f} s vertical legs "
          f"(adiabatic) around {trans.t_cruise_s:.0f} s cruise "
          f"(Re_L = {trans.Re_L:.3g}, laminar)")
    print(f"Currents           : hover {trans.I_hover_A:.1f} A, "
          f"cruise {trans.I_cruise_A:.1f} A  (wiring-law bus voltage)")
    print()
    print(f"{'case':<14}{'R [mOhm]':>10}{'Q_hov [W]':>11}{'dT/leg [C]':>12}"
          f"{'after leg1':>12}{'after cruise':>14}{'final [C]':>11}{'margin':>9}")
    print("-" * 93)
    for c in trans.cases.values():
        print(f"{c.label:<14}{c.R_pack_ohm*1e3:>10.0f}{c.Q_hover_W:>11.1f}"
              f"{c.dT_leg_C:>12.1f}{c.T_after_leg1_C:>12.1f}"
              f"{c.T_after_cruise_C:>14.1f}{c.T_final_C:>11.1f}"
              f"{c.temp_margin_C:>+9.1f}")
    print("-" * 93)
    print(f"Limit: pack average <= {tp.T_batt_max_C:.0f} C at "
          f"{tp.T_ambient_C:.0f} C hot-day ambient (spatial average -- "
          f"cell cores/busbars run 5-10 C hotter until validated by test).")


def print_thermal_findings(res, tp, m_esc_alloc, trans=None) -> None:
    """Standing-finding narration for the two thermal paths (NB7)."""
    e = res["esc"]
    b = res["battery"]
    print("FINDINGS")
    print("=" * 64)
    if b.ok:
        print(f"[OK]   Battery bay vents comfortably: {b.A_req_m2*1e4:.1f} cm^2 needed of "
              f"{b.A_avail_m2*1e4:.0f} cm^2 available ({b.area_headroom:.0f}x), "
              f"{b.temp_margin_C:+.0f} C margin. Free in the current geometry.")
    else:
        print("[FAIL] Battery bay cannot be vented within the available wall.")

    if e.ok:
        print(f"[OK]   ESC cold-plate fits: {e.A_req_m2*1e4:.0f} cm^2 of "
              f"{e.A_avail_m2*1e4:.0f} cm^2, {e.plate_mass_kg*1e3:.0f} g within allocation, "
              f"{e.temp_margin_C:+.0f} C margin.")
    else:
        print("[WARN] ESC cold-plate is MARGINAL at the current design point:")
        if not e.mass_within_alloc:
            print(f"       - plate mass {e.plate_mass_kg*1e3:.0f} g EXCEEDS the ESC allocation "
                  f"{m_esc_alloc*1e3:.0f} g by {(e.plate_mass_kg-m_esc_alloc)*1e3:.0f} g")
        if e.area_headroom < 1.0:
            print(f"       - required {e.A_req_m2*1e4:.0f} cm^2 > available {e.A_avail_m2*1e4:.0f} cm^2 wall")
        print(f"       - only {e.temp_margin_C:+.0f} C margin at {tp.T_ambient_C:.0f} C ambient")
        print("       Root cause: ~51 W ESC loss at the 3.06 kg / ~1 kW-hover design")
        print("       point, cooled mid-body at only half the induced velocity.")
        print("       Levers (next design pass, not resolved here):")
        print("       * higher eta_esc (a 98% ESC drops the load to ~20 W)")
        print("       * ESC nearer the inlet (higher esc_inflow_frac) -- CG cost")
        print("       * a lighter/finned spreader, or accept a larger propulsion")
        print("         fraction for the cold-plate")

    if trans is not None:
        n = trans.cases["nominal"]
        if trans.ok_nominal:
            print(f"[OK]   Pack mission transient (ADR-0014): nominal "
                  f"{n.R_pack_ohm*1e3:.0f} mOhm case ends the mission at "
                  f"{n.T_final_C:.0f} C ({n.temp_margin_C:+.0f} C margin).")
        else:
            print(f"[WARN] Pack mission transient (ADR-0014) EXCEEDS the "
                  f"{tp.T_batt_max_C:.0f} C pack limit at the hot-day ambient:")
            print(f"       - nominal {n.R_pack_ohm*1e3:.0f} mOhm pack ends the "
                  f"mission at {n.T_final_C:.0f} C ({n.temp_margin_C:+.0f} C)")
            print("       - the vent check above is air-side only; the pack's")
            print("         own thermal mass integrates the I^2 R hover heat")
            print("       Levers (next pass / procurement, not resolved here):")
            print("       * build/buy a low-resistance pack (thick nickel+copper")
            print("         busbars; measure DCIR -- the optimistic case passes)")
            print("       * duct part of the hover vent flow directly over the")
            print("         pack (the legs are modelled adiabatic)")
            print("       * mission-plan around the limit at high ambient")
    print("=" * 64)


def print_thermal_summary(res, tp, trans=None) -> None:
    """The thermal-path design summary card (NB7)."""
    e = res["esc"]
    b = res["battery"]
    bar = "=" * 60
    print(bar)
    print("  THERMAL-PATH DESIGN SUMMARY".center(60))
    print(bar)
    print(f"  {'Ambient (design)':<34}: {tp.T_ambient_C:8.0f} degC")
    print(f"  {'Induced inflow v_h':<34}: {res['v_h_ms']:8.1f} m/s")
    print()
    print(f"  {'ESC heat load':<34}: {e.Q_W:8.1f} W")
    print(f"  {'ESC cold-plate':<34}: {e.A_req_m2*1e4:.0f} cm^2, {e.plate_mass_kg*1e3:.0f} g {tp.plate_material}")
    print(f"  {'ESC temp / limit':<34}: {e.T_at_avail_C:.0f} / {tp.T_esc_max_C:.0f} degC "
          f"(margin {e.temp_margin_C:+.0f})")
    print(f"  {'ESC path':<34}: {'OK' if e.ok else 'MARGINAL':>8s}")
    print()
    print(f"  {'Battery heat load':<34}: {b.Q_W:8.1f} W")
    print(f"  {'Battery vent area':<34}: {b.A_req_m2*1e4:.1f} cm^2")
    print(f"  {'Battery temp / limit':<34}: {b.T_at_avail_C:.0f} / {tp.T_batt_max_C:.0f} degC "
          f"(margin {b.temp_margin_C:+.0f})")
    print(f"  {'Battery path':<34}: {'OK' if b.ok else 'FAIL':>8s}")
    if trans is not None:
        n = trans.cases["nominal"]
        print()
        print(f"  {'Pack transient (nominal R)':<34}: {n.T_final_C:8.0f} degC "
              f"end-of-mission (limit {tp.T_batt_max_C:.0f})")
        print(f"  {'Pack transient check':<34}: "
              f"{'OK' if trans.ok_nominal else 'FINDING':>8s}")
    print(bar)


# =====================================================================
#  NB9 -- mass properties
# =====================================================================

def print_component_table(components) -> None:
    """Primitive-by-primitive component inertia table (NB9)."""
    print(f"{'component':<14}{'primitive':<22}{'m [kg]':>8}{'x_s [mm]':>10}"
          f"{'Ixx':>12}{'Iyy':>12}{'Izz':>12}   [kg m^2, about own CG]")
    print("-" * 92)
    for c in components:
        print(f"{c.name:<14}{c.primitive:<22}{c.mass_kg:>8.4f}{c.x_s*1e3:>10.1f}"
              f"{c.I_cg[0]:>12.3e}{c.I_cg[1]:>12.3e}{c.I_cg[2]:>12.3e}")


def print_cg_trim_report(mp, result, fus) -> None:
    """As-packaged CG, battery-tray trim solution, and trimmed tensor (NB9)."""
    print(f"Total mass              : {mp.m_tot0:.4f} kg   "
          f"(mass closure {result.m_total_kg:.4f} kg)")
    print(f"As-packaged CG station  : {mp.x_cg0*1e3:.2f} mm from nose")
    print(f"  NB4 handoff           : {fus['x_CG_m']*1e3:.2f} mm  "
          f"-> refined CG is {mp.dx_cg0*1e3:+.1f} mm "
          f"({'aft' if mp.dx_cg0 > 0 else 'fwd'}) of the NB4 estimate")
    print(f"As-packaged static margin: {mp.SM_ref0:+.3f} MAC  (NB4 target {mp.SM_nb4:+.3f})")

    print()
    if mp.trim_ok:
        print(f"Battery-tray trim       : {mp.x_battery_trim*1e3:+.1f} mm "
              f"({'aft' if mp.x_battery_trim > 0 else 'fwd'} of nominal), "
              f"{abs(mp.x_battery_trim)/mp.travel*100:.0f}% of +/-{mp.travel*1e3:.0f} mm rail travel used")
        print(f"Trimmed CG station      : {mp.x_cg*1e3:.2f} mm from nose")
        print(f"Trimmed static margin   : {mp.SM_ref:+.3f} MAC  (target {mp.SM_nb4:+.3f})  -> OK")
    else:
        print(f"*** Required trim {mp.x_battery_trim*1e3:+.1f} mm EXCEEDS the "
              f"+/-{mp.travel*1e3:.0f} mm rail travel -- mechanical trim alone cannot")
        print("*** restore the target margin.  Re-balance the NB4 packing "
              "(battery/payload forward, or wing aft), then re-run NB4 -> NB6.")
    print(f"CG body-frame r_CG      : [{mp.r_cg_body[0]:.5f}, 0, 0] m  (FRD, nose origin)")
    print()
    print("Inertia tensor about CG, body FRD [kg m^2]:")
    import numpy as np
    with np.printoptions(precision=5, suppress=True):
        print(mp.I_cg)
    print()
    print(f"  Ixx (roll)  = {mp.Ixx:.5f} kg m^2   k_x = {mp.k[0]*1e3:.1f} mm")
    print(f"  Iyy (pitch) = {mp.Iyy:.5f} kg m^2   k_y = {mp.k[1]*1e3:.1f} mm")
    print(f"  Izz (yaw)   = {mp.Izz:.5f} kg m^2   k_z = {mp.k[2]*1e3:.1f} mm")
    print(f"  Iyy/Ixx     = {mp.Iyy/mp.Ixx:.1f}  -> slender tail-sitter: pitch/yaw dominated")


def print_top_iyy_contributors(mp, n=4) -> None:
    """Ranked Iyy contributions from the inertia build-up (NB9)."""
    import numpy as np
    from .mass_properties import inertia_contributions
    names, own, transp = inertia_contributions(mp)
    print("Largest Iyy contributors:")
    tot_yy = own[:, 1] + transp[:, 1]
    for i in np.argsort(tot_yy)[::-1][:n]:
        print(f"  {names[i]:<14}{tot_yy[i]:.4f} kg m^2 "
              f"({100 * tot_yy[i] / mp.Iyy:.0f} %)")
