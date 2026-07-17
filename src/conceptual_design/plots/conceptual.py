# conceptual_design/plots/conceptual.py
"""Figures for the conceptual-design notebook (NB1)."""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

from ..notebook import PALETTE as COLORS
from ..vtol_power import (
    climb_pw_terms,
    vtol_climb_power_to_weight,
    vtol_hover_power_to_weight,
)


def plot_size_matching_diagram(smd, save_path):
    """Constraint curves, upper envelope, stall limit and design point."""
    WS = smd.WS_array

    fig, ax = plt.subplots(figsize=(9, 5))

    curves = [
        (smd.tw_climb,   'Climb',            COLORS[0], '-'),
        (smd.tw_cruise,  'Cruise',           COLORS[1], '-'),
        (smd.tw_ceiling, 'Service ceiling',  COLORS[2], '--'),
        (smd.tw_vmax,    'Max speed',        COLORS[3], '-.'),
        (smd.tw_takeoff, 'Take-off run',     COLORS[4], ':'),
    ]
    for arr, label, c, ls in curves:
        ax.plot(WS, arr, color=c, linestyle=ls, lw=1.8, label=label)

    ax.plot(WS, smd.tw_envelope, 'k-', lw=2.5, label='Upper envelope', zorder=5)
    ax.axvline(smd.WS_stall, color='navy', lw=1.5, ls='--',
               label=f'Stall limit ({smd.WS_stall:.0f} N/m^2)')

    mask = WS <= smd.WS_stall
    ax.fill_between(WS[mask], 0, smd.tw_envelope[mask], alpha=0.08,
                    color='steelblue', label='Feasible region')

    ax.plot(smd.WS_design, smd.TW_design, 'r*', ms=14, zorder=10,
            label=f'Design point ({smd.WS_design:.0f} N/m^2, T/W={smd.TW_design:.3f})')
    ax.annotate(f' Design\n ({smd.WS_design:.0f}, {smd.TW_design:.3f})',
                xy=(smd.WS_design, smd.TW_design),
                xytext=(smd.WS_design + 25, smd.TW_design + 0.03),
                fontsize=8, color='crimson',
                arrowprops=dict(arrowstyle='->', color='crimson', lw=1.2))

    ax.set_xlim(30, 400)
    ax.set_ylim(0, 0.6)
    ax.set_xlabel('Wing loading  $W/S$  [N/m^2]', fontsize=11)
    ax.set_ylabel('Thrust-to-weight ratio  $T/W$  [-]', fontsize=11)
    ax.set_title('Size Matching Diagram -- Forward Flight Performance Constraints',
                 fontsize=12, fontweight='bold')
    ax.legend(fontsize=8, loc='upper right', framealpha=0.9)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    return fig


def plot_vtol_power_analysis(vtol_p, mission, smd, D_rotor_m, DL_est,
                             DL_range, save_path):
    """P/W vs disk loading plus the term-breakdown pie at DL_est."""
    PW_hover = np.array([vtol_hover_power_to_weight(dl, vtol_p)
                         for dl in DL_range])
    PW_climb = np.array([vtol_climb_power_to_weight(
        mission.rate_of_climb, dl, smd.WS_design, D_rotor_m, vtol_p)
        for dl in DL_range])
    terms = climb_pw_terms(mission.rate_of_climb, DL_est, smd.WS_design,
                           D_rotor_m, vtol_p)

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))

    ax = axes[0]
    ax.plot(DL_range, PW_hover, color=COLORS[0], lw=2.0, label='Hover  (RoC = 0)')
    ax.plot(DL_range, PW_climb, color=COLORS[1], lw=2.0, ls='--',
            label=f'Climb  (RoC = {mission.rate_of_climb} m/s)')
    ax.axvline(DL_est, color='gray', lw=1.2, ls=':',
               label=f'Provisional DL = {DL_est:.0f} N/m^2')
    ax.set_xlabel('Disk loading  $W/S_{rotor}$  [N/m^2]', fontsize=10)
    ax.set_ylabel('P/W (shaft)  [W/N]', fontsize=10)
    ax.set_title('VTOL Power-to-Weight vs Disk Loading',
                 fontsize=11, fontweight='bold')
    ax.legend(fontsize=9)

    ax2 = axes[1]
    labels = ['Induced\n+ climb', 'Blade\nprofile', 'Fuselage\nparasite',
              'Wing\nparasite']
    colors_pie = [COLORS[0], COLORS[1], COLORS[2], COLORS[3]]
    wedges, texts, autotexts = ax2.pie(
        list(terms.values()), labels=labels, colors=colors_pie,
        autopct='%1.0f%%', startangle=90, textprops={'fontsize': 9},
    )
    for at in autotexts:
        at.set_fontsize(9)
    ax2.set_title(f'P/W Term Breakdown at DL={DL_est:.0f} N/m^2',
                  fontsize=11, fontweight='bold')

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    return fig


def plot_mass_closure_convergence(result, save_path):
    """MTOW/battery-mass iteration history and the residual on a log axis."""
    iters    = [s.iteration     for s in result.history]
    m_totals = [s.m_total_kg    for s in result.history]
    m_batts  = [s.m_batt_kg     for s in result.history]
    m_new    = [s.m_batt_new_kg for s in result.history]
    deltas   = [abs(s.m_batt_new_kg - s.m_batt_kg) for s in result.history]

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))

    ax = axes[0]
    ax.plot(iters, m_totals, 'o-', color=COLORS[0], lw=2.0, ms=5, label='MTOW')
    ax.plot(iters, m_batts, 's--', color=COLORS[1], lw=1.8, ms=5,
            label='Battery mass (old)')
    ax.plot(iters, m_new, '^:', color=COLORS[2], lw=1.6, ms=4,
            label='Battery mass (new)')
    ax.axhline(result.m_total_kg,   color=COLORS[0], lw=0.8, ls='--', alpha=0.5)
    ax.axhline(result.m_battery_kg, color=COLORS[1], lw=0.8, ls='--', alpha=0.5)
    ax.set_xlabel('Iteration', fontsize=10)
    ax.set_ylabel('Mass  [kg]', fontsize=10)
    ax.set_title('Mass Closure Convergence', fontsize=11, fontweight='bold')
    ax.legend(fontsize=9)

    ax2 = axes[1]
    ax2.semilogy(iters, deltas, 'o-', color='crimson', lw=2.0, ms=5)
    ax2.axhline(1e-6, color='gray', lw=1.2, ls='--',
                label='Convergence tol = 1e-6 kg')
    ax2.set_xlabel('Iteration', fontsize=10)
    ax2.set_ylabel('|delta m_battery|  [kg]', fontsize=10)
    ax2.set_title('Residual -- Battery Mass Change per Iteration',
                  fontsize=11, fontweight='bold')
    ax2.legend(fontsize=9)

    ax.annotate(f'MTOW = {result.m_total_kg:.3f} kg',
                xy=(iters[-1], result.m_total_kg),
                xytext=(iters[-1] - len(iters) * 0.35, result.m_total_kg + 0.2),
                fontsize=8, color=COLORS[0],
                arrowprops=dict(arrowstyle='->', color=COLORS[0], lw=1.0))

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    return fig


def plot_sensitivity_grid(sweeps, base_mtow, save_path):
    """2x2 MTOW sensitivity grid over the four standard sweeps."""
    panels = [
        ("specific_energy", 1.0,  'Battery specific energy  [Wh/kg]',
         'Sensitivity to Battery Technology'),
        ("D_rotor",         1e3,  'EDF diameter  [mm]',
         'Sensitivity to EDF Diameter (COTS choice)'),
        ("LD",              1.0,  'Cruise L/D ratio  [-]',
         'Sensitivity to Aerodynamic Efficiency'),
        ("fs",              100., 'Structural weight fraction  $f_s$  [%]',
         'Sensitivity to Structural Weight Fraction'),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    for ax, color, (key, scale, xlabel, title) in zip(
            axes.flat, COLORS, panels):
        s = sweeps[key]
        ax.plot(np.asarray(s.values) * scale, s.mtow, color=color, lw=2.0)
        bx = s.baseline * scale
        ax.axvline(bx, color='gray', lw=1.0, ls=':', alpha=0.7)
        ax.axhline(base_mtow, color='gray', lw=1.0, ls=':', alpha=0.7)
        ax.plot(bx, base_mtow, 'r*', ms=12, zorder=10,
                label=f'Baseline ({bx}, {base_mtow:.3f} kg)')
        ax.legend(fontsize=8)
        ax.set_xlabel(xlabel, fontsize=10)
        ax.set_ylabel('MTOW  [kg]', fontsize=10)
        ax.set_title(title, fontsize=11, fontweight='bold')

    fig.suptitle('MTOW Sensitivity Analysis -- Baseline = {:.3f} kg'.format(base_mtow),
                 fontsize=13, fontweight='bold', y=1.01)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    return fig


def plot_mass_budget(result, save_path):
    """Mass-budget pie and per-phase power/energy bars."""
    fig = plt.figure(figsize=(13, 5))
    gs = GridSpec(1, 3, figure=fig, wspace=0.35)

    ax1 = fig.add_subplot(gs[0, :2])
    labels_pie = [
        f'Battery\n{result.m_battery_kg:.3f} kg',
        f'Structure\n{result.m_structure_kg:.3f} kg',
        f'Propulsion HW\n{result.m_propulsion_kg:.3f} kg',
        f'Avionics\n{result.m_avionics_kg:.3f} kg',
        f'Misc\n{result.m_misc_kg:.3f} kg',
        f'Payload\n{result.m_payload_kg:.3f} kg',
    ]
    sizes_pie = [
        result.m_battery_kg, result.m_structure_kg, result.m_propulsion_kg,
        result.m_avionics_kg, result.m_misc_kg, result.m_payload_kg,
    ]
    pcolors = ['#f68b33', '#2c7bb6', '#d7191c', '#1a9641', '#762a83', '#aaaaaa']
    explode = [0.04] * len(sizes_pie)

    wedges, texts, autotexts = ax1.pie(
        sizes_pie, labels=labels_pie, colors=pcolors,
        autopct='%1.1f%%', startangle=140, explode=explode,
        textprops={'fontsize': 8.5},
    )
    for at in autotexts:
        at.set_fontsize(8)
    ax1.set_title(f'Mass Budget  --  MTOW = {result.m_total_kg:.3f} kg',
                  fontsize=12, fontweight='bold')

    ax2 = fig.add_subplot(gs[0, 2])
    phases   = ['Hover\n(VTOL)', 'Cruise\n(FW)']
    powers   = [result.P_hover_W, result.P_cruise_W]
    energies = [result.E_hover_Wh, result.E_cruise_Wh]
    bars = ax2.bar(phases, powers, color=[COLORS[0], COLORS[1]],
                   width=0.5, edgecolor='white')
    for bar, pw, ew in zip(bars, powers, energies):
        ax2.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 4,
                 f'{pw:.0f} W\n{ew:.1f} Wh', ha='center', va='bottom', fontsize=9)
    ax2.set_ylabel('Electrical power  [W]', fontsize=10)
    ax2.set_title('Power & Energy\nby Phase', fontsize=11, fontweight='bold')
    ax2.set_ylim(0, max(powers) * 1.30)

    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    return fig
