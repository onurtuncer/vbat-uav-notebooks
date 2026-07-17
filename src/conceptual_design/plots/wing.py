# conceptual_design/plots/wing.py
"""Figures for the wing/airfoil design notebook (NB2)."""

import math

import numpy as np
import matplotlib.pyplot as plt

from ..airfoil_selection import naca4_coordinates, oswald_efficiency
from ..wing_sizing import wing_mass_raymer_kg


def plot_airfoil_profile(M, P, t, designation, save_path):
    """Upper/lower surface geometry of the selected NACA 4-digit section."""
    xu, yu, xl, yl = naca4_coordinates(M, P, t, n=200)

    fig, ax = plt.subplots(figsize=(10, 3))
    ax.plot(xu, yu, color='steelblue', lw=2, label='Upper surface')
    ax.plot(xl, yl, color='tomato',    lw=2, label='Lower surface')
    ax.axhline(0, color='gray', lw=0.8, ls='--')
    ax.set_aspect('equal')
    ax.set_xlabel('x/c  [-]')
    ax.set_ylabel('y/c  [-]')
    ax.set_title(f'Airfoil profile: {designation}', fontweight='bold')
    ax.legend(fontsize=9)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    return fig


def plot_lift_curve(Cl_a_2D, CL_a_3D, aL0_deg, AR, designation, save_path):
    """2D section vs 3D finite-wing lift curves."""
    alpha_deg = np.linspace(-6, 16, 200)
    alpha_rad = np.radians(alpha_deg)

    Cl_2D = Cl_a_2D * (alpha_rad - math.radians(aL0_deg))
    CL_3D = CL_a_3D * (alpha_rad - math.radians(aL0_deg))

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(alpha_deg, Cl_2D, lw=2,
            label=f'2D section  $C_{{l,\\alpha}}$ = {Cl_a_2D:.3f}/rad')
    ax.plot(alpha_deg, CL_3D, lw=2, ls='--',
            label=f'3D wing (AR={AR})  $C_{{L,\\alpha}}$ = {CL_a_3D:.3f}/rad')
    ax.axhline(0, color='gray', lw=0.6)
    ax.axvline(0, color='gray', lw=0.6)
    ax.set_xlabel(r'Angle of attack $\alpha$  [deg]')
    ax.set_ylabel(r'Lift coefficient  $C_L$  [-]')
    ax.set_title(f'Lift curve slope — {designation}', fontweight='bold')
    ax.legend(fontsize=9)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    return fig


def plot_drag_polar(CD0_total, k_induced, CL_max, CL_cruise, CD_cruise,
                    LD_cruise, designation, save_path):
    """Parabolic drag polar and L/D vs CL with the cruise point marked."""
    CL_range = np.linspace(0, CL_max * 0.95, 200)
    CD_range = CD0_total + k_induced * CL_range**2

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))

    ax = axes[0]
    ax.plot(CD_range, CL_range, lw=2, color='steelblue')
    ax.plot(CD_cruise, CL_cruise, 'r*', ms=12, zorder=5,
            label=f'Cruise  CL={CL_cruise:.3f}, CD={CD_cruise:.4f}')
    ax.set_xlabel(r'$C_D$  [-]')
    ax.set_ylabel(r'$C_L$  [-]')
    ax.set_title(f'Drag polar — {designation}', fontweight='bold')
    ax.legend(fontsize=9)

    LD_range = CL_range / CD_range
    ax2 = axes[1]
    ax2.plot(CL_range, LD_range, lw=2, color='darkorange')
    ax2.axvline(CL_cruise, color='red', lw=1.2, ls='--',
                label=f'Cruise  L/D = {LD_cruise:.2f}')
    ax2.set_xlabel(r'$C_L$  [-]')
    ax2.set_ylabel(r'$L/D$  [-]')
    ax2.set_title('Lift-to-drag ratio', fontweight='bold')
    ax2.legend(fontsize=9)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    return fig


def plot_ar_trade(AR_design, AR_values, CD0_total, CL_cruise, W_N, WS_design,
                  MTOW_kg, V_cruise, rho, ws, save_path):
    """Cruise L/D, span, and Raymer wing mass as functions of aspect ratio."""
    LD_vals, span_vals, m_wing_vals = [], [], []
    for AR_i in AR_values:
        e_i  = oswald_efficiency(AR_i)
        k_i  = 1.0 / (math.pi * AR_i * e_i)
        CD_i = CD0_total + k_i * CL_cruise**2
        S_i  = W_N / WS_design
        LD_vals.append(CL_cruise / CD_i)
        span_vals.append(math.sqrt(AR_i * S_i))
        m_wing_vals.append(
            wing_mass_raymer_kg(S_i, AR_i, MTOW_kg, V_cruise, rho, ws))

    fig, axes = plt.subplots(1, 3, figsize=(13, 4))

    for ax_i, ydata, ylabel, title, color in zip(
        axes,
        [LD_vals, span_vals, m_wing_vals],
        ['L/D  [-]', 'Span  [m]', 'Wing mass  [kg]'],
        ['Cruise L/D vs AR', 'Wing span vs AR', 'Wing mass vs AR'],
        ['darkorange', 'steelblue', 'tomato'],
    ):
        ax_i.plot(AR_values, ydata, lw=2, color=color)
        ax_i.axvline(AR_design, color='gray', lw=1.2, ls='--',
                     label=f'Design AR={AR_design}')
        ax_i.set_xlabel('Aspect ratio  AR  [-]')
        ax_i.set_ylabel(ylabel)
        ax_i.set_title(title, fontweight='bold')
        ax_i.legend(fontsize=9)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    return fig
