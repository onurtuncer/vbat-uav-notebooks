# conceptual_design/plots/vanes.py
"""Figures for the control-vane design notebook (NB3)."""

import math

import numpy as np
import matplotlib.pyplot as plt

from ..notebook import PALETTE as C


def plot_vane_authority(d, save_path):
    """Cl(delta), forces/moments, and blockage thrust loss vs deflection."""
    delta_v = np.linspace(0, d.p.delta_max_deg, 300)
    delta_r = np.radians(delta_v)

    Cl_v      = d.Cl_alpha_eff * delta_r
    F_v       = d.q_jet * d.S_vane * Cl_v
    M_pitch_v = 2.0 * F_v * d.L_CG
    M_roll_v  = 4.0 * F_v * d.L_roll
    thrust_loss = d.thrust_loss_pct(delta_v)

    fig, axs = plt.subplots(1, 3, figsize=(13, 4))

    axs[0].plot(delta_v, Cl_v, color=C[0], lw=2)
    axs[0].axvline(d.p.delta_stall_deg, ls="--", color=C[1], lw=1.2,
                   label=f"Stall ~{d.p.delta_stall_deg:.0f} deg")
    axs[0].axvline(d.p.delta_design_deg, ls=":", color=C[2], lw=1.2,
                   label=f"Design {d.p.delta_design_deg:.0f} deg")
    axs[0].set_xlabel("Deflection [deg]")
    axs[0].set_ylabel("Cl")
    axs[0].set_title("Flat-plate Cl(delta)")
    axs[0].legend(fontsize=8)

    axs[1].plot(delta_v, F_v * 1e3,       color=C[0], lw=2, label="F per vane [mN]")
    axs[1].plot(delta_v, M_pitch_v * 1e3, color=C[1], lw=2, ls="--",
                label="M_pitch 2-vane [N*mm]")
    axs[1].plot(delta_v, M_roll_v * 1e3,  color=C[2], lw=2, ls=":",
                label="M_roll 4-vane [N*mm]")
    axs[1].set_xlabel("Deflection [deg]")
    axs[1].set_ylabel("Force [mN] / Moment [N*mm]")
    axs[1].set_title("Forces and Moments")
    axs[1].legend(fontsize=8)

    axs[2].plot(delta_v, thrust_loss, color=C[3], lw=2)
    axs[2].axhline(5.0, ls="--", color=C[1], lw=1.2, label="5% guideline")
    axs[2].set_xlabel("Deflection [deg]")
    axs[2].set_ylabel("Thrust loss [%]")
    axs[2].set_title("Blockage Thrust Loss")
    axs[2].legend(fontsize=8)

    plt.tight_layout()
    plt.savefig(save_path, dpi=120, bbox_inches="tight")
    return fig


def plot_vane_geometry(d, save_path):
    """Aft view of the duct/vane arrangement plus the flat-plate profile
    at a fan of deflections."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5.5))

    # duct cross-section
    th = np.linspace(0, 2 * math.pi, 500)
    ax1.plot(d.R_tip * np.cos(th) * 1e3, d.R_tip * np.sin(th) * 1e3,
             color="#222222", lw=2.5, label="Duct wall")
    ax1.fill(d.R_hub * np.cos(th) * 1e3, d.R_hub * np.sin(th) * 1e3,
             color="#bbbbbb", alpha=0.6)
    ax1.plot(d.R_hub * np.cos(th) * 1e3, d.R_hub * np.sin(th) * 1e3,
             color="#444444", lw=2, label="Nacelle/hub")

    vane_defs = [(90, "T", C[0]), (270, "B", C[1]),
                 (180, "L", C[2]), (0,   "R", C[3])]

    for ang_deg, label, col in vane_defs:
        a  = math.radians(ang_deg)
        x0 = d.R_hub * math.cos(a) * 1e3
        y0 = d.R_hub * math.sin(a) * 1e3
        x1 = (d.R_hub + d.h_vane) * math.cos(a) * 1e3
        y1 = (d.R_hub + d.h_vane) * math.sin(a) * 1e3
        ax1.plot([x0, x1], [y0, y1], color=col, lw=6,
                 solid_capstyle="round", label=f"Vane {label}")
        ax1.text(x1 * 1.18, y1 * 1.18, label, ha="center", va="center",
                 fontsize=12, fontweight="bold", color=col)

    ax1.set_aspect("equal")
    ax1.set_xlabel("y [mm]")
    ax1.set_ylabel("z [mm]")
    ax1.set_title("EDF Duct -- Aft View")
    ax1.legend(loc="lower right", fontsize=8)

    # flat-plate profile at various deflections
    c_mm   = d.c_vane * 1e3
    half_t = 0.5 * d.p.tc_vane * c_mm
    colors_d = plt.cm.Blues(np.linspace(0.3, 0.95, 5))

    for d_deg, col in zip([0, 5, 10, 15, 20], colors_d):
        a   = math.radians(d_deg)
        Rot = np.array([[math.cos(a), -math.sin(a)],
                        [math.sin(a),  math.cos(a)]])
        corners = np.array([[0,    -half_t],
                            [c_mm, -half_t],
                            [c_mm,  half_t],
                            [0,     half_t],
                            [0,    -half_t]])
        rot = (Rot @ corners.T).T
        ax2.fill(rot[:, 0], rot[:, 1], alpha=0.35, color=col)
        ax2.plot(rot[:, 0], rot[:, 1], color=col, lw=1.5, label=f"d = {d_deg} deg")

    ax2.set_aspect("equal")
    ax2.set_xlabel("x [mm]")
    ax2.set_ylabel("y [mm]")
    ax2.set_title("Flat-plate vane profile")
    ax2.legend(loc="upper left", fontsize=8)

    plt.tight_layout()
    plt.savefig(save_path, dpi=120, bbox_inches="tight")
    return fig
