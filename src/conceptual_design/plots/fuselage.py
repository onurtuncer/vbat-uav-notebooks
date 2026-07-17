# conceptual_design/plots/fuselage.py
"""Figures for the fuselage design notebooks (NB6 and its COTS re-solve,
NB14) -- both draw the same side-view layout, so it lives here once."""

import numpy as np
import matplotlib.pyplot as plt

from ..fuselage_design import fuselage_radius
from ..notebook import PALETTE as C


def plot_fineness_trade(FR_sweep, cd0_sweep, mshell_sweep, fus, fus_p,
                        save_path):
    """CD0 and shell mass vs fineness ratio, with the design point marked."""
    fig, ax1 = plt.subplots(figsize=(8, 4.5))
    ax1.plot(FR_sweep, cd0_sweep, color=C[0], lw=2, label=r"$C_{D0,fus}$")
    ax1.axvline(fus_p.fineness_ratio, color="gray", ls="--", lw=1)
    ax1.plot([fus.fineness], [fus.CD0_fus], "o", color=C[1], ms=8, zorder=5)
    ax1.annotate(f"  design: f = {fus.fineness:.1f}", (fus.fineness, fus.CD0_fus),
                 fontsize=9, va="bottom")
    ax1.set_xlabel("fineness ratio  $f = L/D$  [-]")
    ax1.set_ylabel(r"$C_{D0,fus}$  (on $S_{wing}$)  [-]", color=C[0])
    ax1.tick_params(axis="y", labelcolor=C[0])

    ax2 = ax1.twinx()
    ax2.plot(FR_sweep, mshell_sweep, color=C[3], lw=2, label="shell mass")
    ax2.axhline(fus.m_struct_budget_kg, color=C[3], ls=":", lw=1.2)
    ax2.annotate("  structural budget", (FR_sweep[0], fus.m_struct_budget_kg),
                 fontsize=8, color=C[3], va="bottom")
    ax2.set_ylabel("shell + frames mass  [kg]", color=C[3])
    ax2.tick_params(axis="y", labelcolor=C[3])
    ax2.spines.right.set_visible(True)
    ax2.grid(False)

    ax1.set_title("Fuselage fineness ratio trade (D fixed by EDF hub)")
    fig.tight_layout()
    fig.savefig(save_path, bbox_inches="tight")
    return fig


def plot_fuselage_layout(fus, fus_p, vanes, chord_mean_m, title, save_path):
    """Side view: outline, clamshell lid, internal bays, duct, vanes,
    wing chord, fan plane and CG."""
    xs = np.linspace(0.0, fus.L_fus, 400)
    rr = np.array([fuselage_radius(x, fus.D_fus, fus.L_fus,
                                   fus_p.f_nose, fus_p.f_tail, fus.r_hub)
                   for x in xs])
    by_name = {it.name: it for it in fus.items}

    fig, ax = plt.subplots(figsize=(11, 5))
    mm = 1e3

    # clamshell joint line (ADR-0010): full-length hinged lid over [0, x_clam_aft]
    ax.plot([0, fus.x_clam_aft_m * mm], [-fus.D_fus / 2 * mm - 6] * 2,
            color=C[3], lw=3, solid_capstyle='butt', zorder=6)
    ax.axvline(fus.x_clam_aft_m * mm, color=C[3], ls='-.', lw=1.4, zorder=6)
    ax.annotate('clamshell lid (hinged)',
                (fus.x_clam_aft_m * mm / 2, -fus.D_fus / 2 * mm - 14),
                ha='center', fontsize=8, color=C[3])

    # fuselage outline
    ax.fill_between(xs * mm, -rr * mm, rr * mm, color="#d9e4ee", zorder=1)
    ax.plot(xs * mm,  rr * mm, color=C[0], lw=1.8, zorder=3)
    ax.plot(xs * mm, -rr * mm, color=C[0], lw=1.8, zorder=3)

    # internal bays
    bay_colors = {"payload": C[2], "avionics": C[4], "battery": C[3], "esc": C[1]}
    r_int = fus.D_fus / 2 - fus_p.t_shell_m
    for it in fus.items:
        if it.name in bay_colors and it.length > 0:
            ax.add_patch(plt.Rectangle((it.x_start * mm, -0.78 * r_int * mm),
                                       it.length * mm, 1.56 * r_int * mm,
                                       facecolor=bay_colors[it.name], alpha=0.55,
                                       edgecolor="k", lw=0.5, zorder=4))
            ax.annotate(it.name, ((it.x_start + it.length / 2) * mm, 0),
                        ha="center", va="center", fontsize=8, rotation=90,
                        zorder=6)

    # duct annulus (section view)
    x_d0 = (by_name["duct"].x_cg - fus.duct_chord / 2) * mm
    for sgn in (+1, -1):
        ax.add_patch(plt.Rectangle((x_d0, sgn * fus.D_duct_inner / 2 * mm),
                                   fus.duct_chord * mm, sgn * fus_p.t_duct_m * mm,
                                   facecolor="#666", edgecolor="k", lw=0.5,
                                   zorder=4))
    ax.annotate("duct", (x_d0 + fus.duct_chord * mm / 2,
                         fus.D_duct_outer / 2 * mm + 6),
                ha="center", fontsize=8)

    # control vanes (T/B pair in side view)
    for sgn in (+1, -1):
        ax.add_patch(plt.Rectangle(((fus.x_vane - vanes["c_vane_m"] / 2) * mm,
                                    sgn * vanes["R_hub_m"] * mm),
                                   vanes["c_vane_m"] * mm,
                                   sgn * (vanes["R_tip_m"] - vanes["R_hub_m"]) * mm,
                                   facecolor=C[1], alpha=0.8, edgecolor="k",
                                   lw=0.5, zorder=4))
    ax.annotate("vanes", (fus.x_vane * mm, vanes["R_tip_m"] * mm + 6),
                ha="center", fontsize=8, color=C[1])

    # wing chord (side view)
    ax.plot([fus.x_wing_LE * mm, (fus.x_wing_LE + chord_mean_m) * mm], [0, 0],
            color=C[2], lw=5, solid_capstyle="butt", zorder=5)
    ax.annotate("wing chord", ((fus.x_wing_LE + chord_mean_m / 2) * mm, -14),
                ha="center", fontsize=8, color=C[2])

    # fan plane
    x_fan = by_name["motor_fan"].x_cg * mm
    ax.plot([x_fan, x_fan], [-fus.r_hub * mm, fus.r_hub * mm],
            color="k", ls="-.", lw=1)
    ax.annotate("fan", (x_fan, -fus.r_hub * mm - 10), ha="center", fontsize=8)

    # CG
    ax.plot(fus.x_CG * mm, 0, "o", color="k", ms=9, zorder=7,
            markerfacecolor="w", markeredgewidth=2)
    ax.axvline(fus.x_CG * mm, color="k", ls="--", lw=0.8, alpha=0.6)
    ax.annotate(f"CG  (x_s = {fus.x_CG * mm:.0f} mm)",
                (fus.x_CG * mm, fus.D_fus / 2 * mm + 18),
                ha="center", fontsize=9, fontweight="bold")

    ax.set_xlabel(r"station from nose, +aft  [mm]     ($x_{body} = -x_s$,  FRD)")
    ax.set_ylabel("z  [mm]")
    ax.set_title(title)
    ax.set_aspect("equal")
    ax.set_xlim(-30, (fus.x_vane + 0.06) * mm)
    fig.tight_layout()
    fig.savefig(save_path, bbox_inches="tight")
    return fig
