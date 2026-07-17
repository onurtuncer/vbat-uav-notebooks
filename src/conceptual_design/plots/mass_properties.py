# conceptual_design/plots/mass_properties.py
"""Figure for the mass-properties notebook (NB9)."""

import numpy as np
import matplotlib.pyplot as plt

from ..fuselage_design import fuselage_radius
from ..mass_properties import inertia_contributions
from ..notebook import PALETTE as C


def plot_mass_overview(mp, fus, vanes, ail, save_path):
    """Side profile with component CGs plus the per-component inertia
    build-up (own-CG term vs parallel-axis transport)."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 4.4))

    # --- side profile with CG markers -------------------------------------
    xs = np.linspace(0.0, mp.L_fus, 400)
    rs = np.array([fuselage_radius(x, mp.D_fus, mp.L_fus, fus["f_nose"],
                                   fus["f_tail"], fus["r_hub_m"]) for x in xs])
    ax1.fill_between(xs * 1e3, -rs * 1e3, rs * 1e3, color="0.85", zorder=1)
    ax1.plot(xs * 1e3,  rs * 1e3, "k-", lw=1)
    ax1.plot(xs * 1e3, -rs * 1e3, "k-", lw=1)

    # wing chord line at its station
    x_le = fus["x_wing_LE_m"]
    ax1.plot([x_le * 1e3, (x_le + mp.chord_wing_m) * 1e3], [0, 0],
             color=C[2], lw=6, alpha=0.5, solid_capstyle="butt", zorder=2)

    # clusters plot at their true radial offset (mirrored), on-axis at r=0
    r_plot = {"vanes (4x)": vanes["R_mid_m"], "servos (4x)": mp.r_servo_cg,
              "linkages (4x)": vanes["R_hub_m"], "aileron_hw (2x)": ail["y_arm_m"]}
    for i, c in enumerate(mp.components):
        r_off = r_plot.get(c.name, 0.0)
        s = 40 + 1200 * c.mass_kg / mp.m_tot
        ax1.scatter(c.x_s * 1e3,  r_off * 1e3, s=s, color=C[i % len(C)],
                    zorder=3, label=c.name)
        if r_off > 0:
            ax1.scatter(c.x_s * 1e3, -r_off * 1e3, s=s, color=C[i % len(C)],
                        zorder=3)
    ax1.annotate("servos on hinge line,\nrecessed in hub (NB5)",
                 (mp.x_hinge * 1e3, mp.r_servo_cg * 1e3), xytext=(-95, 18),
                 textcoords="offset points", fontsize=7,
                 arrowprops=dict(arrowstyle="->", lw=0.8))

    ax1.axvline(mp.x_cg * 1e3, color="k", ls="--", lw=1.2)
    ax1.annotate("CG", (mp.x_cg * 1e3, mp.D_fus / 2 * 1e3), xytext=(0, 6),
                 textcoords="offset points", ha="center", fontweight="bold")
    ax1.axvline(fus["x_wing_AC_m"] * 1e3, color=C[1], ls=":", lw=1.2)
    ax1.annotate("wing AC", (fus["x_wing_AC_m"] * 1e3, -mp.D_fus / 2 * 1e3),
                 xytext=(0, -14), textcoords="offset points", ha="center",
                 color=C[1])
    ax1.set_xlabel("station from nose, $x_s$ [mm]")
    ax1.set_ylabel("[mm]")
    ax1.set_title("Component CGs (marker area $\\propto$ mass)")
    ax1.set_aspect("equal")
    ax1.legend(loc="upper right", fontsize=7, ncol=2, framealpha=0.9)

    # --- inertia breakdown -------------------------------------------------
    names, own, transp = inertia_contributions(mp)

    x = np.arange(len(names))
    w = 0.27
    for j, (axis, col) in enumerate(zip(["$I_{xx}$", "$I_{yy}$", "$I_{zz}$"],
                                        [C[0], C[1], C[2]])):
        ax2.bar(x + (j - 1) * w, own[:, j],   w, color=col, alpha=0.95,
                label=axis + " (own CG)")
        ax2.bar(x + (j - 1) * w, transp[:, j], w, bottom=own[:, j],
                color=col, alpha=0.35,
                label=axis + " (transport)")
    ax2.set_xticks(x)
    ax2.set_xticklabels(names, rotation=40, ha="right", fontsize=8)
    ax2.set_ylabel("contribution to $I_{CG}$ [kg m$^2$]")
    ax2.set_title("Inertia build-up (solid = own CG, faded = parallel-axis)")
    ax2.legend(fontsize=7, ncol=3)

    fig.tight_layout()
    fig.savefig(save_path, bbox_inches="tight")
    return fig
