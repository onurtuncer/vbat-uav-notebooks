# conceptual_design/plots/thermal.py
"""Figure for the thermal-design notebook (NB7)."""

import numpy as np
import matplotlib.pyplot as plt

from ..notebook import PALETTE as C


def plot_thermal_paths(e, b, tp, save_path):
    """Hover heat loads and reached-vs-limit temperatures for the two paths."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))

    ax1.bar(["ESC", "battery"], [e.Q_W, b.Q_W], color=[C[1], C[0]], width=0.55)
    for i, q in enumerate([e.Q_W, b.Q_W]):
        ax1.annotate(f"{q:.0f} W", (i, q), ha="center", va="bottom", fontsize=10)
    ax1.set_ylabel("hover heat load [W]")
    ax1.set_title("Continuous heat sources (hover)")

    comps  = ["ESC", "battery"]
    temps  = [e.T_at_avail_C, b.T_at_avail_C]
    limits = [tp.T_esc_max_C, tp.T_batt_max_C]
    x = np.arange(len(comps))
    ax2.bar(x, temps,
            color=[C[1] if t > lim else C[2] for t, lim in zip(temps, limits)],
            width=0.55, label="reached (@ available wall)")
    ax2.hlines(limits, x - 0.3, x + 0.3, color="k", lw=2, label="limit")
    ax2.axhline(tp.T_ambient_C, color="0.6", ls="--", lw=1, label="ambient")
    for i, (t, lim) in enumerate(zip(temps, limits)):
        ax2.annotate(f"{t:.0f} / {lim:.0f} C", (i, t), ha="center",
                     va="bottom", fontsize=9)
    ax2.set_xticks(x)
    ax2.set_xticklabels(comps)
    ax2.set_ylabel("temperature [degC]")
    ax2.set_title("Component temperature vs limit")
    ax2.legend(fontsize=8, loc="lower right")

    fig.tight_layout()
    fig.savefig(save_path, bbox_inches="tight")
    return fig
