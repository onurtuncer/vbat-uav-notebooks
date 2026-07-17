# conceptual_design/plots/cots.py
"""Budget-margin bar chart shared by the COTS-selection notebook (NB11)
and the design summary (NB15)."""

import numpy as np
import matplotlib.pyplot as plt

from ..notebook import PALETTE as C


def plot_budget_margins(budgets, groups, labels, title, save_path):
    """Horizontal bars of actual COTS mass vs mass-closure allocation.

    bar = actual, tick = allocation; red = over allocation.
    """
    fig, ax = plt.subplots(figsize=(7.5, 3.6))
    y = np.arange(len(groups))[::-1]
    actual = [budgets[g]["actual_g"] for g in groups]
    alloc  = [budgets[g]["alloc_g"]  for g in groups]
    colors = [C[0] if budgets[g]["within"] else C[1] for g in groups]

    ax.barh(y, actual, height=0.55, color=colors)
    ax.plot(alloc, y, "k|", markersize=22, markeredgewidth=2.2, ls="none",
            zorder=3)
    for yi, g in zip(y, groups):
        ax.annotate(f"{budgets[g]['margin_g']:+.0f} g",
                    (max(budgets[g]['actual_g'], budgets[g]['alloc_g']), yi),
                    xytext=(6, 0), textcoords="offset points", va="center",
                    fontsize=9)
    ax.set_yticks(y, [labels[g] for g in groups])
    ax.set_xlabel("mass [g]")
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(save_path, bbox_inches="tight")
    return fig
