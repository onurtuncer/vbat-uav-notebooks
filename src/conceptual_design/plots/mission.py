# conceptual_design/plots/mission.py
"""Figure for the design-summary notebook (NB16): mission profile + battery track."""

import matplotlib.pyplot as plt
from matplotlib.patches import Patch

from ..notebook import PALETTE as C

#: House-palette colour per leg phase (blue / orange / green).
_PHASE_FILL = {"vtol": C[0], "transition": C[3], "cruise": C[2]}

#: The cruise leg is drawn this many times the summed width of all the other
#: legs, so the ~1-minute vertical/transition legs stay legible next to the
#: 15-minute cruise.  Cosmetic only: the x-axis is a compressed timeline (tick
#: labels report the TRUE elapsed minutes), never a distortion of the physics.
_CRUISE_PLOT_RATIO = 1.6


def _plot_x_boundaries(legs):
    """Leg-boundary x positions on the compressed axis, with their true times.

    Every leg keeps its real width except the (single) cruise leg, which is
    squeezed to ``_CRUISE_PLOT_RATIO`` x the other legs' total.  Returns
    ``(px, tx)``: plotted-x boundaries [s-equivalent] and the matching true
    elapsed times [s], both with ``len(legs) + 1`` entries.
    """
    noncruise_s = sum(leg.duration_s for leg in legs if leg.phase != "cruise")
    cruise_plot_s = (_CRUISE_PLOT_RATIO * noncruise_s if noncruise_s > 0
                     else sum(leg.duration_s for leg in legs))
    px, tx = [0.0], [0.0]
    for leg in legs:
        w = cruise_plot_s if leg.phase == "cruise" else leg.duration_s
        px.append(px[-1] + w)
        tx.append(leg.t_end_s)
    return px, tx


def plot_mission_profile(profile, save_path):
    """Altitude-vs-time mission sketch with pack state-of-charge overlaid.

    Left axis: the flown altitude profile (vertical climb, cruise at
    ``h_cruise``, descent).  Right axis: pack state-of-charge, referenced to the
    sized-with-reserve pack, annotated at every leg boundary.  Legs are shaded
    by phase.  The x-axis is a *compressed* timeline -- the long cruise leg is
    squeezed so the short vertical/transition legs stay readable -- with tick
    labels giving the true elapsed minutes and the cruise band marked as such.
    """
    legs = profile.legs
    px, tx = _plot_x_boundaries(legs)

    fig, ax = plt.subplots(figsize=(10.5, 4.8))
    ax_soc = ax.twinx()
    ax_soc.grid(False)

    # -- altitude polyline (left axis) -------------------------------------
    h_alt = [legs[0].alt_start_m] + [leg.alt_end_m for leg in legs]
    ax.plot(px, h_alt, color=C[0], lw=2.2, zorder=5, label="altitude")
    ax.fill_between(px, h_alt, color=C[0], alpha=0.06, zorder=1)

    # -- state-of-charge polyline (right axis) -----------------------------
    soc = [legs[0].soc_start * 100.0] + [leg.soc_end * 100.0 for leg in legs]
    ax_soc.plot(px, soc, color=C[1], lw=2.0, ls="--", marker="o", ms=4,
                zorder=6, label="battery SoC")

    # reserve floor: the pack is sized so the mission ends here
    ax_soc.axhline(profile.reserve_frac * 100.0, color=C[1], lw=1.0, ls=":",
                   alpha=0.7, zorder=2)
    ax_soc.annotate(f"reserve {profile.reserve_frac*100:.0f}%",
                    (px[-1] * 0.30, profile.reserve_frac * 100.0),
                    xytext=(0, -12), textcoords="offset points",
                    ha="center", va="top", fontsize=8, color=C[1])

    # -- SoC value at each leg boundary ------------------------------------
    for xi, si in zip(px, soc):
        ax_soc.annotate(f"{si:.0f}%", (xi, si), xytext=(0, 8),
                        textcoords="offset points", ha="center", fontsize=8,
                        color=C[1])

    # -- phase shading + leg labels ----------------------------------------
    # Narrow legs get a rotated label above the axis; the wide cruise band
    # gets a horizontal label inside it (with its true duration), so nothing
    # collides with the title.
    h_top = max(h_alt) if max(h_alt) > 0 else 1.0
    for leg, x0, x1 in zip(legs, px[:-1], px[1:]):
        ax.axvspan(x0, x1, color=_PHASE_FILL[leg.phase], alpha=0.07, zorder=0)
        xc = (x0 + x1) / 2.0
        if leg.phase == "cruise":
            ax.annotate(f"cruise\n{leg.duration_s/60:.0f} min (axis compressed)",
                        (xc, h_top * 0.30), ha="center", va="center",
                        fontsize=8, color="0.35")
        else:
            ax.annotate(leg.name, (xc, h_top), xytext=(0, 10),
                        textcoords="offset points", ha="center", va="bottom",
                        rotation=90, fontsize=7.5, color="0.35")

    # -- axis: compressed x, but tick labels carry the TRUE elapsed time ----
    ax.set_xticks(px)
    ax.set_xticklabels([f"{t/60:.1f}" for t in tx])
    ax.set_xlabel("mission time [min]  (cruise leg compressed for legibility)")
    ax.set_ylabel("altitude AGL [m]", color=C[0])
    ax.tick_params(axis="y", labelcolor=C[0])
    ax.set_ylim(0, h_top * 1.15)
    ax.set_xlim(0, px[-1])

    ax_soc.set_ylabel("pack state-of-charge [%]", color=C[1])
    ax_soc.tick_params(axis="y", labelcolor=C[1])
    ax_soc.set_ylim(0, 105)

    ax.set_title(
        f"Mission profile -- {profile.range_km:.1f} km range, "
        f"{profile.endurance_min:.1f} min, cruise {profile.h_cruise_m:.0f} m AGL")

    phase_legend = [Patch(facecolor=_PHASE_FILL[p], alpha=0.25, label=p)
                    for p in ("vtol", "transition", "cruise")]
    lines = ax.get_lines() + ax_soc.get_lines()[:1]
    ax.legend(handles=[*lines, *phase_legend],
              loc="center left", fontsize=8, framealpha=0.9)

    fig.tight_layout()
    fig.savefig(save_path, bbox_inches="tight")
    return fig
