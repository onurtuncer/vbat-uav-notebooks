# conceptual_design/plots/mission.py
"""Figure for the design-summary notebook (NB16): mission profile + battery track."""

import matplotlib.pyplot as plt
from matplotlib.patches import Patch

from ..notebook import PALETTE as C

#: House-palette colour per leg phase (blue / orange / green).
_PHASE_FILL = {"vtol": C[0], "transition": C[3], "cruise": C[2]}


def plot_mission_profile(profile, save_path):
    """Altitude-vs-time mission sketch with pack state-of-charge overlaid.

    Left axis: the flown altitude profile (vertical climb, cruise at
    ``h_cruise``, descent).  Right axis: pack state-of-charge, referenced to the
    sized-with-reserve pack, annotated at every leg boundary.  Legs are shaded
    by phase so the reader sees at a glance where the energy goes -- the flat
    cruise dominates the timeline, exactly as the mass closure bills it.
    """
    legs = profile.legs
    fig, ax = plt.subplots(figsize=(10.5, 4.8))
    ax_soc = ax.twinx()
    ax_soc.grid(False)

    # -- altitude polyline (left axis) -------------------------------------
    t_alt = [legs[0].t_start_s / 60.0]
    h_alt = [legs[0].alt_start_m]
    for leg in legs:
        t_alt.append(leg.t_end_s / 60.0)
        h_alt.append(leg.alt_end_m)
    ax.plot(t_alt, h_alt, color=C[0], lw=2.2, zorder=5, label="altitude")
    ax.fill_between(t_alt, h_alt, color=C[0], alpha=0.06, zorder=1)

    # -- state-of-charge polyline (right axis) -----------------------------
    t_soc = [legs[0].t_start_s / 60.0] + [leg.t_end_s / 60.0 for leg in legs]
    soc   = [legs[0].soc_start * 100.0] + [leg.soc_end * 100.0 for leg in legs]
    ax_soc.plot(t_soc, soc, color=C[1], lw=2.0, ls="--", marker="o", ms=4,
                zorder=6, label="battery SoC")

    # reserve floor: the pack is sized so the mission ends here
    ax_soc.axhline(profile.reserve_frac * 100.0, color=C[1], lw=1.0, ls=":",
                   alpha=0.7, zorder=2)
    ax_soc.annotate(f"reserve {profile.reserve_frac*100:.0f}%",
                    (profile.endurance_min * 0.30, profile.reserve_frac * 100.0),
                    xytext=(0, -12), textcoords="offset points",
                    ha="center", va="top", fontsize=8, color=C[1])

    # -- SoC value at each leg boundary ------------------------------------
    for ti, si in zip(t_soc, soc):
        ax_soc.annotate(f"{si:.0f}%", (ti, si), xytext=(0, 8),
                        textcoords="offset points", ha="center", fontsize=8,
                        color=C[1])

    # -- phase shading + leg labels ----------------------------------------
    h_top = max(h_alt) if max(h_alt) > 0 else 1.0
    for leg in legs:
        ax.axvspan(leg.t_start_s / 60.0, leg.t_end_s / 60.0,
                   color=_PHASE_FILL[leg.phase], alpha=0.07, zorder=0)
        ax.annotate(leg.name, ((leg.t_start_s + leg.t_end_s) / 2.0 / 60.0, h_top),
                    xytext=(0, 10), textcoords="offset points",
                    ha="center", va="bottom", rotation=90, fontsize=7.5,
                    color="0.35")

    ax.set_xlabel("mission time [min]")
    ax.set_ylabel("altitude AGL [m]", color=C[0])
    ax.tick_params(axis="y", labelcolor=C[0])
    ax.set_ylim(0, h_top * 1.15)
    ax.set_xlim(0, profile.endurance_min)

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
