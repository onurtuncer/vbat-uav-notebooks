# conceptual_design/plots/aileron.py
"""Figure for the aileron design notebook (NB4)."""

import numpy as np
import matplotlib.pyplot as plt

from ..notebook import PALETTE as C


def plot_aileron_authority(ail, ail_p, I_roll, ddot_roll_vane_cruise,
                           ddot_min, save_path):
    """Aileron force/moment and combined cruise roll authority vs deflection."""
    delta_v = np.linspace(0, ail_p.delta_max_deg, 300)
    delta_r = np.radians(delta_v)

    F_v      = ail.q_cruise_Pa * ail.S_aileron_m2 * ail.Cl_delta_per_rad * delta_r
    M_roll_v = 2.0 * F_v * ail.y_arm_m
    ddot_v   = np.degrees(M_roll_v / I_roll)
    ddot_combined_v = ddot_v + ddot_roll_vane_cruise

    fig, axs = plt.subplots(1, 2, figsize=(10, 4.2))

    axs[0].plot(delta_v, F_v * 1e3, color=C[0], lw=2, label="F per aileron [mN]")
    axs[0].plot(delta_v, M_roll_v * 1e3, color=C[1], lw=2, ls="--",
                label="M_roll (2-aileron) [N*mm]")
    axs[0].axvline(ail_p.delta_design_deg, ls=":", color=C[2], lw=1.2,
                   label=f"Design {ail_p.delta_design_deg:.0f} deg")
    axs[0].set_xlabel("Deflection [deg]")
    axs[0].set_ylabel("Force [mN] / Moment [N*mm]")
    axs[0].set_title("Aileron force and roll moment")
    axs[0].legend(fontsize=8)

    axs[1].plot(delta_v, ddot_combined_v, color=C[0], lw=2,
                label="Aileron + residual vane (cruise)")
    axs[1].axhline(ddot_min, ls="--", color=C[1], lw=1.2,
                   label=f"Requirement {ddot_min:.0f} deg/s^2")
    axs[1].axvline(ail_p.delta_design_deg, ls=":", color=C[2], lw=1.2,
                   label=f"Design {ail_p.delta_design_deg:.0f} deg")
    axs[1].set_xlabel("Deflection [deg]")
    axs[1].set_ylabel("ddot_roll [deg/s^2]")
    axs[1].set_title("Cruise roll authority vs. deflection")
    axs[1].legend(fontsize=8)

    plt.tight_layout()
    plt.savefig(save_path, dpi=120, bbox_inches="tight")
    return fig
