# conceptual_design/plots/vibration.py
"""Figure for the vibration-isolation notebook (NB5)."""

import math

import numpy as np
import matplotlib.pyplot as plt

from ..notebook import PALETTE as C
from ..vibration_isolation import transmissibility


def plot_transmissibility(res, vib_p, save_path):
    """Isolator transmissibility curve against the EDF forcing lines."""
    r = np.linspace(0.2, res['f_shaft_hz'] / max(res['f_n_hz'], 1e-6) * 1.4, 600)
    T = np.array([transmissibility(ri, vib_p.damping_ratio) for ri in r])
    f_hz = r * res['f_n_hz']

    fig, ax = plt.subplots(figsize=(8.5, 4.6))
    ax.semilogy(f_hz, T, color=C[0], lw=2,
                label=f"isolator (f_n={res['f_n_hz']:.0f} Hz, "
                      f"zeta={vib_p.damping_ratio:.2f})")
    ax.axhline(1.0, color="0.6", lw=1, ls="-")
    ax.axhline(vib_p.target_transmissibility, color=C[2], lw=1.2, ls="--",
               label=f"target T={vib_p.target_transmissibility:.2f}")
    ax.axvline(res['f_shaft_hz'], color=C[1], lw=1.4, ls=":",
               label=f"1/rev {res['f_shaft_hz']:.0f} Hz")
    ax.axvline(res['f_blade_hz'], color=C[3], lw=1.4, ls=":",
               label=f"blade-pass {res['f_blade_hz']:.0f} Hz")
    ax.axvline(res['f_n_hz'] * math.sqrt(2), color="0.5", lw=1, ls="-.",
               label=r"isolation threshold $r=\sqrt{2}$")
    ax.set_xlabel("frequency [Hz]")
    ax.set_ylabel("transmissibility  T  [-]")
    ax.set_title("Isolator transmissibility vs. EDF forcing")
    ax.set_xlim(0, res['f_blade_hz'] * 1.05)
    ax.legend(fontsize=8, loc="lower left")
    fig.tight_layout()
    fig.savefig(save_path, bbox_inches="tight")
    return fig
