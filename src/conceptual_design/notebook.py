# conceptual_design/notebook.py
"""Shared setup for the design notebooks: repo paths and house plot style.

Every notebook opens with the same prelude (resolve repo paths, apply the
matplotlib house style, create the figures directory).  ``nb_setup()`` is
that prelude as a single call, so the notebooks carry no boilerplate.
"""

from dataclasses import dataclass, field
from pathlib import Path

#: House palette shared by all notebook figures (ColorBrewer-derived).
PALETTE = [
    "#2c7bb6", "#d7191c", "#1a9641", "#f68b33", "#762a83",
    "#5aae61", "#9970ab", "#e08214", "#35978f",
]

#: House matplotlib style for all notebook figures.
RC_STYLE = {
    "figure.dpi":        120,
    "axes.spines.top":   False,
    "axes.spines.right": False,
    "axes.grid":         True,
    "grid.alpha":        0.25,
    "font.size":         10,
}


@dataclass(frozen=True)
class NotebookEnv:
    """Resolved repo paths and the shared figure palette for one notebook."""

    repo_root: Path
    config: Path                    # config/  (YAML inputs)
    out: Path                       # out/     (YAML handoffs between notebooks)
    fig_dir: Path                   # notebooks/figures/ (relative to notebook cwd)
    colors: list = field(default_factory=lambda: list(PALETTE))


def nb_setup(style: bool = True) -> NotebookEnv:
    """Resolve repo paths from a notebook's working directory (notebooks/).

    With ``style=True`` (default) also applies the house matplotlib style.
    """
    repo_root = Path.cwd().resolve().parents[0]
    fig_dir = Path("figures")       # per-notebook figures directory
    fig_dir.mkdir(exist_ok=True)

    if style:
        import matplotlib.pyplot as plt
        plt.rcParams.update(RC_STYLE)

    return NotebookEnv(
        repo_root=repo_root,
        config=repo_root / "config",
        out=repo_root / "out",
        fig_dir=fig_dir,
    )
