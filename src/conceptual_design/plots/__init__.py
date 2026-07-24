# conceptual_design/plots/
"""Notebook figure functions.

Presentation only: every function takes already-computed design objects,
draws the house-style figure, saves it, and returns the matplotlib
Figure.  No physics lives here (ADR-0002) -- anything numeric is either
axis cosmetics or a call into the physics modules for display curves.
"""

from .aileron import plot_aileron_authority
from .conceptual import (
    plot_mass_budget,
    plot_mass_closure_convergence,
    plot_sensitivity_grid,
    plot_size_matching_diagram,
    plot_vtol_power_analysis,
)
from .cots import plot_budget_margins
from .fuselage import plot_fineness_trade, plot_fuselage_layout
from .mass_properties import plot_mass_overview
from .mission import plot_mission_profile
from .thermal import plot_thermal_paths
from .vanes import plot_vane_authority, plot_vane_geometry
from .vibration import plot_transmissibility
from .wing import (
    plot_airfoil_profile,
    plot_ar_trade,
    plot_drag_polar,
    plot_lift_curve,
)

__all__ = [
    "plot_aileron_authority",
    "plot_airfoil_profile",
    "plot_ar_trade",
    "plot_budget_margins",
    "plot_drag_polar",
    "plot_fineness_trade",
    "plot_fuselage_layout",
    "plot_lift_curve",
    "plot_mass_budget",
    "plot_mass_closure_convergence",
    "plot_mass_overview",
    "plot_mission_profile",
    "plot_sensitivity_grid",
    "plot_size_matching_diagram",
    "plot_thermal_paths",
    "plot_transmissibility",
    "plot_vane_authority",
    "plot_vane_geometry",
    "plot_vtol_power_analysis",
]
