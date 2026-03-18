# conceptual_design/__init__.py

from .models import (
    Environment,
    Mission,
    PropulsiveSystemParameters,
    WeightFraction,
    Aerodynamics,
    Propulsor,
    MassBreakdown,
    Battery,
    RotorParams,
    WingSizing,
)

from .vtol_power import (
    VTOLParams,
    rpm_from_diameter,
    vtip_from_rpm_and_diameter,
    vtol_hover_power_to_weight,
    vtol_climb_power_to_weight,
    vtol_power_requirements,
)

from .forward_flight_power import (
    ForwardFlightParams,
    SizeMatchingResult,
    compute_size_matching_diagram,
    cruise_power_W,
    tw_cruise,
    tw_climb,
    tw_takeoff,
    tw_ceiling,
    tw_vmax,
    ws_stall,
)

from .wing_sizing import (
    WingStructureParams,
    size_wing,
    wing_mass_raymer_kg,
    wing_mass_nicolai_kg,
)

from .mass_closure import (
    IterStep,
    SizingResult,
    battery_mass_from_energy,
    mtow_from_components,
    run_sizing_loop,
)