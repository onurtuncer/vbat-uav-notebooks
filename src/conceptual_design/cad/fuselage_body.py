"""
fuselage_body.py  --  Fuselage body of revolution (CadQuery / OCCT)

The meridian profile r(x_s) comes from conceptual_design.fuselage_design.
fuselage_radius -- the SAME function the sizing notebook uses -- so the CAD
solid is by construction the sized fuselage.

AXIS CONVENTION (Aetherion): body FRD -- x forward (out the nose), y right,
z down.  Origin at the nose tip; the body occupies x in [-L_fus, 0].
Stations x_s (from nose, +aft) map to body x = -x_s.

Units: millimetres (STEP/STL convention).  Inputs are in metres, as in the
out/fuselage.yaml handoff, and are scaled internally.
"""

from __future__ import annotations

import cadquery as cq

from conceptual_design.fuselage_design import fuselage_radius

MM = 1000.0   # metres -> millimetres


def fuselage_solid(
    D_fus_m:  float,
    L_fus_m:  float,
    f_nose:   float,
    f_tail:   float,
    r_hub_m:  float,
    n:        int = 160,
) -> cq.Workplane:
    """
    Revolve the sized meridian profile about the body x-axis.

    The closed profile runs: nose tip -> outer surface (n segments) ->
    flat tail base at the hub radius (the fan mounting face) -> back
    along the axis to the nose tip.
    """
    pts = [(0.0, 0.0)]
    for i in range(1, n + 1):
        x_s = L_fus_m * i / n
        r = fuselage_radius(x_s, D_fus_m, L_fus_m, f_nose, f_tail, r_hub_m)
        pts.append((-x_s * MM, r * MM))
    pts.append((-L_fus_m * MM, 0.0))

    profile = cq.Workplane("XY").polyline(pts).close()
    return profile.revolve(360.0, (0, 0, 0), (1, 0, 0))
