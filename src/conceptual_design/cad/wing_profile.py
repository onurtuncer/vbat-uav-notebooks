"""
wing_profile.py  --  Airfoil sections and simple wing solids (CadQuery / OCCT)

The airfoil MATH lives in conceptual_design.airfoil_selection (single source
of truth); this module only turns those coordinates into OCCT geometry.

Bug fixed vs. original: airfoil_face() annotated/used `cq.Face` while the
module imports `cadquery as cad` (NameError on call).

Units: callers choose.  The functions are unit-agnostic; the vehicle assembly
builds everything in millimetres for STEP/STL export.
"""

import math
from typing import List, Tuple

import cadquery as cad

from conceptual_design.airfoil_selection import parse_naca4, naca4_coordinates


def naca4_points(designation: str, n: int = 120) -> List[Tuple[float, float]]:
    """
    CLOSED loop of (x, y) points on the unit chord for any NACA 4-digit
    section (cambered or symmetric): upper surface LE->TE, then lower
    surface TE->LE, without duplicating LE/TE points.
    """
    M, P, t = parse_naca4(designation)
    xu, yu, xl, yl = naca4_coordinates(M, P, t, n=n)
    upper = list(zip(xu, yu))
    lower = list(zip(xl, yl))[1:-1][::-1]   # TE -> LE, endpoints dropped
    return upper + lower


def naca4_symmetric_points(t: float, n: int = 200) -> List[Tuple[float, float]]:
    """
    Symmetric NACA 00xx airfoil on unit chord [0,1].
    t: thickness ratio (e.g. 0.12 for NACA 0012)
    Returns a CLOSED loop of (x,y) points: upper surface then lower.

    Kept for backwards compatibility; naca4_points("0012") is equivalent.
    """
    xs = [(1.0 - math.cos(math.pi * i / (n - 1))) * 0.5 for i in range(n)]

    def yt(x: float) -> float:
        return 5.0 * t * (
            0.2969 * math.sqrt(max(x, 0.0))
            - 0.1260 * x
            - 0.3516 * x**2
            + 0.2843 * x**3
            - 0.1015 * x**4
        )

    upper = [(x, yt(x)) for x in xs]
    lower = [(x, -yt(x)) for x in reversed(xs[1:-1])]  # avoid duplicate LE/TE
    return upper + lower


def airfoil_face(chord: float = 0.18, thickness: float = 0.12,
                 n: int = 250, designation: str = None) -> cad.Face:
    """
    Create a planar Face for an airfoil section in the XY plane.

    If `designation` is given (e.g. "NACA 2412") the cambered section is
    used and `thickness` is ignored; otherwise a symmetric NACA 00xx of
    the given thickness ratio is built.
    """
    if designation is not None:
        pts = naca4_points(designation, n=n)
    else:
        pts = naca4_symmetric_points(thickness, n=n)
    pts = [(chord * x, chord * y) for x, y in pts]

    wp = cad.Workplane("XY").polyline(pts).close()
    wire = wp.wire().val()  # underlying OCCT wire
    face = cad.Face.makeFromWires(wire)
    return face


def extrude_wing(span: float = 0.6, chord: float = 0.18, thickness: float = 0.12,
                 n: int = 250, designation: str = None) -> cad.Solid:
    """
    Simple straight wing: extrude the airfoil section along +Z by 'span'.
    """
    face = airfoil_face(chord=chord, thickness=thickness, n=n,
                        designation=designation)
    solid = cad.Solid.extrudeLinear(face, cad.Vector(0, 0, 0), cad.Vector(0, 0, span))
    return solid


def export_step(shape, path: str) -> None:
    """
    Export any CadQuery shape to STEP.
    """
    cad.exporters.export(shape, path)
