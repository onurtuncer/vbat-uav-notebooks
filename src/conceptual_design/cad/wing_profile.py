import math
from typing import List, Tuple

import cadquery as cad


def naca4_symmetric_points(t: float, n: int = 200) -> List[Tuple[float, float]]:
    """
    Symmetric NACA 00xx airfoil on unit chord [0,1].
    t: thickness ratio (e.g. 0.12 for NACA 0012)
    Returns a CLOSED loop of (x,y) points: upper surface then lower.
    """
    # cosine spacing for good leading-edge resolution
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


def airfoil_face(chord: float = 0.18, thickness: float = 0.12, n: int = 250) -> cq.Face:
    """
    Create a planar Face for a symmetric NACA 00xx airfoil.
    """
    pts = naca4_symmetric_points(thickness, n=n)
    pts = [(chord * x, chord * y) for x, y in pts]

    wp = cad.Workplane("XY").polyline(pts).close()
    wire = wp.wire().val()  # underlying OCCT wire
    face = cad.Face.makeFromWires(wire)
    return face


def extrude_wing(span: float = 0.6, chord: float = 0.18, thickness: float = 0.12, n: int = 250) -> cad.Solid:
    """
    Simple straight wing: extrude the airfoil section along +Z by 'span'.
    """
    face = airfoil_face(chord=chord, thickness=thickness, n=n)
    solid = cad.Solid.extrudeLinear(face, cad.Vector(0, 0, 0), cad.Vector(0, 0, span))
    return solid


def export_step(shape, path: str) -> None:
    """
    Export any CadQuery shape to STEP.
    """
    cad.exporters.export(shape, path)
