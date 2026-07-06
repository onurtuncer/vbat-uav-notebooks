#!/usr/bin/env python3
"""
wing_lighten.py -- Structural lightening for the V-BAT wing (the remedy for
parts that cannot be blind-shelled: sharp trailing edges break 3-D BRep
offsets, and a uniform shell is ill-defined where the section is thinner
than two skins).

Method (robust by construction):
  1. Slice the actual wing solid at mid-span; chain the section edges into
     a closed airfoil polygon.
  2. Offset the polygon INWARD in 2-D by the skin thickness (shapely
     negative buffer). The offset naturally vanishes toward the sharp TE,
     so the TE region stays solid -- no degenerate geometry.
  3. Remove chordwise web bands at the spar fraction(s) (--webs, default
     30% and 70% chord) -> fore/mid/aft pocket polygons.
  4. Extrude the pockets spanwise between ribs and subtract from the wing.
     Rib stations can be forced onto the print segmentation planes
     (--align-cuts N for an N-segment print) so step2print's alignment
     pins land in solid ribs.
  5. Optional --spar-hole D: through-bore along the span at the first web
     fraction, at the local mid-thickness, for a carbon tube spar.

Only valid for straight constant-section wings (the V-BAT wing is
rectangular). For tapered/twisted wings, section-per-bay lofting would be
needed.

Usage:
  wing_lighten.py assembly.step [--part wing] [--skin 1.2] [--rib-t 2.0]
      [--bays 10 | --align-cuts 5] [--webs 0.30 0.70] [--web-w 6.0]
      [--spar-hole 8.0] [--out wing_light.step]

Then segment for printing:
  step2print.py wing_light.step --out out/print --stl
"""

import argparse
import math
import sys

from shapely.geometry import Polygon, box as shp_box
from shapely.ops import unary_union

from OCP.BRepAlgoAPI import (BRepAlgoAPI_Section, BRepAlgoAPI_Cut)
from OCP.gp import gp_Pln, gp_Pnt, gp_Dir, gp_Vec, gp_Ax2
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.GCPnts import GCPnts_QuasiUniformDeflection
from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakePolygon,
                                BRepBuilderAPI_MakeFace)
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism, BRepPrimAPI_MakeCylinder
from OCP.BRepCheck import BRepCheck_Analyzer

from step2print import read_assembly, bbox, volume, write_step


# --------------------------------------------------------------------------
# Section extraction: edges -> chained closed polygon in (x, z)
# --------------------------------------------------------------------------

def section_polygon(shape, y, defl=0.05):
    sec = BRepAlgoAPI_Section(shape, gp_Pln(gp_Pnt(0, y, 0),
                                            gp_Dir(0, 1, 0)))
    sec.Build()
    polylines = []
    ex = TopExp_Explorer(sec.Shape(), TopAbs_EDGE)
    while ex.More():
        c = BRepAdaptor_Curve(TopoDS.Edge_s(ex.Current()))
        d = GCPnts_QuasiUniformDeflection(c, defl)
        pts = [(d.Value(i).X(), d.Value(i).Z())
               for i in range(1, d.NbPoints() + 1)]
        if len(pts) >= 2:
            polylines.append(pts)
        ex.Next()
    if not polylines:
        sys.exit(f"no section at y={y}")

    # chain polylines end-to-end into one closed loop
    tol = 1e-3
    loop = polylines.pop(0)
    while polylines:
        end = loop[-1]
        best, rev, dist = None, False, 1e9
        for i, pl in enumerate(polylines):
            d0 = math.dist(end, pl[0])
            d1 = math.dist(end, pl[-1])
            if d0 < dist:
                best, rev, dist = i, False, d0
            if d1 < dist:
                best, rev, dist = i, True, d1
        if dist > 1.0:
            print(f"!! chaining gap {dist:.3f} mm; section may be open",
                  file=sys.stderr)
        pl = polylines.pop(best)
        loop += (pl[::-1] if rev else pl)[1:]
    if math.dist(loop[0], loop[-1]) > tol:
        loop.append(loop[0])
    poly = Polygon(loop)
    if not poly.is_valid:
        poly = poly.buffer(0)
    return poly.simplify(0.02)


# --------------------------------------------------------------------------
# Pocket prisms
# --------------------------------------------------------------------------

def prism_from_polygon(poly, y0, y1):
    """Extrude a shapely polygon (x,z plane) from y0 to y1."""
    mk = BRepBuilderAPI_MakePolygon()
    coords = list(poly.exterior.coords)
    if math.dist(coords[0], coords[-1]) < 1e-9:
        coords = coords[:-1]
    for x, z in coords:
        mk.Add(gp_Pnt(x, y0, z))
    mk.Close()
    face = BRepBuilderAPI_MakeFace(mk.Wire()).Face()
    return BRepPrimAPI_MakePrism(face, gp_Vec(0, y1 - y0, 0)).Shape()


def polys_of(geom):
    if geom.is_empty:
        return []
    if geom.geom_type == "Polygon":
        return [geom]
    return [g for g in geom.geoms if g.geom_type == "Polygon"]


# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("assembly")
    ap.add_argument("--part", default="wing")
    ap.add_argument("--skin", type=float, default=1.2, help="skin, mm")
    ap.add_argument("--rib-t", type=float, default=2.0, help="rib, mm")
    ap.add_argument("--bays", type=int, default=10)
    ap.add_argument("--align-cuts", type=int, metavar="NSEG",
                    help="place ribs on the N-segment print cut planes "
                         "(overrides --bays to a multiple)")
    ap.add_argument("--webs", nargs="*", type=float, default=[0.30, 0.70],
                    help="spar web chord fractions")
    ap.add_argument("--web-w", type=float, default=6.0, help="web width, mm")
    ap.add_argument("--spar-hole", type=float, metavar="D_MM",
                    help="through-bore for a carbon tube at webs[0]")
    ap.add_argument("--min-pocket", type=float, default=30.0,
                    help="skip pockets smaller than this area, mm^2")
    ap.add_argument("--out", default="wing_light.step")
    args = ap.parse_args()

    parts = [p for p in read_assembly(args.assembly)
             if p["name"] == args.part]
    if not parts:
        sys.exit(f"part '{args.part}' not found")
    wing = parts[0]["shape"]
    xmin, ymin, zmin, xmax, ymax, zmax = bbox(wing)
    span, chord = ymax - ymin, xmax - xmin
    v0 = volume(wing)
    print(f">> {args.part}: chord {chord:.1f} mm (LE x={xmax:.1f}), "
          f"span {span:.1f} mm, solid {v0/1000:.1f} cm3")

    # --- section & pockets in 2-D ---------------------------------------
    poly = section_polygon(wing, 0.5 * (ymin + ymax))
    inner = poly.buffer(-args.skin)
    if inner.is_empty:
        sys.exit("skin too thick: inward offset vanished")

    webs = []
    for f in args.webs:
        xw = xmax - f * chord          # LE at xmax (body FRD, x fwd)
        webs.append(shp_box(xw - args.web_w / 2, zmin - 5,
                            xw + args.web_w / 2, zmax + 5))
    pockets2d = [p for p in polys_of(inner.difference(unary_union(webs)))
                 if p.area >= args.min_pocket]
    print(f"   section {poly.area:.0f} mm2 -> {len(pockets2d)} pocket "
          f"regions, skin {args.skin} mm, webs at "
          f"{', '.join(f'{f:.0%}' for f in args.webs)} chord")

    # --- rib stations -----------------------------------------------------
    if args.align_cuts:
        nseg = args.align_cuts
        per = max(1, round(args.bays / nseg))
        nbays = nseg * per             # rib lands on every cut plane
    else:
        nbays = args.bays
    stations = [ymin + span * k / nbays for k in range(nbays + 1)]
    print(f"   {nbays} bays, ribs {args.rib_t} mm at "
          f"{len(stations)} stations"
          + (f" (aligned to {args.align_cuts} print segments)"
             if args.align_cuts else ""))

    # --- subtract pockets ---------------------------------------------------
    work = wing
    ncut = 0
    for k in range(nbays):
        ya = stations[k] + args.rib_t / 2
        yb = stations[k + 1] - args.rib_t / 2
        if yb - ya < 2.0:
            continue
        for p2 in pockets2d:
            work = BRepAlgoAPI_Cut(work,
                                   prism_from_polygon(p2, ya, yb)).Shape()
            ncut += 1

    # --- spar bore ----------------------------------------------------------
    if args.spar_hole:
        xw = xmax - args.webs[0] * chord
        line = shp_box(xw - 0.01, zmin - 5, xw + 0.01, zmax + 5)
        seg = poly.intersection(line)
        zc = 0.5 * (seg.bounds[1] + seg.bounds[3])
        ax = gp_Ax2(gp_Pnt(xw, ymin - 1, zc), gp_Dir(0, 1, 0))
        bore = BRepPrimAPI_MakeCylinder(ax, args.spar_hole / 2,
                                        span + 2).Shape()
        work = BRepAlgoAPI_Cut(work, bore).Shape()
        print(f"   spar bore d{args.spar_hole} mm at x={xw:.1f}, "
              f"z={zc:.1f}, full span")

    v1 = volume(work)
    ok = BRepCheck_Analyzer(work).IsValid()
    print(f">> lightened: {v1/1000:.1f} cm3 ({100*v1/v0:.0f}% of solid, "
          f"{ncut} pocket cuts), valid={ok}")
    write_step(work, args.out, f"{args.part}_light")
    print(f">> wrote {args.out}")


if __name__ == "__main__":
    main()
