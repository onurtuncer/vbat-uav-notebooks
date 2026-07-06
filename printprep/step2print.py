#!/usr/bin/env python3
"""
step2print.py -- Produce per-part, 3D-print-ready STEP (and STL) files from
the vbat-uav-notebooks assembly STEP.

Reads out/cad/step/vbat_assembly.step via XCAF so part NAMES and instance
counts survive: identical instances (leg_1..4, strut_1..4, vane_T/B/L/R)
are deduplicated to one exported prototype with a print quantity in the
manifest.

For every unique part:
  * millimetre units preserved (repo CAD is already in mm)
  * optional uniform print scale (--scale)
  * parts exceeding the printer build volume (--build X Y Z, default
    250x250x250 mm) are segmented by planar cuts along their longest axis,
    with cylindrical alignment-pin sockets (--pin-d/--pin-len, default
    4x12 mm; suppress with --no-pins) at each interface.  Sockets are
    placed about the local section centroid and are only kept where they
    actually remove material (verified by volume change), so hollow or
    thin sections degrade gracefully.
  * STEP written to <out>/step/, STL (--stl, linear deflection --defl mm)
    to <out>/stl/, plus <out>/manifest.yaml with bbox, volume, quantity,
    segment count and per-segment files.

Usage:
  step2print.py path/to/vbat_assembly.step [--out out/print]
                [--build 250 250 250] [--scale 1.0]
                [--stl] [--defl 0.1] [--parts wing fuselage] [--no-pins]

Requires: cadquery (brings the OCP/OCCT bindings). Tested with cadquery 2.8.
"""

import argparse
import math
import os
import sys

from OCP.STEPCAFControl import STEPCAFControl_Reader, STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import (TCollection_ExtendedString,
                             TCollection_AsciiString)
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence, TDF_Label, TDF_Tool
from OCP.TDataStd import TDataStd_Name
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepPrimAPI import (BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder)
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut
from OCP.BRepOffsetAPI import BRepOffsetAPI_MakeThickSolid
from OCP.BRepOffset import BRepOffset_Skin
from OCP.TopTools import TopTools_ListOfShape
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Pnt, gp_Dir, gp_Ax2, gp_Trsf
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from OCP.Interface import Interface_Static
from OCP.Message import Message, Message_Alarm

# silence OCCT transfer chatter
for _pr in Message.DefaultMessenger_s().Printers():
    _pr.SetTraceLevel(Message_Alarm)


# --------------------------------------------------------------------------
# XCAF assembly reading
# --------------------------------------------------------------------------

def _name(label):
    n = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), n):
        return TCollection_AsciiString(n.Get()).ToCString()
    return "unnamed"


def _entry(label):
    a = TCollection_AsciiString()
    TDF_Tool.Entry_s(label, a)
    return a.ToCString()


def read_assembly(path):
    """Return list of dicts: {name, shape, qty, instances[names]} of unique
    prototypes (instances deduplicated by referred label entry)."""
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    rdr = STEPCAFControl_Reader()
    if rdr.ReadFile(path) != IFSelect_RetDone:
        sys.exit(f"cannot read {path}")
    rdr.Transfer(doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)

    protos = {}
    for r in range(1, roots.Length() + 1):
        root = roots.Value(r)
        if not tool.IsAssembly_s(root):
            key = _entry(root)
            protos[key] = dict(name=_name(root),
                               shape=tool.GetShape_s(root),
                               instances=[_name(root)])
            continue
        comps = TDF_LabelSequence()
        tool.GetComponents_s(root, comps)
        for i in range(1, comps.Length() + 1):
            ref = TDF_Label()
            tool.GetReferredShape_s(comps.Value(i), ref)
            key = _entry(ref)
            inst = _name(comps.Value(i)) if _name(comps.Value(i)) != "unnamed" \
                else _name(ref)
            if key in protos:
                protos[key]["instances"].append(inst)
            else:
                protos[key] = dict(name=_name(ref),
                                   shape=tool.GetShape_s(ref),
                                   instances=[inst])
    parts = []
    for p in protos.values():
        p["qty"] = len(p["instances"])
        parts.append(p)
    return parts


# --------------------------------------------------------------------------
# Geometry helpers
# --------------------------------------------------------------------------

def bbox(shape):
    b = Bnd_Box()
    BRepBndLib.Add_s(shape, b)
    return b.Get()          # xmin ymin zmin xmax ymax zmax


def volume(shape):
    p = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, p)
    return p.Mass()


def centroid(shape):
    p = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, p)
    c = p.CentreOfMass()
    return c.X(), c.Y(), c.Z()


def scaled(shape, s):
    if abs(s - 1.0) < 1e-12:
        return shape
    t = gp_Trsf()
    t.SetScale(gp_Pnt(0, 0, 0), s)
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def shelled(shape, wall):
    """Hollow `shape` to a closed shell of `wall` mm by subtracting the
    inward-offset core.  Returns (shape, ok).  Fails cleanly on geometry
    that cannot be offset (e.g. sharp trailing edges) -> solid + warning."""
    try:
        mk = BRepOffsetAPI_MakeThickSolid()
        mk.MakeThickSolidByJoin(shape, TopTools_ListOfShape(), -wall,
                                1e-4, BRepOffset_Skin, False, False)
        if not mk.IsDone():
            return shape, False
        core = mk.Shape()
        if core is None or volume(core) < 1.0 or volume(core) >= volume(shape):
            return shape, False
        hollow = BRepAlgoAPI_Cut(shape, core).Shape()
        if not BRepCheck_Analyzer(hollow).IsValid():
            return shape, False
        return hollow, True
    except Exception:
        return shape, False


def box_solid(x0, y0, z0, x1, y1, z1):
    return BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), gp_Pnt(x1, y1, z1)).Shape()


def cyl_along(axis, p0, d, length):
    """Cylinder of diameter d, axis unit vector along `axis` starting p0."""
    ax = gp_Ax2(gp_Pnt(*p0), gp_Dir(*axis))
    return BRepPrimAPI_MakeCylinder(ax, d / 2.0, length).Shape()


AXES = {0: (1, 0, 0), 1: (0, 1, 0), 2: (0, 0, 1)}


def _section_solids(section):
    """Disjoint solids of a slab intersection (hollow parts -> several)."""
    out = []
    ex = TopExp_Explorer(section, TopAbs_SOLID)
    while ex.More():
        out.append(ex.Current())
        ex.Next()
    return out or [section]


def _slice_once(shape, build, pin_d, pin_len, pins, margin):
    """One slicing pass along the worst-violating axis.
    Returns (segments, cut_positions, pin_report); segments may still
    violate other axes (caller recurses)."""
    xmin, ymin, zmin, xmax, ymax, zmax = bbox(shape)
    ext = [xmax - xmin, ymax - ymin, zmax - zmin]
    lo = [xmin, ymin, zmin]
    ratios = [ext[i] / (build[i] - margin) for i in range(3)]
    a = max(range(3), key=lambda i: ratios[i])
    if ratios[a] <= 1.0:
        return [shape], [], []

    nseg = math.ceil(ext[a] / (build[a] - margin))
    cuts = [lo[a] + ext[a] * k / nseg for k in range(1, nseg)]
    BIG = 1e5
    report = []
    work = shape
    if pins:
        for xc in cuts:
            slab_lo = [-BIG, -BIG, -BIG]
            slab_hi = [BIG, BIG, BIG]
            slab_lo[a], slab_hi[a] = xc - 1.5, xc + 1.5
            sect = BRepAlgoAPI_Common(work,
                                      box_solid(*slab_lo, *slab_hi)).Shape()
            chunks = [c for c in _section_solids(sect) if volume(c) > 1.0]
            if not chunks:
                report.append(f"cut@axis{a}={xc:.1f}: empty section, no pins")
                continue
            got, want = 0, 0
            for ch in chunks[:4]:                     # one pin per chunk
                cx = list(centroid(ch))
                cx[a] = xc - pin_len / 2.0
                # if a single chunk (solid section), add a second offset pin
                targets = [cx]
                if len(chunks) == 1:
                    t1, t2 = [i for i in range(3) if i != a]
                    sxmin, symin, szmin, sxmax, symax, szmax = bbox(ch)
                    sext = [sxmax - sxmin, symax - symin, szmax - szmin]
                    toff = t1 if sext[t1] >= sext[t2] else t2
                    off = 0.25 * sext[toff]
                    p2 = cx[:]
                    targets[0][toff] += off
                    p2[toff] -= off
                    targets.append(p2)
                # thin/hollow sections: retry along the wall midline if
                # the centroid candidate is in the cavity
                sxmin, symin, szmin, sxmax, symax, szmax = bbox(sect)
                mid = [(sxmin+sxmax)/2, (symin+symax)/2, (szmin+szmax)/2]
                half = [(sxmax-sxmin)/2, (symax-symin)/2, (szmax-szmin)/2]
                t1, t2 = [i for i in range(3) if i != a]
                edge_cands = []
                for t in (t1, t2):
                    for sgn in (+1, -1):
                        q = mid[:]
                        q[t] += sgn * (half[t] - 1.5 * pin_d)
                        q[a] = xc - pin_len / 2.0
                        edge_cands.append(q)

                full = 0.25 * math.pi * (pin_d / 2) ** 2 * pin_len
                for p0 in targets:
                    want += 1
                    for cand in [p0] + edge_cands:
                        socket = cyl_along(AXES[a], cand, pin_d, pin_len)
                        v0 = volume(work)
                        res = BRepAlgoAPI_Cut(work, socket).Shape()
                        if v0 - volume(res) > 0.9 * full:
                            work = res
                            got += 1
                            edge_cands = [c for c in edge_cands
                                          if c is not cand]
                            break
            report.append(f"cut@axis{a}={xc:.1f}: {got}/{want} pin sockets")

    segs = []
    for k in range(nseg):
        s_lo = [-BIG, -BIG, -BIG]
        s_hi = [BIG, BIG, BIG]
        s_lo[a] = lo[a] + ext[a] * k / nseg if k else -BIG
        s_hi[a] = lo[a] + ext[a] * (k + 1) / nseg if k < nseg - 1 else BIG
        seg = BRepAlgoAPI_Common(work, box_solid(*s_lo, *s_hi)).Shape()
        if volume(seg) > 1.0:
            segs.append(seg)
    return segs, cuts, report


def segment_part(shape, build, pin_d, pin_len, pins=True, margin=2.0,
                 _depth=0):
    """Recursively split until every segment fits the build volume."""
    segs, cuts, report = _slice_once(shape, build, pin_d, pin_len,
                                     pins, margin)
    if not cuts:
        return [shape], 0, []
    out, ncuts = [], len(cuts)
    if _depth > 3:
        return segs, ncuts, report + ["!! recursion limit"]
    for seg in segs:
        sub, subcuts, subrep = segment_part(seg, build, pin_d, pin_len,
                                            pins, margin, _depth + 1)
        out += sub
        ncuts += subcuts
        report += subrep
    return out, ncuts, report


# --------------------------------------------------------------------------
# Export
# --------------------------------------------------------------------------

def write_step(shape, path, name):
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    w = STEPCAFControl_Writer()
    doc = TDocStd_Document(TCollection_ExtendedString("out"))
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    lbl = tool.AddShape(shape, False)
    TDataStd_Name.Set_s(lbl, TCollection_ExtendedString(name))
    w.Transfer(doc, STEPControl_AsIs)
    if w.Write(path) != IFSelect_RetDone:
        raise RuntimeError(f"STEP write failed: {path}")


def write_stl(shape, path, defl):
    BRepMesh_IncrementalMesh(shape, defl, False, 0.5, True)
    wr = StlAPI_Writer()
    wr.ASCIIMode = False
    wr.Write(shape, path)


# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("assembly", help="vbat_assembly.step")
    ap.add_argument("--out", default="out/print")
    ap.add_argument("--build", nargs=3, type=float, default=[250, 250, 250],
                    metavar=("X", "Y", "Z"), help="build volume, mm")
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--stl", action="store_true")
    ap.add_argument("--defl", type=float, default=0.1,
                    help="STL linear deflection, mm")
    ap.add_argument("--parts", nargs="*",
                    help="only these part names (default: all)")
    ap.add_argument("--no-pins", action="store_true")
    ap.add_argument("--shell", type=float, metavar="WALL_MM",
                    help="hollow parts to a closed shell of this wall "
                         "thickness where the offset succeeds (fuselage, "
                         "centerbody...); parts with sharp thin edges "
                         "(wing, vanes) fall back to solid with a warning "
                         "-- use slicer infill for those. NOTE: closed "
                         "cavities; add drain holes downstream for resin/"
                         "SLS, and prefer slicer infill for FDM.")
    ap.add_argument("--pin-d", type=float, default=4.0)
    ap.add_argument("--pin-len", type=float, default=12.0)
    args = ap.parse_args()

    step_dir = os.path.join(args.out, "step")
    stl_dir = os.path.join(args.out, "stl")
    os.makedirs(step_dir, exist_ok=True)
    if args.stl:
        os.makedirs(stl_dir, exist_ok=True)

    parts = read_assembly(args.assembly)
    if args.parts:
        parts = [p for p in parts if p["name"] in args.parts]
        if not parts:
            sys.exit(f"no parts matched {args.parts}")

    manifest = [f"# generated by step2print.py, scale={args.scale}, "
                f"build={args.build} mm", "parts:"]
    for p in sorted(parts, key=lambda q: -volume(q["shape"])):
        shp = scaled(p["shape"], args.scale)
        shell_note = ""
        if args.shell:
            shp, ok = shelled(shp, args.shell)
            shell_note = (f"  [shelled {args.shell} mm]" if ok
                          else "  [shell FAILED -> solid]")
        xmin, ymin, zmin, xmax, ymax, zmax = bbox(shp)
        ext = (xmax - xmin, ymax - ymin, zmax - zmin)
        vol = volume(shp)
        print(f">> {p['name']:12s} x{p['qty']}  "
              f"bbox {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} mm  "
              f"vol {vol/1000:.1f} cm3" + shell_note)

        segs, ncuts, rep = segment_part(shp, args.build,
                                        args.pin_d, args.pin_len,
                                        pins=not args.no_pins)
        for r in rep:
            print(f"     {r}")

        files = []
        for i, seg in enumerate(segs):
            suffix = f"_seg{i+1:02d}of{len(segs):02d}" if len(segs) > 1 else ""
            fname = f"{p['name']}{suffix}.step"
            fpath = os.path.join(step_dir, fname)
            if not BRepCheck_Analyzer(seg).IsValid():
                print(f"     !! {fname}: BRep check reports issues",
                      file=sys.stderr)
            write_step(seg, fpath, p["name"] + suffix)
            files.append(fname)
            if args.stl:
                write_stl(seg, os.path.join(
                    stl_dir, fname.replace(".step", ".stl")), args.defl)
            print(f"     -> {fname}"
                  + ("  (+stl)" if args.stl else ""))

        manifest += [f"  - name: {p['name']}",
                     f"    quantity: {p['qty']}",
                     f"    instances: [{', '.join(p['instances'])}]",
                     f"    bbox_mm: [{ext[0]:.2f}, {ext[1]:.2f}, "
                     f"{ext[2]:.2f}]",
                     f"    volume_cm3: {vol/1000:.2f}",
                     f"    shelled: "
                     f"{'true' if shell_note.startswith('  [shelled') else 'false'}",
                     f"    segments: {len(segs)}",
                     f"    files: [{', '.join(files)}]"]
        if not args.no_pins and ncuts:
            manifest.append(f"    alignment_pins: d{args.pin_d} x "
                            f"{args.pin_len} mm sockets "
                            f"(print {2*ncuts} pins of d"
                            f"{args.pin_d - 0.2:.1f} x "
                            f"{args.pin_len - 0.5:.1f} mm)")

    mpath = os.path.join(args.out, "manifest.yaml")
    with open(mpath, "w") as f:
        f.write("\n".join(manifest) + "\n")
    print(f">> manifest: {mpath}")


if __name__ == "__main__":
    main()
