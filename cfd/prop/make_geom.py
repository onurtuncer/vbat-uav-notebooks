#!/usr/bin/env python3
"""
make_geom.py -- Parametric duct + centerbody STL for the actuator-disk
propulsion case. Dimensions (duct_chord_m, D_duct_inner_m,
D_duct_outer_m, r_hub_m) are read at run time from the design-pipeline
handoff `out/fuselage.yaml`, so the CFD geometry tracks the converged
design point instead of drifting; CLI flags override single values.

Local propulsion frame (FRD-consistent): x along the duct axis, inflow
from +x, jet exits toward -x.  Duct mid-chord (= actuator disk plane)
at x = 0.

Duct ring: NACA0012-style symmetric thickness distribution (closed TE)
revolved about x, mean radius R_mid, max thickness = R_outer - R_inner.
Centerbody: tangent-ogive nose + cylinder + conical tail, hub radius r_hub.

Usage:  make_geom.py [--out constant/triSurface] [--nseg 96]
        make_geom.py --print-dims    # one line for Allrun.prop:
                                     # chord D_in D_out r_hub r_disk A_disk
Writes: duct.stl, centerbody.stl
"""
import argparse
import math
import os
import re

FUSELAGE_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "..", "..", "out", "fuselage.yaml")


def read_fuselage_dims(path):
    """Duct/hub dimensions from the fuselage handoff. Minimal top-level
    `key: number` parser (same idiom as postprocess/foam2dml.py) -- the
    OpenFOAM host only has a bare python3, no PyYAML."""
    vals = {}
    with open(path) as f:
        for line in f:
            m = re.match(r"([A-Za-z_]\w*):\s*([-+\d.eE]+)\s*$", line)
            if m:
                vals[m.group(1)] = float(m.group(2))
    try:
        return dict(chord=vals["duct_chord_m"],
                    r_inner=vals["D_duct_inner_m"] / 2.0,
                    r_outer=vals["D_duct_outer_m"] / 2.0,
                    r_hub=vals["r_hub_m"])
    except KeyError as e:
        raise SystemExit(
            f"{path}: missing key {e} -- regenerate the design handoffs "
            "(notebooks/fuselage_design.ipynb)")


def naca00_halfthickness(xi, t):
    """Closed-TE NACA 00xx half-thickness at chord fraction xi."""
    return 5*t*(0.2969*math.sqrt(xi) - 0.1260*xi - 0.3516*xi**2
                + 0.2843*xi**3 - 0.1036*xi**4)


def duct_profile(chord, r_in, r_out, n=60):
    """(x, r) polyline of the duct section, LE at +x/2, TE at -x/2."""
    r_mid = 0.5*(r_in + r_out)
    t = (r_out - r_in)/chord
    upper, lower = [], []
    for i in range(n + 1):
        xi = i/n
        h = naca00_halfthickness(xi, t)*chord
        x = chord/2 - xi*chord          # LE at +chord/2
        upper.append((x, r_mid + h))
        lower.append((x, r_mid - h))
    # closed loop: LE -> TE along outer, back TE -> LE along inner
    return upper + lower[::-1][1:]


def centerbody_profile(r_hub, l_nose=0.08, l_cyl=0.10, l_tail=0.14, n=24):
    """(x, r) polyline nose(+x) -> tail(-x); tangent ogive nose, cone tail."""
    pts = [(l_cyl/2 + l_nose, 0.0)]
    rho = (r_hub**2 + l_nose**2)/(2*r_hub)          # ogive radius
    for i in range(1, n + 1):
        s = i/n*l_nose                               # distance from tip
        r = math.sqrt(max(rho**2 - (l_nose - s)**2, 0.0)) + r_hub - rho
        pts.append((l_cyl/2 + l_nose - s, max(r, 0.0)))
    pts.append((-l_cyl/2, r_hub))                    # cylinder aft end
    pts.append((-l_cyl/2 - l_tail, 0.0))             # tail cone apex
    return pts


def revolve_to_stl(profile, path, name, nseg):
    """Revolve an (x, r) polyline about the x-axis; write ASCII STL."""
    tris = []
    for k in range(nseg):
        a0 = 2*math.pi*k/nseg
        a1 = 2*math.pi*(k + 1)/nseg
        for (x0, r0), (x1, r1) in zip(profile, profile[1:]):
            p00 = (x0, r0*math.cos(a0), r0*math.sin(a0))
            p01 = (x0, r0*math.cos(a1), r0*math.sin(a1))
            p10 = (x1, r1*math.cos(a0), r1*math.sin(a0))
            p11 = (x1, r1*math.cos(a1), r1*math.sin(a1))
            if r0 > 1e-9:
                tris.append((p00, p10, p01))
            if r1 > 1e-9:
                tris.append((p01, p10, p11))
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            u = [b[i]-a[i] for i in range(3)]
            v = [c[i]-a[i] for i in range(3)]
            nvec = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2],
                    u[0]*v[1]-u[1]*v[0])
            mag = math.sqrt(sum(x*x for x in nvec)) or 1.0
            f.write(f"  facet normal {nvec[0]/mag:.6e} {nvec[1]/mag:.6e} "
                    f"{nvec[2]/mag:.6e}\n    outer loop\n")
            for p in (a, b, c):
                f.write(f"      vertex {p[0]:.6e} {p[1]:.6e} {p[2]:.6e}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")
    return len(tris)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="constant/triSurface")
    ap.add_argument("--nseg", type=int, default=96)
    ap.add_argument("--fuselage-yaml", default=FUSELAGE_YAML,
                    help="design handoff supplying the duct/hub dimensions")
    # dimensions default to the out/fuselage.yaml handoff; flags are
    # one-off overrides for sensitivity studies, not a place to pin values
    ap.add_argument("--chord", type=float, default=None)
    ap.add_argument("--r-inner", type=float, default=None)
    ap.add_argument("--r-outer", type=float, default=None)
    ap.add_argument("--r-hub", type=float, default=None)
    ap.add_argument("--print-dims", action="store_true",
                    help="print 'chord D_in D_out r_hub r_disk A_disk' "
                         "(SI) and exit; consumed by Allrun.prop")
    a = ap.parse_args()

    dims = read_fuselage_dims(a.fuselage_yaml)
    chord = dims["chord"] if a.chord is None else a.chord
    r_in = dims["r_inner"] if a.r_inner is None else a.r_inner
    r_out = dims["r_outer"] if a.r_outer is None else a.r_outer
    r_hub = dims["r_hub"] if a.r_hub is None else a.r_hub

    if a.print_dims:
        print(f"{chord:.6g} {2*r_in:.6g} {2*r_out:.6g} {r_hub:.6g} "
              f"{r_in:.6g} {math.pi*r_in**2:.6g}")
        return

    os.makedirs(a.out, exist_ok=True)
    n1 = revolve_to_stl(duct_profile(chord, r_in, r_out),
                        os.path.join(a.out, "duct.stl"), "duct", a.nseg)
    n2 = revolve_to_stl(centerbody_profile(r_hub),
                        os.path.join(a.out, "centerbody.stl"),
                        "centerbody", a.nseg)
    print(f"duct.stl: {n1} tris; centerbody.stl: {n2} tris "
          f"(disk plane x=0, r_disk = {r_in:.4f} m)")


if __name__ == "__main__":
    main()
