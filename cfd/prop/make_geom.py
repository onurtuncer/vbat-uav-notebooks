#!/usr/bin/env python3
"""
make_geom.py -- Parametric duct + centerbody STL for the actuator-disk
propulsion case, dimensions from vbat-uav-notebooks outputs:

    D_duct_inner = 0.286 m, D_duct_outer = 0.302 m, duct_chord = 0.126 m
    r_hub = 0.056 m                                (out/fuselage.yaml)

Local propulsion frame (FRD-consistent): x along the duct axis, inflow
from +x, jet exits toward -x.  Duct mid-chord (= actuator disk plane)
at x = 0.

Duct ring: NACA0012-style symmetric thickness distribution (closed TE)
revolved about x, mean radius R_mid, max thickness = R_outer - R_inner.
Centerbody: tangent-ogive nose + cylinder + conical tail, hub radius r_hub.

Usage:  make_geom.py [--out constant/triSurface] [--nseg 96]
Writes: duct.stl, centerbody.stl
"""
import argparse
import math
import os


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
    # repo values (out/fuselage.yaml); override if the design changes
    ap.add_argument("--chord", type=float, default=0.126)
    ap.add_argument("--r-inner", type=float, default=0.143)
    ap.add_argument("--r-outer", type=float, default=0.151)
    ap.add_argument("--r-hub", type=float, default=0.056)
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    n1 = revolve_to_stl(duct_profile(a.chord, a.r_inner, a.r_outer),
                        os.path.join(a.out, "duct.stl"), "duct", a.nseg)
    n2 = revolve_to_stl(centerbody_profile(a.r_hub),
                        os.path.join(a.out, "centerbody.stl"),
                        "centerbody", a.nseg)
    print(f"duct.stl: {n1} tris; centerbody.stl: {n2} tris "
          f"(disk plane x=0, r_disk = {a.r_inner:.3f} m)")


if __name__ == "__main__":
    main()
