#!/usr/bin/env python3
"""
foam2dml.py -- Export an OpenFOAM AoA polar to DAVE-ML 2.0 RC.

Reads postProcessing/forceCoeffs/*/coefficient.dat from each polar/aoa_*/
run, averages the converged tail of the iteration history, and writes a
DAVE-ML file (gridded tables over angle of attack) suitable for the
Aetherion DAVEml loader -- the aero sibling of vbat_aero.dml.

Contents of the generated file:
  * variableDefs : angleOfAttack [deg] input; CL, CD, Cm outputs
  * breakpointDef: ALPHA1 (the swept AoA vector)
  * griddedTableDef + function for each of CL(alpha), CD(alpha), Cm(alpha)
  * fileHeader with provenance (solver, Re, references) and derived
    derivatives (CL_alpha, Cm_alpha, CD0, k) as documentation
  * checkData: one staticShot per computed point, so the DAVEml consumer
    (Aetherion) can verify its own table interpolation against the source

Reference quantities match Allrun.case:
  MAC = 0.16704 m, S_ref = 0.16740 m2, CofR = CG (-0.23402 0 0), FRD axes.

Usage:
  foam2dml.py --polar-dir polar [--avg-frac 0.25] [--out vbat_aero_cfd.dml]
              [--plot]
Also accepts a single-run case dir via --case (one AoA, from run_meta.yaml).
"""

import argparse
import glob
import math
import os
import re
import sys
from datetime import date
from xml.sax.saxutils import escape

# --------------------------------------------------------------------------
# forceCoeffs parsing
# --------------------------------------------------------------------------

def parse_coefficient_dat(path):
    """Return (colnames, rows) from an OpenFOAM coefficient.dat.

    Column layout differs across OpenFOAM versions, so the header comment
    line naming the columns is parsed rather than assuming positions.
    """
    colnames, rows = None, []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                toks = line.lstrip("#").split()
                if toks and toks[0] == "Time":
                    colnames = toks
                continue
            vals = line.split()
            try:
                rows.append([float(v) for v in vals])
            except ValueError:
                continue
    if colnames is None or not rows:
        raise RuntimeError(f"could not parse {path}")
    return colnames, rows


def averaged_coeffs(case_dir, avg_frac):
    """Average the last avg_frac of iterations of Cd, Cl, CmPitch."""
    pats = [
        os.path.join(case_dir, "postProcessing", "forceCoeffs", "*",
                     "coefficient.dat"),
        os.path.join(case_dir, "postProcessing", "forceCoeffs", "*",
                     "forceCoeffs.dat"),   # older versions
    ]
    files = sorted(sum((glob.glob(p) for p in pats), []))
    if not files:
        raise RuntimeError(f"no forceCoeffs output under {case_dir}")

    cols, rows = parse_coefficient_dat(files[-1])
    idx = {c: i for i, c in enumerate(cols)}

    def col(*names):
        for n in names:
            if n in idx:
                return idx[n]
        raise KeyError(f"none of {names} in columns {cols}")

    icd, icl = col("Cd"), col("Cl")
    icm = col("CmPitch", "Cm")

    n = max(1, int(len(rows) * avg_frac))
    tail = rows[-n:]

    def stats(i):
        vals = [r[i] for r in tail]
        mean = sum(vals) / len(vals)
        sd = (sum((v - mean) ** 2 for v in vals) / len(vals)) ** 0.5
        return mean, sd

    (cd, cd_sd), (cl, cl_sd), (cm, cm_sd) = stats(icd), stats(icl), stats(icm)
    return dict(CD=cd, CL=cl, Cm=cm,
                CD_sd=cd_sd, CL_sd=cl_sd, Cm_sd=cm_sd,
                n_avg=n, n_total=len(rows))


def read_meta(case_dir):
    meta = {}
    p = os.path.join(case_dir, "run_meta.yaml")
    if os.path.isfile(p):
        for line in open(p):
            m = re.match(r"(\w+):\s*([-\d.eE+]+)", line)
            if m:
                meta[m.group(1)] = float(m.group(2))
    return meta


# --------------------------------------------------------------------------
# Derived derivatives (documentation only; tables carry the data)
# --------------------------------------------------------------------------

def linear_fit(x, y):
    n = len(x)
    sx, sy = sum(x), sum(y)
    sxx = sum(v * v for v in x)
    sxy = sum(a * b for a, b in zip(x, y))
    den = n * sxx - sx * sx
    if abs(den) < 1e-30:
        return 0.0, sy / n
    slope = (n * sxy - sx * sy) / den
    return slope, (sy - slope * sx) / n


def derived(points):
    """CL_alpha, Cm_alpha (per rad, linear range |a|<=8 deg), CD0, k."""
    lin = [p for p in points if abs(p["aoa"]) <= 8.0] or points
    a_rad = [math.radians(p["aoa"]) for p in lin]
    cla, _ = linear_fit(a_rad, [p["CL"] for p in lin])
    cma, _ = linear_fit(a_rad, [p["Cm"] for p in lin])
    k, cd0 = linear_fit([p["CL"] ** 2 for p in lin], [p["CD"] for p in lin])
    return dict(CL_alpha=cla, Cm_alpha=cma, CD0=cd0, k=k)


# --------------------------------------------------------------------------
# DAVE-ML writing
# --------------------------------------------------------------------------

DAVE_NS = "http://daveml.org/2010/DAVEML"

def fmt(v, p=6):
    return f"{v:.{p}g}"


def write_dml(points, drv, out, v_mps, avg_frac):
    alphas = [p["aoa"] for p in points]
    today = date.today().isoformat()

    def table(vals):
        return ", ".join(fmt(v) for v in vals)

    lines = []
    w = lines.append
    w('<?xml version="1.0" encoding="UTF-8"?>')
    w(f'<DAVEfunc xmlns="{DAVE_NS}">')

    # ---- header ----
    w('  <fileHeader name="vbat_aero_cfd">')
    w('    <author name="foam2dml exporter" org="ITU Aeronautical Engineering"/>')
    w(f'    <fileCreationDate date="{today}"/>')
    w('    <description>')
    w(escape(
        f"Static longitudinal aerodynamic coefficients of the V-BAT-like "
        f"tail-sitter from steady RANS (OpenFOAM simpleFoam, k-omega SST). "
        f"Freestream {fmt(v_mps)} m/s (Re_MAC ~ {fmt(v_mps*0.16704/1.5e-5,3)}). "
        f"References: S_ref = 0.16740 m^2, MAC = 0.16704 m, moment reference "
        f"at CG (body FRD x = -0.23402 m). Stability-axis CL/CD, body-axis "
        f"pitching moment Cm. Coefficients are the mean of the final "
        f"{int(avg_frac*100)}% of SIMPLE iterations. Derived (linear range, "
        f"|alpha| <= 8 deg): CL_alpha = {fmt(drv['CL_alpha'],4)} /rad, "
        f"Cm_alpha = {fmt(drv['Cm_alpha'],4)} /rad, "
        f"CD0 = {fmt(drv['CD0'],4)}, k = {fmt(drv['k'],4)}. "
        f"Companion to vbat_propulsion.dml and vbat_controls.dml."))
    w('    </description>')
    w('  </fileHeader>')

    # ---- variables ----
    w('  <variableDef name="angleOfAttack" varID="alpha" units="deg" '
      'symbol="&#x3B1;">')
    w('    <description>Body angle of attack (FRD; positive nose-up '
      'relative wind).</description>')
    w('    <isStdAIAA/>')
    w('  </variableDef>')
    for vid, name, desc in [
        ("CL", "liftCoefficient",
         "Stability-axis lift coefficient, positive up (-z_FRD at alpha=0)."),
        ("CD", "dragCoefficient",
         "Stability-axis drag coefficient."),
        ("Cm", "pitchMomentCoefficient",
         "Pitching-moment coefficient about the CG, lRef = MAC."),
    ]:
        w(f'  <variableDef name="{name}" varID="{vid}" units="nd">')
        w(f'    <description>{escape(desc)}</description>')
        w('    <isOutput/>')
        w('  </variableDef>')

    # ---- breakpoints ----
    w('  <breakpointDef name="alphaBreakpoints" bpID="ALPHA1" units="deg">')
    w(f'    <bpVals>{table(alphas)}</bpVals>')
    w('  </breakpointDef>')

    # ---- tables + functions ----
    for vid in ("CL", "CD", "Cm"):
        w(f'  <griddedTableDef name="{vid}_table" gtID="{vid}_T">')
        w('    <breakpointRefs><bpRef bpID="ALPHA1"/></breakpointRefs>')
        w('    <dataTable>')
        w('      ' + table([p[vid] for p in points]))
        w('    </dataTable>')
        w('  </griddedTableDef>')
    for vid in ("CL", "CD", "Cm"):
        w(f'  <function name="{vid}_basic">')
        w(f'    <description>{vid} vs angle of attack, CFD polar.'
          '</description>')
        w(f'    <independentVarRef varID="alpha" min="{fmt(min(alphas))}" '
          f'max="{fmt(max(alphas))}" extrapolate="neither"/>')
        w(f'    <dependentVarRef varID="{vid}"/>')
        w(f'    <functionDefn name="{vid}_fn">')
        w(f'      <griddedTableRef gtID="{vid}_T"/>')
        w('    </functionDefn>')
        w('  </function>')

    # ---- checkData: every computed point becomes a verification case ----
    w('  <checkData>')
    for p in points:
        w(f'    <staticShot name="alpha_{fmt(p["aoa"])}_deg">')
        w('      <checkInputs>')
        w('        <signal><signalName>angleOfAttack</signalName>'
          '<signalUnits>deg</signalUnits>'
          f'<signalValue>{fmt(p["aoa"])}</signalValue></signal>')
        w('      </checkInputs>')
        w('      <checkOutputs>')
        for vid, name in [("CL", "liftCoefficient"),
                          ("CD", "dragCoefficient"),
                          ("Cm", "pitchMomentCoefficient")]:
            tol = max(3 * p[f"{vid}_sd"], 1e-6)   # 3-sigma of iteration tail
            w(f'        <signal><signalName>{name}</signalName>'
              '<signalUnits>nd</signalUnits>'
              f'<signalValue>{fmt(p[vid])}</signalValue>'
              f'<tol>{fmt(tol,3)}</tol></signal>')
        w('      </checkOutputs>')
        w('    </staticShot>')
    w('  </checkData>')

    w('</DAVEfunc>')
    with open(out, "w") as fh:
        fh.write("\n".join(lines) + "\n")


# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--polar-dir", default="polar",
                    help="directory containing aoa_*/ runs")
    ap.add_argument("--case", help="single-run case directory instead")
    ap.add_argument("--avg-frac", type=float, default=0.25,
                    help="fraction of final iterations to average")
    ap.add_argument("--out", default="vbat_aero_cfd.dml")
    ap.add_argument("--plot", action="store_true",
                    help="write polar plots next to the .dml")
    args = ap.parse_args()

    dirs = ([args.case] if args.case else
            sorted(glob.glob(os.path.join(args.polar_dir, "aoa_*"))))
    if not dirs:
        sys.exit(f"no runs found under {args.polar_dir}")

    points, v_mps = [], 20.0
    for d in dirs:
        meta = read_meta(d)
        if "aoa_deg" not in meta:
            m = re.search(r"aoa_([+-]?\d+(?:\.\d+)?)", d)
            if not m:
                print(f"!! skipping {d}: no AoA metadata", file=sys.stderr)
                continue
            meta["aoa_deg"] = float(m.group(1))
        v_mps = meta.get("V_mps", v_mps)
        try:
            c = averaged_coeffs(d, args.avg_frac)
        except RuntimeError as e:
            print(f"!! skipping {d}: {e}", file=sys.stderr)
            continue
        c["aoa"] = meta["aoa_deg"]
        points.append(c)
        print(f"  alpha={c['aoa']:+6.1f}  CL={c['CL']:+.4f}  "
              f"CD={c['CD']:.5f}  Cm={c['Cm']:+.4f}  "
              f"(avg {c['n_avg']}/{c['n_total']} iters, "
              f"sd CL {c['CL_sd']:.1e})")

    if len(points) < 2:
        sys.exit("need at least 2 converged points for a table")
    points.sort(key=lambda p: p["aoa"])

    drv = derived(points)
    print(f"\nDerived (|alpha|<=8 deg): CL_alpha={drv['CL_alpha']:.3f}/rad  "
          f"Cm_alpha={drv['Cm_alpha']:.3f}/rad  "
          f"CD0={drv['CD0']:.5f}  k={drv['k']:.5f}")

    write_dml(points, drv, args.out, v_mps, args.avg_frac)
    print(f">> wrote {args.out}")

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            a = [p["aoa"] for p in points]
            fig, ax = plt.subplots(1, 3, figsize=(13, 4))
            for i, (vid, lbl) in enumerate([("CL", "$C_L$"),
                                            ("CD", "$C_D$"),
                                            ("Cm", "$C_m$")]):
                ax[i].plot(a, [p[vid] for p in points], "o-")
                ax[i].set_xlabel(r"$\alpha$ [deg]")
                ax[i].set_ylabel(lbl)
                ax[i].grid(alpha=0.3)
            fig.tight_layout()
            png = os.path.splitext(args.out)[0] + "_polar.png"
            fig.savefig(png, dpi=150)
            print(f">> wrote {png}")
        except ImportError:
            print("!! matplotlib unavailable; skipped plots",
                  file=sys.stderr)


if __name__ == "__main__":
    main()
