#!/usr/bin/env python3
"""
foam2dml_prop.py -- Export ducted-fan and control-vane CFD sweeps to DAVE-ML.

Inputs (either or both):
  --prop-dir prop/sweep    dirs named V*_T* with run_meta.yaml
                           (V_mps, T_disk_N) and a `forces` functionObject
                           on (duct centerbody)
  --vane-dir vanes/sweep   dirs named Vj*_d* with run_meta.yaml
                           (V_jet_mps, delta_deg, S_vane_m2, c_vane_m,
                           h_vane_m, hinge_xc, tc_design) and
                           `forces` on (vane), CofR at the hinge line

Outputs:
  vbat_propulsion_cfd.dml
      installedThrust(V, T_disk)   2-D gridded table [N]
          T_installed = T_disk + Fx(duct + centerbody)
          (x = duct axis, thrust positive +x; the momentum source reaction
          is exactly T_disk, the surfaces add lip suction / ram drag)
      thrustAugmentation(V, T_disk) = T_installed / T_disk  (documentation
          table for the same grid)
  vbat_controls_cfd.dml
      vaneNormalForceCoeff(delta[, V_jet])  CN = Fz / (q_jet S_vane)
      vaneHingeMomentCoeff(delta[, V_jet])  CH = My / (q_jet S_vane c_vane)
      per-vane, single vane in uniform jet; apply the mixing matrix from
      out/control_vanes.yaml downstream.

2-D DAVE-ML gridded tables are written with breakpointRefs order
(first, second) and data in row-major order: the LAST breakpoint varies
fastest, per the DAVE-ML 2.0 reference convention.

Both files carry checkData staticShots (3-sigma tolerances from the
converged iteration tail) for verification by the Aetherion DAVEml loader.
"""

import argparse
import glob
import math
import os
import re
import sys
from datetime import date
from xml.sax.saxutils import escape

DAVE_NS = "http://daveml.org/2010/DAVEML"


def fmt(v, p=6):
    return f"{v:.{p}g}"


# --------------------------------------------------------------------------
# forces functionObject parsing (handles parenthesised and flat layouts)
# --------------------------------------------------------------------------

def _rows(path):
    rows = []
    for line in open(path):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        vals = line.replace("(", " ").replace(")", " ").split()
        try:
            rows.append([float(v) for v in vals])
        except ValueError:
            continue
    return rows


def forces_tail(case_dir, avg_frac):
    """Mean & sd of total force (Fx,Fy,Fz) and moment (Mx,My,Mz)."""
    out = {}
    for kind, fname in (("F", "force.dat"), ("M", "moment.dat")):
        files = sorted(glob.glob(os.path.join(
            case_dir, "postProcessing", "forces", "*", fname)))
        if not files:
            raise RuntimeError(f"no {fname} under {case_dir}")
        rows = _rows(files[-1])
        n = max(1, int(len(rows) * avg_frac))
        tail = rows[-n:]
        for ax, i in (("x", 1), ("y", 2), ("z", 3)):   # cols after Time = total
            vals = [r[i] for r in tail]
            mean = sum(vals) / len(vals)
            sd = (sum((v - mean) ** 2 for v in vals) / len(vals)) ** 0.5
            out[f"{kind}{ax}"] = mean
            out[f"{kind}{ax}_sd"] = sd
    return out


def read_meta(d):
    meta = {}
    p = os.path.join(d, "run_meta.yaml")
    if os.path.isfile(p):
        for line in open(p):
            m = re.match(r"(\w+):\s*([-\d.eE+]+)", line)
            if m:
                meta[m.group(1)] = float(m.group(2))
    return meta


# --------------------------------------------------------------------------
# DAVE-ML document builder
# --------------------------------------------------------------------------

class Dml:
    def __init__(self, name, description):
        self.l = ['<?xml version="1.0" encoding="UTF-8"?>',
                  f'<DAVEfunc xmlns="{DAVE_NS}">',
                  f'  <fileHeader name="{name}">',
                  '    <author name="foam2dml_prop exporter" '
                  'org="ITU Aeronautical Engineering"/>',
                  f'    <fileCreationDate date="{date.today().isoformat()}"/>',
                  '    <description>', escape(description),
                  '    </description>', '  </fileHeader>']
        self.shots = []

    def var(self, name, vid, units, desc, output=False):
        self.l += [f'  <variableDef name="{name}" varID="{vid}" '
                   f'units="{units}">',
                   f'    <description>{escape(desc)}</description>']
        if output:
            self.l.append('    <isOutput/>')
        self.l.append('  </variableDef>')

    def bp(self, bpid, units, vals):
        self.l += [f'  <breakpointDef name="{bpid}" bpID="{bpid}" '
                   f'units="{units}">',
                   f'    <bpVals>{", ".join(fmt(v) for v in vals)}</bpVals>',
                   '  </breakpointDef>']

    def table_fn(self, vid, bpids, data, bounds, desc):
        gt = f"{vid}_T"
        self.l += [f'  <griddedTableDef name="{vid}_table" gtID="{gt}">',
                   '    <breakpointRefs>']
        self.l += [f'      <bpRef bpID="{b}"/>' for b in bpids]
        self.l += ['    </breakpointRefs>',
                   '    <dataTable>',
                   '      ' + ", ".join(fmt(v) for v in data),
                   '    </dataTable>', '  </griddedTableDef>',
                   f'  <function name="{vid}_fn">',
                   f'    <description>{escape(desc)}</description>']
        for (varid, mn, mx) in bounds:
            self.l.append(f'    <independentVarRef varID="{varid}" '
                          f'min="{fmt(mn)}" max="{fmt(mx)}" '
                          'extrapolate="neither"/>')
        self.l += [f'    <dependentVarRef varID="{vid}"/>',
                   f'    <functionDefn name="{vid}_defn">',
                   f'      <griddedTableRef gtID="{gt}"/>',
                   '    </functionDefn>', '  </function>']

    def shot(self, name, inputs, outputs):
        s = [f'    <staticShot name="{name}">', '      <checkInputs>']
        for n, u, v in inputs:
            s.append('        <signal>'
                     f'<signalName>{n}</signalName>'
                     f'<signalUnits>{u}</signalUnits>'
                     f'<signalValue>{fmt(v)}</signalValue></signal>')
        s += ['      </checkInputs>', '      <checkOutputs>']
        for n, u, v, tol in outputs:
            s.append('        <signal>'
                     f'<signalName>{n}</signalName>'
                     f'<signalUnits>{u}</signalUnits>'
                     f'<signalValue>{fmt(v)}</signalValue>'
                     f'<tol>{fmt(max(tol, 1e-6), 3)}</tol></signal>')
        s += ['      </checkOutputs>', '    </staticShot>']
        self.shots += s

    def write(self, path):
        if self.shots:
            self.l += ['  <checkData>'] + self.shots + ['  </checkData>']
        self.l.append('</DAVEfunc>')
        with open(path, "w") as f:
            f.write("\n".join(self.l) + "\n")
        print(f">> wrote {path}")


def grid(points, xkey, ykey, vkey):
    """Sorted unique axes + row-major data (last axis fastest); None-checked."""
    xs = sorted({p[xkey] for p in points})
    ys = sorted({p[ykey] for p in points})
    lut = {(p[xkey], p[ykey]): p[vkey] for p in points}
    data = []
    for x in xs:
        for y in ys:
            if (x, y) not in lut:
                sys.exit(f"grid incomplete: missing ({xkey}={x}, {ykey}={y})")
            data.append(lut[(x, y)])
    return xs, ys, data


# --------------------------------------------------------------------------

def export_prop(prop_dir, avg_frac, out):
    points = []
    geo = None
    for d in sorted(glob.glob(os.path.join(prop_dir, "V*_T*"))):
        meta = read_meta(d)
        try:
            f = forces_tail(d, avg_frac)
        except RuntimeError as e:
            print(f"!! skipping {d}: {e}", file=sys.stderr)
            continue
        if geo is None:
            geo = meta
        Td = meta["T_disk_N"]
        Ti = Td + f["Fx"]           # thrust +x; surfaces add lip suction/drag
        points.append(dict(V=meta["V_mps"], Td=Td, Ti=Ti,
                           tau=Ti / Td if Td else 1.0,
                           sd=f["Fx_sd"]))
        print(f"  V={meta['V_mps']:5.1f}  T_disk={Td:5.1f} N  "
              f"T_inst={Ti:6.2f} N  tau={Ti/Td:5.3f}")
    if len(points) < 4:
        sys.exit("need a filled V x T_disk grid (>=2x2)")

    Vs, Ts, tdata = grid(points, "V", "Td", "Ti")
    _, _, adata = grid(points, "V", "Td", "tau")

    # geometry sentence from run_meta.yaml (Allrun.prop writes these from
    # out/fuselage.yaml) -- never hardcode dimensions here, they drift
    try:
        geom_txt = (
            f"(r_disk = {math.sqrt(geo['A_disk_m2'] / math.pi):.4f} m, "
            f"A = {geo['A_disk_m2']:.5f} m^2, duct mid-chord). Duct "
            f"D_inner/D_outer = {geo['D_duct_inner_m']:.3f}/"
            f"{geo['D_duct_outer_m']:.3f} m, chord {geo['duct_chord_m']:.4f} "
            f"m, NACA0012-section ring; hub r = {geo['r_hub_m']:.4f} m "
            "(out/fuselage.yaml). ")
    except KeyError as e:
        sys.exit(f"run_meta.yaml missing geometry key {e}; sweep predates "
                 "the fuselage.yaml-sourced geometry -- re-run Allrun.prop")

    dml = Dml("vbat_propulsion_cfd",
              "Installed thrust map of the V-BAT ducted fan from steady RANS "
              "(OpenFOAM simpleFoam, k-omega SST) with a uniform actuator-"
              "disk momentum source " + geom_txt +
              "T_installed = T_disk + Fx(duct+centerbody): includes duct lip "
              "suction and ram drag, excludes fan/blade efficiency (no blade "
              "CAD in the conceptual model -- combine with the momentum-"
              "theory fan model). Axis: thrust positive along +x_FRD. "
              "2-D tables, row-major, last breakpoint (diskThrust) fastest. "
              "Companion to vbat_aero_cfd.dml and vbat_controls_cfd.dml.")
    dml.var("axialVelocity", "V", "m_s", "Axial inflow speed along the duct axis.")
    dml.var("diskThrust", "Tdisk", "N", "Momentum imparted by the actuator disk.")
    dml.var("installedThrust", "Tinst", "N",
            "Net installed thrust: disk + duct + centerbody axial force.",
            output=True)
    dml.var("thrustAugmentation", "tau", "nd",
            "T_installed / T_disk (duct augmentation factor).", output=True)
    dml.bp("V_BP", "m_s", Vs)
    dml.bp("TD_BP", "N", Ts)
    dml.table_fn("Tinst", ["V_BP", "TD_BP"], tdata,
                 [("V", min(Vs), max(Vs)), ("Tdisk", min(Ts), max(Ts))],
                 "Installed thrust vs axial speed and disk thrust.")
    dml.table_fn("tau", ["V_BP", "TD_BP"], adata,
                 [("V", min(Vs), max(Vs)), ("Tdisk", min(Ts), max(Ts))],
                 "Duct thrust augmentation ratio.")
    for p in points:
        dml.shot(f"V{fmt(p['V'])}_T{fmt(p['Td'])}",
                 [("axialVelocity", "m_s", p["V"]),
                  ("diskThrust", "N", p["Td"])],
                 [("installedThrust", "N", p["Ti"], 3 * p["sd"])])
    dml.write(out)


def export_vanes(vane_dir, avg_frac, out):
    points = []
    geo = None
    for d in sorted(glob.glob(os.path.join(vane_dir, "Vj*_d*"))):
        meta = read_meta(d)
        try:
            f = forces_tail(d, avg_frac)
        except RuntimeError as e:
            print(f"!! skipping {d}: {e}", file=sys.stderr)
            continue
        if geo is None:
            geo = meta
        rho = meta.get("rho", 1.225)
        S = meta["S_vane_m2"]
        c = meta["c_vane_m"]
        q = 0.5 * rho * meta["V_jet_mps"] ** 2
        CN = f["Fz"] / (q * S)
        CH = f["My"] / (q * S * c)
        points.append(dict(d=meta["delta_deg"], Vj=meta["V_jet_mps"],
                           CN=CN, CH=CH,
                           CN_sd=f["Fz_sd"] / (q * S),
                           CH_sd=f["My_sd"] / (q * S * c)))
        print(f"  Vj={meta['V_jet_mps']:5.1f}  d={meta['delta_deg']:+6.1f}  "
              f"CN={CN:+.4f}  CH={CH:+.5f}")
    if len(points) < 2:
        sys.exit("need >=2 vane points")

    vjs = sorted({p["Vj"] for p in points})
    two_d = len(vjs) > 1

    # geometry sentence from run_meta.yaml (Allrun.vanes writes these from
    # out/control_vanes.yaml) -- never hardcode dimensions here, they drift
    try:
        hinge = geo["hinge_xc"]
        re_c = geo["V_jet_mps"] * geo["c_vane_m"] / 1.5e-5   # nu, case value
        geom_txt = (
            f"vane per out/control_vanes.yaml: span {geo['h_vane_m']:.4f} m, "
            f"chord {geo['c_vane_m']:.5f} m, S = {geo['S_vane_m2']:.6f} m^2, "
            f"hinge at {hinge:.2f}c; 1.5 mm plate thickness for meshability "
            f"(t/c = {geo['tc_design']:.2f} design). ")
    except KeyError as e:
        sys.exit(f"run_meta.yaml missing geometry key {e}; sweep predates "
                 "the control_vanes.yaml-sourced geometry -- re-run "
                 "Allrun.vanes")

    dml = Dml("vbat_controls_cfd",
              "Single control-vane force and hinge-moment coefficients from "
              "steady RANS (OpenFOAM simpleFoam, k-omega SST). Flat-plate "
              + geom_txt +
              "Uniform jet inflow (slip outer boundary), 5% turbulence "
              f"intensity, Re_c ~ {re_c:.2g}. CN = Fz/(q_jet S), CH = "
              "My/(q_jet S c) about the hinge; positive delta deflects the "
              "LE toward -z (positive CN), jet along -x. Per-vane data: "
              "apply the T/B/L/R mixing matrix from control_vanes.yaml "
              "downstream. Companion to vbat_aero_cfd.dml and "
              "vbat_propulsion_cfd.dml.")
    dml.var("vaneDeflection", "delta", "deg",
            f"Vane deflection about the {hinge:.2f}c hinge line.")
    if two_d:
        dml.var("jetVelocity", "Vjet", "m_s", "Local jet speed at the vane.")
    dml.var("vaneNormalForceCoeff", "CN", "nd",
            "Vane normal-force coefficient (per vane).", output=True)
    dml.var("vaneHingeMomentCoeff", "CH", "nd",
            f"Vane hinge-moment coefficient about {hinge:.2f}c.", output=True)

    if two_d:
        ds = sorted({p["d"] for p in points})
        _, _, cndata = grid(points, "Vj", "d", "CN")
        _, _, chdata = grid(points, "Vj", "d", "CH")
        dml.bp("VJ_BP", "m_s", vjs)
        dml.bp("DELTA_BP", "deg", ds)
        bounds = [("Vjet", min(vjs), max(vjs)), ("delta", min(ds), max(ds))]
        dml.table_fn("CN", ["VJ_BP", "DELTA_BP"], cndata, bounds,
                     "Vane normal force coefficient.")
        dml.table_fn("CH", ["VJ_BP", "DELTA_BP"], chdata, bounds,
                     "Vane hinge moment coefficient.")
    else:
        pts = sorted(points, key=lambda p: p["d"])
        ds = [p["d"] for p in pts]
        dml.bp("DELTA_BP", "deg", ds)
        bounds = [("delta", min(ds), max(ds))]
        dml.table_fn("CN", ["DELTA_BP"], [p["CN"] for p in pts], bounds,
                     "Vane normal force coefficient vs deflection.")
        dml.table_fn("CH", ["DELTA_BP"], [p["CH"] for p in pts], bounds,
                     "Vane hinge moment coefficient vs deflection.")

    for p in sorted(points, key=lambda p: (p["Vj"], p["d"])):
        ins = [("vaneDeflection", "deg", p["d"])]
        if two_d:
            ins.append(("jetVelocity", "m_s", p["Vj"]))
        dml.shot(f"Vj{fmt(p['Vj'])}_d{fmt(p['d'])}", ins,
                 [("vaneNormalForceCoeff", "nd", p["CN"], 3 * p["CN_sd"]),
                  ("vaneHingeMomentCoeff", "nd", p["CH"], 3 * p["CH_sd"])])
    dml.write(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--prop-dir")
    ap.add_argument("--vane-dir")
    ap.add_argument("--avg-frac", type=float, default=0.25)
    ap.add_argument("--prop-out", default="vbat_propulsion_cfd.dml")
    ap.add_argument("--vane-out", default="vbat_controls_cfd.dml")
    args = ap.parse_args()
    if not (args.prop_dir or args.vane_dir):
        sys.exit("give --prop-dir and/or --vane-dir")
    if args.prop_dir:
        export_prop(args.prop_dir, args.avg_frac, args.prop_out)
    if args.vane_dir:
        export_vanes(args.vane_dir, args.avg_frac, args.vane_out)


if __name__ == "__main__":
    main()
