"""
sizing_studies.py  --  Parameter Studies Around the Mass Closure
================================================================

Re-runs of the sizing loop with one input varied at a time, used by the
conceptual-design notebook (NB1):

  * construction-method trade (monocoque vs segmented FDM, ADR-0008)
  * MTOW sensitivity sweeps (battery Wh/kg, rotor diameter, L/D, fs)
  * normalised local elasticities at the baseline

Divergent points (the mission does not close) are carried as None so a
sweep can cross the closure boundary without aborting.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np
import yaml

from .design_point import DesignInputs
from .mass_closure import SizingResult


# ---------------------------------------------
#  Construction-method trade (ADR-0008)
# ---------------------------------------------
@dataclass(frozen=True)
class ConstructionTradeRow:
    method:      str
    k:           float    # structural knock-up factor
    m_total_kg:  float
    m_batt_kg:   float
    P_hover_W:   float
    C_rate_peak: float
    T_hover_N:   float    # hover thrust need (s_ratio * W)
    b_wing_m:    float
    baseline:    bool     # configured construction method?


def construction_trade(inputs: DesignInputs, config_dir) -> list[ConstructionTradeRow]:
    """Re-converge the closure for BOTH construction methods, with k for
    each read straight from config so both use configured values."""
    with open(f"{config_dir}/initial_weight_fraction_estimation.yaml",
              encoding="utf-8") as f:
        k_table = yaml.safe_load(f)["k_construction"]

    rows = []
    for method in ("monocoque", "segmented_fdm"):
        k_m = float(k_table[method])
        wf_m = replace(inputs.wf, fs=inputs.wf.fs_base * k_m,
                       k_construction=k_m, construction_method=method)
        r_m = inputs.solve(wf=wf_m)
        W_m = r_m.m_total_kg * inputs.env.g
        rows.append(ConstructionTradeRow(
            method=method, k=k_m,
            m_total_kg=r_m.m_total_kg, m_batt_kg=r_m.m_battery_kg,
            P_hover_W=r_m.P_hover_W, C_rate_peak=r_m.C_rate_peak,
            T_hover_N=inputs.prop.s_ratio * W_m, b_wing_m=r_m.wing.b_wing,
            baseline=(method == inputs.wf.construction_method),
        ))
    return rows


def hover_thrust_guard(inputs: DesignInputs,
                       rows: list[ConstructionTradeRow]) -> float:
    """The baseline method's hover thrust need, checked against the COTS
    rotor guard T_max_N (config/rotor.yaml, ADR-0003).  Fails loudly
    rather than sizing past the purchasable hardware."""
    T_need = next(r.T_hover_N for r in rows if r.baseline)
    assert T_need < inputs.rotor.T_max_N, (
        f"hover thrust need {T_need:.1f} N exceeds the COTS EDF guard "
        f"T_max_N={inputs.rotor.T_max_N:.0f} N (config/rotor.yaml, ADR-0003) -- "
        f"revisit k_construction or the mission")
    return T_need


# ---------------------------------------------
#  One-at-a-time sensitivity sweeps
# ---------------------------------------------
@dataclass
class SweepSeries:
    label:    str            # axis label for plotting
    values:   np.ndarray     # swept parameter values
    baseline: float          # the configured value of the parameter
    results:  list           # SizingResult | None per point

    @property
    def mtow(self) -> list[float]:
        """MTOW per point; NaN where the loop diverged / did not converge."""
        return [r.m_total_kg if (r is not None and r.converged) else float("nan")
                for r in self.results]


def sweep(inputs: DesignInputs, values, modifier) -> list[SizingResult | None]:
    """Run the sizing loop for each value; ``modifier(v)`` returns the
    ``inputs.solve`` overrides.  Divergent points yield None."""
    results = []
    for v in values:
        try:
            results.append(inputs.solve(**modifier(v)))
        except ValueError:   # divergence guard tripped
            results.append(None)
    return results


def mtow_sensitivity_sweeps(inputs: DesignInputs,
                            ranges: dict) -> dict[str, SweepSeries]:
    """The four standard MTOW sweeps keyed by parameter name.

    ``ranges`` maps any of 'specific_energy', 'D_rotor', 'LD', 'fs' to the
    array of values to sweep (the analysis choice stays with the caller).
    """
    factories = {
        "specific_energy": (
            "Battery specific energy  [Wh/kg]", inputs.batt.specific_energy,
            lambda v: {"batt": replace(inputs.batt, specific_energy=float(v))}),
        "D_rotor": (
            "EDF diameter  [m]", inputs.rotor.D_rotor_m,
            lambda v: {"D_rotor_m": float(v)}),
        "LD": (
            "Cruise L/D ratio  [-]", inputs.aero.LD,
            lambda v: {"aero": replace(inputs.aero, LD=float(v))}),
        "fs": (
            "Structural weight fraction  $f_s$  [-]", inputs.wf.fs,
            lambda v: {"wf": replace(inputs.wf, fs=float(v))}),
    }
    out = {}
    for name, values in ranges.items():
        label, baseline, modifier = factories[name]
        out[name] = SweepSeries(label=label, values=np.asarray(values),
                                baseline=baseline,
                                results=sweep(inputs, values, modifier))
    return out


def local_sensitivity(series: SweepSeries, base_mtow: float) -> float:
    """Normalised elasticity dMTOW/dX * X/MTOW via a central difference at
    the sweep point nearest the baseline (NaN at the sweep edges)."""
    m = series.mtow
    i = int(np.argmin(np.abs(series.values - series.baseline)))
    if i == 0 or i >= len(series.values) - 1:
        return float("nan")
    dm = m[i + 1] - m[i - 1]
    dp = series.values[i + 1] - series.values[i - 1]
    return (dm / dp) * (series.values[i] / base_mtow)
