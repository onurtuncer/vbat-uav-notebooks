"""
cots_integration.py  --  As-Selected Re-Solve Support (post COTS freeze)
=========================================================================

Support for the post-freeze re-solve notebooks (NB12-NB14): once
cots_selection.py has frozen the COTS hardware into out/components.yaml,
the aileron, vibration-isolation, and fuselage designs are re-solved
DOWNSTREAM with the as-selected masses and catalog envelopes instead of
the conceptual estimates.

MOTIVATION
----------
NB4-NB6 size against configured estimates (9g-class servo mass, a 60 g
FC/IMU cluster, an effective pack density) because nothing better exists
before the hardware freeze.  NB11 replaces those estimates with real
parts.  Re-solving NB4-NB6 *after* the freeze -- as new notebooks, not
by re-running the old ones -- keeps the design pipeline one-way
(ADR-0001): the conceptual solution stays untouched as the record of
what the estimates gave, the as-selected solution lands in parallel
*_cots.yaml handoffs, and the deltas between the two are findings
against the estimates, exactly like the mass-allocation findings of the
selection itself.  Nothing feeds back automatically; folding a delta
into config/ is a deliberate next design iteration.

WHAT THIS MODULE PROVIDES
-------------------------
* FrozenComponents      -- typed reader for out/components.yaml
* effective_density     -- catalog envelope -> as-selected packing density
* avionics_budget_bottom_up -- rebuild the avionics "fraction" from real
                           hardware so size_fuselage's carve-out
                           arithmetic nets back to the actual bay content
* propulsion_item_masses -- actual motor/prop/ESC masses for the
                           size_fuselage layout (duct stays modeled: it
                           is airframe structure booked in the propulsion
                           fraction, not a COTS part)
* bay_part_envelopes    -- the catalog envelopes that must physically fit
                           the fuselage bay stack (battery pack, FC board)
* bay_fit_report        -- does each frozen part fit its bay?  Findings,
                           never filters -- same discipline as the
                           selection's mass-allocation report.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from .fuselage_design import (
    PROP_SPLIT,
    FuselageParams,
    FuselageSizing,
    min_axial_length_m,
)


# ---------------------------------------------
#  Frozen-hardware reader (out/components.yaml)
# ---------------------------------------------
@dataclass
class FrozenPart:
    """One frozen COTS part as written by write_components_yaml."""
    id:      str
    name:    str
    category: str
    mass_kg: float
    shape:   str          # "box" | "cylinder"
    dims_m:  tuple
    ratings: dict
    frozen:  bool

    def envelope_volume_m3(self) -> float:
        """Catalog envelope volume (box, or cylinder for motor/prop)."""
        if self.shape == "cylinder":
            d, length = self.dims_m
            return math.pi * (d / 2.0) ** 2 * length
        a, b, c = self.dims_m
        return a * b * c


@dataclass
class FrozenComponents:
    """The frozen COTS hardware list (out/components.yaml)."""
    parts:              dict    # category -> FrozenPart
    supporting_mass_kg: float   # rigid supporting avionics (GPS/RX/...)
    design_point:       dict    # MTOW/P_hover echo, provenance
    budgets:            dict    # mass-allocation findings of the freeze

    @classmethod
    def from_yaml(cls, path) -> "FrozenComponents":
        with open(Path(path), "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        parts = {}
        for cat, e in data["selected"].items():
            parts[cat] = FrozenPart(
                id       = str(e["id"]),
                name     = str(e["name"]),
                category = cat,
                mass_kg  = float(e["mass_g"]) * 1e-3,
                shape    = str(e["shape"]),
                dims_m   = tuple(float(d) * 1e-3 for d in e["dims_mm"]),
                ratings  = dict(e["ratings"]),
                frozen   = bool(e["frozen"]),
            )
        supporting = sum(float(e["mass_g"]) * 1e-3
                         for e in data["supporting_avionics"])
        return cls(parts=parts, supporting_mass_kg=supporting,
                   design_point=dict(data["design_point"]),
                   budgets=dict(data["budgets"]))

    def __getitem__(self, category: str) -> FrozenPart:
        return self.parts[category]


# ---------------------------------------------
#  As-selected packing density
# ---------------------------------------------
def effective_density(part: FrozenPart) -> float:
    """As-selected packing density [kg/m^3]: catalog mass over catalog
    envelope volume.  Replaces the corresponding configured estimate
    (e.g. rho_battery_pack) in the fuselage re-solve -- derived from the
    frozen part, never configured."""
    return part.mass_kg / part.envelope_volume_m3()


# ---------------------------------------------
#  Bottom-up mass reconstruction
# ---------------------------------------------
def avionics_budget_bottom_up(
    m_fc_kg:                 float,   # frozen flight controller [kg]
    m_supporting_kg:         float,   # rigid supporting avionics [kg]
    n_vane_servos:           int,
    n_aileron_servos:        int,
    m_servo_each_kg:         float,   # frozen servo [kg]
    m_isolation_avionics_kg: float,   # FC-tray isolators (NB13 re-solve) [kg]
) -> float:
    """
    Rebuild the avionics 'fraction' bottom-up from the frozen hardware.

    size_fuselage nets its avionics-bay content as budget - carved where
    carved = vane servos + aileron servos + FC-tray isolators.  Feeding
    it (actual bay content + those same carve-outs) makes the net bay
    mass equal the real FC + supporting stack, while keeping the
    carve-out traceability (ADR-0005) intact.
    """
    m_bay = m_fc_kg + m_supporting_kg
    m_carved = ((n_vane_servos + n_aileron_servos) * m_servo_each_kg
                + m_isolation_avionics_kg)
    return m_bay + m_carved


def propulsion_item_masses(
    m_propulsion_closure_kg: float,   # mass-closure propulsion fraction [kg]
    comps: FrozenComponents,
) -> dict:
    """
    Actual propulsion layout masses for size_fuselage's
    prop_item_masses override: motor + prop/fan from the frozen parts,
    duct still modeled as its PROP_SPLIT share of the closure fraction
    (the duct is printed airframe structure booked under propulsion,
    not a COTS part).
    """
    return {
        "motor_fan": comps["edf_motor"].mass_kg + comps["propeller"].mass_kg,
        "esc":       comps["esc"].mass_kg,
        "duct":      PROP_SPLIT["duct"] * m_propulsion_closure_kg,
    }


# ---------------------------------------------
#  Physical fit: envelopes into the bay stack
# ---------------------------------------------
def bay_part_envelopes(comps: FrozenComponents) -> dict:
    """
    Catalog envelopes that must physically fit the fuselage bay stack,
    keyed by bay name (size_fuselage's part_envelopes input).  The
    battery pack and the FC board are the two frozen parts living in
    stacked bays; the payload bay stays volume-based (the payload is
    not a COTS selection), the ESC has its own mid-body slot and the
    motor lives in the tail centerbody (both checked by
    bay_fit_report instead).
    """
    return {
        "battery":  [(comps["battery"].shape, comps["battery"].dims_m)],
        "avionics": [(comps["flight_controller"].shape,
                      comps["flight_controller"].dims_m)],
    }


def bay_fit_report(
    fus:   FuselageSizing,
    p:     FuselageParams,
    comps: FrozenComponents,
) -> list:
    """
    Check each frozen part against the space the re-solved fuselage
    actually gives it.  Returns a list of finding dicts (part, where,
    need_mm, have_mm, ok) -- REPORTED, never filtered on, same
    discipline as the selection's mass-allocation report.

      battery / FC : best-orientation axial length (+ clearance) vs the
                     bay length from the layout
      ESC          : best-orientation axial length (+ clearance) vs its
                     fixed mid-body slot
      motor        : outer diameter vs the tail centerbody (2 * r_hub)
    """
    D_int = fus.D_fus - 2.0 * p.t_shell_m
    bays = {it.name: it for it in fus.items}
    report = []

    def _bay_check(part: FrozenPart, bay_name: str) -> None:
        need = min_axial_length_m(part.shape, part.dims_m, D_int)
        need = need + p.part_clearance_m if math.isfinite(need) else need
        have = bays[bay_name].length
        report.append({
            "part": part.id, "category": part.category, "where": bay_name,
            "need_mm": round(need * 1e3, 1) if math.isfinite(need) else None,
            "have_mm": round(have * 1e3, 1),
            "ok": bool(math.isfinite(need) and need <= have),
        })

    _bay_check(comps["battery"], "battery")
    _bay_check(comps["flight_controller"], "avionics")
    _bay_check(comps["esc"], "esc")

    motor = comps["edf_motor"]
    d_motor = motor.dims_m[0]          # cylinder: (diameter, length)
    report.append({
        "part": motor.id, "category": motor.category,
        "where": "tail centerbody (2 x r_hub)",
        "need_mm": round(d_motor * 1e3, 1),
        "have_mm": round(2.0 * fus.r_hub * 1e3, 1),
        "ok": bool(d_motor <= 2.0 * fus.r_hub),
    })
    return report
