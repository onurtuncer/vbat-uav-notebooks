"""
cots_selection.py  --  COTS component selection & freeze
=========================================================

Selects the COTS hardware the conceptual design leaves open -- flight
controller, ESC, EDF drive motor, propeller (the fan unit itself), the
(single-type) vane/aileron servo, and the battery pack -- from the
per-category database files in config/components/, and freezes the
winners (with their mass and own-CG inertia tensors) into
out/components.yaml.

MOTIVATION
----------
ADR-0011 commits the first flights to PX4 on a COTS Pixhawk-class FC
with a telemetry ESC, and notes that the electrical placeholders "stop
being placeholders once the COTS hardware list is frozen".  This module
is that freeze: a deterministic, requirement-checked selection whose
output is pinned by the design-regression tests, so a design change
that outgrows a chosen part turns CI red instead of drifting silently.

SELECTION RULE
--------------
Hard requirements are DERIVED from the converged design point (never
configured -- only the margins in config/components/selection.yaml are
inputs):

    flight controller : PX4-supported, >= fc_min_imu_count IMUs
    ESC               : I_cont  >= esc_rating_a (the wiring module's
                        margined hover current), pack cell count in
                        [s_min, s_max], live RPM/current telemetry
    EDF motor         : P_max   >= motor_power_margin * P_design,
                        pack cell count in [s_min, s_max]
    propeller (fan)   : rotor diameter == D_rotor_m (ADR-0003 froze the
                        class; the disk loading is derived from it, so a
                        different diameter is a different design point),
                        P_max >= motor_power_margin * P_design
    servo             : stall torque at the 5 V BEC rail
                        >= servo_torque_margin * worst hinge moment
                        (jet vane vs aileron, whichever is larger)
    battery           : cell count == the configured series bus
                        (config/electrical.yaml), capacity >= the sized
                        mission capacity (the wiring module's
                        pack_capacity_ah), continuous discharge current
                        (C_cont * capacity) >= esc_rating_a -- the pack
                        feeds the ESC, so the same margined hover
                        current applies

Among the candidates passing every hard requirement the LIGHTEST wins
(ties broken alphabetically by id).  A `frozen` id in the config pins a
category to a specific part -- the pin is still validated against the
derived requirements and raises if a design change has outgrown it.

Mass-allocation fit (avionics bay, ESC / motor_fan layout allocations,
per-servo carve-out) is REPORTED, never filtered on: if the lightest
feasible part busts its weight-fraction allocation that is a standing
finding against config/initial_weight_fraction_estimation.yaml, exactly
like the marginal ESC cold-plate of ADR-0009.

INERTIA
-------
Each frozen part gets its inertia tensor about its own CG from its
catalog envelope (solid box, or solid cylinder for the motor and fan),
axes parallel to the envelope's local axes.  Mounting orientation inside
the airframe is an installation detail left to the NB9 mass-properties
refinement.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


# Category name (singular, as used in policy/frozen/report keys) ->
# database list key AND per-category file stem in config/components/
# (e.g. "servo" -> servos.yaml with a top-level `servos:` list).
CATEGORIES = {
    "flight_controller": "flight_controllers",
    "esc":               "escs",
    "edf_motor":         "edf_motors",
    "propeller":         "propellers",
    "servo":             "servos",
    "battery":           "batteries",
}

# Rotor-diameter match tolerance for the propeller category [m].  A
# numerical comparison guard, not a design freedom: the fan must BE the
# ADR-0003 diameter class, this only absorbs catalog rounding (195.0 vs
# 195.5 mm style differences).
D_ROTOR_TOL_M = 1.0e-3


# ---------------------------------------------
#  Database containers
# ---------------------------------------------
@dataclass
class ComponentSpec:
    """One catalog candidate.  dims_m: box = (a, b, c) envelope;
    cylinder = (diameter, length).  ratings: category-specific fields
    (imu_count, i_cont_a, p_max_w, stall_torque_gcm, ...).  url:
    procurement link (prefer an in-country retailer), None if the part
    has no known local source."""
    id:       str
    name:     str
    category: str
    mass_kg:  float
    shape:    str          # "box" | "cylinder"
    dims_m:   tuple
    ratings:  dict
    url:      str | None = None

    def inertia_cg(self) -> tuple:
        """(I1, I2, I3) about the part's own CG [kg m^2], axes along the
        envelope's local axes (cylinder: axis 1 = rotation axis)."""
        m = self.mass_kg
        if self.shape == "cylinder":
            d, length = self.dims_m
            r = d / 2.0
            I_ax = 0.5 * m * r ** 2
            I_tr = m * (3.0 * r ** 2 + length ** 2) / 12.0
            return (I_ax, I_tr, I_tr)
        if self.shape == "box":
            a, b, c = self.dims_m
            return (
                m * (b ** 2 + c ** 2) / 12.0,
                m * (a ** 2 + c ** 2) / 12.0,
                m * (a ** 2 + b ** 2) / 12.0,
            )
        raise ValueError(f"{self.id}: unknown shape '{self.shape}'")


@dataclass
class SelectionPolicy:
    fc_min_imu_count:      int
    fc_require_px4:        bool
    esc_require_telemetry: bool
    motor_power_margin:    float
    servo_torque_margin:   float
    frozen:                dict   # category -> part id or None


@dataclass
class ComponentDB:
    policy:     SelectionPolicy
    supporting: list          # [(name, mass_kg)] -- avionics-bay fit only
    candidates: dict          # category -> [ComponentSpec]

    @classmethod
    def from_dir(cls, dir_path) -> "ComponentDB":
        """Load the database from a config/components/ directory:
        selection.yaml (policy), supporting_avionics.yaml, and one
        <list-key>.yaml per category (e.g. servos.yaml)."""
        dir_path = Path(dir_path)

        def _load(name):
            with open(dir_path / name, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}

        sel = _load("selection.yaml")["selection"]
        policy = SelectionPolicy(
            fc_min_imu_count      = int(sel["fc_min_imu_count"]),
            fc_require_px4        = bool(sel["fc_require_px4"]),
            esc_require_telemetry = bool(sel["esc_require_telemetry"]),
            motor_power_margin    = float(sel["motor_power_margin"]),
            servo_torque_margin   = float(sel["servo_torque_margin"]),
            frozen = {cat: sel["frozen"].get(cat) for cat in CATEGORIES},
        )
        supporting = [
            (str(e["name"]), float(e["mass_g"]) * 1e-3)
            for e in _load("supporting_avionics.yaml")["supporting_avionics"]]
        candidates = {}
        for cat, key in CATEGORIES.items():
            specs = []
            for e in _load(f"{key}.yaml")[key]:
                base = {"id", "name", "mass_g", "dims_mm", "shape", "url"}
                specs.append(ComponentSpec(
                    id       = str(e["id"]),
                    name     = str(e["name"]),
                    category = cat,
                    mass_kg  = float(e["mass_g"]) * 1e-3,
                    shape    = str(e["shape"]),
                    dims_m   = tuple(float(d) * 1e-3 for d in e["dims_mm"]),
                    ratings  = {k: v for k, v in e.items() if k not in base},
                    url      = str(e["url"]) if e.get("url") else None,
                ))
            ids = [s.id for s in specs]
            if len(ids) != len(set(ids)):
                raise ValueError(f"duplicate part ids in '{key}'")
            candidates[cat] = specs
        return cls(policy=policy, supporting=supporting, candidates=candidates)


# ---------------------------------------------
#  Requirement derivation
# ---------------------------------------------
def derive_requirements(
    P_design_W:          float,   # sizing max electrical power [W]
    esc_rating_a:        float,   # margined hover current (wiring module) [A]
    series_cells:        int,     # pack cell count (config/electrical.yaml)
    D_rotor_m:           float,   # frozen rotor diameter (config/rotor.yaml)
    tau_vane_req_gcm:    float,   # vane servo torque requirement [g cm]
    tau_aileron_req_gcm: float,   # aileron servo torque requirement [g cm]
    pack_capacity_ah:    float,   # sized mission capacity (wiring module) [Ah]
    policy:              SelectionPolicy,
) -> dict:
    """Category -> {requirement name: value}.  All values derived from
    the converged design point; only the policy margins are inputs."""
    tau_worst = max(tau_vane_req_gcm, tau_aileron_req_gcm)
    return {
        "flight_controller": {
            "px4_required":     policy.fc_require_px4,
            "min_imu_count":    policy.fc_min_imu_count,
        },
        "esc": {
            "i_cont_min_a":     esc_rating_a,
            "series_cells":     series_cells,
            "telemetry_required": policy.esc_require_telemetry,
        },
        "edf_motor": {
            "p_max_min_w":      policy.motor_power_margin * P_design_W,
            "series_cells":     series_cells,
        },
        "propeller": {
            "d_rotor_mm":       D_rotor_m * 1e3,
            "p_max_min_w":      policy.motor_power_margin * P_design_W,
        },
        "servo": {
            "stall_torque_min_gcm": policy.servo_torque_margin * tau_worst,
            "tau_vane_req_gcm":     tau_vane_req_gcm,
            "tau_aileron_req_gcm":  tau_aileron_req_gcm,
        },
        "battery": {
            "capacity_min_ah":  pack_capacity_ah,
            "series_cells":     series_cells,
            "i_dis_min_a":      esc_rating_a,
        },
    }


def _rejection_reason(spec: ComponentSpec, req: dict) -> str | None:
    """None if the candidate passes every hard requirement, else why not."""
    r = spec.ratings
    if spec.category == "flight_controller":
        if req["px4_required"] and not r.get("px4", False):
            return "not a supported PX4 target (ADR-0011)"
        if int(r["imu_count"]) < req["min_imu_count"]:
            return (f"{r['imu_count']} IMU(s) < required "
                    f"{req['min_imu_count']}")
    elif spec.category == "esc":
        if float(r["i_cont_a"]) < req["i_cont_min_a"]:
            return (f"{r['i_cont_a']:.0f} A cont < required "
                    f"{req['i_cont_min_a']:.1f} A")
        if not (int(r["s_min"]) <= req["series_cells"] <= int(r["s_max"])):
            return (f"{req['series_cells']}S outside "
                    f"{r['s_min']}-{r['s_max']}S range")
        if req["telemetry_required"] and not r.get("telemetry", False):
            return "no live RPM/current telemetry (ADR-0011)"
    elif spec.category == "edf_motor":
        if float(r["p_max_w"]) < req["p_max_min_w"]:
            return (f"{r['p_max_w']:.0f} W max < required "
                    f"{req['p_max_min_w']:.0f} W")
        if not (int(r["s_min"]) <= req["series_cells"] <= int(r["s_max"])):
            return (f"{req['series_cells']}S outside "
                    f"{r['s_min']}-{r['s_max']}S range")
    elif spec.category == "propeller":
        if abs(float(r["d_rotor_mm"]) * 1e-3 - req["d_rotor_mm"] * 1e-3) > D_ROTOR_TOL_M:
            return (f"{r['d_rotor_mm']:.0f} mm rotor != frozen "
                    f"{req['d_rotor_mm']:.0f} mm class (ADR-0003: different "
                    "diameter = different disk loading = different design point)")
        if float(r["p_max_w"]) < req["p_max_min_w"]:
            return (f"{r['p_max_w']:.0f} W rating < required "
                    f"{req['p_max_min_w']:.0f} W")
    elif spec.category == "servo":
        if float(r["stall_torque_gcm"]) < req["stall_torque_min_gcm"]:
            return (f"{r['stall_torque_gcm']:.0f} g cm stall < required "
                    f"{req['stall_torque_min_gcm']:.0f} g cm")
    elif spec.category == "battery":
        if int(r["s_cells"]) != req["series_cells"]:
            return (f"{r['s_cells']}S pack != configured "
                    f"{req['series_cells']}S bus")
        cap_ah = float(r["capacity_mah"]) * 1e-3
        if cap_ah < req["capacity_min_ah"]:
            return (f"{cap_ah:.2f} Ah < required {req['capacity_min_ah']:.2f} "
                    "Ah mission capacity")
        i_dis = float(r["c_rate_cont"]) * cap_ah
        if i_dis < req["i_dis_min_a"]:
            return (f"{i_dis:.0f} A cont discharge < required "
                    f"{req['i_dis_min_a']:.1f} A")
    else:
        raise ValueError(f"unknown category '{spec.category}'")
    return None


# ---------------------------------------------
#  Selection
# ---------------------------------------------
@dataclass
class CategorySelection:
    category:     str
    requirements: dict
    selected:     ComponentSpec
    frozen:       bool               # True if pinned via config `frozen`
    feasible_ids: list               # all ids passing, lightest first
    rejected:     dict               # id -> human-readable reason


def select_components(db: ComponentDB, requirements: dict) -> dict:
    """category -> CategorySelection.  Deterministic: lightest feasible
    candidate wins (ties by id); a config-frozen id is validated and
    used instead.  Raises if a category has no feasible candidate or a
    frozen id is unknown/infeasible -- a design change that outgrows
    the hardware must fail loudly."""
    out = {}
    for cat in CATEGORIES:
        req = requirements[cat]
        rejected, feasible = {}, []
        for spec in db.candidates[cat]:
            reason = _rejection_reason(spec, req)
            if reason is None:
                feasible.append(spec)
            else:
                rejected[spec.id] = reason
        feasible.sort(key=lambda s: (s.mass_kg, s.id))

        frozen_id = db.policy.frozen.get(cat)
        if frozen_id is not None:
            by_id = {s.id: s for s in db.candidates[cat]}
            if frozen_id not in by_id:
                raise ValueError(f"{cat}: frozen id '{frozen_id}' not in database")
            if frozen_id in rejected:
                raise ValueError(
                    f"{cat}: frozen part '{frozen_id}' no longer meets the "
                    f"derived requirements ({rejected[frozen_id]}) -- the "
                    "design has outgrown the frozen hardware")
            selected, frozen = by_id[frozen_id], True
        else:
            if not feasible:
                raise ValueError(
                    f"{cat}: no candidate meets the derived requirements: "
                    + "; ".join(f"{i}: {r}" for i, r in rejected.items()))
            selected, frozen = feasible[0], False

        out[cat] = CategorySelection(
            category=cat, requirements=req, selected=selected, frozen=frozen,
            feasible_ids=[s.id for s in feasible], rejected=rejected,
        )
    return out


# ---------------------------------------------
#  Mass-allocation report
# ---------------------------------------------
def budget_report(
    selections:       dict,
    db:               ComponentDB,
    m_avionics_net_kg: float,   # avionics bay net budget (out/fuselage.yaml)
    m_esc_alloc_kg:    float,   # ESC layout allocation (out/fuselage.yaml)
    m_motor_fan_alloc_kg: float,  # motor+fan layout allocation
    servo_alloc_kg:    float,   # per-servo carve-out (config/fuselage.yaml)
    n_servos:          int,     # vanes + ailerons
    m_batt_sized_kg:   float,   # mass-closure battery mass (sizing result)
) -> dict:
    """Compare the frozen hardware against the weight-fraction
    allocations.  `within` flags are findings, not selection filters."""
    fc = selections["flight_controller"].selected
    m_supporting = sum(m for _, m in db.supporting)
    m_avionics = fc.mass_kg + m_supporting

    esc = selections["esc"].selected
    motor = selections["edf_motor"].selected
    prop = selections["propeller"].selected
    servo = selections["servo"].selected
    battery = selections["battery"].selected

    def entry(actual, alloc, note=""):
        e = {
            "actual_g": actual * 1e3,
            "alloc_g":  alloc * 1e3,
            "margin_g": (alloc - actual) * 1e3,
            "within":   actual <= alloc,
        }
        if note:
            e["note"] = note
        return e

    report = {
        "avionics_bay": entry(
            m_avionics, m_avionics_net_kg,
            "FC + supporting avionics vs the net avionics-bay budget"),
        "esc": entry(esc.mass_kg, m_esc_alloc_kg),
        "motor_fan": entry(
            motor.mass_kg + prop.mass_kg, m_motor_fan_alloc_kg,
            "EDF motor + fan/prop unit vs the motor_fan layout allocation"),
        "servo_each": entry(servo.mass_kg, servo_alloc_kg,
                            f"per servo, x{n_servos} installed"),
        "battery": entry(
            battery.mass_kg, m_batt_sized_kg,
            "COTS pack vs the mass-closure battery mass (capacity "
            "quantisation makes a small overshoot expected)"),
    }
    report["all_within"] = all(
        v["within"] for k, v in report.items() if isinstance(v, dict))
    return report


# ---------------------------------------------
#  Handoff writer
# ---------------------------------------------
def write_components_yaml(
    selections: dict,
    budgets:    dict,
    db:         ComponentDB,
    design_point: dict,   # MTOW/P_hover/P_design/pack echo for provenance
    path,
) -> None:
    """Write out/components.yaml -- the frozen COTS hardware list with
    mass and own-CG inertia, pinned by tests/test_design_outputs.py."""
    sel_block = {}
    for cat, sel in selections.items():
        s = sel.selected
        I1, I2, I3 = s.inertia_cg()
        sel_block[cat] = {
            "id":         s.id,
            "name":       s.name,
            "frozen":     bool(sel.frozen),
            **({"url": s.url} if s.url else {}),
            "mass_g":     round(s.mass_kg * 1e3, 2),
            "shape":      s.shape,
            "dims_mm":    [round(d * 1e3, 2) for d in s.dims_m],
            "I_cg_kgm2":  [round(inertia, 9) for inertia in (I1, I2, I3)],
            "ratings":    dict(s.ratings),
            "feasible_alternatives": [i for i in sel.feasible_ids if i != s.id],
            "rejected":   dict(sel.rejected),
        }
    reqs = {cat: {k: (round(v, 3) if isinstance(v, float) else v)
                  for k, v in sel.requirements.items()}
            for cat, sel in selections.items()}
    budgets_r = {
        k: ({kk: (round(vv, 2) if isinstance(vv, float) else vv)
             for kk, vv in v.items()} if isinstance(v, dict) else bool(v))
        for k, v in budgets.items()
    }
    data = {
        "design_point": design_point,
        "requirements": reqs,
        "selected":     sel_block,
        "supporting_avionics": [
            {"name": n, "mass_g": round(m * 1e3, 2)} for n, m in db.supporting],
        "budgets":      budgets_r,
        "all_within_allocations": bool(budgets["all_within"]),
    }
    with open(path, "w", encoding="utf-8") as f:
        f.write("# AUTO-GENERATED -- do not edit by hand.\n")
        f.write("# Source : src/conceptual_design/cots_selection.py\n")
        f.write("# Input  : config/components/ + converged design point\n")
        f.write("# Regen  : re-run notebooks/cots_selection.ipynb\n\n")
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)


# Rating columns shown per category in the notebook candidate tables.
RATING_COLS = {
    "flight_controller": ["imu_count", "px4"],
    "esc":               ["i_cont_a", "s_min", "s_max", "telemetry"],
    "edf_motor":         ["p_max_w", "s_min", "s_max", "kv"],
    "propeller":         ["d_rotor_mm", "p_max_w", "n_blades"],
    "servo":             ["stall_torque_gcm", "torque_at_v"],
    "battery":           ["capacity_mah", "s_cells", "c_rate_cont"],
}


def candidate_tables(db, selections):
    """Per-category candidate DataFrames with a SELECTED/ok/REJECTED verdict
    column, for notebook display."""
    import pandas as pd

    tables = {}
    for cat in CATEGORIES:
        sel = selections[cat]
        rows = []
        for spec in db.candidates[cat]:
            verdict = ("SELECTED" if spec.id == sel.selected.id
                       else "ok" if spec.id in sel.feasible_ids
                       else f"REJECTED: {sel.rejected[spec.id]}")
            row = {"id": spec.id, "mass_g": spec.mass_kg * 1e3}
            row.update({k: spec.ratings.get(k) for k in RATING_COLS[cat]})
            row["verdict"] = verdict
            rows.append(row)
        tables[cat] = pd.DataFrame(rows).set_index("id")
    return tables


def validate_components_yaml(path, selections, reqs, esc_rating_a,
                             pack_capacity_ah, D_rotor_m):
    """Self-checks on the written out/components.yaml handoff: the frozen
    ids round-trip, and every selection still clears its derived
    requirement (a failure means the design outgrew the freeze)."""
    frozen = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    assert set(frozen["selected"]) == set(CATEGORIES)
    for cat, sel in selections.items():
        assert frozen["selected"][cat]["id"] == sel.selected.id
    esc_sel = selections["esc"].selected
    assert esc_sel.ratings["i_cont_a"] >= esc_rating_a
    srv_sel = selections["servo"].selected
    assert srv_sel.ratings["stall_torque_gcm"] >= reqs["servo"]["stall_torque_min_gcm"]
    prop_sel = selections["propeller"].selected
    assert abs(prop_sel.ratings["d_rotor_mm"] * 1e-3 - D_rotor_m) < 1.5e-3
    assert selections["propeller"].frozen, (
        "propeller must stay pinned to the ADR-0003 amendment decision")
    bat_sel = selections["battery"].selected
    assert bat_sel.ratings["capacity_mah"] * 1e-3 >= pack_capacity_ah
