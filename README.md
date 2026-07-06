# Electric V-BAT-Like Tail-Sitter (EDF) – Conceptual Design Study

[![CI](https://github.com/onurtuncer/vbat-uav-notebooks/actions/workflows/ci.yml/badge.svg)](https://github.com/onurtuncer/vbat-uav-notebooks/actions/workflows/ci.yml)
[![Design Pipeline](https://github.com/onurtuncer/vbat-uav-notebooks/actions/workflows/design-pipeline.yml/badge.svg)](https://github.com/onurtuncer/vbat-uav-notebooks/actions/workflows/design-pipeline.yml)
[![Release](https://img.shields.io/github/v/tag/onurtuncer/vbat-uav-notebooks?label=release)](https://github.com/onurtuncer/vbat-uav-notebooks/releases)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue.svg)](pyproject.toml)

**📖 Documentation:** rendered design notebooks are published at
**<https://onurtuncer.github.io/vbat-uav-notebooks/>**

This repository contains a **first-principles conceptual sizing study** for a **small electric tail-sitter VTOL UAV** inspired by the *very early electric ancestors* of the V-BAT concept.

The focus is **not** on a production UAV, but on:
- architectural feasibility,
- mass / power / energy closure,
- and understanding design trade-offs at the **pre-SBIR / technology-demonstrator level**.

The code is intentionally simple, transparent, and extensible.

## Theory & Models

The low-order physics models used in this study are documented in
[`docs/`](docs/).

See:
- Hover power derivation
- Cruise power estimation
- Battery sizing assumptions

---

## 1. Problem Definition

### Objective
Design a **small electric ducted-fan tail-sitter UAV** capable of:

- **True VTOL** (vertical takeoff and landing)
- **Short forward flight** after transition
- Carrying **0.5 kg of payload**
- Using **electric propulsion only**
- Using **first-order physics models** suitable for early design

This mirrors the *earliest feasibility phase* of tail-sitter concepts before scaling to fuel engines, endurance optimization, or operational payloads.

---

### Design Philosophy
This study intentionally:
- avoids CFD or high-fidelity tools,
- avoids vendor-specific components,
- emphasizes **physics-based intuition**,
- and keeps all assumptions explicit.

The goal is to answer:

> *“Is this architecture feasible at this scale, and where are the dominant penalties?”*

---

## 2. Vehicle Concept (High Level)

- **Configuration:** Tail-sitter VTOL
- **Propulsor:** Electric ducted fan (EDF)
- **Wing:** Fixed wing for forward flight
- **Takeoff / Landing:** Vertical, tail-down
- **Primary regime:** Hover → transition → short cruise

This corresponds to the **electric technology demonstrators** typically built *before* SBIR Phase I funding.

---

## 3. Assumed Mission Profile

The baseline mission used for sizing is:

| Segment | Description |
|------|-------------|
| Hover | 120 s total (takeoff, landing, margin) |
| Cruise | 1680 s forward flight (~33.6 km at cruise speed) |
| Cruise speed | 20 m/s (~72 km/h) |
| Energy reserve | 20% |

All mission parameters are **editable** in [`config/mission.yaml`](config/mission.yaml).

---

## 4. Payload & Mass Target

- **Payload:** 0.5 kg  
  (sensor + avionics representative of early demonstrators)

Initial mass breakdown assumptions:
- avionics
- structure
- EDF + motor + ESC
- battery
- miscellaneous margin

Mass closure is solved iteratively.

---

## 5. Modeling Approach

### Hover
Hover power is estimated using **momentum (actuator disk) theory**:

$$
P_{ideal} = \frac{T^{3/2}}{\sqrt{2 \rho A}}
$$

with an overall hover efficiency applied to account for:
- duct losses,
- motor/ESC losses,
- non-ideal inflow.

A minimum **thrust-to-weight ratio** is enforced for control margin.

---

### Cruise
Cruise power is estimated using a simple **L/D model**:

$$
P = \frac{W}{(L/D)} \cdot V
$$

This is appropriate for:
- early conceptual sizing,
- rapid trade studies,
- understanding sensitivity to aerodynamic quality.

---

### Battery
Battery mass is computed from:
- total mission energy,
- pack-level specific energy,
- usable fraction (reserve).

No thermal or current limits are enforced at this stage.

---

## 6. Repository Structure

```
config/                  Design inputs (mission, aero, battery, rotor, ... as YAML)
src/conceptual_design/   All physics models (sizing, power, mass closure)
src/conceptual_design/cad/  CadQuery solid-model generators
notebooks/               Design notebooks (import from src/, write to out/)
out/                     Generated design outputs (YAML, STEP/STL, plots)
docs/                    Theory notes (hover, cruise, battery models)
tests/                   Unit tests for the physics modules (pytest)
cfd/                     OpenFOAM cases (vehicle, prop, vanes) + post-processing
printprep/               3D-print preparation scripts
```

### Design rule
- **All physics lives in `.py` files**
- **The notebook only imports and experiments**

This mirrors best practice in research and aerospace R&D codebases.

---

## 7. Requirements

### Python
- Python **3.10+**

### Python packages
Install the full stack (numpy, scipy, matplotlib, pandas, pyyaml, jupyter,
cadquery) with:

```bash
pip install -r requirements.txt
```

or install the package itself in editable mode:

```bash
pip install -e .
```

CadQuery is only needed for the solid-model notebook
(`vehicle_solid_model.ipynb`); everything else runs on the plain
scientific stack.

### OpenFOAM (optional)
The `Allrun.*` scripts under [`cfd/`](cfd/) run external-aero RANS
analyses of the generated geometry. They require a sourced
**OpenFOAM.com (v2306+)** environment and are independent of the Python
toolchain. See [`cfd/README.md`](cfd/README.md).

## 8. How to Run

Open a terminal in the repository root and launch Jupyter:

```bash
jupyter lab
```

Run the notebooks top-to-bottom **in this order** (each one writes YAML
handoff files to `out/` that the next one reads):

1. `notebooks/vbat_conceptual_design.ipynb` — mission sizing & mass closure
2. `notebooks/wing_design.ipynb` — airfoil selection & wing design
3. `notebooks/control_vane_design.ipynb` — control vane sizing
4. `notebooks/fuselage_design.ipynb` — fuselage layout & drag trade (reads the vane results)
5. `notebooks/vehicle_solid_model.ipynb` — full-vehicle CAD (STEP/STL export)

Adjust parameters in `config/*.yaml` and re-run to explore trade-offs.

### Tests

```bash
pip install -e ".[dev]"
pytest
```

### Versioning & releases

The package version derives from **git tags** via `setuptools-scm`
(`v0.1.0` → `0.1.0`; commits after a tag get a `.devN` suffix — never
edit a version number by hand). Tagging a release freezes the design:

```bash
git tag v0.2.0 && git push --tags
```

CI then re-runs the whole design pipeline and attaches the design
snapshot (design-point YAMLs, STEP/STL geometry, rendered notebooks) to
the [GitHub Release](https://github.com/onurtuncer/vbat-uav-notebooks/releases).
Suggested convention: bump **minor** for a new design point or analysis
capability, **patch** for corrections that do not move the design.

## 9. Intended Extensions

This framework is designed to grow. Natural next steps include:

- wing sizing & stall constraints
- EDF continuous vs burst power limits
- disk loading vs noise trade-offs
- thermal limits (motor & ESC)
- parametric sweeps and Pareto fronts
- hybrid-electric or fuel-based scaling

Each extension should live in new modules

## 10. Scope & Limitations

This study:

❌ is not a flight-ready design

❌ ignores structural stress analysis

❌ ignores control law implementation

❌ ignores certification or safety

It is appropriate for:

- concept feasibility
- academic exploration
- architecture comparison

## 11. Motivation

Tail-sitter VTOL aircraft occupy a unique design space where:

- hover power dominates,
- transition dynamics are critical,
- and endurance scaling is non-trivial.

This repository exists to make those trade-offs explicit and quantitative.

## 12. License

Please see LICENSE file. Open for research and educational use.
(No warranty, no fitness for flight.)

## Author Notes

This project intentionally mirrors how early electric V-BAT-like demonstrators would have been sized before fuel engines, autonomy stacks, or other payloads were introduced.

## 👤 Author

**Prof.Dr. Onur Tuncer**  
Aerospace Engineer, Researcher & C++ Systems Developer  
Email: **onur.tuncer@itu.edu.tr**

<p align="left">
  <img src="assets/itu_logo.png" width="180" alt="Istanbul Technical University"/>
</p>
