# Electric V-BAT-Like Tail-Sitter (EDF) – Conceptual Design Study

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
| Cruise | 480 s forward flight |
| Cruise speed | 20 m/s (~72 km/h) |
| Energy reserve | 20% |

All mission parameters are **editable** in the notebook.

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


### Design rule
- **All physics lives in `.py` files**
- **The notebook only imports and experiments**

This mirrors best practice in research and aerospace R&D codebases.

---

## 7. Requirements

### Python
- Python **3.9+** recommended

### Python packages
Minimal scientific stack:

```txt
numpy
matplotlib
```

Optional but recommended:
```txt
jupyterlab
```

Install everything with:
```txt
pip install numpy matplotlib jupyterlab
```

## 8. How to Run

Open a terminal in vbat_uav_notebooks/

Launch Jupyter:
```txt
jupyter lab
```

Open notebook_vbat_demo.ipynb

Run cells top-to-bottom

Adjust parameters in the notebook and re-run to explore trade-offs.

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