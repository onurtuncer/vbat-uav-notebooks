# Cruise Power Model

Cruise power is estimated using a **lift-to-drag (L/D) formulation**.

---

## Drag Estimate

Assuming steady level flight:

\[
D = \frac{W}{L/D}
\]

---

## Power Required

Aerodynamic power:

\[
P_{aero} = D \cdot V
\]

Electrical power:

\[
P_{elec} = \frac{P_{aero}}{\eta_{cruise}}
\]

---

## Applicability
This model is suitable for:
- early sizing
- architecture comparison
- parametric studies

It is not suitable for:
- envelope prediction
- detailed performance analysis
