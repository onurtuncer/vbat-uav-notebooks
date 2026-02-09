# Hover Power Model

Hover power is estimated using **actuator disk (momentum) theory**.

---

## Ideal Hover Power

For thrust \( T \) and disk area \( A \):

\[
P_{ideal} = \frac{T^{3/2}}{\sqrt{2 \rho A}}
\]

where:
- \( \rho \) is air density
- \( A = \pi (D/2)^2 \)

---

## Electrical Power

Non-ideal effects are lumped into an efficiency factor:

\[
P_{elec} = \frac{P_{ideal}}{\eta_{hover}}
\]

The efficiency term includes:
- duct losses
- motor efficiency
- ESC losses
- non-uniform inflow

---

## Thrust Margin

A minimum thrust-to-weight ratio is enforced:

\[
T = (T/W)_{min} \cdot W
\]

Typical values:
- \( T/W = 1.3 \)–\( 1.6 \)

---

## Limitations
- No ground effect modeling
- No swirl losses modeled
- No transient inflow effects
