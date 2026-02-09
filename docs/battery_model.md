# Battery & Energy Model

Battery mass is computed from total mission energy.

---

## Mission Energy

\[
E_{mission} = E_{hover} + E_{cruise}
\]

A reserve factor is applied:

\[
E_{req} = f_{res} \cdot E_{mission}
\]

---

## Battery Mass

\[
m_{batt} = \frac{E_{req}}
{\epsilon_{pack} \cdot f_{usable}}
\]

where:
- \( \epsilon_{pack} \) is pack-level specific energy
- \( f_{usable} \) is usable depth of discharge

---

## Notes
- Pack-level energy includes casing and wiring
- C-rate and thermal limits are ignored
