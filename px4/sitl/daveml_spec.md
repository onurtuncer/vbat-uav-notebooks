# DAVE-ML model database spec (draft, ADR-0011)

Contract between this repo's CFD results and the Aetherion 6-DoF plant
used for PX4 SITL. Producer: a generator script in this repo (follow-up
work item) writing `out/daveml/*.dml` from CFD sweep results + existing
`out/` handoffs. Consumer: Aetherion's `Serialization/DAVEML` readers
(`DAVEMLAeroModel` via the generic `evaluateRaw` map interface,
`LoadInertiaFromDAVEML`, a prop-style wrapper).

Conformance: **DAVE-ML 2.0 (AIAA S-119)**, gridded tables + MathML,
embedded check-cases mandatory (Aetherion's F-16 files are the style
reference). Units: **SI native** (m, m/s, N, N·m, deg for angles,
matching the F-16 file convention of degrees for aero angles).
NOTE: Aetherion's `getValueSI` assumes imperial native units (slug, ft);
verify its readers pass SI-native units through unchanged — flagged as
an Aetherion-side check.

## Frames, references, provenance

- Body **FRD**, x out the nose (project-wide convention). Moments about
  the CG; forces/moments and their coefficient forms use the same signs
  as the F-16 reference set (`cx` +fwd, `cz` +down, `cm` +nose-up, …).
- Reference quantities pinned to `cfd/vehicle/Allrun.case` (and the
  regression pins in `tests/test_design_outputs.py`):
  `S_ref = 0.18925 m²`, `c_ref = 0.17760 m` (MAC = chord, rectangular
  wing), `b_ref = 1.0655 m`, moment centre `(-0.23841, 0, 0)` = CG.
- Every file carries the standard AUTO-GENERATED provenance header
  (source case, mesh/solver settings hash, generation date) and is
  re-generated — never hand-edited — when the design point moves.

## The hover-normalisation split (structural decision)

At hover `q_∞ → 0`, so `C = F/(q_∞ S)` is singular exactly where a
tailsitter lives. The database is therefore split:

| File | Form | Valid regime |
|---|---|---|
| `vbat_aero.dml` | coefficients (power-off airframe) | V ≥ ~5 m/s |
| `vbat_prop.dml` | **dimensional** N / N·m vs throttle state | all speeds |
| `vbat_vanes.dml` | **dimensional** N / N·m vs deflection + jet state | all speeds |
| `vbat_inertia.dml` | constants | — |

Total loads = airframe coefficients (faded in with airspeed) +
prop/duct + vane increments. Aetherion composes the superposition; the
fade law is defined here (breakpointed on airspeed), not hardcoded there.

## vbat_aero.dml — power-off airframe

Inputs: `alpha` [deg], `beta` [deg], `vt` [m/s], `p`,`q`,`r` [rad/s],
`da_l`,`da_r` [deg] (aileron deflections).
Outputs: `cx cy cz cl cm cn`.

Draft breakpoint grid (transition corridor gets the density):

- `alpha`: −180…180; 2.5° steps in [−10, +30], 5° in [30, 90], 15°
  beyond (tail-sitter passes through 90° — the post-stall band is the
  point of doing this at all)
- `beta`: 0, ±5, ±10, ±20 (symmetry exploited in generation)
- `vt`: 5, 10, 16, 20, 26 m/s (Re sensitivity check decides whether
  the vt axis survives or collapses to one Re)
- aileron: 0, ±10, ±20° (increments; rate derivatives `p,q,r` are NOT
  CFD-derived initially — first fill from analytic/DATCOM estimates,
  flagged `estimated` in the varDef description, replaced if forced
  oscillation or system-ID data arrives)

## vbat_prop.dml — EDF + duct, power-on

Inputs: `throttle` [0–1] (or `rpm`), `vt` [m/s], `alpha` [deg].
Outputs: `fx_N fy_N fz_N mx_Nm my_Nm mz_Nm` **plus `q_jet_Pa`** (feeds
the vane model — the F-16 prop file has no such coupling; this one must).
Must capture duct normal force and pitching moment at incidence
(dominant tailsitter transition effect) — this is the actuator-disk CFD
sweep, power-on minus power-off.

Draft grid: throttle {idle, cruise ≈ 0.15, hover 0.61, max 1.0} ×
vt {0, 5, 10, 16, 20, 26} × alpha {0, 15, 30, 60, 90}.
Static check-cases: hover 30.3 N at throttle 0.61; `T_max_N` 50 at 1.0
(config/rotor.yaml); `q_jet` 780.3 Pa at hover (out/control_vanes.yaml).

## vbat_vanes.dml — jet vane increments

Inputs: `dv_t dv_b dv_l dv_r` [deg, ±20 hard stop], `q_jet_Pa` (from the
prop model), `vt`, `alpha`.
Outputs: incremental `fx_N … mz_Nm` about the CG.

Draft grid: deflection −20…+20 in 5° steps (one vane swept, others
zero; cross-coupling spot-checked), q_jet {cruise, 0.5·hover, hover,
max}. First-issue tables may come from the analytic model in
`src/conceptual_design` (provenance-flagged), replaced by CFD
(resolved-vane or momentum-source runs) sweep by sweep — the .dml
interface stays fixed while fidelity improves underneath.
Check-case: 10° single vane at hover q_jet → 0.719 N
(out/control_vanes.yaml) for the analytic issue; re-pinned when CFD
replaces it.

## vbat_inertia.dml

Straight transcription of `out/mass_properties.yaml`: mass 2.37626 kg,
Ixx 0.022453, Iyy 0.041776, Izz 0.054148 kg·m² (products zero), CG
(-0.23841, 0, 0). Battery-tray CG travel (±0.025 m) exposed as the one
input variable, mirroring the F-16 file's `CG_PCT_MAC` pattern.

## Validation gates (Aetherion side)

1. S-119 embedded check-cases pass in Aetherion's Catch2 suite.
2. Static hover trim: thrust = weight at throttle ≈ 0.61, vanes ≈ 0.
3. Cruise trim at 20 m/s reproduces CL ≈ 0.50 and L/D consistent with
   `out/airfoil.yaml` (13.2 wing-only, less airframe drag).
4. Vane authority at hover reproduces the `ddot_*` design values of
   `out/control_vanes.yaml` before any PX4 gain tuning is attempted.
