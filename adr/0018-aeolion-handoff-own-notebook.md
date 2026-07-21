# ADR-0018: Aeolion handoff as its own pipeline notebook stage

Date: 2026-07-22
Status: Accepted

## Context

`build_aeolion_geometry`/`export_aeolion_geometry` (ADR-0016) were
called from a tail cell of `vehicle_solid_model` (NB8) because that's
where the CAD assembly and the parametric prop rotor were already
being built, and the ADR-0016 rationale explicitly framed the JSON as
sitting "beside the existing assembly and fused STEP files."

Since then the contract has grown: six schema revisions in three days
(1.0.0 → 1.7.0, ADR-0016/0017), a real design decision of its own
(the Clark Y blade section, ADR-0017), and a numerically-derived
physical fact (rotation sense) — enough surface area that it no longer
reads as an afterthought at the end of the CAD notebook.

It's also, on inspection, not actually coupled to CAD at all:
`build_aeolion_geometry`'s every input — wing span/chord from the
sizing result, `out/airfoil.yaml`, `out/control_vanes.yaml`,
`out/aileron.yaml`, `out/vibration.yaml`, `out/fuselage.yaml`,
`config/prop_geometry.yaml`, `config/airfoils/clarky.dat`,
`config/aeolion.yaml` — comes from `config/` or an upstream `out/*.yaml`
handoff, never from the CAD solid or its STEP/STL exports. Both
`aeolion_handoff.py` and `prop_geometry.py` are already, deliberately,
CadQuery-free (their own docstrings say so, "so the handoff and its
tests run in environments without OCCT").

## Decision

Split the Aeolion export into its own notebook, **`aeolion_handoff`**,
inserted as **NB9** (immediately after `vehicle_solid_model`,
renumbering `mass_properties` → NB10 through `design_summary` → NB16).
It uses the same thin-notebook pattern as every post-ADR-0013 notebook
(`design_point.solve_design_point` as the sizing-loop call site,
`design_point.load_handoff` for the upstream `out/*.yaml` reads) rather
than NB8's special inline `run_sizing_loop` block.

`vehicle_solid_model` keeps building the parametric prop rotor solid
(`PropGeometry`/`ClarkYSection` are still needed there, for the CAD
loft) and exporting STEP/STL; it just drops the trailing Aeolion-export
cell. `out/cad/aeolion_geometry.json` keeps its existing path — the
"beside the STEP files" convention is about file layout, not about
which notebook writes it.

Because NB9 has no CadQuery dependency, it can regenerate
`out/cad/aeolion_geometry.json` **even when NB8 is skipped** (no OCP
wheel, local Python 3.14 venv) — a genuine practical improvement, the
same property `wiring_diagram` (NB11) already has.

## Consequences

- Pipeline is now sixteen notebooks. `CLAUDE.md`,
  `.github/workflows/design-pipeline.yml`'s `NOTEBOOK_ORDER`,
  `.github/workflows/release.yml`'s minimal notebook list, and
  `.claude/skills/run-vbat-uav-notebooks/driver.py`'s `NOTEBOOKS` list
  all updated in the same commit.
- `out/cad/aeolion_geometry.json`'s content is unaffected by the
  split — same inputs, same code, same `design_id` — verified by
  running the full pipeline and diffing against the pre-split file.
- Future Aeolion-side schema/BEMT work (further fields, another
  section decision, etc.) has a natural home that doesn't require
  touching the CAD notebook or a CadQuery-capable environment to
  iterate on.
