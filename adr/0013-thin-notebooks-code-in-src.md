# 13. Thin notebooks: all notebook code moves to src

Status: Accepted

## Context

ADR-0002 already requires all *physics* to live in `src/`, but by 2026-07
the fifteen notebooks had accumulated ~4,000 lines of Python anyway:

- an identical ~25-line path/matplotlib-style prelude in every notebook;
- an identical ~35-line "load every config, call `run_sizing_loop`"
  block in fourteen notebooks — the *"if you change the sizing API,
  update all fifteen call sites"* problem called out in CLAUDE.md;
- genuine physics that had leaked into notebooks in violation of
  ADR-0002: the entire jet-vane sizing model (NB3), the inertia
  primitives / assembly / battery-trim / BOM logic (NB9), the
  construction trade and sensitivity-sweep machinery (NB1);
- large matplotlib figure blocks and ASCII summary cards, including a
  ~70-line fuselage layout plot duplicated verbatim in NB6 and NB14 and
  a budget-margin chart duplicated in NB11 and NB15.

The notebooks are the rendered documentation (GitHub Pages); walls of
code bury the narrative they exist to tell.

## Decision

Notebooks contain **only** parameter setting, module calls, and short
interpretive prints. Everything else moves into `src/conceptual_design/`:

- `notebook.py` — `nb_setup()`: repo paths + house plot style + palette
  (the shared prelude as one call).
- `design_point.py` — `DesignInputs.from_config()` /
  `solve_design_point()`: the single call site for re-running the
  converged design point from `config/`; `load_handoff()` for `out/`
  reads. Sweeps override single inputs via `DesignInputs.solve(**kw)`.
- `control_vane_design.py` — NB3's vane physics (`VaneParams`,
  `design_vanes`, `mixing_check`, `write_control_vanes_yaml`).
- `mass_properties.py` — NB9's inertia primitives, assembly,
  battery-tray trim, BOM builder and handoff writer.
- `sizing_studies.py` — construction trade (ADR-0008), MTOW sensitivity
  sweeps, local elasticities; `vtol_power.climb_pw_terms()` exposes the
  P/W term breakdown NB1 used to re-derive inline.
- `plots/` — one module per notebook family; pure presentation, every
  function takes computed results and a save path and returns the
  Figure. The NB6/NB14 layout plot and NB11/NB15 budget chart exist
  once each here.
- `reports.py` and `design_summary.py` — the console tables and summary
  cards; NB15's margin table and standing-findings collection are logic
  and live in `design_summary.py`.

Design parameters that had been hardcoded in notebook cells (vane t/c,
hub ratio, servo safety factors, …) stay visible as notebook-set
parameters — now grouped in one commented `VaneParams`-style block per
notebook — so ADR-0002's "no magic numbers in src" holds.

**Exception:** NB8 (`vehicle_solid_model`) keeps its original inline
sizing block for now — it only executes in CI (CadQuery, Python 3.12),
so its conversion should land in a PR where CI can actually run it.

## Verification discipline

The refactor was landed with a byte-diff gate: after each notebook's
conversion the pipeline re-ran it and every `out/*.yaml` / `bom.csv`
handoff had to be byte-identical to the committed baseline (modulo CRLF
on Windows checkouts). The full 14-notebook pipeline, the design
regression + geometry pins, and ruff all pass unchanged. The same
discipline applies to future extractions: moving code between notebooks
and `src/` must not move a single number.

## Consequences

- Notebook code drops from ~4,050 to roughly a third of that; rendered
  pages read as narrative with parameter blocks and results.
- A sizing-API change now touches `design_point.py` (plus NB8's block)
  instead of fifteen notebooks.
- Physics extracted from NB3/NB9 becomes importable and unit-testable.
- The notebooks import more names per cell; the coupling is explicit
  (`from conceptual_design.plots import …`) rather than copy-pasted.
