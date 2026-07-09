# 6. Tag-driven versioning and tiered CI

Status: Accepted

## Context

The design point drifts as models and config evolve, and CAD/CFD outputs are
expensive to regenerate (CadQuery needs Python 3.12; converged CFD needs real
hardware). We need versions that cannot be hand-edited out of sync with the
code, and a CI strategy that validates the whole pipeline without ever
attempting production CFD on shared runners.

## Decision

- **Version comes from git tags via `setuptools-scm`** — never edited by
  hand. A frozen design snapshot is a `v*` tag.
- **Three CI tiers:**
  - `ci.yml` — ruff, pytest (3.10/3.12/3.14), ShellCheck on `cfd/**/Allrun*`.
  - `design-pipeline.yml` — executes all eight notebooks, runs
    design-regression + geometry tests, uploads `out/`; on `main` also runs
    the coarse OpenFOAM smoke run and deploys the GitHub Pages report.
  - `release.yml` — a `v*` tag re-runs the pipeline and attaches the frozen
    snapshot to a GitHub Release.
- **Never run full/converged CFD in CI.** The coarse-mesh, 60-iteration
  smoke run validates case *setup* only and is the ceiling.
- Regression pins (`tests/test_design_outputs.py`) lock the design point and
  cross-check `cfd/vehicle/Allrun.case`; an intentional design change updates
  the pins and Allrun.case in the same commit.

## Consequences

- A release is reproducible: the tag pins the code, and the pipeline
  regenerates `out/` from `config/` to prove it still converges.
- CadQuery-dependent outputs (`out/cad/`) are validated against a fresh CI
  regeneration; committed copies may lag locally (the 3.14 venv can't build
  them) but CI is the source of truth.
- Each version tag is accompanied by a `CHANGELOG.md` entry (project policy).
- CI cost is bounded and predictable; expensive converged CFD stays off-CI on
  the user's own hardware.
