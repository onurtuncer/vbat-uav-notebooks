# Architecture Decision Records

This folder holds Architecture Decision Records (ADRs) — short documents
capturing an architecturally significant decision, its context, and its
consequences. New ADRs are added when an architectural decision is made;
existing ADRs are never rewritten (a superseding decision gets a new ADR
that marks the old one `Superseded`).

Format: one file per decision, `NNNN-kebab-title.md`, each with
**Status / Context / Decision / Consequences**.

| ADR | Title | Status |
|-----|-------|--------|
| [0001](0001-one-way-design-pipeline.md) | One-way design pipeline (config → src → notebooks → out → cfd) | Accepted |
| [0002](0002-all-physics-in-src-no-magic-numbers.md) | All physics in `src/`; no magic numbers in Python | Accepted |
| [0003](0003-cots-edf-derived-disk-loading.md) | COTS 195 mm EDF; disk loading derived, not configured | Accepted |
| [0004](0004-dual-roll-control-vanes-and-ailerons.md) | Dual roll control: jet vanes (hover) + ailerons (cruise) | Accepted |
| [0005](0005-mass-fraction-carve-out-with-traceability.md) | Mass-fraction carve-out with traceability and a guard | Accepted |
| [0006](0006-tag-driven-versioning-and-ci-tiers.md) | Tag-driven versioning and tiered CI | Accepted |
| [0007](0007-vibration-isolation.md) | Soft-mount the FC/IMU and payload against EDF imbalance | Accepted |
