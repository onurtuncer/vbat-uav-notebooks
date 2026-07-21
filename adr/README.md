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
| [0008](0008-segmented-fdm-construction.md) | Segmented-FDM construction as the baseline airframe | Amended by 0010 |
| [0009](0009-thermal-paths-as-structure.md) | Thermal paths as structure (ESC cold-plate + vented battery bay) | Accepted |
| [0010](0010-semi-monocoque-clamshell.md) | Semi-monocoque clamshell fuselage (external review) | Accepted |
| [0011](0011-px4-flight-validation-before-custom-fc.md) | Fly on PX4 + COTS Pixhawk before a custom flight controller | Accepted |
| [0012](0012-post-freeze-as-selected-resolve.md) | Post-freeze as-selected re-solve as new downstream notebooks | Accepted |
| [0013](0013-thin-notebooks-code-in-src.md) | Thin notebooks: physics/plots/reports code lives in src | Accepted |
| [0014](0014-battery-pack-transient-thermal-check.md) | Battery pack mission-transient thermal check | Accepted |
| [0015](0015-naca-4412-airfoil-reselection.md) | Airfoil re-selection: NACA 2412 → NACA 4412 | Accepted |
| [0016](0016-aeolion-geometry-handoff.md) | Versioned JSON plus STEP for the Aeolion handoff | Accepted |
| [0017](0017-prop-blade-clark-y-section-and-rotation-sense.md) | Propeller blade section (Clark Y) and rotation sense | Accepted |
