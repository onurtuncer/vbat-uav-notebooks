# CAD Assets – VBAT Conceptual Design

This directory contains all CAD geometry related to the electric
tail-sitter (VBAT-style) UAV conceptual design.

Geometry is generated programmatically using **CadQuery**
(built on OpenCascade / OCCT).

---

## Philosophy

All primary geometry should be:

- Parametric
- Reproducible
- Script-generated when possible
- Version-controlled at the source level

The CAD models are treated as **derived artifacts** from the Python
conceptual design codebase.

---

## Directory Structure

This package contains the geometry **generators**:

cad/
├─ wing_profile.py      # NACA sections (math from airfoil_selection.py)
├─ fuselage_body.py     # body of revolution (meridian from fuselage_design.py)
├─ vehicle_assembly.py  # full vehicle: fuselage, wing, duct, vanes, legs
└─ README.md

Generated artifacts are **derived outputs** and live under `out/` (gitignored):

out/cad/
├─ step/ # STEP solids for CAD exchange (assembly + fused)
└─ stl/  # Mesh exports for visualization or 3D printing

## Axis Convention (Aetherion-compatible)

Body frame **FRD**: x forward (out the nose), y right, z down.
Origin at the nose tip; the EDF exhaust points in -x.
World frame is NED.

## Units

- Public function arguments and all `out/*.yaml` handoffs: **meters (m)**
- OCCT geometry and STEP/STL exports: **millimetres (mm)** (CAD convention);
  scaling happens inside the modules.

---

## Generation Workflow

CAD geometry is generated from Python using:


Typical workflow:

1. Adjust parameters in `config/*.yaml`.
2. Re-run the upstream notebooks (sizing -> airfoil -> vanes -> fuselage).
3. Run `notebooks/vehicle_solid_model.ipynb` to regenerate
   `out/cad/step/` and `out/cad/stl/`.

Example (full vehicle from the YAML handoffs):

```python
import yaml
from conceptual_design.cad.vehicle_assembly import build_vehicle, export_vehicle

fus   = yaml.safe_load(open("out/fuselage.yaml"))
vanes = yaml.safe_load(open("out/control_vanes.yaml"))
asm, fused = build_vehicle(fus, vanes, b_wing_m=1.002,
                           chord_wing_m=0.167, wing_designation="NACA 2412")
export_vehicle(asm, fused, "out/cad")
```

## Planned Geometry Modules

- Wing (lofted root–tip sections)
- Spar cutouts
- Rib generation
- Fuselage parametric body
- Duct / fan housing
- Mounting structures

## Long-Term Goal

CAD is tightly coupled to the conceptual design:

- Mission → mass → wing loading → chord/span → geometry

Geometry should be derivable from YAML configuration

This ensures:

-Design traceability
-Repeatability
-Parametric trade studies