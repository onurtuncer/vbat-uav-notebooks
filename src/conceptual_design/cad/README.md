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

cad/
├─ step/ # STEP solids for CAD exchange
├─ stl/ # Mesh exports for visualization or 3D printing
├─ dxf/ # 2D profiles (laser / CNC)
├─ drawings/ # Rendered drawings (PDF, PNG)
└─ README.md

## Units

All geometry is defined in:

- **meters (m)**

Unless otherwise stated.

---

## Generation Workflow

CAD geometry is generated from Python using:


Typical workflow:

1. Adjust parameters in notebook or YAML.
2. Run CAD generation script.
3. Export to:
   - `cad/step/`
   - `cad/stl/`
   - etc.

Example:

```python
from conceptual_design.cad.wing_profile import extrude_wing, export_step

wing = extrude_wing(span=0.6, chord=0.18, thickness=0.12)
export_step(wing, "../cad/step/wing.step")
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