# CFD Cases (OpenFOAM)

RANS (simpleFoam, k-omega SST) cases characterizing the V-BAT tail-sitter,
using geometry and reference quantities produced by the notebooks
(`out/cad/`, `out/fuselage.yaml`, `out/control_vanes.yaml`,
`config/mission.yaml`). Tested against OpenFOAM.com v2306+.

| Directory      | Purpose |
|----------------|---------|
| `vehicle/`     | Full-vehicle external aero: mesh from `out/cad/stl/vbat_fused.stl`, single-AoA runs, and AoA polar sweeps |
| `prop/`        | Ducted-fan propulsion map via actuator disk (parametric duct + centerbody geometry, thrust x velocity sweep) |
| `vanes/`       | Control-vane characterization: flat-plate vane in the duct jet, deflection sweep |
| `postprocess/` | Exporters from `postProcessing/` force histories to DAVE-ML (`foam2dml.py` for the vehicle polar, `foam2dml_prop.py` for prop/vane sweeps) |

## Run order

```bash
# Full-vehicle aero polar (requires repo STL/STEP under out/cad/)
cd vehicle
./Allrun.mesh                      # geometry -> snappyHexMesh -> checkMesh
./Allrun.case -a 0 -V 20 -n 8 -s   # single AoA, or:
./Allrun.polar -n 8 -V 20 -- -4 -2 0 2 4 6 8 10 12
python3 ../postprocess/foam2dml.py --polar-dir polar --out vbat_aero_cfd.dml --plot

# Propulsion map (duct/hub dims read from out/fuselage.yaml via make_geom.py)
cd prop
./Allrun.prop -n 8
python3 ../postprocess/foam2dml_prop.py --prop-dir sweep

# Vane sweep
cd vanes
./Allrun.vanes -n 8
python3 ../postprocess/foam2dml_prop.py --vane-dir sweep
```

Run artifacts (meshes, `log/`, `processor*/`, `postProcessing/`, sweep
directories) are generated in place and gitignored; only the Allrun
scripts, `system/` dictionaries, and Python tooling are tracked.
