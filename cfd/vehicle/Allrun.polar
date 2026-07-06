#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# AoA polar sweep for the V-BAT case (run AFTER Allrun.mesh).
#
#   ./Allrun.polar [-n nProcs] [-V speed] -- AOA1 AOA2 ...
#   e.g.  ./Allrun.polar -n 8 -V 20 -- -4 -2 0 2 4 6 8 10 12
#
# For each AoA: clones the meshed base case into polar/aoa_<a>/ (sharing the
# mesh via symlink), runs Allrun.case -s, leaves postProcessing/ in place.
# Then export DAVE-ML with:  ../postprocess/foam2dml.py --polar-dir polar
# ---------------------------------------------------------------------------
set -euo pipefail
cd "$(dirname "$0")"

NPROCS=8; VMAG=20
while getopts "n:V:" opt; do
    case $opt in
        n) NPROCS=$OPTARG ;;
        V) VMAG=$OPTARG ;;
        *) exit 1 ;;
    esac
done
shift $((OPTIND-1))
[[ "${1:-}" == "--" ]] && shift
[[ $# -gt 0 ]] || { echo "usage: $0 [-n N] [-V V] -- AOA1 AOA2 ..."; exit 1; }

[[ -d constant/polyMesh ]] || { echo "ERROR: mesh missing. Run ./Allrun.mesh."; exit 1; }

for AOA in "$@"; do
    TAG=$(printf "aoa_%+06.1f" "$AOA")
    DIR="polar/$TAG"
    echo "=============================================================="
    echo ">> $TAG  (V=$VMAG m/s, $NPROCS procs)"
    mkdir -p "$DIR"
    # share the (large, identical) mesh; copy everything else
    cp -r system "$DIR/"
    mkdir -p "$DIR/constant"
    cp constant/transportProperties "$DIR/constant/" 2>/dev/null || true
    cp constant/turbulenceProperties "$DIR/constant/" 2>/dev/null || true
    ln -sfn "$(pwd)/constant/polyMesh"   "$DIR/constant/polyMesh"
    ln -sfn "$(pwd)/constant/triSurface" "$DIR/constant/triSurface"
    cp Allrun.case "$DIR/"
    ( cd "$DIR" && ./Allrun.case -a "$AOA" -V "$VMAG" -n "$NPROCS" -s )
    # record run metadata for the exporter
    cat > "$DIR/run_meta.yaml" << EOF
aoa_deg: $AOA
V_mps: $VMAG
EOF
done

echo
echo ">> Sweep complete. Export DAVE-ML:"
echo "     python3 ../postprocess/foam2dml.py --polar-dir polar --out vbat_aero_cfd.dml --plot"
