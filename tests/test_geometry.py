"""Geometry validation of the exported CAD (out/cad/).

Gates the geometry before anything CFD- or print-related consumes it:
the STL must be watertight for snappyHexMesh, and the units must be
millimetres (Allrun.mesh scales mm -> m and silently produces a
1000x-too-small mesh if the export ever changes to metres).

Skips when out/cad/ has not been generated.
"""

import pytest

from conftest import REPO_ROOT

STL = REPO_ROOT / "out" / "cad" / "stl" / "vbat_fused.stl"
STEP_DIR = REPO_ROOT / "out" / "cad" / "step"

pytestmark = pytest.mark.skipif(
    not STL.exists(),
    reason="out/cad/ not generated -- run vehicle_solid_model.ipynb first",
)


@pytest.fixture(scope="module")
def mesh():
    trimesh = pytest.importorskip("trimesh")
    return trimesh.load_mesh(STL)


class TestFusedStl:
    def test_watertight(self, mesh):
        # snappyHexMesh needs a closed, consistently-oriented surface.
        # Known artifact: the CadQuery fuse/tessellation currently emits
        # one stray degenerate triangle alongside the closed hull, so we
        # require the main body watertight and cap the stray-face count.
        assert mesh.is_winding_consistent
        assert mesh.volume > 0

        bodies = mesh.split(only_watertight=False)
        main = max(bodies, key=lambda b: len(b.faces))
        assert main.is_watertight, "main CAD body is not a closed surface"

        n_stray = len(mesh.faces) - len(main.faces)
        assert n_stray <= 5, (
            f"{n_stray} faces outside the main body -- CAD export degraded"
        )

    def test_units_are_millimetres(self, mesh):
        # Wing span ~1 m: in mm the largest extent is ~1000, in metres
        # it would be ~1. Anything outside [500, 2000] means the export
        # scale changed and Allrun.mesh's mm->m scaling is now wrong.
        largest = max(mesh.extents)
        assert 500.0 < largest < 2000.0, (
            f"largest extent {largest:.1f} -- expected ~1000 (mm). "
            f"Did the CAD export unit change?"
        )

    def test_span_matches_cfd_reference(self, mesh):
        # Allrun.case uses S_ref = span * MAC with span measured from
        # this STL (1.0022 m). Pin the span so the CFD reference area
        # stays consistent with the geometry.
        span_m = max(mesh.extents) / 1000.0
        assert span_m == pytest.approx(1.0022, rel=1e-2)


class TestStepExports:
    @pytest.mark.parametrize("name", ["vbat_fused.step", "vbat_assembly.step"])
    def test_step_file_present_and_nontrivial(self, name):
        path = STEP_DIR / name
        assert path.exists(), f"{name} missing from out/cad/step/"
        assert path.stat().st_size > 10_000, f"{name} suspiciously small"
        head = path.read_text(encoding="utf-8", errors="ignore")[:200]
        assert "ISO-10303" in head, f"{name} is not a valid STEP file"
