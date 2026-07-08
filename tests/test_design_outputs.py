"""Design regression tests on the notebook outputs in out/.

These pin the converged design so an unintended change to a model or a
config value shows up as a red CI run instead of silent drift. They skip
when out/ has not been generated (fresh checkout); the design-pipeline
workflow executes the notebooks first, then runs these.

If a design change is INTENTIONAL, update the expected values here in
the same commit -- that is the point: the diff then documents the new
design point.
"""

import re

import pytest
import yaml

from conftest import REPO_ROOT

OUT = REPO_ROOT / "out"

pytestmark = pytest.mark.skipif(
    not (OUT / "fuselage.yaml").exists(),
    reason="out/ not generated -- run the design notebooks first",
)


@pytest.fixture(scope="module")
def airfoil() -> dict:
    return yaml.safe_load((OUT / "airfoil.yaml").read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def fuselage() -> dict:
    return yaml.safe_load((OUT / "fuselage.yaml").read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def vanes() -> dict:
    return yaml.safe_load((OUT / "control_vanes.yaml").read_text(encoding="utf-8"))


class TestAirfoil:
    def test_design_point(self, airfoil):
        assert airfoil["designation"] == "NACA 2412"
        assert airfoil["tc_ratio"] == pytest.approx(0.12, rel=1e-3)
        assert airfoil["CL_max_3D"] == pytest.approx(1.2186, rel=1e-2)
        assert airfoil["LD_cruise"] == pytest.approx(13.22, rel=1e-2)
        assert airfoil["e_oswald"] == pytest.approx(0.8691, rel=1e-2)


class TestFuselage:
    def test_design_point(self, fuselage):
        # 195 mm COTS EDF design point (2026-07 design review)
        assert fuselage["D_fus_m"] == pytest.approx(0.10145, rel=1e-2)
        assert fuselage["L_fus_m"] == pytest.approx(0.50725, rel=1e-2)
        assert fuselage["x_CG_m"] == pytest.approx(0.24003, rel=1e-2)
        assert fuselage["static_margin"] == pytest.approx(0.05, rel=1e-2)

    def test_internal_consistency(self, fuselage):
        # Segments must sum to total length
        segments = fuselage["L_nose_m"] + fuselage["L_mid_m"] + fuselage["L_tail_m"]
        assert segments == pytest.approx(fuselage["L_fus_m"], rel=1e-3)
        # Fineness ratio must match L/D
        assert fuselage["fineness"] == pytest.approx(
            fuselage["L_fus_m"] / fuselage["D_fus_m"], rel=1e-3
        )
        # CG must sit between wing leading edge and the vanes, ahead of the AC
        assert fuselage["x_wing_LE_m"] < fuselage["x_CG_m"] < fuselage["x_wing_AC_m"]

    def test_mac_from_ac_le_spacing(self, fuselage):
        # Rectangular wing: x_AC - x_LE = MAC/4. This MAC feeds the CFD
        # force-coefficient setup, so pin it explicitly.
        mac = 4.0 * (fuselage["x_wing_AC_m"] - fuselage["x_wing_LE_m"])
        assert mac == pytest.approx(0.1822, rel=1e-2)


class TestControlVanes:
    def test_design_point(self, vanes):
        assert vanes["n_vanes"] == 4
        assert vanes["AR_vane"] == pytest.approx(2.5, rel=1e-3)
        assert vanes["servo_torque_req_gcm"] == pytest.approx(461.0, rel=2e-2)

    def test_deflection_ordering(self, vanes):
        assert vanes["delta_design_deg"] < vanes["delta_stall_deg"] <= vanes["delta_max_deg"]

    def test_control_authority_positive_and_symmetric(self, vanes):
        assert vanes["M_pitch_design_Nm"] > 0
        assert vanes["M_roll_design_Nm"] > 0
        # X-configuration: pitch and yaw authority identical by symmetry
        assert vanes["M_pitch_design_Nm"] == pytest.approx(vanes["M_yaw_design_Nm"])


class TestCrossFileConsistency:
    def test_vane_hub_matches_fuselage_hub(self, fuselage, vanes):
        assert vanes["R_hub_m"] == pytest.approx(fuselage["r_hub_m"], rel=1e-6)


@pytest.fixture(scope="module")
def allrun_case() -> str:
    path = REPO_ROOT / "cfd" / "vehicle" / "Allrun.case"
    if not path.exists():
        pytest.skip("cfd/vehicle/Allrun.case not present")
    return path.read_text(encoding="utf-8")


class TestCfdCaseConsistency:
    """The CFD case hardcodes reference values derived from out/.

    If the design drifts, cfd/vehicle/Allrun.case must be updated by
    hand -- these tests catch the mismatch before an expensive CFD run.
    """

    def test_mac_matches_fuselage(self, allrun_case, fuselage):
        # lRef            0.16704;          // MAC
        m = re.search(r"lRef\s+([\d.]+)", allrun_case)
        assert m, "lRef not found in Allrun.case"
        mac_design = 4.0 * (fuselage["x_wing_AC_m"] - fuselage["x_wing_LE_m"])
        assert float(m.group(1)) == pytest.approx(mac_design, rel=1e-3)

    def test_cofr_matches_cg(self, allrun_case, fuselage):
        # CofR            (-0.23402 0 0);   body x = -station
        m = re.search(r"CofR\s+\((-?[\d.]+)", allrun_case)
        assert m, "CofR not found in Allrun.case"
        assert float(m.group(1)) == pytest.approx(-fuselage["x_CG_m"], rel=1e-3)
