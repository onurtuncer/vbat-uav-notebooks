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


@pytest.fixture(scope="module")
def aileron() -> dict:
    return yaml.safe_load((OUT / "aileron.yaml").read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def vibration() -> dict:
    return yaml.safe_load((OUT / "vibration.yaml").read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def thermal() -> dict:
    return yaml.safe_load((OUT / "thermal.yaml").read_text(encoding="utf-8"))


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
        assert fuselage["D_fus_m"] == pytest.approx(0.10247, rel=1e-2)
        assert fuselage["L_fus_m"] == pytest.approx(0.51235, rel=1e-2)
        assert fuselage["x_CG_m"] == pytest.approx(0.25255, rel=1e-2)
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
        assert mac == pytest.approx(0.2014, rel=1e-2)


class TestControlVanes:
    def test_design_point(self, vanes):
        assert vanes["n_vanes"] == 4
        assert vanes["AR_vane"] == pytest.approx(2.5, rel=1e-3)
        assert vanes["servo_torque_req_gcm"] == pytest.approx(563.0, rel=2e-2)

    def test_deflection_ordering(self, vanes):
        assert vanes["delta_design_deg"] < vanes["delta_stall_deg"] <= vanes["delta_max_deg"]

    def test_control_authority_positive_and_symmetric(self, vanes):
        assert vanes["M_pitch_design_Nm"] > 0
        assert vanes["M_roll_design_Nm"] > 0
        # X-configuration: pitch and yaw authority identical by symmetry
        assert vanes["M_pitch_design_Nm"] == pytest.approx(vanes["M_yaw_design_Nm"])


class TestAileron:
    def test_design_point(self, aileron):
        assert aileron["n_ailerons"] == 2
        assert aileron["span_frac_wing"] == pytest.approx(0.12, rel=1e-3)
        assert aileron["chord_frac"] == pytest.approx(0.12, rel=1e-3)
        assert aileron["servo_torque_req_gcm"] == pytest.approx(115.8, rel=2e-2)

    def test_cruise_roll_authority(self, aileron):
        # The whole point of NB4: combined (aileron + residual jet-vane)
        # cruise roll authority must clear the shared requirement.
        assert aileron["cruise_authority_ok"] is True
        assert (aileron["ddot_roll_total_cruise_deg_s2"]
                >= aileron["ddot_min_deg_s2"])
        # Jet-vane authority alone is thin in cruise -- confirm the
        # aileron is doing real work, not just riding on vane margin.
        assert aileron["ddot_roll_aileron_deg_s2"] > aileron["ddot_min_deg_s2"]


class TestVibration:
    def test_forcing_and_corner(self, vibration):
        # 1/rev forcing derived from rotor RPM; corner freq must sit in the
        # valid window (above control bandwidth, below isolation threshold).
        assert vibration["f_shaft_hz"] == pytest.approx(211.3, rel=2e-2)
        lo, hi = vibration["f_n_window_hz"]
        assert lo < vibration["f_n_hz"] < hi
        assert vibration["window_ok"] is True

    def test_meets_transmissibility_target(self, vibration):
        # every isolated module must meet the configured attenuation target
        # at the forcing frequency.
        target = vibration["target_transmissibility"]
        for mod in vibration["modules"].values():
            assert mod["transmissibility"] <= target * (1.0 + 1e-3)

    def test_sway_is_small(self, vibration):
        # high forcing frequency -> small static deflection -> small sway;
        # guard that the packing cost stays a few mm, not centimetres.
        assert 0.0 < vibration["sway_mm"] < 5.0


class TestThermal:
    def test_heat_loads_derived_and_sane(self, thermal):
        # Heat loads follow from hover power and the efficiencies; pin the
        # order of magnitude (ESC ~50 W, battery ~30 W at the 3.06 kg point).
        assert thermal["esc"]["Q_W"] == pytest.approx(51.4, rel=5e-2)
        assert thermal["battery"]["Q_W"] == pytest.approx(31.8, rel=5e-2)

    def test_battery_bay_vents_comfortably(self, thermal):
        b = thermal["battery"]
        assert b["ok"] is True
        assert b["area_headroom"] > 5.0          # lots of wall to spare
        assert b["temp_margin_C"] > 5.0

    def test_esc_path_is_marginal_finding(self, thermal):
        # KNOWN finding (ADR-0009): the ESC cold-plate is marginal at the
        # current design point -- pin that state so a change either way is
        # caught. This is a documented finding, not a passing design.
        e = thermal["esc"]
        assert e["ok"] is False
        assert e["mass_within_alloc"] is False    # plate heavier than ESC alloc
        assert 0.0 < e["temp_margin_C"] < 10.0     # positive but tight
        assert e["A_req_cm2"] == pytest.approx(346.6, rel=5e-2)


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
