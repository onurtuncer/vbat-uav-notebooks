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
        # 203 mm COTS 3-blade prop-in-duct design point (ADR-0003 as
        # amended 2026-07-12; MTOW 2.303 kg, hover ~652 W)
        assert fuselage["D_fus_m"] == pytest.approx(0.09565, rel=1e-2)
        assert fuselage["L_fus_m"] == pytest.approx(0.47827, rel=1e-2)
        assert fuselage["x_CG_m"] == pytest.approx(0.23661, rel=1e-2)
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
        assert mac == pytest.approx(0.17484, rel=1e-2)


class TestControlVanes:
    def test_design_point(self, vanes):
        assert vanes["n_vanes"] == 4
        assert vanes["AR_vane"] == pytest.approx(2.5, rel=1e-3)
        assert vanes["servo_torque_req_gcm"] == pytest.approx(441.5, rel=2e-2)

    def test_deflection_ordering(self, vanes):
        assert vanes["delta_design_deg"] < vanes["delta_stall_deg"] <= vanes["delta_max_deg"]

    def test_control_authority_positive_and_symmetric(self, vanes):
        assert vanes["M_pitch_design_Nm"] > 0
        assert vanes["M_roll_design_Nm"] > 0
        # X-configuration: pitch and yaw authority identical by symmetry
        assert vanes["M_pitch_design_Nm"] == pytest.approx(vanes["M_yaw_design_Nm"])


class TestStructure:
    """Semi-monocoque clamshell (ADR-0010): explicit member model."""

    def test_members_positive_and_close(self, fuselage):
        members = fuselage["semimono_members_kg"]
        for name in ("skin", "longerons", "crossbeams", "half_rings"):
            assert members[name] > 0, f"{name} mass must be positive"
        parts_sum = sum(v for k, v in members.items() if k != "total")
        assert members["total"] == pytest.approx(parts_sum, rel=1e-3)
        assert fuselage["m_shell_kg"] == pytest.approx(members["total"], rel=1e-3)

    def test_carbon_realizes_the_mass_gain(self, fuselage):
        # ADR-0010's claim, pinned: the 2-ply CFRP semi-monocoque is
        # lighter than the old monocoque estimate (k=1.30 x 0.8 mm CFRP
        # over the same wetted area), while the printed skin is heavier
        # (buildable now, carbon later).
        mono_equiv = 1.30 * 1600.0 * 0.0008 * fuselage["S_wet_m2"]
        assert fuselage["m_semimono_cfrp_kg"] < mono_equiv
        assert fuselage["m_semimono_fdm_kg"] > fuselage["m_semimono_cfrp_kg"]

    def test_budget_utilization(self, fuselage):
        # structure estimate must fit the top-down structural budget
        budget = fuselage["m_struct_pool_kg"] - fuselage["m_struct_carved_kg"]
        assert 0.0 < fuselage["m_shell_kg"] < budget

    def test_clamshell_geometry(self, fuselage):
        # lid spans nose tip -> tail-cone start
        assert 0.0 < fuselage["x_clam_aft_m"] < fuselage["L_fus_m"]
        assert fuselage["x_clam_aft_m"] == pytest.approx(
            fuselage["L_fus_m"] - fuselage["L_tail_m"], rel=1e-3)


class TestAileron:
    def test_design_point(self, aileron):
        assert aileron["n_ailerons"] == 2
        assert aileron["span_frac_wing"] == pytest.approx(0.12, rel=1e-3)
        assert aileron["chord_frac"] == pytest.approx(0.12, rel=1e-3)
        assert aileron["servo_torque_req_gcm"] == pytest.approx(75.7, rel=2e-2)

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
        assert vibration["f_shaft_hz"] == pytest.approx(203.5, rel=2e-2)
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
        # Heat loads follow from hover power and the efficiencies; pin
        # them at the 2.303 kg / ~652 W prop-in-duct point.
        assert thermal["esc"]["Q_W"] == pytest.approx(32.6, rel=5e-2)
        assert thermal["battery"]["Q_W"] == pytest.approx(20.2, rel=5e-2)

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
        assert 0.0 < e["temp_margin_C"] < 20.0     # positive but modest
        assert e["A_req_cm2"] == pytest.approx(212.9, rel=5e-2)


@pytest.fixture(scope="module")
def components() -> dict:
    path = OUT / "components.yaml"
    if not path.exists():
        pytest.skip("out/components.yaml not generated -- run cots_selection")
    return yaml.safe_load(path.read_text(encoding="utf-8"))


class TestCotsSelection:
    def test_frozen_hardware_ids(self, components):
        # The frozen COTS stack (lightest feasible per derived requirement).
        # An intentional database/requirement change must update these ids
        # in the same commit -- the diff is the procurement record.
        sel = components["selected"]
        assert sel["flight_controller"]["id"] == "pixhawk_6c"
        assert sel["esc"]["id"] == "apd_80f3x"
        assert sel["edf_motor"]["id"] == "sunnysky_x4120_465"
        assert sel["propeller"]["id"] == "ma_3blade_8x6"   # ADR-0003 amendment
        assert sel["propeller"]["frozen"] is True          # pinned, not auto
        assert sel["servo"]["id"] == "kst_x08_v6"
        assert sel["battery"]["id"] == "gens_ace_6s_4000_30c"

    def test_requirements_derived_from_design_point(self, components):
        req = components["requirements"]
        assert req["esc"]["i_cont_min_a"] == pytest.approx(47.0, rel=2e-2)
        assert req["propeller"]["d_rotor_mm"] == pytest.approx(203.0, rel=1e-3)
        assert req["servo"]["stall_torque_min_gcm"] == pytest.approx(662.3, rel=2e-2)
        assert req["battery"]["capacity_min_ah"] == pytest.approx(3.50, rel=2e-2)

    def test_selected_parts_meet_their_requirements(self, components):
        sel, req = components["selected"], components["requirements"]
        assert sel["esc"]["ratings"]["i_cont_a"] >= req["esc"]["i_cont_min_a"]
        assert sel["edf_motor"]["ratings"]["p_max_w"] >= req["edf_motor"]["p_max_min_w"]
        assert (sel["servo"]["ratings"]["stall_torque_gcm"]
                >= req["servo"]["stall_torque_min_gcm"])

    def test_budget_findings_pinned(self, components):
        # KNOWN findings (same discipline as the ADR-0009 thermal pin):
        # ESC and servos fit; the avionics bay is over; motor + 3-blade
        # prop still land ~1.5x over the motor_fan allocation (down from
        # ~6x pre-amendment -- the motor now dominates). Pin the state so
        # a change either way is caught.
        b = components["budgets"]
        assert b["esc"]["within"] is True
        assert b["servo_each"]["within"] is True
        assert b["avionics_bay"]["within"] is False
        assert b["motor_fan"]["within"] is False
        assert b["motor_fan"]["actual_g"] < 2 * b["motor_fan"]["alloc_g"]
        # COTS capacity quantisation: the pack lands a little over the
        # sized battery mass, but bounded (< 15%).
        assert b["battery"]["within"] is False
        assert b["battery"]["actual_g"] < 1.15 * b["battery"]["alloc_g"]
        assert components["all_within_allocations"] is False

    def test_heavy_fan_rejected_only_on_diameter(self, components):
        # The ADR-0003 amendment's paper trail: the DS-215 heavy-lift fan
        # (the pre-amendment selection) must stay visible as a
        # diameter-only rejection at the 203 mm prop-in-duct point.
        rej = components["selected"]["propeller"]["rejected"]
        assert "schuebeler_ds215_dia_hst_fan" in rej
        assert "mm rotor" in rej["schuebeler_ds215_dia_hst_fan"]


@pytest.fixture(scope="module")
def aileron_cots() -> dict:
    path = OUT / "aileron_cots.yaml"
    if not path.exists():
        pytest.skip("out/aileron_cots.yaml not generated -- run aileron_design_cots")
    return yaml.safe_load(path.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def vibration_cots() -> dict:
    path = OUT / "vibration_cots.yaml"
    if not path.exists():
        pytest.skip("out/vibration_cots.yaml not generated -- run vibration_isolation_cots")
    return yaml.safe_load(path.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def fuselage_cots() -> dict:
    path = OUT / "fuselage_cots.yaml"
    if not path.exists():
        pytest.skip("out/fuselage_cots.yaml not generated -- run fuselage_design_cots")
    return yaml.safe_load(path.read_text(encoding="utf-8"))


class TestAsSelectedResolve:
    """Post-freeze re-solve (NB12/NB13, ADR-0012): the conceptual
    solutions stay in aileron/vibration.yaml; the as-selected ones land
    in the *_cots.yaml handoffs, tied to the frozen hardware ids."""

    def test_aileron_resolve_uses_frozen_servo(self, aileron_cots, components):
        servo = components["selected"]["servo"]
        assert aileron_cots["cots_servo"]["id"] == servo["id"]
        assert aileron_cots["servo_mass_kg_each"] == pytest.approx(
            servo["mass_g"] * 1e-3, rel=1e-6)

    def test_aileron_surface_is_servo_independent(self, aileron, aileron_cots):
        # the re-solve must not move the aerodynamic solution -- only the
        # hardware mass and the torque check change
        for key in ("c_aileron_m", "b_aileron_m", "S_aileron_m2",
                    "servo_torque_req_gcm", "ddot_roll_total_cruise_deg_s2"):
            assert aileron_cots[key] == pytest.approx(aileron[key], rel=1e-6)

    def test_frozen_servo_margins(self, aileron_cots):
        s = aileron_cots["cots_servo"]
        assert s["margin_vs_vane_req"] >= 1.0        # vane torque sizes the servo
        assert s["margin_vs_aileron_req"] >= s["margin_vs_vane_req"]
        assert aileron_cots["cruise_authority_ok"] is True

    def test_vibration_resolve_uses_frozen_fc(self, vibration_cots, components):
        fc = components["selected"]["flight_controller"]
        assert vibration_cots["cots_fc"]["id"] == fc["id"]
        assert vibration_cots["modules"]["fc_imu"]["m_isolated_kg"] == pytest.approx(
            fc["mass_g"] * 1e-3, rel=1e-3)

    def test_corner_frequency_is_mass_independent(self, vibration, vibration_cots):
        # f_n depends only on forcing/target/damping -- the re-solve must
        # reproduce it (and the sway pad NB6/NB14 pack against) exactly
        assert vibration_cots["f_n_hz"] == pytest.approx(vibration["f_n_hz"], rel=1e-6)
        assert vibration_cots["sway_pad_total_m"] == pytest.approx(
            vibration["sway_pad_total_m"], rel=1e-6)
        assert vibration_cots["window_ok"] is True

    def test_fc_isolator_softens_with_the_real_board(self, vibration, vibration_cots):
        # k scales with the isolated mass; the Pixhawk 6C is lighter than
        # the 60 g estimate, so the procured isolators must be softer
        k_est  = vibration["modules"]["fc_imu"]["k_isolator_N_m"]
        k_cots = vibration_cots["modules"]["fc_imu"]["k_isolator_N_m"]
        assert k_cots < k_est
        assert vibration_cots["modules"]["payload"]["k_isolator_N_m"] == pytest.approx(
            vibration["modules"]["payload"]["k_isolator_N_m"], rel=1e-6)


class TestAsSelectedFuselage:
    """Post-freeze fuselage re-solve (NB14, ADR-0012): real masses,
    envelope-floored bays, physical fit. The conceptual out/fuselage.yaml
    stays the CAD/CFD geometry; these pin the as-selected hull and its
    standing findings."""

    def test_design_point(self, fuselage_cots):
        assert fuselage_cots["D_fus_m"] == pytest.approx(0.10627, rel=1e-2)
        assert fuselage_cots["L_fus_m"] == pytest.approx(0.53133, rel=1e-2)
        assert fuselage_cots["x_CG_m"] == pytest.approx(0.25678, rel=1e-2)
        assert fuselage_cots["active_constraint"] == "packaging"

    def test_internal_consistency(self, fuselage_cots):
        segments = (fuselage_cots["L_nose_m"] + fuselage_cots["L_mid_m"]
                    + fuselage_cots["L_tail_m"])
        assert segments == pytest.approx(fuselage_cots["L_fus_m"], rel=1e-3)
        assert fuselage_cots["fineness"] == pytest.approx(
            fuselage_cots["L_fus_m"] / fuselage_cots["D_fus_m"], rel=1e-3)

    def test_hull_grows_to_hold_the_real_hardware(self, fuselage, fuselage_cots):
        # rigid COTS envelopes floor the bay stack -- the as-selected hull
        # must be at least as big as the conceptual one, never smaller
        assert fuselage_cots["D_fus_m"] >= fuselage["D_fus_m"]
        assert fuselage_cots["L_fus_m"] >= fuselage["L_fus_m"]
        assert fuselage_cots["S_wet_m2"] >= fuselage["S_wet_m2"]

    def test_battery_bay_holds_the_real_pack(self, fuselage_cots, components):
        layout = {e["name"]: e for e in fuselage_cots["layout"]}
        battery = components["selected"]["battery"]
        assert layout["battery"]["mass_kg"] == pytest.approx(
            battery["mass_g"] * 1e-3, rel=1e-6)
        assert layout["battery"]["length_m"] * 1e3 >= max(battery["dims_mm"])

    def test_all_frozen_parts_fit(self, fuselage_cots):
        for entry in fuselage_cots["bay_fit"]:
            assert entry["ok"] is True, (
                f"{entry['part']} no longer fits {entry['where']} "
                f"(needs {entry['need_mm']} mm, has {entry['have_mm']} mm)")

    def test_as_selected_mass_rollup(self, fuselage_cots):
        # the all-up delta IS the sum of the freeze's standing mass
        # findings; pin its state (positive, bounded) like the thermal pin
        s = fuselage_cots["as_selected"]
        assert s["m_items_total_kg"] == pytest.approx(
            s["m_closure_mtow_kg"] + s["m_delta_kg"], rel=1e-3)
        assert 0.0 < s["m_delta_kg"] < 0.2
        assert s["m_delta_kg"] == pytest.approx(0.1246, rel=5e-2)

    def test_structure_over_budget_is_pinned_finding(self, fuselage_cots):
        # KNOWN finding (ADR-0012): the hull that holds the real hardware
        # outgrows the structural fraction at the closure MTOW. Pin the
        # state so a change either way is caught.
        budget = (fuselage_cots["m_struct_pool_kg"]
                  - fuselage_cots["m_struct_carved_kg"])
        assert fuselage_cots["m_shell_kg"] > budget
        assert fuselage_cots["m_shell_kg"] < 1.25 * budget   # over, but bounded


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
