"""
fuselage_design.py  --  Parametric Fuselage Sizing for the V-BAT Tail-Sitter
=============================================================================

Sizes the fuselage of the electric tail-sitter from the converged mass
closure (mass_closure.py) and the EDF geometry.

AXIS CONVENTION  (consistent with the Aetherion 6-DoF library)
--------------------------------------------------------------
    Body frame:  x forward (out the nose), y right, z down   (FRD)
    World frame: NED (x North, y East, z Down)

    Internally this module uses the STATION coordinate x_s, measured
    from the nose tip, POSITIVE AFT (standard fuselage-station
    practice).  Conversion to the body frame is simply:

        x_body = -x_s        (origin at the nose tip)

    The EDF exhaust and control vanes therefore point in -x_body.

THEORY
------

1. PACKAGING (drives length, not diameter)
    Each internal component occupies a cylindrical bay:

        L_bay = V_comp / (packing_factor * A_internal)

    where V_comp = m_comp / rho_comp and A_internal is the internal
    cross-section of the mid-body.  The stack of bays must fit between
    the nose equipment station and the start of the tail cone.

2. DIAMETER (driven by the EDF hub, checked against packaging)
    A tail-sitter EDF fuselage must taper down to the fan hub -- the
    tail cone IS the fan centerbody.  Therefore:

        D_fus >= d_hub_margin * (2 * R_hub)

    The smallest diameter that satisfies BOTH the hub constraint and
    the packaging constraint (at the target fineness ratio) is chosen.
    At this vehicle scale the hub constraint is almost always active,
    i.e. the fuselage has volume to spare.

3. SHAPE  (body of revolution, three segments)
        nose  : semi-ellipsoid,             x_s in [0, L_nose]
        mid   : cylinder,                   x_s in [L_nose, L_nose+L_mid]
        tail  : smoothstep boattail to hub, x_s in [L-L_tail, L]

    The boattail radius uses the C1-continuous smoothstep
        r(s) = R - (R - r_hub) * s^2 (3 - 2 s)
    which is tangent to the cylinder at s=0 and level at s=1 -- this
    is also directly used as the CAD revolve profile.

4. ZERO-LIFT DRAG  (Raymer component buildup, single component)
        Re_L  = rho V L / mu
        Cf    = 0.455 / log10(Re_L)^2.58            (turbulent plate)
        FF    = 1 + 60/f^3 + f/400                  (Raymer eq. 12.31)
        CD0_f = Cf * FF * Q * S_wet / S_ref

5. STRUCTURE MASS  (semi-monocoque member model, ADR-0010)
        m_struct = k_skin*rho_skin*t_skin*S_wet          (skin)
                 + lambda_frame * (2 L_clam + 2 D)       (longerons)
                 + lambda_frame * n_cross * 0.8 D        (crossbeams)
                 + lambda_frame * n_rings * pi D / 2     (half-rings)

    Explicit members replace the old monocoque area-density estimate;
    compared against the structural weight fraction budget
    (m_structure - m_wing) from the mass closure.

6. LONGITUDINAL LAYOUT AND CG
    Components are stacked nose-to-tail:
        payload -> avionics -> battery -> ESC ... motor/fan at tail.
    The propulsion fraction is split motor+fan / ESC / duct.
    The wing is then PLACED so that the wing aerodynamic center sits
    static_margin * MAC behind the total CG (wing mass at the CG makes
    this a mild fixed-point problem -- 3 iterations suffice).

References
----------
  Raymer, "Aircraft Design: A Conceptual Approach", ch. 12 (drag buildup)
  Hoerner, "Fluid-Dynamic Drag", ch. 6 (bodies of revolution)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import yaml


MU_AIR = 1.789e-5   # dynamic viscosity of air, ISA sea level [Pa s]

# Split of the propulsion mass fraction between its hardware items.
# Motor + fan rotor sit at the fan plane, the ESC sits mid-body where
# cooling air is available, the duct ring is at the tail.
PROP_SPLIT = {"motor_fan": 0.60, "esc": 0.25, "duct": 0.15}

# Guard on how much of a top-down mass-fraction budget
# (config/initial_weight_fraction_estimation.yaml: structural/avionics/misc)
# may be re-allocated to named hardware that used to hide inside it, before
# the fraction assumption itself needs revisiting rather than being
# silently netted down.
MAX_HARDWARE_CARVE_FRACTION = 0.5


# ---------------------------------------------
#  Input parameters (config/fuselage.yaml)
# ---------------------------------------------
@dataclass
class FuselageParams:
    # packaging
    rho_battery_pack: float   # [kg/m^3]
    rho_payload:      float   # [kg/m^3]
    rho_avionics:     float   # [kg/m^3]
    packing_factor:   float   # [-]
    # shape
    fineness_ratio:   float   # L/D target [-]
    f_nose:           float   # nose fraction of L [-]
    f_tail:           float   # tail-cone fraction of L [-]
    d_hub_margin:     float   # D_fus >= margin * d_hub [-]
    part_clearance_m: float   # axial clearance per COTS part in a bay [m]
    # shell
    t_shell_m:        float   # packaging wall for r_int [m]
    rho_shell:        float   # [kg/m^3] (vane plates etc.)
    # aero
    Q_interference:   float   # [-]
    # stability
    static_margin:    float   # (x_AC - x_CG)/MAC [-]
    # battery CG trim
    battery_tray_travel_m: float   # +/- rail travel [m]
    battery_tray_mass_kg:  float   # sliding-rail hardware [kg]
    # control hardware (vane + servo + linkage) mass model
    t_vane_plate_m:   float   # CFRP vane plate thickness [m]
    servo_mass_kg:    float   # per-vane servo + horn + wiring [kg]
    linkage_mass_kg:  float   # per-vane pushrod + hinge pin [kg]
    # duct geometry
    duct_chord_ratio: float   # duct chord / D_rotor [-]
    t_duct_m:         float   # [m]
    tip_clearance_m:  float   # [m]

    @property
    def f_mid(self) -> float:
        return 1.0 - self.f_nose - self.f_tail

    @classmethod
    def from_yaml(cls, path) -> "FuselageParams":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(
            rho_battery_pack = float(data["rho_battery_pack"]),
            rho_payload      = float(data["rho_payload"]),
            rho_avionics     = float(data["rho_avionics"]),
            packing_factor   = float(data["packing_factor"]),
            part_clearance_m = float(data["part_clearance_m"]),
            fineness_ratio   = float(data["fineness_ratio"]),
            f_nose           = float(data["f_nose"]),
            f_tail           = float(data["f_tail"]),
            d_hub_margin     = float(data["d_hub_margin"]),
            t_shell_m        = float(data["t_shell_m"]),
            rho_shell        = float(data["rho_shell"]),
            Q_interference   = float(data["Q_interference"]),
            static_margin    = float(data["static_margin"]),
            battery_tray_travel_m = float(data["battery_tray_travel_m"]),
            battery_tray_mass_kg  = float(data["battery_tray_mass_kg"]),
            t_vane_plate_m   = float(data["t_vane_plate_m"]),
            servo_mass_kg    = float(data["servo_mass_kg"]),
            linkage_mass_kg  = float(data["linkage_mass_kg"]),
            duct_chord_ratio = float(data["duct_chord_ratio"]),
            t_duct_m         = float(data["t_duct_m"]),
            tip_clearance_m  = float(data["tip_clearance_m"]),
        )


# ---------------------------------------------
#  Modularity / structure parameters (config/modularity.yaml, ADR-0010)
# ---------------------------------------------
@dataclass
class ModularityParams:
    """Clamshell split, semi-monocoque frame, and joint hardware.

    Architecture (ADR-0010, external review C. Ucler): longitudinal
    clamshell -- structural lower half + full-length hinged upper lid; a
    rectangular profile around the joint line works as the longerons,
    with crossbeams as equipment mounts and half-rings tying the battery
    rail to the frame. The skin is a thin covering (semi-monocoque), not
    the primary load path. Member masses are COMPUTED from geometry;
    only the discrete hinge/latch hardware is a configured mass, carved
    from the structural fraction (ADR-0005 discipline).

    The two-piece wing on the CFRP carry-through spar is unchanged from
    ADR-0008.
    """
    # clamshell
    clam_aft_frac:      float   # clamshell region / L_fus [-] (nose tip -> here)
    lid_hinge_mass_kg:  float   # piano-hinge strip along one longeron [kg]
    lid_latch_mass_kg:  float   # latches + reinforcement, other longeron [kg]
    # longeron frame (semi-monocoque)
    frame_profile_side_m: float   # square box-profile outer side [m]
    frame_profile_wall_m: float   # profile wall [m]
    frame_material:       str     # key into rho_frame
    rho_frame:            dict    # material -> density [kg/m^3]
    n_crossbeams:         int     # transverse equipment-mount beams [-]
    n_half_rings:         int     # battery-rail / shell-stabilizing rings [-]
    # skin, per construction method
    skin:               dict    # method -> {t_skin_m, rho_skin}
    k_skin:             float   # bonding/overlap allowance on the skin [-]
    # wing spar (unchanged, ADR-0008)
    spar_od_m:                 float   # CFRP tube OD [m] (= wing_lighten --spar-hole)
    spar_wall_m:               float   # tube wall [m]
    spar_span_frac:            float   # spar length / wing span [-]
    spar_chord_frac:           float   # spar chordwise station / chord [-]
    rho_spar:                  float   # CFRP tube density [kg/m^3]
    wing_root_fitting_mass_kg: float   # per-side root socket + screw [kg]

    @classmethod
    def from_yaml(cls, path) -> "ModularityParams":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        p = cls(
            clam_aft_frac        = float(data["clam_aft_frac"]),
            lid_hinge_mass_kg    = float(data["lid_hinge_mass_kg"]),
            lid_latch_mass_kg    = float(data["lid_latch_mass_kg"]),
            frame_profile_side_m = float(data["frame_profile_side_m"]),
            frame_profile_wall_m = float(data["frame_profile_wall_m"]),
            frame_material       = str(data["frame_material"]),
            rho_frame            = {k: float(v) for k, v in data["rho_frame"].items()},
            n_crossbeams         = int(data["n_crossbeams"]),
            n_half_rings         = int(data["n_half_rings"]),
            skin                 = {m: {k: float(v) for k, v in s.items()}
                                    for m, s in data["skin"].items()},
            k_skin               = float(data["k_skin"]),
            spar_od_m                 = float(data["spar_od_m"]),
            spar_wall_m               = float(data["spar_wall_m"]),
            spar_span_frac            = float(data["spar_span_frac"]),
            spar_chord_frac           = float(data["spar_chord_frac"]),
            rho_spar                  = float(data["rho_spar"]),
            wing_root_fitting_mass_kg = float(data["wing_root_fitting_mass_kg"]),
        )
        if p.frame_material not in p.rho_frame:
            raise ValueError(
                f"frame_material '{p.frame_material}' has no rho_frame entry "
                f"(available: {sorted(p.rho_frame)})"
            )
        return p

    def spar_length_m(self, b_wing_m: float) -> float:
        return self.spar_span_frac * b_wing_m

    def spar_tube_mass_kg(self, b_wing_m: float) -> float:
        """Computed from geometry -- never configured."""
        r_o = self.spar_od_m / 2.0
        r_i = r_o - self.spar_wall_m
        area = math.pi * (r_o**2 - r_i**2)
        return area * self.spar_length_m(b_wing_m) * self.rho_spar

    def frame_linear_density(self) -> float:
        """Box-profile linear density [kg/m], computed from the section."""
        s, w = self.frame_profile_side_m, self.frame_profile_wall_m
        area = s * s - (s - 2.0 * w) ** 2
        return area * self.rho_frame[self.frame_material]

    def semi_monocoque_masses(self, S_wet_m2: float, D_fus_m: float,
                              L_fus_m: float, method: str) -> dict:
        """
        Per-member semi-monocoque structure mass [kg] for a construction
        method key in `skin` (ADR-0010, all computed from geometry):

          skin       : k_skin * rho_skin * t_skin * S_wet
          longerons  : picture-frame perimeter ~ 2*L_clam + 2*D
          crossbeams : n * 0.8*D each (transverse, between longerons)
          half_rings : n * (pi*D/2) each (lower shell)
        """
        if method not in self.skin:
            raise ValueError(
                f"construction method '{method}' has no skin entry "
                f"(available: {sorted(self.skin)})"
            )
        sk = self.skin[method]
        lam = self.frame_linear_density()
        L_clam = self.clam_aft_frac * L_fus_m

        m_skin       = self.k_skin * sk["rho_skin"] * sk["t_skin_m"] * S_wet_m2
        m_longerons  = (2.0 * L_clam + 2.0 * D_fus_m) * lam
        m_crossbeams = self.n_crossbeams * 0.8 * D_fus_m * lam
        m_half_rings = self.n_half_rings * (math.pi * D_fus_m / 2.0) * lam
        return {
            "skin": m_skin,
            "longerons": m_longerons,
            "crossbeams": m_crossbeams,
            "half_rings": m_half_rings,
            "total": m_skin + m_longerons + m_crossbeams + m_half_rings,
        }


# ---------------------------------------------
#  Layout entry (one mass item on the axis)
# ---------------------------------------------
@dataclass
class LayoutItem:
    name:     str
    mass_kg:  float
    x_start:  float    # station of bay start [m from nose, +aft]
    length:   float    # bay length [m]  (0 for point masses)

    @property
    def x_cg(self) -> float:
        return self.x_start + 0.5 * self.length


# ---------------------------------------------
#  Sizing output container
# ---------------------------------------------
@dataclass
class FuselageSizing:
    # primary geometry
    D_fus:        float   # max outer diameter          [m]
    L_fus:        float   # total length                [m]
    L_nose:       float   # nose length                 [m]
    L_mid:        float   # cylinder length             [m]
    L_tail:       float   # tail-cone length            [m]
    r_hub:        float   # tail exit (fan hub) radius  [m]
    fineness:     float   # L/D actual                  [-]
    # which constraint set the diameter
    active_constraint: str   # "hub" | "packaging"
    # volumes
    V_total_m3:   float   # enclosed volume             [m^3]
    V_bays_m3:    float   # required bay volume         [m^3]
    L_stack_m:    float   # required bay stack length   [m]
    L_avail_m:    float   # available stack length      [m]
    # aero
    S_wet_m2:     float   # wetted area                 [m^2]
    Re_L:         float   # length Reynolds number      [-]
    Cf:           float   # skin friction coefficient   [-]
    FF:           float   # form factor                 [-]
    CD0_fus:      float   # fuselage CD0 on S_wing      [-]
    # structure
    m_shell_kg:   float   # shell + frames mass                    [kg]
    m_struct_pool_kg:   float  # (m_structure - m_wing) pool        [kg]
    m_struct_carved_kg: float  # vanes + linkages pulled out        [kg]
    m_struct_budget_kg: float  # pool - carved: left for the shell  [kg]
    # avionics allocation traceability (top-down fraction vs. carved servos)
    m_avionics_budget_kg: float   # original mass-closure avionics fraction [kg]
    m_avionics_carved_kg: float   # vane servos pulled out                 [kg]
    m_avionics_net_kg:    float   # what remains as the "avionics" bay     [kg]
    # misc allocation traceability (top-down fraction vs. carved allowances)
    m_misc_budget_kg: float   # original mass-closure misc fraction    [kg]
    m_misc_carved_kg: float   # battery_tray pulled out                [kg]
    m_misc_net_kg:    float   # what remains as the "misc" LayoutItem  [kg]
    # layout / balance
    items:        List[LayoutItem]
    x_CG:         float   # CG station from nose        [m, +aft]
    x_wing_LE:    float   # wing LE station             [m, +aft]
    x_wing_AC:    float   # wing AC station             [m, +aft]
    x_vane:       float   # vane mid-chord station      [m, +aft]
    L_vane_arm:   float   # |x_vane - x_CG| control arm [m]
    # battery CG trim
    x_battery_trim_m:      float   # applied trim offset       [m, +aft]
    battery_tray_travel_m: float   # available +/- rail travel [m]
    # vibration-isolation packing cost
    sway_pad_m:            float   # rattle space added to the bay stack [m]
    m_isolation_hw_kg:     float   # total isolator hardware (both bays) [kg]
    # clamshell + semi-monocoque structure (ADR-0010)
    x_clam_aft_m:     float   # clamshell aft end (lid length) [m, +aft]
    frame_profile_side_m: float  # frame box-profile outer side [m]
    n_crossbeams:     int     # transverse frame beams [-]
    construction_method: str  # skin/frame method the structure was sized for
    semimono_members: Dict    # per-member masses for that method [kg]
    m_semimono_fdm_kg:  float # total, segmented_fdm skin        [kg]
    m_semimono_cfrp_kg: float # total, cfrp_2ply skin            [kg]
    # wing spar + joint hardware (ADR-0008 spar unchanged)
    spar_od_m:        float   # wing carry-through spar OD  [m]
    spar_chord_frac:  float   # spar chordwise station / chord [-]
    spar_length_m:    float   # spar tube length            [m]
    m_spar_kg:        float   # spar tube mass (computed)   [kg]
    m_joint_hw_kg:    float   # lid hinge + latches         [kg]
    m_spar_hw_kg:     float   # spar tube + 2 root fittings [kg]
    # duct geometry (for CAD)
    duct_chord:   float   # duct axial length           [m]
    D_duct_inner: float   # duct inner diameter         [m]
    D_duct_outer: float   # duct outer diameter         [m]


# ---------------------------------------------
#  Body-of-revolution profile
# ---------------------------------------------

def fuselage_radius(x_s: float, D: float, L: float,
                    f_nose: float, f_tail: float, r_hub: float) -> float:
    """
    Local outer radius at station x_s (from nose, +aft) of the
    3-segment body of revolution.  Used by the notebook plots AND the
    CAD revolve profile, so geometry is defined in exactly one place.
    """
    R      = D / 2.0
    L_nose = f_nose * L
    L_tail = f_tail * L
    x_tail = L - L_tail

    if x_s <= 0.0:
        return 0.0
    if x_s < L_nose:
        # semi-ellipse:  r = R sqrt(1 - ((L_nose - x)/L_nose)^2)
        u = (L_nose - x_s) / L_nose
        return R * math.sqrt(max(0.0, 1.0 - u * u))
    if x_s <= x_tail:
        return R
    if x_s <= L:
        # smoothstep boattail: tangent at cylinder, level at hub
        s = (x_s - x_tail) / L_tail
        return R - (R - r_hub) * s * s * (3.0 - 2.0 * s)
    return r_hub


def _integrate_profile(D: float, L: float, f_nose: float, f_tail: float,
                       r_hub: float, n: int = 400) -> Tuple[float, float]:
    """
    Trapezoidal integration of the profile.
    Returns (V_total [m^3], S_wet [m^2]).
        V     = int  pi r^2      dx
        S_wet = int  2 pi r ds,   ds = sqrt(1 + (dr/dx)^2) dx
    """
    dx = L / n
    V = 0.0
    S = 0.0
    r_prev = fuselage_radius(0.0, D, L, f_nose, f_tail, r_hub)
    for i in range(1, n + 1):
        r = fuselage_radius(i * dx, D, L, f_nose, f_tail, r_hub)
        V += 0.5 * math.pi * (r_prev**2 + r**2) * dx
        ds = math.sqrt(dx * dx + (r - r_prev) ** 2)
        S += math.pi * (r_prev + r) * ds
        r_prev = r
    return V, S


# ---------------------------------------------
#  Component volumes
# ---------------------------------------------

def component_volumes(m_battery_kg: float, m_payload_kg: float,
                      m_avionics_kg: float, p: FuselageParams) -> Dict[str, float]:
    """Raw component volumes [m^3] from mass / density."""
    return {
        "payload":  m_payload_kg  / p.rho_payload,
        "avionics": m_avionics_kg / p.rho_avionics,
        "battery":  m_battery_kg  / p.rho_battery_pack,
    }


def min_axial_length_m(shape: str, dims_m: tuple, D_int_m: float) -> float:
    """
    Shortest axial bay length a catalog envelope can occupy inside a
    circular cross-section of internal diameter D_int_m, over all
    orientations whose cross-section fits (a rectangle fits a circle
    iff its diagonal does).  Returns math.inf if the part fits in no
    orientation -- the diameter solver then has to grow the hull.

    Used by the post-freeze fuselage re-solve (cots_integration):
    volume-based bay lengths assume the contents can be shaped to the
    bay, which a rigid COTS part (a long LiPo pack, an FC board)
    cannot.
    """
    best = math.inf
    if shape == "cylinder":
        d, length = dims_m
        if d <= D_int_m:                       # axis along the fuselage
            best = length
        if math.hypot(d, length) <= D_int_m:   # lying transverse (box bound)
            best = min(best, d)
        return best
    a, b, c = dims_m
    for axial, u, v in ((a, b, c), (b, a, c), (c, a, b)):
        if math.hypot(u, v) <= D_int_m:
            best = min(best, axial)
    return best


def _envelope_floor_m(envelopes: list, D_int_m: float,
                      clearance_m: float) -> float:
    """Minimum axial length for a bay that must hold the given rigid
    catalog envelopes [(shape, dims_m), ...], stacked axially, each
    with its own clearance.  0 for an empty list; inf if any envelope
    cannot fit the cross-section in any orientation."""
    total = 0.0
    for shape, dims in envelopes:
        need = min_axial_length_m(shape, dims, D_int_m)
        if not math.isfinite(need):
            return math.inf
        total += need + clearance_m
    return total


def _stack_length(volumes: Dict[str, float], D: float, p: FuselageParams,
                  sway_pad_m: float = 0.0,
                  part_envelopes: Dict[str, list] = None) -> float:
    """Total bay stack length needed at internal diameter of D.

    `sway_pad_m` is fixed extra length (independent of D) reserved as
    rattle space around the soft-mounted bays (vibration isolation).
    `part_envelopes` (bay name -> [(shape, dims_m), ...]) floors each
    bay's volume-based length at what its rigid COTS contents need
    (post-freeze re-solve; None keeps the conceptual volume model).
    """
    r_int = D / 2.0 - p.t_shell_m
    A_int = math.pi * r_int * r_int
    env = part_envelopes or {}
    total = 0.0
    for name, v in volumes.items():
        L_vol = v / (p.packing_factor * A_int)
        L_min = _envelope_floor_m(env.get(name, []), 2.0 * r_int,
                                  p.part_clearance_m)
        total += max(L_vol, L_min)
    return total + sway_pad_m


# ---------------------------------------------
#  Diameter solver
# ---------------------------------------------

def solve_diameter(volumes: Dict[str, float], r_hub: float,
                   p: FuselageParams, sway_pad_m: float = 0.0,
                   part_envelopes: Dict[str, list] = None) -> Tuple[float, str]:
    """
    Smallest D that satisfies BOTH constraints at L = fineness * D:

      hub      : D >= d_hub_margin * 2 r_hub
      packaging: stack length (incl. isolator sway pad and any rigid
                 COTS envelope floors) <= L_mid + 0.5 L_nose

    Returns (D, active_constraint).
    """
    D_hub = p.d_hub_margin * 2.0 * r_hub

    def packaging_ok(D: float) -> bool:
        L = p.fineness_ratio * D
        L_avail = p.f_mid * L + 0.5 * p.f_nose * L
        return _stack_length(volumes, D, p, sway_pad_m, part_envelopes) <= L_avail

    # bisection for the packaging-limited diameter
    lo, hi = 0.02, 1.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if packaging_ok(mid):
            hi = mid
        else:
            lo = mid
    D_pack = hi

    if D_hub >= D_pack:
        return D_hub, "hub"
    return D_pack, "packaging"


# ---------------------------------------------
#  Drag buildup
# ---------------------------------------------

def fuselage_cd0(L: float, S_wet: float, S_ref: float, V: float,
                 rho: float, Q: float) -> Tuple[float, float, float, float]:
    """
    Raymer single-component drag buildup.
    Returns (CD0_fus, Re_L, Cf, FF).
    """
    Re_L = rho * V * L / MU_AIR
    Cf   = 0.455 / (math.log10(Re_L) ** 2.58)
    # fineness ratio for FF uses an effective diameter from S_wet/L
    d_eff = S_wet / (math.pi * L)
    f     = L / d_eff
    FF    = 1.0 + 60.0 / f**3 + f / 400.0
    CD0   = Cf * FF * Q * S_wet / S_ref
    return CD0, Re_L, Cf, FF


# ---------------------------------------------
#  Main sizing routine
# ---------------------------------------------

def size_fuselage(
    # from mass closure
    m_battery_kg:   float,
    m_payload_kg:   float,
    m_avionics_kg:  float,
    m_propulsion_kg: float,
    m_structure_kg: float,
    m_wing_kg:      float,
    m_misc_kg:      float,
    # geometry inputs
    R_hub_m:        float,   # EDF hub radius (from control vane design) [m]
    D_rotor_m:      float,   # EDF rotor diameter [m]
    c_vane_m:       float,   # vane chord [m]
    n_vanes:        int,     # vane count (control vane design) [-]
    S_vane_m2:      float,   # single vane planform area [m^2]
    hinge_xc:       float,   # vane hinge line, fraction of chord [-]
    chord_mean_m:   float,   # wing MAC [m]
    m_aileron_servo_kg:   float,  # total aileron servo mass (both) [kg]
    m_aileron_linkage_kg: float,  # total aileron linkage mass (both) [kg]
    # vibration isolation (out/vibration.yaml)
    m_isolation_avionics_kg: float,  # FC/IMU-tray isolator hardware [kg]
    m_isolation_struct_kg:   float,  # payload-mount isolator hardware [kg]
    sway_pad_m:              float,  # total extra bay length for isolator sway [m]
    # flight condition
    V_cruise:       float,
    rho:            float,
    S_wing:         float,
    # parameters
    p:              FuselageParams = None,
    mod:            ModularityParams = None,   # config/modularity.yaml (ADR-0010)
    # skin/frame construction method (WeightFraction.construction_method):
    # keys the per-method skin table in config/modularity.yaml
    construction_method: str = "segmented_fdm",
    # battery CG trim (0 = nominal, no trim)
    x_battery_trim_m: float = 0.0,
    # post-freeze re-solve (cots_integration): rigid catalog envelopes
    # flooring the bay lengths (bay name -> [(shape, dims_m), ...]),
    # and actual propulsion item masses replacing the PROP_SPLIT
    # allocation ({"motor_fan": kg, "esc": kg, "duct": kg}).  None keeps
    # the conceptual models.
    part_envelopes:   Dict[str, list] = None,
    prop_item_masses: Dict[str, float] = None,
) -> FuselageSizing:
    """
    Complete fuselage sizing.  See module docstring for the model.
    """
    if p is None:
        raise ValueError("FuselageParams required (config/fuselage.yaml)")
    if mod is None:
        raise ValueError("ModularityParams required (config/modularity.yaml)")
    if abs(x_battery_trim_m) > p.battery_tray_travel_m:
        raise ValueError(
            f"x_battery_trim_m={x_battery_trim_m*1e3:.1f} mm exceeds the "
            f"battery tray travel of +/-{p.battery_tray_travel_m*1e3:.1f} mm"
        )

    # -- 0. control hardware (vane + servo + linkage) mass model -----------
    # This hardware physically sits at the aft hinge line (recessed in the
    # exhaust centerbody -- config/fuselage.yaml), not where the top-down
    # mass-closure fractions nominally book it.  Per
    # config/initial_weight_fraction_estimation.yaml: servos are documented
    # under the AVIONICS fraction ("... + servos + wiring"); vanes and
    # linkages are control-surface/mechanical hardware closest to the
    # STRUCTURAL fraction ("wing + fuselage + booms + landing legs").  Carve
    # each piece from the budget that already documents it, not from misc.
    m_vane_each   = S_vane_m2 * p.t_vane_plate_m * p.rho_shell
    m_servo_total = n_vanes * p.servo_mass_kg
    m_vanelink_total = n_vanes * (m_vane_each + p.linkage_mass_kg)
    m_ctl_total   = m_servo_total + m_vanelink_total

    # Vibration-isolation hardware (out/vibration.yaml) is booked the same
    # way: the FC/IMU-tray isolators sit with the avionics, the payload-mount
    # isolators with the structure.
    m_avionics_budget = m_avionics_kg
    m_avionics_carved = m_servo_total + m_aileron_servo_kg + m_isolation_avionics_kg
    m_avionics_net    = m_avionics_budget - m_avionics_carved
    assert 0.0 < m_avionics_carved < MAX_HARDWARE_CARVE_FRACTION * m_avionics_budget, (
        f"vane + aileron servos + FC isolators ({m_avionics_carved*1e3:.1f} g) "
        f"take more than {MAX_HARDWARE_CARVE_FRACTION*100:.0f}% of the avionics "
        f"budget ({m_avionics_budget*1e3:.1f} g) -- the avionics "
        f"mass-fraction assumption in "
        f"config/initial_weight_fraction_estimation.yaml needs revisiting, "
        f"not a silent net-down"
    )

    m_aileron_hw   = m_aileron_servo_kg + m_aileron_linkage_kg
    m_isolation_hw = m_isolation_avionics_kg + m_isolation_struct_kg

    # Modularity joint hardware (config/modularity.yaml, ADR-0010): the
    # clamshell lid's hinge strip + latches, and the wing carry-through
    # spar tube + root fittings.  All structural hardware, carved from the
    # structural pool.  The spar tube mass is computed from geometry
    # (rectangular wing: span = S_wing / MAC), never configured.
    b_wing_m   = S_wing / chord_mean_m
    m_spar     = mod.spar_tube_mass_kg(b_wing_m)
    m_joint_hw = mod.lid_hinge_mass_kg + mod.lid_latch_mass_kg
    m_spar_hw  = m_spar + 2.0 * mod.wing_root_fitting_mass_kg

    m_struct_pool   = m_structure_kg - m_wing_kg
    m_struct_carved = (m_vanelink_total + m_aileron_linkage_kg
                       + m_isolation_struct_kg + m_joint_hw + m_spar_hw)
    assert 0.0 < m_struct_carved < MAX_HARDWARE_CARVE_FRACTION * m_struct_pool, (
        f"vanes + linkages + aileron linkages + payload isolators + joint "
        f"hardware + spar ({m_struct_carved*1e3:.1f} g) take more than "
        f"{MAX_HARDWARE_CARVE_FRACTION*100:.0f}% of the non-wing structural "
        f"pool ({m_struct_pool*1e3:.1f} g) -- the structural mass-fraction "
        f"assumption in config/initial_weight_fraction_estimation.yaml needs "
        f"revisiting, not a silent net-down"
    )

    # m_misc_kg is the top-down mass-closure fraction assumption (fasteners,
    # bonding, harness, margin only -- servos/vanes/linkages now come from
    # their own documented fractions above); the battery tray is the one
    # piece of new hardware with no better-documented home, so it alone is
    # carved from misc.  The original budget and the carve-out are both
    # kept on the returned FuselageSizing/handoff so the history of the
    # fraction assumption isn't lost -- only the *net* misc item shrinks.
    m_misc_budget = m_misc_kg
    m_misc_carved = p.battery_tray_mass_kg
    m_misc_net    = m_misc_budget - m_misc_carved
    assert 0.0 < m_misc_carved < MAX_HARDWARE_CARVE_FRACTION * m_misc_budget, (
        f"battery tray ({m_misc_carved*1e3:.1f} g) takes more than "
        f"{MAX_HARDWARE_CARVE_FRACTION*100:.0f}% of the misc budget "
        f"({m_misc_budget*1e3:.1f} g) -- the misc mass-fraction assumption "
        f"in config/initial_weight_fraction_estimation.yaml needs revisiting, "
        f"not a silent net-down"
    )

    # -- 1. volumes and diameter ---------------------------------------
    # sway_pad_m is rattle space reserved around the two soft-mounted bays
    # (payload + FC/IMU); it enlarges the required bay stack.
    vols = component_volumes(m_battery_kg, m_payload_kg, m_avionics_net, p)
    D, active = solve_diameter(vols, R_hub_m, p, sway_pad_m, part_envelopes)
    L = p.fineness_ratio * D

    L_nose = p.f_nose * L
    L_mid  = p.f_mid  * L
    L_tail = p.f_tail * L

    # -- 2. volume / wetted area ---------------------------------------
    V_tot, S_wet = _integrate_profile(D, L, p.f_nose, p.f_tail, R_hub_m)
    L_stack = _stack_length(vols, D, p, sway_pad_m, part_envelopes)
    L_avail = L_mid + 0.5 * L_nose

    # -- 3. drag ---------------------------------------------------------
    CD0_fus, Re_L, Cf, FF = fuselage_cd0(
        L, S_wet, S_wing, V_cruise, rho, p.Q_interference)

    # -- 4. structure mass (semi-monocoque, ADR-0010) ----------------------
    # Explicit member model replaces the monocoque area-density estimate
    # (m_shell = k_struct*rho*t*S_wet): thin skin + longeron frame +
    # crossbeams + half-rings, each computed from geometry. Evaluated for
    # the configured construction method AND for both skin options so the
    # print-first / carbon-later trade stays visible on every run.
    semimono = mod.semi_monocoque_masses(S_wet, D, L, construction_method)
    m_shell  = semimono["total"]     # structure estimate (was: monocoque)
    m_semimono_by_method = {
        m: mod.semi_monocoque_masses(S_wet, D, L, m)["total"]
        for m in mod.skin
    }
    m_struct_budget = m_struct_pool - m_struct_carved

    # -- 5. layout -------------------------------------------------------
    r_int = D / 2.0 - p.t_shell_m
    A_int = math.pi * r_int * r_int
    env = part_envelopes or {}

    def bay_len(name: str, v: float) -> float:
        L_vol = v / (p.packing_factor * A_int)
        L_min = _envelope_floor_m(env.get(name, []), 2.0 * r_int,
                                  p.part_clearance_m)
        return max(L_vol, L_min)

    # sway rattle space is split evenly between the two soft-mounted bays
    # (payload + FC/IMU avionics); it lengthens those bays without adding
    # packed component volume.
    bay_pad = {"payload": 0.5 * sway_pad_m, "avionics": 0.5 * sway_pad_m}

    x = 0.5 * L_nose * 0.5   # first usable station: quarter of the nose
    items: List[LayoutItem] = []
    battery_item: LayoutItem = None
    bay_item: Dict[str, LayoutItem] = {}
    for name in ("payload", "avionics", "battery"):   # nose-to-tail order
        li = LayoutItem(name, {"payload": m_payload_kg,
                               "avionics": m_avionics_net,
                               "battery": m_battery_kg}[name],
                        x_start=x, length=bay_len(name, vols[name]) + bay_pad.get(name, 0.0))
        items.append(li)
        bay_item[name] = li
        x = li.x_start + li.length
        if name == "battery":
            battery_item = li

    # battery tray: slides the packed bay +/- x_battery_trim_m on its rail
    battery_item.x_start += x_battery_trim_m

    if prop_item_masses is None:
        m_motor = PROP_SPLIT["motor_fan"] * m_propulsion_kg
        m_esc   = PROP_SPLIT["esc"]       * m_propulsion_kg
        m_duct  = PROP_SPLIT["duct"]      * m_propulsion_kg
    else:
        # post-freeze re-solve: actual motor/prop and ESC masses, duct
        # still modeled (structure booked in the propulsion fraction)
        m_motor = prop_item_masses["motor_fan"]
        m_esc   = prop_item_masses["esc"]
        m_duct  = prop_item_masses["duct"]

    duct_chord = p.duct_chord_ratio * D_rotor_m
    x_fan      = L - 0.25 * L_tail            # fan plane in the boattail
    x_duct_c   = x_fan + 0.15 * duct_chord    # duct centered around fan

    # vanes sit just aft of the duct exit; servos/linkages recess in the
    # hub with their shaft on the hinge line -- real aft-mounted hardware,
    # placed at its true station instead of the forward bays it's booked
    # against above.
    x_vane    = L + 0.5 * c_vane_m + 0.015
    x_vane_te = x_vane + 0.5 * c_vane_m
    x_hinge   = x_vane_te - (1.0 - hinge_xc) * c_vane_m

    items.append(LayoutItem("esc",        m_esc,   x_start=0.5 * L - 0.02, length=0.04))
    items.append(LayoutItem("motor_fan",  m_motor, x_start=x_fan,   length=0.0))
    items.append(LayoutItem("duct",       m_duct,  x_start=x_duct_c, length=0.0))
    items.append(LayoutItem("shell_struct", m_struct_budget,
                            x_start=0.45 * L, length=0.0))
    items.append(LayoutItem("battery_tray", p.battery_tray_mass_kg,
                            x_start=battery_item.x_cg, length=0.0))
    items.append(LayoutItem("control_hw", m_ctl_total,
                            x_start=x_hinge, length=0.0))
    # vibration isolators: FC-tray set at the avionics bay, payload set at
    # the payload bay -- placed at the mass-weighted station of the two.
    x_iso = ((m_isolation_avionics_kg * bay_item["avionics"].x_cg
              + m_isolation_struct_kg * bay_item["payload"].x_cg)
             / m_isolation_hw) if m_isolation_hw > 0 else bay_item["avionics"].x_cg
    items.append(LayoutItem("isolation_hw", m_isolation_hw,
                            x_start=x_iso, length=0.0))

    # clamshell (ADR-0010): the lid runs from the nose tip to
    # clam_aft_frac * L; the hinge strip and latches are distributed along
    # the two longerons, so the joint hardware acts at mid-lid.  The spar
    # tube + root fittings ride with the wing (added after the CG solve).
    x_clam_aft = mod.clam_aft_frac * L
    x_joint = 0.5 * x_clam_aft
    items.append(LayoutItem("joint_hw", m_joint_hw,
                            x_start=x_joint, length=0.0))

    items.append(LayoutItem("misc",       m_misc_net, x_start=0.40 * L, length=0.0))

    # -- 6. CG and wing placement (fixed point on wing position) --------
    # Ailerons and the carry-through spar are chordwise co-located with the
    # wing (mounted on/in it), so their hardware acts at the same station
    # in this fixed-point solve.
    m_nonwing = sum(it.mass_kg for it in items)
    mom_nonwing = sum(it.mass_kg * it.x_cg for it in items)
    m_wing_group = m_wing_kg + m_aileron_hw + m_spar_hw

    x_cg = mom_nonwing / m_nonwing            # initial guess: no wing
    for _ in range(5):
        x_wing_cg = x_cg                      # wing group acts at ~its own AC
        x_cg = (mom_nonwing + m_wing_group * x_wing_cg) / (m_nonwing + m_wing_group)

    x_ac      = x_cg + p.static_margin * chord_mean_m
    x_wing_le = x_ac - 0.25 * chord_mean_m

    items.append(LayoutItem("wing", m_wing_kg, x_start=x_cg, length=0.0))
    items.append(LayoutItem("aileron_hw", m_aileron_hw, x_start=x_cg, length=0.0))
    items.append(LayoutItem("spar_hw", m_spar_hw, x_start=x_cg, length=0.0))

    # -- 7. vane arm cross-check -----------------------------------------
    L_arm  = abs(x_vane - x_cg)

    D_duct_inner = D_rotor_m + 2.0 * p.tip_clearance_m
    D_duct_outer = D_duct_inner + 2.0 * p.t_duct_m

    return FuselageSizing(
        D_fus=D, L_fus=L, L_nose=L_nose, L_mid=L_mid, L_tail=L_tail,
        r_hub=R_hub_m, fineness=L / D, active_constraint=active,
        V_total_m3=V_tot, V_bays_m3=sum(vols.values()),
        L_stack_m=L_stack, L_avail_m=L_avail,
        S_wet_m2=S_wet, Re_L=Re_L, Cf=Cf, FF=FF, CD0_fus=CD0_fus,
        m_shell_kg=m_shell,
        m_struct_pool_kg=m_struct_pool, m_struct_carved_kg=m_struct_carved,
        m_struct_budget_kg=m_struct_budget,
        m_avionics_budget_kg=m_avionics_budget, m_avionics_carved_kg=m_avionics_carved,
        m_avionics_net_kg=m_avionics_net,
        m_misc_budget_kg=m_misc_budget, m_misc_carved_kg=m_misc_carved,
        m_misc_net_kg=m_misc_net,
        items=items, x_CG=x_cg, x_wing_LE=x_wing_le, x_wing_AC=x_ac,
        x_vane=x_vane, L_vane_arm=L_arm,
        x_battery_trim_m=x_battery_trim_m,
        battery_tray_travel_m=p.battery_tray_travel_m,
        sway_pad_m=sway_pad_m, m_isolation_hw_kg=m_isolation_hw,
        x_clam_aft_m=x_clam_aft,
        frame_profile_side_m=mod.frame_profile_side_m,
        n_crossbeams=mod.n_crossbeams,
        construction_method=construction_method,
        semimono_members=semimono,
        m_semimono_fdm_kg=m_semimono_by_method.get("segmented_fdm", 0.0),
        m_semimono_cfrp_kg=m_semimono_by_method.get("cfrp_2ply", 0.0),
        spar_od_m=mod.spar_od_m, spar_chord_frac=mod.spar_chord_frac,
        spar_length_m=mod.spar_length_m(b_wing_m),
        m_spar_kg=m_spar, m_joint_hw_kg=m_joint_hw, m_spar_hw_kg=m_spar_hw,
        duct_chord=duct_chord,
        D_duct_inner=D_duct_inner, D_duct_outer=D_duct_outer,
    )


# ---------------------------------------------
#  Handoff writer
# ---------------------------------------------

def write_fuselage_yaml(fus: FuselageSizing, p: FuselageParams, path,
                        regen_notebook: str = "notebooks/fuselage_design.ipynb",
                        extra: dict = None) -> None:
    """
    Write out/fuselage.yaml -- the handoff consumed by the CAD notebook.
    Stations are from the nose tip, +aft;  x_body = -station  (FRD).

    The post-freeze re-solve reuses this writer for out/fuselage_cots.yaml
    (same schema) with its own `regen_notebook` provenance and an `extra`
    block (fit report, deltas vs the conceptual solution) appended.
    """
    data = {
        "axis_convention": "body FRD (x fwd, y right, z down); stations from nose, +aft; x_body = -station",
        # shape
        "D_fus_m":        round(fus.D_fus, 5),
        "L_fus_m":        round(fus.L_fus, 5),
        "L_nose_m":       round(fus.L_nose, 5),
        "L_mid_m":        round(fus.L_mid, 5),
        "L_tail_m":       round(fus.L_tail, 5),
        "r_hub_m":        round(fus.r_hub, 5),
        "f_nose":         round(p.f_nose, 4),
        "f_tail":         round(p.f_tail, 4),
        "fineness":       round(fus.fineness, 3),
        "active_constraint": fus.active_constraint,
        # aero
        "S_wet_m2":       round(fus.S_wet_m2, 5),
        "CD0_fus":        round(fus.CD0_fus, 5),
        # structure (semi-monocoque, ADR-0010): m_shell_kg is the explicit
        # member-model total for the configured construction method;
        # t_shell_m remains the PACKAGING wall (r_int), decoupled from the
        # structural skin thickness at conceptual stage
        "m_shell_kg":     round(fus.m_shell_kg, 4),
        "t_shell_m":      p.t_shell_m,
        "construction_method": fus.construction_method,
        "semimono_members_kg": {k: round(v, 5)
                                for k, v in fus.semimono_members.items()},
        "m_semimono_fdm_kg":  round(fus.m_semimono_fdm_kg, 5),
        "m_semimono_cfrp_kg": round(fus.m_semimono_cfrp_kg, 5),
        # mass-fraction allocation traceability: top-down budget (from
        # config/initial_weight_fraction_estimation.yaml) vs. the named
        # hardware carved out of each, per its documented fraction scope
        "m_struct_pool_kg":   round(fus.m_struct_pool_kg, 5),
        "m_struct_carved_kg": round(fus.m_struct_carved_kg, 5),
        "m_avionics_budget_kg": round(fus.m_avionics_budget_kg, 5),
        "m_avionics_carved_kg": round(fus.m_avionics_carved_kg, 5),
        "m_avionics_net_kg":    round(fus.m_avionics_net_kg, 5),
        "m_misc_budget_kg": round(fus.m_misc_budget_kg, 5),
        "m_misc_carved_kg": round(fus.m_misc_carved_kg, 5),
        "m_misc_net_kg":     round(fus.m_misc_net_kg, 5),
        # balance
        "x_CG_m":         round(fus.x_CG, 5),
        "x_wing_LE_m":    round(fus.x_wing_LE, 5),
        "x_wing_AC_m":    round(fus.x_wing_AC, 5),
        "x_vane_m":       round(fus.x_vane, 5),
        "L_vane_arm_m":   round(fus.L_vane_arm, 5),
        "static_margin":  p.static_margin,
        # battery CG trim
        "x_battery_trim_m":      round(fus.x_battery_trim_m, 5),
        "battery_tray_travel_m": p.battery_tray_travel_m,
        # vibration-isolation packing cost
        "sway_pad_m":         round(fus.sway_pad_m, 5),
        "m_isolation_hw_kg":  round(fus.m_isolation_hw_kg, 5),
        # clamshell + joint hardware (ADR-0010)
        "x_clam_aft_m":    round(fus.x_clam_aft_m, 5),
        "frame_profile_side_m": fus.frame_profile_side_m,
        "n_crossbeams":    fus.n_crossbeams,
        "spar_od_m":       fus.spar_od_m,
        "spar_chord_frac": fus.spar_chord_frac,
        "spar_length_m":   round(fus.spar_length_m, 5),
        "m_spar_kg":       round(fus.m_spar_kg, 5),
        "m_joint_hw_kg":   round(fus.m_joint_hw_kg, 5),
        "m_spar_hw_kg":    round(fus.m_spar_hw_kg, 5),
        # duct
        "duct_chord_m":   round(fus.duct_chord, 5),
        "D_duct_inner_m": round(fus.D_duct_inner, 5),
        "D_duct_outer_m": round(fus.D_duct_outer, 5),
        # layout table
        "layout": [
            {"name": it.name, "mass_kg": round(it.mass_kg, 4),
             "x_start_m": round(it.x_start, 5), "length_m": round(it.length, 5),
             "x_cg_m": round(it.x_cg, 5)}
            for it in fus.items
        ],
    }
    if extra:
        data.update(extra)
    with open(path, "w", encoding="utf-8") as f:
        f.write("# AUTO-GENERATED -- do not edit by hand.\n")
        f.write("# Source : src/conceptual_design/fuselage_design.py\n")
        f.write("# Input  : config/fuselage.yaml + mass closure + control vanes\n")
        f.write(f"# Regen  : re-run {regen_notebook}\n\n")
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)


def fineness_sweep(fus_kwargs: dict, p: FuselageParams, FR_values):
    """Re-size the fuselage across a fineness-ratio range (drag trade).

    ``fus_kwargs`` is the full ``size_fuselage`` keyword set minus ``p``;
    returns (CD0_fus, m_shell_kg) lists over ``FR_values``.
    """
    from dataclasses import replace

    cd0_sweep, mshell_sweep = [], []
    for fr in FR_values:
        f_i = size_fuselage(**fus_kwargs, p=replace(p, fineness_ratio=float(fr)))
        cd0_sweep.append(f_i.CD0_fus)
        mshell_sweep.append(f_i.m_shell_kg)
    return cd0_sweep, mshell_sweep
