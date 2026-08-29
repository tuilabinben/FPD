"""Inverse / forward kinematics for STCR4000S twin frog-leg arm.

Direct port of `solve_ik_frogleg` from MATLAB_v4_final/mophong_init.m. MATLAB helper:

    function [d1, th2, th3_cad] = solve_ik_frogleg(X, Y, Z, Z_offset, a3, a4, a5, a6)
        d1 = Z - Z_offset;
        if d1 < 0, d1 = 0; elseif d1 > 285, d1 = 285; end
        th2 = atan2(Y, X);
        R = sqrt(X^2 + Y^2);
        cos_th3_math = (R - (a3 + a6)) / (a4 + a5);
        ... clamp to [-1, 1] ...
        th3_math = acos(cos_th3_math);
        th3_cad  = pi - th3_math;
    end

Two deliberate diffs from MATLAB original, both about safety not maths:

1. MATLAB *clamps* silently — out-of-range Z or unreachable radius quietly becomes nearest
   legal pose. Fine for plotting script; not for machine — operator would press RUN believing
   point accepted. Here same conditions raise ValueError, GUI refuses LOAD.
   `clamp_like_matlab=True` restores original clamping for A/B comparison.

2. Per-arm Z offsets honoured: arm 2's deck sits 9mm below arm 1's, same Cartesian Z maps to
   different d1 per arm.

Pure maths — no tkinter, no serial. Safe to unit-test in isolation.
"""

import math

from .config import (
    ARM2_Z_DROP_MM,
    ARM_GEAR_RATIO,
    ARM_LINK_SUM_MM,
    ARM_RADIAL_OFFSET_MM,
    D1_MAX_MM,
    D1_MIN_MM,
    ARM_ZERO_CAD_DEG,
    FOLD_ANGLE_HOME_DEG,
    FOLD_ANGLE_MAX_DEG,
    FOLD_ANGLE_MIN_DEG,
    FOLD_ANGLE_SPEC_MAX_DEG,
    FOLD_ANGLE_SINGULARITY_WARN_DEG,
    ROT_MAX_DEG,
    ROT_MIN_DEG,
    Z_OFFSET_ARM1_MM,
    Z_OFFSET_ARM2_MM,
)

EPS = 1e-6

#: Cartesian Z offset applied per arm, keyed by the P2P arm selector.
Z_OFFSET_BY_ARM = {"A1M": Z_OFFSET_ARM1_MM, "A2M": Z_OFFSET_ARM2_MM}


def normalize_angle(deg):
    """Wraps an angle into (-180, 180]. For angular DIFFERENCES only — for
    an RM position use rot_from_bearing()."""
    a = (deg + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


def rot_from_bearing(deg):
    """Compass bearing (any range) -> RM reading in [0, 360).

    RM zero = CCW stop, counts up to 340 — never negative. atan2 returns
    (-180,180], would report point just CW of home as -5, refused as
    "below CCW limit" though actually 355 and reachable.
    """
    return deg % 360.0


def clamp(value, low, high):
    return max(low, min(high, value))


def z_offset_for(arm):
    """Absolute end-effector height of `arm` when the lift is at d1 = 0."""
    try:
        return Z_OFFSET_BY_ARM[arm]
    except KeyError:
        raise ValueError(f"arm must be 'A1M' or 'A2M', got {arm!r}") from None


# ----------------------------------------------------------------------
# Operator's Z frame: 0 AT HOME
# ----------------------------------------------------------------------
# HOME = P2P reference point — X=0, Y=0, Z=0. Typed Z = lift travel up
# from it, 0..285mm. X/Y signed (turntable can put arm behind machine);
# Z never negative (HOME = bottom of stroke, nothing below it).
#
# Maths underneath unchanged, still absolute: solve_ik(),
# forward_kinematics(), MATLAB parity sweep all work in frame where arm 1
# deck sits at 514.3mm w/ lift down — deliberate, sweep proves module
# still reproduces mophong_init.m to machine precision, only works
# speaking .m's frame. Translation lives HERE, at edge, applied when
# number crosses operator <-> maths.
#
# Z_HOME per arm: one carriage, two decks 9mm apart — same typed Z is
# different absolute height for A1M vs A2M.
def z_abs_from_home(z_home_mm, arm):
    """Operator Z (0 at HOME, up positive) -> absolute model Z for `arm`."""
    return z_home_mm + z_offset_for(arm)


def z_home_from_abs(z_abs_mm, arm):
    """Absolute model Z -> operator Z. Inverse of the above."""
    return z_abs_mm - z_offset_for(arm)


# ----------------------------------------------------------------------
# Elbow angle <-> radial reach
# ----------------------------------------------------------------------
# Only place CAD frame appears. Callers work in degrees-rotated-from-home
# (0° retracted); +/- ARM_ZERO_CAD_DEG here is the whole translation.
def fold_angle_to_reach(fold_deg):
    """Elbow rotation from home (deg) -> radial reach of the wafer centre.

        R = A3 + A6 - (A4 + A5) * cos(fold + ARM_ZERO_CAD_DEG)

    0° -> 133.2 mm (retracted home), 120° -> 613.2 mm (straight arm).
    """
    return ARM_RADIAL_OFFSET_MM - ARM_LINK_SUM_MM * math.cos(
        math.radians(fold_deg + ARM_ZERO_CAD_DEG))


def reach_to_fold_angle(r_mm):
    """Radial reach (mm) -> elbow rotation from home. Inverse of the above."""
    cos_th3_math = clamp((r_mm - ARM_RADIAL_OFFSET_MM) / ARM_LINK_SUM_MM, -1.0, 1.0)
    return 180.0 - math.degrees(math.acos(cos_th3_math)) - ARM_ZERO_CAD_DEG


# ----------------------------------------------------------------------
# Motor degrees <-> frog-leg degrees
# ----------------------------------------------------------------------
# Board counts step pulses — MOTOR degrees only elbow figure it knows
# exactly. Every frog-leg angle here derived from it via these two fns
# only, so ARM_GEAR_RATIO recalibration can't be half-applied.
#
# ARM_GEAR_RATIO is MEASURED, 7.80 — see config.py for the reach figures it
# came from. The Simscape model's 2 describes the linkage, not the gearbox
# in front of it, and does not match the machine.
def fold_angle_from_motor_deg(motor_deg):
    """Motor rotation from home (deg) -> frog-leg rotation from home."""
    return motor_deg / ARM_GEAR_RATIO


def motor_deg_from_fold_angle(fold_deg):
    """Frog-leg rotation from home (deg) -> motor rotation from home."""
    return fold_deg * ARM_GEAR_RATIO


# ----------------------------------------------------------------------
# Arm BASE angle — number operator reads off machine
# ----------------------------------------------------------------------
# Base (shoulder) link swings 0deg at HOME (240mm reach) to 90deg at rated
# working reach 575mm. Scale anchored on FOLD_ANGLE_SPEC_MAX_DEG
# (146.68deg fold = 575mm reach = 90deg base) not geometric singularity at
# 180deg — singularity never valid target, anchoring there would make
# "straight out" read ~73deg instead of the 90 operator sees on machine.
#
# DISPLAY ONLY. Wire protocol, taught boundaries, everything board stores
# stay MOTOR degrees — see CLAUDE.md section 1b.
BASE_ANGLE_MAX_DEG = 90.0


def base_angle_from_fold_angle(fold_deg):
    """Frog-leg rotation from home (deg) -> arm base angle (deg).

    0 deg fold -> 0 deg base (240 mm, retracted home).
    FOLD_ANGLE_SPEC_MAX_DEG -> 90 deg base (575 mm, rated working reach).
    """
    return fold_deg * (BASE_ANGLE_MAX_DEG / FOLD_ANGLE_SPEC_MAX_DEG)


def fold_angle_from_base_angle(base_deg):
    """Arm base angle (deg) -> frog-leg rotation from home."""
    return base_deg * (FOLD_ANGLE_SPEC_MAX_DEG / BASE_ANGLE_MAX_DEG)


def base_angle_from_motor_deg(motor_deg):
    """Motor rotation from home (deg) -> arm base angle (deg)."""
    return base_angle_from_fold_angle(fold_angle_from_motor_deg(motor_deg))


def motor_deg_from_base_angle(base_deg):
    """Arm base angle (deg) -> motor rotation from home."""
    return motor_deg_from_fold_angle(fold_angle_from_base_angle(base_deg))


def motor_deg_to_reach(motor_deg):
    """Motor rotation from home (deg) -> radial reach of the wafer centre."""
    return fold_angle_to_reach(fold_angle_from_motor_deg(motor_deg))


def reach_to_motor_deg(r_mm):
    """Radial reach (mm) -> motor rotation from home."""
    return motor_deg_from_fold_angle(reach_to_fold_angle(r_mm))


def is_near_singularity(fold_deg):
    """True once the frog-leg is close enough to straight that radial
    stiffness is collapsing. Advisory only — never blocks a move."""
    return fold_deg >= FOLD_ANGLE_SINGULARITY_WARN_DEG


# ----------------------------------------------------------------------
# Range checks
# ----------------------------------------------------------------------
def _check_rotation(theta2):
    if not (ROT_MIN_DEG - EPS <= theta2 <= ROT_MAX_DEG + EPS):
        raise ValueError(
            f"Rotation (RM) = {theta2:.1f}° is outside the real travel "
            f"[{ROT_MIN_DEG:.0f}°, {ROT_MAX_DEG:.0f}°]. RM sweeps 340° from its "
            f"CCW stop (0° = HOME) to its CW stop, so the 20° wedge between "
            f"{ROT_MAX_DEG:.0f}° and 360° cannot be reached from either side."
        )


#: Radii cosine can actually invert: |(R - 293.2) / 320| <= 1.
#: Outside this NO frog-leg configuration exists, any elbow angle, any
#: machine — acos would clamp, hand back pose not the one asked for,
#: which is MATLAB behaviour this module deliberately doesn't copy.
REACH_SOLVABLE_MIN_MM = ARM_RADIAL_OFFSET_MM - ARM_LINK_SUM_MM   # -26.8
REACH_SOLVABLE_MAX_MM = ARM_RADIAL_OFFSET_MM + ARM_LINK_SUM_MM   # 613.2


def _check_reach(r, label="Point"):
    """Refuses only radii geometry cannot solve AT ALL.

    NO STRUCTURAL REACH ENVELOPE HERE ANY MORE — same decision already
    taken for elbow boundaries (ARM_LIMITS_UNBOUNDED in config.py). Old
    133.2mm floor was R(fold=0°) — it assumed the elbow's zero really is
    the folded home pose. ARM_GEAR_RATIO is measured now, but that one is
    still an assumption, so the floor was still a guess — one that rejects
    a radius the arm is physically standing at, stopping the operator
    using the machine at all.

    Working envelope is operator's, from taught elbow boundaries in
    Settings -> Boundaries, enforced where those live: _limit_violation()
    before LOAD, armBand()/reachBandFor() on board. One system of record,
    not two.

    What survives is arithmetic, not opinion: radius beyond 613.2mm has no
    solution for any elbow angle, acos() would silently clamp it.
    """
    if r < REACH_SOLVABLE_MIN_MM - EPS or r > REACH_SOLVABLE_MAX_MM + EPS:
        raise ValueError(
            f"{label} has no solution: r={r:.1f} mm. The frog-leg spans "
            f"a3+a6 ± (a4+a5) = {ARM_RADIAL_OFFSET_MM:.1f} ± "
            f"{ARM_LINK_SUM_MM:.1f} mm, so no elbow angle whatsoever reaches "
            f"it. Your own working limits are separate — see "
            f"Settings → Boundaries."
        )


def reach_band_from_motor_deg(motor_lo, motor_hi):
    """(min, max) radius from taught elbow band, in MOTOR degrees.

    NOT min/max of two endpoints. Reach = cosine of `fold + 60`,
    monotonic only across half period: once band crosses extreme, extreme
    radius lies INSIDE interval — endpoint-only answer wrong in unsafe
    direction, reports narrower band than arm can actually sweep.
    Extremes sit where `fold + 60` is multiple of 180, i.e. fold = 120,
    300, -60, ...

    Mirrors reachBandFor() in firmware; both must agree or panel would
    advertise band board refuses.
    """
    lo_f = fold_angle_from_motor_deg(min(motor_lo, motor_hi))
    hi_f = fold_angle_from_motor_deg(max(motor_lo, motor_hi))
    radii = [fold_angle_to_reach(lo_f), fold_angle_to_reach(hi_f)]

    # Every fold angle in [lo_f, hi_f] where cos(fold + 60) is +/-1.
    k = math.floor((lo_f + ARM_ZERO_CAD_DEG) / 180.0)
    while True:
        extreme = 180.0 * k - ARM_ZERO_CAD_DEG
        if extreme > hi_f:
            break
        if extreme >= lo_f:
            radii.append(fold_angle_to_reach(extreme))
        k += 1
    return min(radii), max(radii)


def _check_d1(d1, arm, z):
    if not (D1_MIN_MM - EPS <= d1 <= D1_MAX_MM + EPS):
        # d1 IS operator's Z now — lift's travel up from HOME — message
        # leads with that, absolute height only as context. Old message
        # quoted 514.3..799.3 band nobody types anymore, read as though
        # entry rejected for being far too small.
        raise ValueError(
            f"Z={d1:.1f} mm is outside the lift's travel from HOME "
            f"(valid [{D1_MIN_MM:.0f}, {D1_MAX_MM:.0f}] mm; Z is measured UP "
            f"from HOME, so it is never negative). "
            f"That would put {arm}'s deck at an absolute {z:.1f} mm."
        )


# ----------------------------------------------------------------------
# Inverse kinematics
# ----------------------------------------------------------------------
def solve_ik(x, y, z, arm_choice="A1M", reference_deg=0.0, clamp_like_matlab=False,
             idle_deg=None):
    """Inverse kinematics for single-arm move.

    Model: prismatic Z lift (d1) + rotating base (theta2, RM) + one of two
    independent frog-leg arms (A1M or A2M). `arm_choice` selects arm that
    moves.

    `idle_deg` = elbow angle returned for arm NOT moving. Pass its current
    live angle to leave it exactly where it is — default None means "park
    at CAD home angle", what firmware's own IK does but physically MOVES
    an arm operator never commanded. GUI passes live angle for that
    reason.

    Returns (d1_mm, theta2_deg, theta_a1m_deg, theta_a2m_deg), arm angles
    as rotation from home (0° retracted .. 120° straight).

    `clamp_like_matlab=True` reproduces MATLAB helper's silent clamping
    exactly instead of raising.
    """
    z_offset = z_offset_for(arm_choice)

    d1 = z - z_offset
    if clamp_like_matlab:
        d1 = clamp(d1, D1_MIN_MM, D1_MAX_MM)
    else:
        _check_d1(d1, arm_choice, z)

    r = math.hypot(x, y)
    theta2 = (reference_deg if r < EPS
              else rot_from_bearing(math.degrees(math.atan2(y, x))))

    if not clamp_like_matlab:
        _check_rotation(theta2)
        _check_reach(r, f"Point ({x:.1f},{y:.1f})")

    theta_active = reach_to_fold_angle(r)

    # Idle arm's angle passed through UNCLAMPED. Measured position, not
    # request: clamping to 0..120 would hand back target different from
    # where arm stands — command move on arm operator didn't select,
    # exact failure idle_deg exists to prevent. A reading outside 0..120
    # is expected in any case, not a fault.
    idle = FOLD_ANGLE_HOME_DEG if idle_deg is None else idle_deg

    theta_a1m = theta_active if arm_choice == "A1M" else idle
    theta_a2m = theta_active if arm_choice == "A2M" else idle

    return d1, theta2, theta_a1m, theta_a2m


def solve_ik_both(x0, y0, z0, x1, y1, z1, reference_deg=0.0):
    """Inverse kinematics for SIMULTANEOUS dual-arm move: A1M reaches
    (x0,y0,z0) while A2M reaches (x1,y1,z1).

    RM and ZM carriage shared, each arm's elbow has own motor — two
    points must lie same direction from centre (only radius may differ).
    Cartesian Z values must differ by exactly 9mm deck offset, since one
    carriage lifts both decks.

    Returns (d1_mm, theta2_deg, theta_a1m_deg, theta_a2m_deg).
    """
    # Arm 2's deck ARM2_Z_DROP_MM lower — one carriage position, two
    # end-effectors at Z and Z-9.
    expected_dz = ARM2_Z_DROP_MM
    if abs((z0 - z1) - expected_dz) > 0.5:
        raise ValueError(
            f"Both arms share one ZM carriage and arm 2's deck sits "
            f"{expected_dz:.1f} mm lower, so Z0 - Z1 must equal {expected_dz:.1f} mm. "
            f"Got Z0={z0:.1f}, Z1={z1:.1f} (difference {z0 - z1:.1f} mm)."
        )

    d1 = z0 - Z_OFFSET_ARM1_MM
    _check_d1(d1, "A1M", z0)

    r0, r1 = math.hypot(x0, y0), math.hypot(x1, y1)

    if r0 < EPS and r1 < EPS:
        rot = reference_deg
    elif r0 < EPS:
        rot = rot_from_bearing(math.degrees(math.atan2(y1, x1)))
    elif r1 < EPS:
        rot = rot_from_bearing(math.degrees(math.atan2(y0, x0)))
    else:
        principal0 = rot_from_bearing(math.degrees(math.atan2(y0, x0)))
        principal1 = rot_from_bearing(math.degrees(math.atan2(y1, x1)))
        diff = abs(normalize_angle(principal1 - principal0))
        if diff > 1.0:
            raise ValueError(
                f"The two points are not on the same bearing. Both arms share RM, "
                f"so they must be at the same angle and differ only in radius: "
                f"point 1 at {principal0:.1f}°, point 2 at {principal1:.1f}° "
                f"(off by {diff:.1f}°, needs 0°)."
            )
        rot = principal0

    _check_rotation(rot)
    _check_reach(r0, "Point 1 (A1M)")
    _check_reach(r1, "Point 2 (A2M)")

    return d1, rot, reach_to_fold_angle(r0), reach_to_fold_angle(r1)


# ----------------------------------------------------------------------
# Forward kinematics
# ----------------------------------------------------------------------
def forward_kinematics(d1, theta2, theta_a1m, theta_a2m, arm=None):
    """Joint values -> Cartesian (x, y, z) mm of selected arm's wafer
    centre. `arm` = "A1M" or "A2M"; omitted -> more-extended arm (larger
    th3_cad) reported, since idle arm sits at home.
    """
    if arm == "A1M":
        active_theta, z_offset = theta_a1m, Z_OFFSET_ARM1_MM
    elif arm == "A2M":
        active_theta, z_offset = theta_a2m, Z_OFFSET_ARM2_MM
    else:
        if theta_a1m >= theta_a2m:
            active_theta, z_offset = theta_a1m, Z_OFFSET_ARM1_MM
        else:
            active_theta, z_offset = theta_a2m, Z_OFFSET_ARM2_MM

    r = fold_angle_to_reach(active_theta)
    rad = math.radians(theta2)
    return r * math.cos(rad), r * math.sin(rad), d1 + z_offset


def home_pose():
    """(d1, theta2, th3_a1m, th3_a2m) of the retracted CAD home pose."""
    return D1_MIN_MM, 0.0, FOLD_ANGLE_HOME_DEG, FOLD_ANGLE_HOME_DEG


def sample_joint_path(start_j, target_j, arm=None, n=30):
    """(x, y) mm polyline for linear joint-space move start_j -> target_j,
    both (d1, rot, a1_motor_deg, a2_motor_deg).

    Uses SAME per-joint linear interpolation real machine driven with
    (p2p_control._animate_leg: `s + (e - s) * t`) — actual swept path,
    not approximation. Motor degrees converted to fold degrees per sample
    since forward_kinematics works in fold frame.
    """
    pts = []
    for i in range(n + 1):
        t = i / n
        d1, rot, a1, a2 = (s + (e - s) * t for s, e in zip(start_j, target_j))
        x, y, _z = forward_kinematics(d1, rot, fold_angle_from_motor_deg(a1),
                                      fold_angle_from_motor_deg(a2), arm=arm)
        pts.append((x, y))
    return pts


# Kept so callers that only need the elbow span don't import config.
FOLD_ANGLE_RANGE_DEG = (FOLD_ANGLE_MIN_DEG, FOLD_ANGLE_MAX_DEG)
