"""Inverse / forward kinematics for the STCR4000S twin frog-leg arm.

This is a direct port of `solve_ik_frogleg` from
MATLAB_v4_final/mophong_init.m. The MATLAB helper is:

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

Two deliberate differences from the MATLAB original, both about safety
rather than maths:

1.  MATLAB *clamps* silently — an out-of-range Z or an unreachable
    radius quietly becomes the nearest legal pose. A plotting script can
    afford that; a machine cannot, because the operator would press RUN
    believing the commanded point was accepted. Here the same conditions
    raise ValueError so the GUI refuses the LOAD. `clamp_like_matlab=True`
    restores the original clamping behaviour for A/B comparison.

2.  Per-arm Z offsets are honoured: arm 2's deck sits 9 mm below arm 1's,
    so the same Cartesian Z maps to a different d1 for each arm.

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
    """Compass bearing (any range) -> an RM reading in [0, 360).

    RM's zero is its CCW stop and it counts up to 340, so a position is
    never negative. atan2 returns (-180, 180], which would report a point
    just clockwise of home as -5 and get it refused as "below the CCW
    limit" when it is in fact 355 and perfectly reachable.
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
# The operator's Z frame: 0 AT HOME
# ----------------------------------------------------------------------
# HOME is the P2P reference point — X = 0, Y = 0, Z = 0 — and the typed Z
# is the lift's travel up from it, 0..285 mm. X and Y stay signed, because
# the turntable can put the arm behind the machine; Z cannot go negative,
# because HOME is the bottom of the stroke and there is nothing below it.
#
# The maths underneath is unchanged and still absolute: solve_ik(),
# forward_kinematics() and the MATLAB parity sweep all work in the frame
# where the arm 1 deck sits at 514.3 mm with the lift down. That is
# deliberate — the sweep proves this module still reproduces
# mophong_init.m to machine precision, and it can only do that if it is
# speaking the .m's frame. So the translation lives HERE, at the edge, and
# is applied when a number crosses between the operator and the maths.
#
# Z_HOME is therefore per arm: one carriage, two decks 9 mm apart, so the
# same typed Z is a different absolute height for A1M and A2M.
def z_abs_from_home(z_home_mm, arm):
    """Operator Z (0 at HOME, up positive) -> absolute model Z for `arm`."""
    return z_home_mm + z_offset_for(arm)


def z_home_from_abs(z_abs_mm, arm):
    """Absolute model Z -> operator Z. Inverse of the above."""
    return z_abs_mm - z_offset_for(arm)


# ----------------------------------------------------------------------
# Elbow angle <-> radial reach
# ----------------------------------------------------------------------
# These two functions are the ONLY place the CAD frame appears. Callers
# work in degrees-rotated-from-home (0° retracted); the +/- of
# ARM_ZERO_CAD_DEG here is the whole of the translation.
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
# The board counts step pulses, so MOTOR degrees is the only elbow figure
# it knows exactly. Every frog-leg angle in this module is derived from it
# through these two functions and nothing else, so a re-calibration of
# ARM_GEAR_RATIO cannot be half-applied.
#
# The ratio is 2 because the elbow motor drives the KNEE while the reach
# curve is written in terms of the driven LINK angle, and the knee turns
# through twice the link angle — see the derivation in config.py.
def fold_angle_from_motor_deg(motor_deg):
    """Motor rotation from home (deg) -> frog-leg rotation from home."""
    return motor_deg / ARM_GEAR_RATIO


def motor_deg_from_fold_angle(fold_deg):
    """Frog-leg rotation from home (deg) -> motor rotation from home."""
    return fold_deg * ARM_GEAR_RATIO


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


#: The radii the cosine can actually invert: |(R - 293.2) / 320| <= 1.
#: Outside this there is NO frog-leg configuration at all, at any elbow
#: angle, on any machine — acos would clamp and hand back a pose that is
#: not the one asked for, which is the MATLAB behaviour this module
#: deliberately does not copy.
REACH_SOLVABLE_MIN_MM = ARM_RADIAL_OFFSET_MM - ARM_LINK_SUM_MM   # -26.8
REACH_SOLVABLE_MAX_MM = ARM_RADIAL_OFFSET_MM + ARM_LINK_SUM_MM   # 613.2


def _check_reach(r, label="Point"):
    """Refuses only radii the geometry cannot solve AT ALL.

    THERE IS NO STRUCTURAL REACH ENVELOPE HERE ANY MORE, and that is the
    same decision already taken for the elbow boundaries themselves
    (ARM_LIMITS_UNBOUNDED in config.py). The old floor of 133.2 mm was
    R(fold = 0°) — it assumed the elbow's zero really is the folded home
    pose, and that the fold angle is motor degrees over an ARM_GEAR_RATIO
    that has never been measured. Both are assumptions, so the floor was a
    guess, and a guess that rejects a radius the arm is physically
    standing at stops the operator using the machine at all.

    The working envelope is the operator's, from the taught elbow
    boundaries in Settings -> Boundaries, and it is enforced where those
    live: _limit_violation() before LOAD, and armBand()/reachBandFor() on
    the board. That is one system of record instead of two.

    What survives is arithmetic, not opinion: a radius beyond 613.2 mm has
    no solution for any elbow angle, and acos() would silently clamp it.
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
    """(min, max) radius produced by a taught elbow band, in MOTOR degrees.

    NOT min/max of the two endpoints. Reach is a cosine of `fold + 60`, so
    it is monotonic only across half a period: once a band crosses an
    extreme, the extreme radius lies INSIDE the interval and an
    endpoint-only answer is wrong in the unsafe direction — it reports a
    narrower band than the arm can actually sweep. The extremes sit where
    `fold + 60` is a multiple of 180, i.e. fold = 120, 300, -60, ...

    This mirrors reachBandFor() in the firmware; the two must agree or the
    panel would advertise a band the board refuses.
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
        # d1 IS the operator's Z now — the lift's travel up from HOME — so
        # the message leads with that and mentions the absolute height only
        # as context. The old message quoted a 514.3..799.3 band nobody
        # types any more, which read as though the entry had been rejected
        # for being far too small.
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
    """Inverse kinematics for a single-arm move.

    Model: prismatic Z lift (d1) + rotating base (theta2, RM) + one of two
    independent frog-leg arms (A1M or A2M). `arm_choice` selects the arm
    that makes the move.

    `idle_deg` is the elbow angle returned for the arm that is NOT moving.
    Pass its current live angle to leave it exactly where it is — the
    default of None means "park at the CAD home angle", which is what the
    firmware's own IK does but which physically MOVES an arm the operator
    never commanded. The GUI passes the live angle for that reason.

    Returns (d1_mm, theta2_deg, theta_a1m_deg, theta_a2m_deg), with the
    arm angles expressed as rotation from home (0° retracted .. 120°
    straight).

    With `clamp_like_matlab=True` the MATLAB helper's silent clamping is
    reproduced exactly instead of raising.
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

    # The idle arm's angle is passed through UNCLAMPED. It is a measured
    # position, not a request: clamping it to 0..120 would hand back a
    # target different from where the arm is standing, i.e. would command
    # a move on the arm the operator did not select — the exact failure
    # the idle_deg parameter exists to prevent. The reported angle rides
    # on an unmeasured ARM_GEAR_RATIO, so reading outside 0..120 is
    # expected, not a fault.
    idle = FOLD_ANGLE_HOME_DEG if idle_deg is None else idle_deg

    theta_a1m = theta_active if arm_choice == "A1M" else idle
    theta_a2m = theta_active if arm_choice == "A2M" else idle

    return d1, theta2, theta_a1m, theta_a2m


def solve_ik_both(x0, y0, z0, x1, y1, z1, reference_deg=0.0):
    """Inverse kinematics for a SIMULTANEOUS dual-arm move: A1M reaches
    (x0,y0,z0) while A2M reaches (x1,y1,z1).

    RM and the ZM carriage are shared while each arm's elbow has its own
    motor, so the two points must lie in the same direction from the
    centre (only the radius may differ). Their Cartesian Z values must
    differ by exactly the 9 mm deck offset, because a single carriage
    lifts both decks.

    Returns (d1_mm, theta2_deg, theta_a1m_deg, theta_a2m_deg).
    """
    # Arm 2's deck is ARM2_Z_DROP_MM lower, so for one carriage position
    # the two end-effectors are at Z and Z - 9.
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
    """Joint values -> Cartesian (x, y, z) mm of the selected arm's wafer
    centre. `arm` is "A1M" or "A2M"; when omitted the more-extended arm
    (larger th3_cad) is reported, since the idle arm sits at home.
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
    """(x, y) mm polyline for a linear joint-space move from start_j to
    target_j, both (d1, rot, a1_motor_deg, a2_motor_deg).

    Uses the SAME per-joint linear interpolation the real machine is
    driven with (p2p_control._animate_leg: `s + (e - s) * t`), so this is
    the actual swept path, not an approximation of it. Motor degrees are
    converted to fold degrees per sample because forward_kinematics works
    in the fold frame.
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
