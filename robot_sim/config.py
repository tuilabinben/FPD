"""Tuning constants, machine geometry and protocol-level settings.

Everything in here is a plain value — no tkinter, no logic — so any other
module can import it without pulling in the GUI.
"""

# ------------------------------------------------------------------
# Serial / connection
# ------------------------------------------------------------------
DEFAULT_COM_PORT = "COM7"
DEFAULT_BAUD_RATE = "115200"
BAUD_CHOICES = ["9600", "19200", "38400", "57600", "115200", "230400"]

PING_TIMEOUT_MS = 2000
HEARTBEAT_INTERVAL_MS = 3000
MISSED_BEAT_LIMIT = 3
SERIAL_POLL_MS = 50

# ------------------------------------------------------------------
# Jog (software simulation used when no hardware is confirmed)
# ------------------------------------------------------------------
JOG_SIM_TICK_MS = 50
# NOTE: the jog simulation has no speeds of its own. It derives them from
# the universal RPM and the per-motor percentages (SPEED_FIELDS below),
# through the same arithmetic and the same ceilings the firmware uses, so
# offline motion is a faithful preview rather than an optimistic one.

# Jog "Boost" cycles through these multipliers (x1 -> x1.5 -> x2 -> x1 ...)
BOOST_LEVELS = [1.0, 1.5, 2.0]

# How often the GUI refreshes the board's jog dead-man watchdog while an
# axis is held. Must be comfortably below the firmware's JOG_WATCHDOG_MS
# (700 ms) so a single dropped packet doesn't trip it.
JOG_HEARTBEAT_MS = 150

# ------------------------------------------------------------------
# PID — ONE preset, from the Stepper MATLAB report, Table 2
# ("normalised controller parameters as configured in Simulink").
#
# Plant identified in that report (open-loop transfer function of the
# ClearCore -> TB6600 -> stepper -> 1:50 gearbox chain):
#       G(s) = 12.5 / (s * (s + 12.5))          [rad per STEP pulse]
# Designed by pole placement for POT < 5% (zeta = 0.7071), ts ≈ 0.57 s.
#
# The P / PI / PD alternatives and the PARALLEL / I-PD / PREFILTER form
# selector were REMOVED in v9.1. Only this row was ever used on the
# machine, and the form selector configured a controller structure that
# does not exist on an open-loop board — a menu that changed nothing but
# could still be set wrong. PID_ENABLED replaces it: one switch that says
# whether these gains are in play at all.
# ------------------------------------------------------------------
PID_PRESET = {
    "kp": 24.97, "ki": 120.00, "kd": 1.33, "n": 50.0, "ts": 0.57,
    "note": "Pole placement, ζ = 0.7071, ts ≈ 0.57 s. The only gain set "
            "used on this machine — Stepper MATLAB report, Table 2.",
}
PID_PRESET_NAME = "PID"

DEFAULT_KP = PID_PRESET["kp"]
DEFAULT_KI = PID_PRESET["ki"]
DEFAULT_KD = PID_PRESET["kd"]

# Derivative filter coefficient N in the Simulink PID block. The report
# specifies N = 50..100 to suppress encoder quantisation noise on the
# derivative term without changing the designed dynamics.
DEFAULT_N_FILTER = PID_PRESET["n"]
N_FILTER_MIN, N_FILTER_MAX = 1.0, 200.0

# Whether the stored gains are in use. The board is open loop either way,
# so this is about honesty: with PID off nothing reports the gains as
# active, and nobody tunes against a controller that is not running.
DEFAULT_PID_ENABLED = True

# ------------------------------------------------------------------
# SPEED — ONE UNIVERSAL RPM, ONE PERCENTAGE PER MOTOR
#
#     axisMotorRpm = master_rpm * (axis_pct / 100) * AXIS_RPM_SCALE
#
# AXIS_RPM_SCALE is CALIBRATION, not a user setting. The three axes are
# geared completely differently, so a raw percentage of one shared RPM
# would mean nothing:
#
#     RM  28.4375:1  ->  140 motor RPM gives  29.5 °/s     scale 1.000
#     ZM  20 mm/rev  ->  105 motor RPM gives  35.0 mm/s    scale 0.750
#     AM  ratio UNMEASURED -> runs at the master RPM        scale 1.000
#
# AM's scale was 0.030, derived from "25 °/s = 4.17 motor RPM", which only
# held if ARM_GEAR_RATIO were really 1.0. On the machine, 100 °/s was
# still visibly slow — proof the elbow has a real reduction and the motor
# was being throttled to ~17 RPM while every other axis ran at 100+. Its
# scale is now 1.0, so the arm's percentage maps straight to the master
# motor RPM exactly like RM's does, and its old 100 °/s ceiling is gone.
#
# RM and ZM are still clamped to a real engineering ceiling because their
# gearing is known. The arm is bounded in MOTOR RPM instead — the only
# unit on that axis that currently means anything, and the one that
# guards the hazard that actually exists: an open-loop stepper skipping
# steps at high RPM with no encoder to notice.
# ------------------------------------------------------------------
MASTER_RPM_NOMINAL = 140.0

ROT_RPM_SCALE = 140.0 / MASTER_RPM_NOMINAL      # 1.000
Z_RPM_SCALE = 105.0 / MASTER_RPM_NOMINAL        # 0.750
ARM_RPM_SCALE = 1.0                             # master RPM, direct

#: Per-axis conversion for the live "real RPM" readout beside each field.
#: (label, scale) — kept next to the scales so the two cannot drift.
AXIS_RPM_SCALES = {
    "rot_pct": ROT_RPM_SCALE,
    "arm_pct": ARM_RPM_SCALE,
    "z_pct": Z_RPM_SCALE,
}

# ---- The universal speed is now a FIXED CONSTANT ----
#
# It used to be an editable field, which made two knobs that did the same
# job: raising the master and raising a percentage produced identical
# motion, so "150 RPM at 80%" and "120 RPM at 100%" were the same machine
# state reached two ways. That is a setting people get wrong.
#
# There is now exactly one way to change a speed — that axis's percentage
# — and the reference it is a percentage OF never moves.
MASTER_RPM = 150.0                      # motor RPM at 100%
MASTER_ACC_RPM_S = 375.0                # RPM/s, ≈ 0.4 s ramp on every axis

# ---- Per-axis defaults ----
# These are also the RECOMMENDED CEILINGS. Above them the field turns
# amber and warns, but the value is still accepted — see SPEED_WARN_PCT.
#     RM  75% of 150 RPM -> 112.5 motor RPM -> 23.74 °/s
#     AM 125% of 150 RPM -> 187.5 motor RPM
#     ZM  50% of 150 RPM ->  56.25 motor RPM -> 18.75 mm/s
DEFAULT_ARM_PCT = 125.0
DEFAULT_ROT_PCT = 75.0
DEFAULT_Z_PCT = 50.0

# There is NO hard upper bound on a percentage.
#
# Safe for a specific reason rather than by luck: a percentage is a
# multiplier, not a speed, and every axis it feeds still has a real
# backstop underneath it —
#     RM  -> ROT_VEL_MAX_DEG_S   (real gearing, so °/s means something)
#     ZM  -> Z_VEL_MAX_MM_S      (real lead, same)
#     AM  -> ARM_MOTOR_RPM_MAX   (gearing unmeasured, so bound the motor)
#
# The lower bound stays. 0% silently freezes an axis and a negative
# percentage would invert its direction — neither is a speed setting.
AXIS_PCT_MIN = 1.0

#: Above this, the field warns. It is the default value itself: the
#: defaults were chosen as the fastest settings tested stable on this
#: machine, so "above the default" and "past what has been validated" are
#: the same statement.
SPEED_WARN_PCT = {
    "rot_pct": DEFAULT_ROT_PCT,
    "arm_pct": DEFAULT_ARM_PCT,
    "z_pct": DEFAULT_Z_PCT,
}

#: setting key -> (label, unit, default, min, max). max=None means no
#: upper bound; validation leans on the per-axis backstops instead.
SPEED_FIELDS = {
    "rot_pct": ("RM — rotation", "%", DEFAULT_ROT_PCT, AXIS_PCT_MIN, None),
    "arm_pct": ("A1M / A2M — arms", "%", DEFAULT_ARM_PCT, AXIS_PCT_MIN, None),
    "z_pct":   ("ZM — lift", "%", DEFAULT_Z_PCT, AXIS_PCT_MIN, None),
}
SPEED_KEYS = ("rot_pct", "arm_pct", "z_pct")

#: Wire order for SET_SPEED:master,acc,rotPct,armPct,zPct,rotAccPct,armAccPct,zAccPct
SPEED_WIRE_KEYS = ("rot_pct", "arm_pct", "z_pct")

# ------------------------------------------------------------------
# ACCELERATION — independent per-axis percentage of masterAccRpmS.
#
# Until now the firmware silently reused rotPct/armPct/zPct (the SPEED
# percentages) to scale acceleration too. These are a separate family so
# the ramp can be tuned without also changing cruise speed. Defaults equal
# the speed defaults — that reproduces today's real-machine acceleration
# exactly, rather than jumping to a flat 100% nobody has validated.
# ------------------------------------------------------------------
DEFAULT_ROT_ACC_PCT = DEFAULT_ROT_PCT
DEFAULT_ARM_ACC_PCT = DEFAULT_ARM_PCT
DEFAULT_Z_ACC_PCT = DEFAULT_Z_PCT

ACCEL_FIELDS = {
    "rot_acc_pct": ("RM — rotation accel", "%", DEFAULT_ROT_ACC_PCT, AXIS_PCT_MIN, None),
    "arm_acc_pct": ("A1M / A2M — arms accel", "%", DEFAULT_ARM_ACC_PCT, AXIS_PCT_MIN, None),
    "z_acc_pct":   ("ZM — lift accel", "%", DEFAULT_Z_ACC_PCT, AXIS_PCT_MIN, None),
}
ACCEL_KEYS = ("rot_acc_pct", "arm_acc_pct", "z_acc_pct")

#: Appended to SPEED_WIRE_KEYS on the wire, never interleaved.
ACCEL_WIRE_KEYS = ("rot_acc_pct", "arm_acc_pct", "z_acc_pct")

ACCEL_WARN_PCT = {
    "rot_acc_pct": DEFAULT_ROT_ACC_PCT,
    "arm_acc_pct": DEFAULT_ARM_ACC_PCT,
    "z_acc_pct": DEFAULT_Z_ACC_PCT,
}

#: Engineering ceilings for acceleration, mirrored from the firmware's
#: ROT_ACC_MAX / Z_ACC_MAX / ARM_ACC_RPM_MAX.
ROT_ACC_MAX_DEG_S2 = 400.0
Z_ACC_MAX_MM_S2 = 400.0
ARM_ACC_RPM_MAX = 2000.0

# Ceilings mirrored from the firmware so the GUI can warn BEFORE sending
# rather than only after the board clamps.
#
# The arm's is in MOTOR RPM, not °/s. Its old 100 °/s ceiling was computed
# from an unmeasured gear ratio, so it was not protecting the arm from
# anything — it was throttling it while claiming to be a safety margin.
ROT_VEL_MAX_DEG_S = 120.0
Z_VEL_MAX_MM_S = 140.0
ARM_MOTOR_RPM_MAX = 400.0

# ------------------------------------------------------------------
# MACHINE GEOMETRY — STCR4000S twin frog-leg arm
#
# SOURCE OF TRUTH: MATLAB_v4_final/mophong_init.m (the Simscape model
# driven by the SolidWorks assembly). Symbol names below deliberately
# mirror that file so the two can be diffed by eye.
#
# Cross-checked against JEL reference drawing MTCR4160-300-AM (a closely
# related model in the same product line):
#     "340degree (Rotation angle)"           -> ROT span         AGREES
#     "160   160"                            -> A4_MM / A5_MM    AGREES
#     "575 (robot centre -> wafer centre)"   -> see REACH below
#     "315 (robot centre -> 3rd joint)"      -> see REACH below
#     "674.5 / 662.5 end-effector levels"    -> 12 mm arm gap, vs 9 mm in
#         the MATLAB model. The MATLAB value wins because the simulation
#         and this firmware must agree with the CAD that generates them;
#         the drawing is a different model number. CONFIRM ON THE BENCH.
#
# The previous revision of this file modelled the arm as
#     reach = 2 * 157.5 * cos(theta),  theta 0..90, reach 0..315
# which is wrong in three ways: the links are 160 mm (not 157.5), the
# reach carries a fixed +293.2 mm offset from A3 + A6 that was missing
# entirely, and reach therefore never approaches 0. See the audit note
# at the bottom of this section.
# ------------------------------------------------------------------
import math

# ---- Link lengths (mm), names as in mophong_init.m ----
A3_MM = 45.0        # turntable centre -> shoulder pivot
A4_MM = 160.0       # upper frog-leg link
A5_MM = 160.0       # lower frog-leg link
A6_MM = 248.2       # wrist pivot -> wafer centre (end-effector)

# ---- Column / deck heights (mm) ----
D_BASE_MM = 388.0       # column height up to the lift's zero
D3_ARM1_MM = 50.0       # arm 1 deck height above the lift carriage
D3_ARM2_MM = 41.0       # arm 2 deck height — 9 mm BELOW arm 1
D4_MM = 46.5
D5_MM = 24.8
D6_MM = 5.0

# Absolute height of each end-effector when the lift sits at d1 = 0.
Z_OFFSET_ARM1_MM = D_BASE_MM + D3_ARM1_MM + D4_MM + D5_MM + D6_MM   # 514.3
Z_OFFSET_ARM2_MM = D_BASE_MM + D3_ARM2_MM + D4_MM + D5_MM + D6_MM   # 505.3
# Computed from the deck heights directly, not by subtracting the two
# Z offsets — that subtraction lands on 8.999999999999943 in binary
# floating point and the 0.5 mm dual-arm tolerance check reads it raw.
ARM2_Z_DROP_MM = D3_ARM1_MM - D3_ARM2_MM                            # 9.0

# RM drivetrain: motor revolutions per one turntable revolution.
# The Simscape block diagram feeds RM through 1/4.375 then 1/6.5.
# mophong_init.m still writes 6.4 (= 28.0); the diagram and the machine
# agree on 6.5, so that is what the controller uses. The .m file is left
# alone on purpose — it is the simulation's own record.
#
# This is a MANUALLY-MIRRORED constant, same as ARM_GEAR_RATIO below: the
# board's live value is runtime-settable via SET_ROT_RATIO (no re-flash)
# and this GUI copy does not track it — it only feeds the jog-speed
# PREVIEW (rot_speed_deg_s()). If you calibrate on the bench, change it
# here too, or the preview and the board will quietly disagree.
I_RM_TOTAL = 4.375 * 6.5        # 28.4375

# ------------------------------------------------------------------
# THE ELBOW: MOTOR DEGREES vs FROG-LEG DEGREES
#
# These are two different numbers. What the board can MEASURE is motor
# shaft rotation — it counts step pulses and nothing else. The frog-leg
# link angle is DERIVED from it, and the derivation needs this ratio.
#
# WHERE IT COMES FROM (MATLAB_v4_final: mophongv2.slx + mophong_init.m)
# --------------------------------------------------------------------
# The Simscape root diagram drives each arm's TWO revolute joints from the
# one AM1/AM2 signal:
#
#     AM1 --x(-1)--> Revolute3   [banxoay : canhtay1]   the SHOULDER
#         --x(-2)--> Revolute    [canhtay1 : canhtay2]  the KNEE
#
# and mophong_init.m's forward kinematics says the same thing in closed
# form: the upper link sits at (th2 + th3_math), the lower at
# (th2 - th3_math) — symmetric about the radial line, so the knee turns
# through TWICE the angle the driven link does.
#
# The elbow motor is coupled to the knee, so one frog-leg degree costs two
# motor degrees:
#
#     fold_deg  = motor_deg / ARM_GEAR_RATIO
#     motor_deg = fold_deg  * ARM_GEAR_RATIO
#
# >>> CONFIRM ON THE BENCH. <<< This is derived from the model, not
# measured off the machine. Mark the elbow, command a known number of
# motor revolutions and divide by the frog-leg angle actually swept. If it
# is not 2, change it here AND in the firmware (or send SET_ARM_RATIO to a
# running board, which needs no re-flash).
#
# RM is the counter-example that shows this is the right shape: its ratio
# has been in the model all along as 1/4.375 then 1/6.5.
# ------------------------------------------------------------------
ARM_GEAR_RATIO = 2.0            # motor degrees per frog-leg degree

#: Kept as an alias because the name reads better next to I_RM_TOTAL.
I_ARM_TOTAL = ARM_GEAR_RATIO

# ZM ballscrew: carriage travel per motor revolution.
# ------------------------------------------------------------------
# ZM LEAD — MEASURE THIS
#
# Carriage travel per motor revolution. 20 was assumed, never measured, and
# it is the one number that decides whether a commanded millimetre is a real
# millimetre.
#
# If the carriage travels FURTHER than commanded, the true lead is LARGER in
# exact proportion: a 10 mm command that moves 30 mm means 3 * 20 = 60
# mm/rev. A non-power-of-2 factor like 3 points at the mechanics (lead screw
# pitch, pulley ratio) rather than the driver's microstep switches, which
# only ever err by powers of two.
#
# Measure with a rule on the carriage over a long move — 100 mm, not 10, so
# the reading error is small — then set it here AND send SET_Z_LEAD to the
# board (which needs no re-flash). A wrong lead also moves where every ZM
# soft limit physically is, because those are in millimetres.
# ------------------------------------------------------------------
Z_MM_PER_MOTOR_REV = 20.0


# ---- Universal RPM -> the unit each axis actually moves in ----
def rot_speed_deg_s(master_rpm, pct):
    """Turntable speed (°/s) for a master RPM and RM's percentage."""
    return master_rpm * (pct / 100.0) * ROT_RPM_SCALE * 360.0 / (60.0 * I_RM_TOTAL)


def arm_motor_speed_deg_s(master_rpm, pct):
    """MOTOR shaft speed (°/s) of an elbow motor. Exact — no gear ratio."""
    return master_rpm * (pct / 100.0) * ARM_RPM_SCALE * 360.0 / 60.0


def arm_speed_deg_s(master_rpm, pct):
    """FROG-LEG speed (°/s) — the motor speed divided by ARM_GEAR_RATIO.

    Only as good as ARM_GEAR_RATIO, which is derived from the Simscape
    model rather than measured, so quote it with the ratio beside it.
    arm_motor_speed_deg_s() and arm_motor_rpm() are the exact figures.
    """
    return arm_motor_speed_deg_s(master_rpm, pct) / ARM_GEAR_RATIO


def arm_motor_rpm(master_rpm, pct):
    """What the elbow motors are actually asked to turn at.

    Unlike arm_speed_deg_s() this does not depend on the gear ratio at
    all, so it is true whatever the ratio turns out to be.
    """
    return master_rpm * (pct / 100.0) * ARM_RPM_SCALE


def z_speed_mm_s(master_rpm, pct):
    """Lift speed (mm/s) for a master RPM and ZM's percentage."""
    return master_rpm * (pct / 100.0) * Z_RPM_SCALE * Z_MM_PER_MOTOR_REV / 60.0


def axis_motor_rpm(master_rpm, pct, scale):
    """The RPM that motor is actually asked to turn at."""
    return master_rpm * (pct / 100.0) * scale


#: (label, pct key, RPM scale, speed fn, unit, engineering ceiling or
#: None, motor-RPM ceiling or None). Drives the live readout beside each
#: percentage, computed exactly the way the firmware computes it.
#:
#: The arm's engineering ceiling is None on purpose: a °/s bound derived
#: from an unmeasured gear ratio is not a limit, it is a guess wearing a
#: limit's clothing. It is bounded in motor RPM instead.
SPEED_PREVIEW = (
    ("RM",      "rot_pct", ROT_RPM_SCALE, rot_speed_deg_s, "°/s",
     ROT_VEL_MAX_DEG_S, None),
    # Quoted in MOTOR °/s: that figure needs no gear ratio, so it cannot
    # be wrong. The frog-leg °/s is this divided by ARM_GEAR_RATIO.
    ("A1M/A2M", "arm_pct", ARM_RPM_SCALE, arm_motor_speed_deg_s, "motor °/s",
     None, ARM_MOTOR_RPM_MAX),
    ("ZM",      "z_pct",   Z_RPM_SCALE,   z_speed_mm_s,    "mm/s",
     Z_VEL_MAX_MM_S, None),
)
SPEED_PREVIEW_BY_KEY = {p[1]: p for p in SPEED_PREVIEW}

#: Same shape as SPEED_PREVIEW, reusing the same speed functions — they are
#: generic in master_rpm, so passing MASTER_ACC_RPM_S in its place computes
#: the exact acceleration figure the firmware derives, with no new maths.
ACCEL_PREVIEW = (
    ("RM",      "rot_acc_pct", ROT_RPM_SCALE, rot_speed_deg_s, "°/s²",
     ROT_ACC_MAX_DEG_S2, None),
    ("A1M/A2M", "arm_acc_pct", ARM_RPM_SCALE, arm_motor_speed_deg_s, "motor °/s²",
     None, ARM_ACC_RPM_MAX),
    ("ZM",      "z_acc_pct",   Z_RPM_SCALE,   z_speed_mm_s,    "mm/s²",
     Z_ACC_MAX_MM_S2, None),
)
ACCEL_PREVIEW_BY_KEY = {p[1]: p for p in ACCEL_PREVIEW}

# ---- Radial reach ----
# mophong_init.m writes the forward relation as
#       R = a3 + a6 + (a4 + a5) * cos(th3_math),   th3_math = pi - th3_cad
# which is identical to
#       R = A3 + A6 - (A4 + A5) * cos(th3_cad)
# so R grows as the CAD elbow angle th3_cad grows. See the frame note
# below: this project reports rotation from home, not th3_cad.
ARM_LINK_SUM_MM = A4_MM + A5_MM             # 320.0
ARM_RADIAL_OFFSET_MM = A3_MM + A6_MM        # 293.2

# ── THE ARM ANGLE IS MEASURED FROM HOME, NOT FROM CAD ────────────────
#
# Everything in this project that says "arm angle" or "fold angle" now
# means DEGREES ROTATED FROM THE HOME POSE: 0° is fully retracted, and the
# number grows by however far the elbow has turned.
#
# Why not th3_cad (60° retracted, 180° straight), which is what
# mophong_init.m uses? Because the board cannot produce th3_cad. It counts
# steps from wherever it was referenced and scales them by ARM_GEAR_RATIO,
# an unmeasured placeholder — so the "60°" it printed at home was never a
# measured CAD angle, it was zero rotation wearing a CAD label. Reporting
# rotation-from-home is the same number without the false claim, and it
# reads the way the operator thinks: home is nothing, and it counts up.
#
# th3_cad still exists — the frog-leg geometry is genuinely written in it
# — but ONLY inside fold_angle_to_reach() / reach_to_fold_angle(), which
# add and remove the offset below. No other code should mention it.
ARM_ZERO_CAD_DEG = 60.0             # the CAD angle that this project calls 0°

# Travel, in the from-home frame. 120° = th3_cad 180° = straight arm.
#
# NOTE (deliberate choice): 120° is the *mathematical* maximum and puts
# the frog-leg exactly at its straight-arm singularity, where radial
# stiffness collapses and a small elbow error produces a large radial
# error. JEL's drawing stops at 575 mm (= 91.72°). Full math was chosen
# here; set FOLD_ANGLE_MAX_DEG = FOLD_ANGLE_SPEC_MAX_DEG to switch to
# the conservative drawing limit.
FOLD_ANGLE_HOME_DEG = 0.0
FOLD_ANGLE_SPEC_MAX_DEG = 91.72     # -> R = 575 mm, JEL drawing
FOLD_ANGLE_MIN_DEG = FOLD_ANGLE_HOME_DEG
FOLD_ANGLE_MAX_DEG = 120.0          # -> R = 613.2 mm, full math

# Warn (do not block) once the elbow is this close to the singularity.
FOLD_ANGLE_SINGULARITY_WARN_DEG = 110.0


def _reach_at(fold_deg):
    """Reach for an angle in the from-home frame. The +ARM_ZERO_CAD_DEG
    is the one place the CAD frame is reintroduced."""
    return ARM_RADIAL_OFFSET_MM - ARM_LINK_SUM_MM * math.cos(
        math.radians(fold_deg + ARM_ZERO_CAD_DEG))


ARM_MIN_REACH_MM = _reach_at(FOLD_ANGLE_MIN_DEG)    # 133.2 mm at 0°, retracted
ARM_MAX_REACH_MM = _reach_at(FOLD_ANGLE_MAX_DEG)    # 613.2 mm at 120°, straight

# Backwards-compatible name. There is no single "link length" any more,
# so this is the half-sum and must NOT be used in new kinematics code.
ARM_LINK_MM = ARM_LINK_SUM_MM / 2.0

# ---- Z lift ----
# d1 is the carriage stroke; Cartesian Z is d1 + the arm's Z offset.
# mophong_init.m clamps d1 to 0..285 inside solve_ik_frogleg.
Z_STROKE_MM = 285.0
D1_MIN_MM, D1_MAX_MM = 0.0, Z_STROKE_MM

# ---- The operator's Z frame: 0 AT HOME ----
#
# HOME is the P2P reference point: X = 0, Y = 0, Z = 0. X and Y are
# measured from the TURNTABLE AXIS and are signed, because RM can put the
# arm behind the machine. Z is the lift's travel UP from HOME and is
# never negative — HOME is the bottom of the stroke and there is nothing
# below it.
Z_INPUT_MIN_MM = D1_MIN_MM                      # 0, at HOME
Z_INPUT_MAX_MM = D1_MAX_MM                      # 285, top of the stroke
#: The real height of arm 1's deck when the typed Z is 0. Nothing is
#: computed from this — it is what the panel shows beside the entry so the
#: operator can still relate the number to the machine in front of them.
Z_HOME_ABS_MM = Z_OFFSET_ARM1_MM                # 514.3

# The ABSOLUTE Cartesian band, still used by the kinematics and by the
# MATLAB parity sweep. Nothing the operator types is in this frame any
# more; see z_abs_from_home() in kinematics.py for the one crossing point.
Z_MIN_MM = Z_OFFSET_ARM1_MM                     # 514.3
Z_MAX_MM = Z_OFFSET_ARM1_MM + Z_STROKE_MM       # 799.3

# RM stops at real optical-sensor limits: 340° total span, centred on 0.
# ASSUMPTION: adjust the split if the real limits aren't centred on 0.
# ── RM ZERO IS THE CCW STOP, NOT MID-TRAVEL ──────────────────────────
#
# RM reads 0 at its fully counter-clockwise stop and counts up to 340 at
# the clockwise one. It used to be centred, -170..+170.
#
# HOME is that CCW stop, so this makes RM agree with every other axis on
# this machine: home is zero and the number counts up. ZM is already like
# that (0 at the bottom of the stroke) and so are the elbows.
#
# The Cartesian +X axis moves with it: at RM = 0 the arm points along +X.
# HOME is therefore a true X0 Y0 Z0 reference, and a target straight ahead
# of the home direction is (R, 0). The consequence is that any X,Y taught
# under the old centred frame is rotated 170 deg from what it used to mean.
ROT_MIN_DEG, ROT_MAX_DEG = 0.0, 340.0

# Bounds used by the software-only jog simulation. Jog works in JOINT
# space, so the Z pad drives d1 (0..285), not Cartesian Z.
ROT_SIM_MIN_DEG, ROT_SIM_MAX_DEG = ROT_MIN_DEG, ROT_MAX_DEG
Z_SIM_MIN_MM, Z_SIM_MAX_MM = D1_MIN_MM, D1_MAX_MM
# The jog simulation tracks the elbows in MOTOR degrees, because that is
# what the board reports and what the taught limits are stored in.
ARM_MOTOR_MIN_DEG = FOLD_ANGLE_MIN_DEG * ARM_GEAR_RATIO     # 0
ARM_MOTOR_MAX_DEG = FOLD_ANGLE_MAX_DEG * ARM_GEAR_RATIO     # 240 at ratio 2
ARM_SIM_MIN_DEG, ARM_SIM_MAX_DEG = ARM_MOTOR_MIN_DEG, ARM_MOTOR_MAX_DEG

# ------------------------------------------------------------------
# OPERATOR-DEFINED WORKING LIMITS
#
# Everything above is the FACTORY envelope — what the structure allows.
# What the machine may actually use is narrower and depends on what is
# installed around it: a cassette, a chamber port, a cable loop. Those
# limits belong to the operator, so they are editable in Settings and can
# also be captured straight from the machine's current position
# ("set here as lower / upper limit").
#
# Each arm has its OWN pair. Sharing one arm limit is exactly how v8 let
# A2M be driven past its stop while A1M's angle was the one being checked.
#
# The board holds these in RAM only, so the GUI is the system of record:
# it writes them to a JSON file and re-sends them on every connect.
#
#   key -> (label, firmware axis, end, unit, factory floor, factory ceil,
#           default, decimals)
# ------------------------------------------------------------------
# How far a TAUGHT elbow boundary may sit. Deliberately far wider than
# the CAD envelope: the number the board reports for an elbow is scaled by
# ARM_GEAR_RATIO, which has not been measured, so a captured position can
# legitimately land well outside 60°–180°. Narrowing this would reject the
# very teaching it exists to support.
#: Kept only so older saved files and the firmware constant names still
#: resolve. Nothing validates against these any more — see LIMIT_FIELDS.
ARM_LIMIT_FLOOR_DEG = None
ARM_LIMIT_CEIL_DEG = None

# ------------------------------------------------------------------
# DEFAULT BOUNDARIES SIT INSIDE THE FACTORY ENVELOPE
#
# The defaults used to BE the factory envelope, which meant a machine
# nobody had taught yet would happily drive an axis to its mechanical end
# stop — the soft limit and the hard stop were the same position, so the
# soft limit protected nothing. On a first real test that is a collision,
# not a warning.
#
# Each default is now inset by the margin below. The factory envelope is
# unchanged and still the outer bound; these are just the numbers you get
# before anybody teaches better ones, and they are deliberately timid.
#
# Widen them by teaching (SET HERE) once the real stops are known, which is
# the workflow these boundaries exist for.
# ------------------------------------------------------------------
LIMIT_SAFETY_MARGIN = {
    "Z": 5.0,       # mm off each end of the 285 mm stroke
    "ROT": 5.0,     # ° off each end of the 340° sweep
    "A1": 10.0,     # MOTOR ° off each end (= 5 fold° at ratio 2)
    "A2": 10.0,
}

_Z_M = LIMIT_SAFETY_MARGIN["Z"]
_R_M = LIMIT_SAFETY_MARGIN["ROT"]
_A_M = LIMIT_SAFETY_MARGIN["A1"]

# THE MARGIN IS APPLIED AT THE FAR END ONLY, NOT AT HOME.
#
# HOME is the MINIMUM of all four axes: d1 = 0 at the bottom of the stroke,
# RM = 0 at the CCW stop, both elbows = 0 fully retracted. Insetting the
# lower end would therefore put HOME ITSELF outside the working envelope —
# which refuses the home pose, and with it every P2P program, since a run
# is HOME -> A -> B -> HOME.
#
# Nothing is lost by leaving the lower end at the floor: that end is where
# the machine's own physical sensors are (M5 MinZ, M6 OutR), so it is the
# one end that is already protected without a soft limit. The far end has
# no sensor at all on this machine, which is exactly why it gets the
# margin.
DEFAULT_LIM_Z_MIN, DEFAULT_LIM_Z_MAX = D1_MIN_MM, D1_MAX_MM - _Z_M
DEFAULT_LIM_ROT_MIN, DEFAULT_LIM_ROT_MAX = ROT_MIN_DEG, ROT_MAX_DEG - _R_M
DEFAULT_LIM_A_MIN = ARM_MOTOR_MIN_DEG
DEFAULT_LIM_A_MAX = ARM_MOTOR_MAX_DEG - _A_M

#: Per-axis ENFORCEMENT. True = this boundary is stopping the axis.
#: The taught values are kept and still displayed while it is off, so
#: switching an axis off and back on costs nothing.
#:
#: This replaced a per-axis value LOCK (`lim_<axis>_locked`), which froze
#: the number while leaving the limit enforced. The lock was a real
#: protection but it was the *only* per-axis control, so the panel could
#: say UNLOCKED — telling you about the number — while the question an
#: operator actually asks at that button, "is this boundary on?", had no
#: per-axis answer anywhere. Values are now guarded by APPLY alone, the
#: same as every other tab.
LIMIT_ENFORCE_BY_AXIS = {
    "Z": "lim_z_enforced",
    "ROT": "lim_rot_enforced",
    "A1": "lim_a1_enforced",
    "A2": "lim_a2_enforced",
}
LIMIT_ENFORCE_KEYS = tuple(LIMIT_ENFORCE_BY_AXIS[a] for a in ("Z", "ROT", "A1", "A2"))
#: On by default. A fresh install must not be less safe than a configured one.
DEFAULT_LIMIT_ENFORCED = True

#: Master switch: soft-limit ENFORCEMENT on or off, every axis at once.
#: Values are kept and still displayed while it is off. It ANDs with the
#: per-axis flags above rather than overriding them — turning the master
#: back on must not silently re-arm an axis the operator switched off.
LIMITS_ENABLED_KEY = "limits_enabled"
DEFAULT_LIMITS_ENABLED = True

LIMIT_FIELDS = {
    "lim_z_min":   ("Lower limit",     "Z",   "MIN", "mm",
                    D1_MIN_MM, D1_MAX_MM, DEFAULT_LIM_Z_MIN, 2),
    "lim_z_max":   ("Upper limit",     "Z",   "MAX", "mm",
                    D1_MIN_MM, D1_MAX_MM, DEFAULT_LIM_Z_MAX, 2),
    "lim_rot_min": ("CCW limit",       "ROT", "MIN", "°",
                    ROT_MIN_DEG, ROT_MAX_DEG, DEFAULT_LIM_ROT_MIN, 2),
    "lim_rot_max": ("CW limit",        "ROT", "MAX", "°",
                    ROT_MIN_DEG, ROT_MAX_DEG, DEFAULT_LIM_ROT_MAX, 2),
    # ---- The elbows are TAUGHT, and have NO envelope at all ----
    #
    # floor = ceil = None means "accept whatever number the machine
    # reports". This is not laziness, it is the only honest option: the
    # board's elbow angle is a raw count scaled by an unmeasured gear
    # ratio, so it legitimately reads 1000° at a pose the CAD model calls
    # 140°. Any envelope written here would be a guess, and a guess that
    # rejects a real position the operator is standing at is worse than no
    # envelope — it stops them teaching the machine at all.
    #
    # They stay in LIMIT_CAPTURE_ONLY below, which makes the entry
    # read-only. Typing a number would mean typing it against that same
    # meaningless scale.
    # Unit is MOTOR degrees. That is the raw count, so a taught boundary
    # stays exact whatever ARM_GEAR_RATIO turns out to be — correcting the
    # ratio must never cost the operator an afternoon of re-teaching.
    "lim_a1_min":  ("Taught limit A",  "A1",  "MIN", "motor °",
                    None, None, DEFAULT_LIM_A_MIN, 2),
    "lim_a1_max":  ("Taught limit B",  "A1",  "MAX", "motor °",
                    None, None, DEFAULT_LIM_A_MAX, 2),
    "lim_a2_min":  ("Taught limit A",  "A2",  "MIN", "motor °",
                    None, None, DEFAULT_LIM_A_MIN, 2),
    "lim_a2_max":  ("Taught limit B",  "A2",  "MAX", "motor °",
                    None, None, DEFAULT_LIM_A_MAX, 2),
}

#: Boundaries that may ONLY be set by capturing the machine's current
#: position. Their entry is read-only.
#:
#: This is not a UI preference, it is the honest consequence of not
#: knowing the elbow's real scale: a typed 90° means nothing until
#: ARM_GEAR_RATIO is measured, whereas "wherever the arm is standing right
#: now" is exact regardless. Teaching sidesteps the unknown entirely.
LIMIT_CAPTURE_ONLY = frozenset({
    "lim_a1_min", "lim_a1_max", "lim_a2_min", "lim_a2_max",
})
LIMIT_KEYS = tuple(LIMIT_FIELDS)

#: Grouped for the dialog, and for the pair check.
#: (heading, min key, max key, minimum allowed span)
#:
#: span=None means the pair is UNORDERED: either box may hold either end,
#: and whichever is lower becomes the lower limit when the pair is
#: applied. That is what the elbows need — you jog to a stop and press
#: SET HERE, and which of the two stops you happened to teach first is
#: not information the operator should have to keep straight.
LIMIT_GROUPS = (
    ("ZM — lift",         "lim_z_min",   "lim_z_max",   1.0),
    ("RM — turntable",    "lim_rot_min", "lim_rot_max", 1.0),
    ("A1M — arm 1 elbow (motor °)", "lim_a1_min",  "lim_a1_max",  None),
    ("A2M — arm 2 elbow (motor °)", "lim_a2_min",  "lim_a2_max",  None),
)

#: PID gains, each individually lockable. Locking one term freezes just
#: that term — the others stay editable and keep being sent.
PID_FIELDS = (
    ("kp", "Kp — proportional", "kp_locked"),
    ("ki", "Ki — integral", "ki_locked"),
    ("kd", "Kd — derivative", "kd_locked"),
    ("n",  "N — derivative filter", "n_locked"),
)
PID_LOCK_KEYS = tuple(f[2] for f in PID_FIELDS)

#: Which live jog readout each limit is captured from, for the
#: "set current position as this limit" buttons.
LIMIT_LIVE_SOURCE = {
    "Z": "sim_z", "ROT": "sim_rot", "A1": "sim_a1", "A2": "sim_a2",
}

# ------------------------------------------------------------------
# PLC LINK — MELSEC MC Protocol 3E, ASCII, TCP
#
# Mirrors the PLC section of the firmware. Nothing here opens a socket:
# the GUI talks to the ClearCore over serial and the ClearCore is the MC
# protocol client. These constants exist so the operator console can SAY
# what the board is talking to, and so a wrong address is visible in one
# place rather than only in a .ino nobody has open.
#
# If you change one of these, change the matching #define in
# RobotMotionController_v9_ClearCore.ino and re-flash. They are two
# copies on purpose — the board must work driven from a bare terminal
# with no GUI at all — but they must not disagree.
# ------------------------------------------------------------------
PLC_IP = "192.168.3.101"
PLC_PORT = 1025
PLC_CLEARCORE_IP = "192.168.3.200"
#: How often the BOARD polls the PLC's status word. Two rates, and the
#: second one is a correctness requirement, not a tuning knob:
#:
#: Idle, nothing in that word is urgent, so 5 s leaves the Mitsubishi's
#: Ethernet module almost entirely alone and gives it all the time it wants
#: to answer.
#:
#: During a home it must be fast. The board completes a home only after it
#: has SEEN the run bits M10..M13 come on and go off again — that gate is
#: what stops a stale latched M1 ending the next home before the machine
#: has moved. At 5 s, a home sequence shorter than one interval finishes
#: entirely between two polls, the run bits are never observed, and a home
#: that physically succeeded fails on the 30 s timeout — intermittently,
#: depending on where the poll lands.
PLC_POLL_IDLE_MS = 5000
PLC_POLL_HOMING_MS = 200
PLC_POLL_MS = PLC_POLL_IDLE_MS

#: The PLC device map, transcribed from the GX Works comment list.
#: (device, comment, direction as seen from the ClearCore)
#: "wire" is a real direction here, not a label: X0 is driven by a
#: physical output from ClearCore's IO-0 terminal, not by a packet.
PLC_DEVICE_MAP = (
    ("X0",  "HOME request (wired from ClearCore IO-0)", "wire"),
    ("M0",  "RUN",           "plc"),
    ("M1",  "DONE",          "read"),
    ("M2",  "rHOME",         "plc"),
    ("M3",  "STOP",          "plc"),
    ("M4",  "rJOG",          "plc"),
    ("M5",  "MinZ  (ZM home sensor)",  "read"),
    ("M6",  "OutR  (RM home sensor)",  "read"),
    ("M7",  "OutR1 (A1M home sensor)", "read"),
    ("M8",  "OutR2 (A2M home sensor)", "read"),
    ("M10", "Run ZM",        "read"),
    ("M11", "Run RM",        "read"),
    ("M12", "Run A1M",       "read"),
    ("M13", "Run A2M",       "read"),
    ("M20", "AUTO",          "plc"),
    ("M21", "HOME",          "plc"),
    ("M23", "sHOME",         "plc"),
)

#: The per-axis optical HOME sensors. They report that an axis has reached
#: its reference — they are NOT limit switches, and nothing derives a
#: working boundary from them. Boundaries come from the operator only.
PLC_HOME_SENSOR_BITS = {
    "M5": ("Z",   "MinZ",  "ZM lift"),
    "M6": ("ROT", "OutR",  "RM turntable"),
    "M7": ("A1",  "OutR1", "A1M arm 1 elbow"),
    "M8": ("A2",  "OutR2", "A2M arm 2 elbow"),
}

# ------------------------------------------------------------------
# THE M5..M8 SENSOR PANEL, shown in BOTH motion modes
#
# (bit, axis label, firmware axis token, blocked jog command, wired)
#
# M5 and M6 are wired and working: while covered, the axis may not be
# driven FURTHER INTO the stop. The opposite direction stays available —
# a sensor that blocked both would pin the machine on its own home switch.
#
# M7 and M8 are BROKEN on this machine, so they are `wired = False`:
# displayed as a placeholder, never acted on. A dead sensor read as "not at
# the limit" simply never blocks; read as "at the limit" it would freeze
# both elbows for good.
# ------------------------------------------------------------------
# (bit, axis label, firmware axis token, jog command that drives INTO it,
#  which end of the axis it sits at: -1 minimum, +1 maximum)
#
# All four are wired and working, and they sit at OPPOSITE ends:
#   M5 MinZ  -> ZM at the bottom      \ the HOME end
#   M6 OutR  -> RM at the CCW stop    /
#   M7 OutR1 -> A1M fully EXTENDED    \ the FAR end
#   M8 OutR2 -> A2M fully EXTENDED    /
#
# HOME STATE is M5 and M6 covered while M7 and M8 are CLEAR: at home the
# lift is down, the turntable is at 0, and both arms are pulled IN — which
# is the opposite end from where M7/M8 sit.
PLC_SENSOR_PANEL = (
    ("M5", "ZM  lift",      "Z",   "Z_DOWN",   -1),
    ("M6", "RM  turntable", "ROT", "ROT_CCW",  -1),
    ("M7", "A1M arm 1",     "A1",  "A1_FWD",   +1),
    ("M8", "A2M arm 2",     "A2",  "A2_FWD",   +1),
)

#: Which axis index in current_joints each sensor guards.
PLC_SENSOR_JOINT_INDEX = {"Z": 0, "ROT": 1, "A1": 2, "A2": 3}

#: Shown until a PLC device read has actually landed, and again if the reads
#: stop.
#:
#: This is NOT cosmetic. The lamps used to start at "CLEAR" and only change
#: when a poll arrived, so a dead MC-protocol link showed four CLEAR lamps —
#: indistinguishable from "no sensor is covered". On a safety display that is
#: the worst possible failure: it reads as good news. Unknown must look
#: unknown.
PLC_SENSOR_UNKNOWN_TEXT = "NO DATA"

#: A sensor reading older than this is treated as unknown again. Generous
#: relative to the 5 s idle poll so a normal slow poll never trips it.
PLC_SENSOR_STALE_MS = 15000

#: Kept for a sensor that is not wired. All four are wired now, so nothing
#: uses it — left because older saved layouts and the tests name it.
PLC_SENSOR_BROKEN_TEXT = "NO SENSOR"

#: HOME STATE = these covered, and PLC_HOME_STATE_CLEAR_BITS not covered.
PLC_HOME_STATE_ON_BITS = ("M5", "M6")
PLC_HOME_STATE_CLEAR_BITS = ("M7", "M8")

#: How often the GUI asks the board for its PLC link state. Piggybacked on
#: the serial heartbeat cadence rather than a timer of its own — the board
#: is already being spoken to every 3 s, and one more short command costs
#: nothing.
PLC_STATUS_POLL_MS = HEARTBEAT_INTERVAL_MS

#: The PLC link LED. (text, theme colour name) keyed by state.
#:
#: This LED replaced the heartbeat one. The heartbeat itself still runs —
#: it is what notices a dead board and triggers the all-stop — but its
#: PING/PONG state is serial-link health, which the COM PORT and CLEARCORE
#: lamps beside it already convey. Whether the Mitsubishi is reachable is
#: the thing that is NOT visible anywhere else, and it is what decides
#: whether HOME can work at all.
PLC_LED_STATES = {
    "unknown":     ("NO LINK",     "TEXT_MUTED"),
    "connected":   ("CONNECTED",   "ACCENT_GREEN"),
    "no_reply":    ("NO REPLY",    "ACCENT_ORANGE"),
    "unreachable": ("UNREACHABLE", "ACCENT_RED"),
}

#: The HOME request is a WIRE: ClearCore's IO-0 terminal into the PLC's X0
#: input. The Ethernet link is read-only and cannot start a home.
#:
#: It used to be an MC-protocol bit write to X0, and that was broken by
#: design: X devices are refreshed from their physical terminals at the top
#: of every PLC scan, so a written X0 was overwritten within ~10 ms and the
#: home silently never started. It worked only while X0 had no wire on it.
#: An output into that same input is what the scan expects, so there is
#: nothing left to race — and it needs no ladder change, because X0 is the
#: same HOME input it always was, in parallel with the panel pushbutton.
#:
#: Recorded here as well as in the firmware because when HOME does not
#: start, the person debugging it is usually looking at the GUI, and the
#: first thing to check is now a wire and a 24 V return, not the network.
PLC_HOME_REQUEST_DEVICE = "X0"
PLC_HOME_REQUEST_SOURCE = "ClearCore IO-0 (hard-wired)"
PLC_HOME_DONE_DEVICE = "M1"
#: Nothing in this app or the firmware writes a PLC device.
PLC_LINK_IS_READ_ONLY = True

#: Where the GUI persists settings between runs. Kept next to the package
#: in a dev run, so a portable copy of the app carries its own machine
#: setup with it; redirected to %APPDATA% in a frozen build, where
#: "next to the package" is a fresh temp folder every launch — see
#: paths.py.
import os

from . import paths

_HERE = paths.user_data_dir()
SETTINGS_FILE = os.path.join(_HERE, "machine_settings.json")

#: Bumped whenever a saved value changes MEANING rather than just format.
#:
#: v2 moved the elbow angle from th3_cad (60° retracted, 180° straight) to
#: rotation from home (0° retracted, 120° straight). A v1 file's taught
#: elbow limits of 60..180 would silently be read as 60..180 DEGREES OF
#: ROTATION — a band that starts 60° out from home, so the arm could not
#: retract and the operator would be hunting a mechanical fault that isn't
#: there. Numbers that changed meaning must not be loaded quietly, and
#: converting them is not honest either: the old numbers were produced by
#: the same unmeasured gear ratio, so they were never real angles to
#: convert. They are dropped, and the operator re-teaches.
#: NOT bumped for the lock -> enforce change, on purpose. No stored value
#: changed meaning: `lim_<axis>_locked` was removed and is simply ignored
#: on load, and `lim_<axis>_enforced` defaults to True when absent — the
#: safe direction. Bumping would fire the ARM_FRAME_V2_RESET_KEYS drop
#: below and cost the operator their taught elbow boundaries for a change
#: that never touched them.
SETTINGS_SCHEMA = 4
SETTINGS_SCHEMA_KEY = "_schema"

#: The keys stored in an elbow unit that no longer means what it says.
#:
#: v1 -> v2 re-zeroed the elbow angle (th3_cad 60..180 became rotation from
#: home 0..120). v2 -> v3 changed the UNIT: these are now MOTOR degrees,
#: not frog-leg degrees, and at ARM_GEAR_RATIO = 2 a stored 120 that used
#: to mean "straight arm" now means "fold 60°, R = 453 mm". Reading it
#: quietly would leave the operator hunting a mechanical fault that is not
#: there, and converting it is not honest either — the old numbers came out
#: of a gear ratio of 1.0 that was never measured. They are dropped and
#: re-taught, which is two SET HERE presses per arm.
ARM_FRAME_V2_RESET_KEYS = ("lim_a1_min", "lim_a1_max",
                           "lim_a2_min", "lim_a2_max")

#: v3 -> v4: RM's zero moved from mid-travel to its CCW stop, so a stored
#: RM boundary points at a different physical place than it used to.
#:
#: This one bites hard and silently. A file saved with the centred frame
#: holds something like -150..+150; read in the new frame those become
#: "0..150 with a negative lower bound", so every bearing past 150 deg is
#: refused — which is most of the machine, and in particular ANY negative X,
#: whose bearing is 180. The symptom is "it keeps saying there is a limit"
#: on a target that is plainly reachable.
#:
#: Converting is possible in principle (add 170) but not honest: the old
#: numbers were taught against a zero that no longer exists on the machine,
#: and a boundary silently moved to a place nobody chose is worse than one
#: that has to be re-taught.
ROT_FRAME_V4_RESET_KEYS = ("lim_rot_min", "lim_rot_max")

#: Named limit sets, so one machine can carry several working envelopes —
#: a tight one for running against a cassette, a wide one for maintenance
#: — and switch between them instead of retyping eight numbers and getting
#: one of them wrong.
LIMIT_PRESETS_FILE = os.path.join(_HERE, "limit_presets.json")
LIMIT_PRESET_NAME_MAX = 40

# Home / park pose: lift down, turntable centred, elbow at the CAD home
# angle (fully retracted, R = 133.2 mm).
ROT_HOME_DEG = 0.0          # the CCW stop, and the Cartesian +X direction
ARM_HOME_DEG = FOLD_ANGLE_HOME_DEG * ARM_GEAR_RATIO     # motor degrees, 0
Z_HOME_MM = D1_MIN_MM

# Default P2P points shown on startup, in the OPERATOR's frame: X and Y
# from the turntable axis, Z up from HOME. Both are inside the real
# workspace — radius between 133.2 and 613.2 mm, Z between 0 and 285.
#
# These were 560 and 650, which were absolute heights. Left alone they
# would now read as "560 mm above HOME" on a 285 mm stroke and every
# startup would open with two unreachable points.
DEFAULT_POINT_A = (300.0, 0.0, 45.0)
DEFAULT_POINT_B = (250.0, 250.0, 135.0)

# Which arm performs a P2P move: "arm 1", "arm 2", or BOTH simultaneously.
ARM_CONFIGS = ("A1M", "A2M", "BOTH")
ELBOW_CONFIGS = ARM_CONFIGS  # backwards-compatible alias

# ------------------------------------------------------------------
# Jog command vocabulary — single source of truth. Previously three
# separate copies of this map lived in the app class and drifted apart.
# ------------------------------------------------------------------
# A1M and A2M are separate motors on separate frog-leg linkages, so each
# gets its own jog axis. ARM_FWD/ARM_BACK are kept on the wire as the
# "both arms together" gesture that was tested on real hardware, and are
# what the LINK toggle sends.
JOG_STOP_COMMAND = {
    "ROT_CW": "ROT_STOP",
    "ROT_CCW": "ROT_STOP",
    "A1_FWD": "A1_STOP",
    "A1_BACK": "A1_STOP",
    "A2_FWD": "A2_STOP",
    "A2_BACK": "A2_STOP",
    "ARM_FWD": "ARM_STOP",
    "ARM_BACK": "ARM_STOP",
    "Z_UP": "Z_STOP",
    "Z_DOWN": "Z_STOP",
}

# Which physical arm each jog axis drives, and in which direction.
# +1 extends (th3_cad increases), -1 retracts.
JOG_ARM_AXES = {
    "A1_FWD":  (("A1M",), +1),
    "A1_BACK": (("A1M",), -1),
    "A2_FWD":  (("A2M",), +1),
    "A2_BACK": (("A2M",), -1),
    "ARM_FWD":  (("A1M", "A2M"), +1),
    "ARM_BACK": (("A1M", "A2M"), -1),
}

# When LINK is on, a press on either arm's pad is promoted to the
# both-arms command so the two elbows stay in step.
JOG_LINK_PROMOTION = {
    "A1_FWD": "ARM_FWD", "A2_FWD": "ARM_FWD",
    "A1_BACK": "ARM_BACK", "A2_BACK": "ARM_BACK",
}

# Directions guarded by optical limit sensors, and their escape direction.
LIMIT_OPPOSITE = {
    "ROT_CW": "ROT_CCW",
    "ROT_CCW": "ROT_CW",
    "Z_UP": "Z_DOWN",
    "Z_DOWN": "Z_UP",
}

# ------------------------------------------------------------------
# Keyboard bindings
#
# These are no longer hard-coded. They live in `keybinds.py`, are editable
# in Settings → Controls, and persist to keybinds.json. The names below
# are kept because the rest of the app reads them, but they are DERIVED —
# anything that needs the live layout after a rebinding must call
# keybinds.active_map() rather than capture these at import.
# ------------------------------------------------------------------
from . import keybinds as _kb

JOG_KEYMAP = _kb.to_tk_keymap(_kb.active_map())
JOG_KEYCAPS = _kb.to_keycaps(_kb.active_map())
JOG_KEY_HINT = _kb.to_hint(_kb.active_map())

# Log ring-buffer size (lines) before the oldest lines are trimmed.
LOG_MAX_LINES = 800

WINDOW_TITLE = "Robot Motion Controller — P2P + Joystick (v5)"
WINDOW_GEOMETRY = "1400x900"
WINDOW_MIN_SIZE = (900, 500)

# Settings is tabbed rather than one long scroll, so it only ever has to
# be tall enough for its biggest single tab.
SETTINGS_GEOMETRY = "780x650"
SETTINGS_MIN_SIZE = (700, 560)
