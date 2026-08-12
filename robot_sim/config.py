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

BOOST_LEVELS = [1.0, 1.5, 2.0]

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

DEFAULT_N_FILTER = PID_PRESET["n"]
N_FILTER_MIN, N_FILTER_MAX = 1.0, 200.0

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

ROT_RPM_SCALE = 140.0 / MASTER_RPM_NOMINAL     
Z_RPM_SCALE = 105.0 / MASTER_RPM_NOMINAL       
ARM_RPM_SCALE = 1.0                            

AXIS_RPM_SCALES = {
    "rot_pct": ROT_RPM_SCALE,
    "arm_pct": ARM_RPM_SCALE,
    "z_pct": Z_RPM_SCALE,
}

MASTER_RPM = 150.0                     
MASTER_ACC_RPM_S = 375.0               

DEFAULT_ARM_PCT = 125.0
DEFAULT_ROT_PCT = 75.0
DEFAULT_Z_PCT = 50.0

AXIS_PCT_MIN = 1.0

SPEED_WARN_PCT = {
    "rot_pct": DEFAULT_ROT_PCT,
    "arm_pct": DEFAULT_ARM_PCT,
    "z_pct": DEFAULT_Z_PCT,
}

SPEED_FIELDS = {
    "rot_pct": ("RM — rotation", "%", DEFAULT_ROT_PCT, AXIS_PCT_MIN, None),
    "arm_pct": ("A1M / A2M — arms", "%", DEFAULT_ARM_PCT, AXIS_PCT_MIN, None),
    "z_pct":   ("ZM — lift", "%", DEFAULT_Z_PCT, AXIS_PCT_MIN, None),
}
SPEED_KEYS = ("rot_pct", "arm_pct", "z_pct")

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

ACCEL_WIRE_KEYS = ("rot_acc_pct", "arm_acc_pct", "z_acc_pct")

ACCEL_WARN_PCT = {
    "rot_acc_pct": DEFAULT_ROT_ACC_PCT,
    "arm_acc_pct": DEFAULT_ARM_ACC_PCT,
    "z_acc_pct": DEFAULT_Z_ACC_PCT,
}

ROT_ACC_MAX_DEG_S2 = 400.0
Z_ACC_MAX_MM_S2 = 400.0
ARM_ACC_RPM_MAX = 2000.0

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

# SOLVED FROM TWO BENCH MEASUREMENTS, not from mophong_init.m.
#
# Only the two SUMS below are measured; the split within each pair is not,
# because the reach curve only ever uses the sums:
#
#     R = (a3 + a6) - (a4 + a5) * cos(frog-leg angle)
#
#     HOME, frog-leg   0 deg:  (a3+a6) - (a4+a5) = 240 mm   measured
#     straight, 180 deg:       (a3+a6) + (a4+a5) = 605 mm   measured
#     -> a3 + a6 = 422.5,  a4 + a5 = 182.5
#
# a3 keeps its old 45.0 (the base radius is the one part that did not
# change) and a6 carries the correction; a4/a5 stay equal. If someone later
# measures the links individually, only the sums have to be preserved.
#
# The .m file still says 45/160/160/248.2 -> 133.2..613.2 mm. mophong_init.m
# is NOT the reference for this machine - it is wrong in several places and
# was abandoned as ground truth. The bench measurements above are.
A3_MM = 45.0
A4_MM = 91.25
A5_MM = 91.25
A6_MM = 377.5

D_BASE_MM = 388.0      
D3_ARM1_MM = 50.0      
D3_ARM2_MM = 41.0      
D4_MM = 46.5
D5_MM = 24.8
D6_MM = 5.0

Z_OFFSET_ARM1_MM = D_BASE_MM + D3_ARM1_MM + D4_MM + D5_MM + D6_MM  
Z_OFFSET_ARM2_MM = D_BASE_MM + D3_ARM2_MM + D4_MM + D5_MM + D6_MM  
ARM2_Z_DROP_MM = D3_ARM1_MM - D3_ARM2_MM                           

I_RM_TOTAL = 1 * 6.5       

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
ARM_GEAR_RATIO = 7.80       # MEASURED: physical arm reaches 575 mm (straight out) at the
                             # motor position the old 10.0 ratio mapped to 498 mm.
                             # Derived: new = old * fold_angle(498mm) / fold_angle(575mm)
                             #              = 10.0 * 114.45 / 146.68 = 7.80


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


SPEED_PREVIEW = (
    ("RM",      "rot_pct", ROT_RPM_SCALE, rot_speed_deg_s, "°/s",
     ROT_VEL_MAX_DEG_S, None),
    ("A1M/A2M", "arm_pct", ARM_RPM_SCALE, arm_motor_speed_deg_s, "motor °/s",
     None, ARM_MOTOR_RPM_MAX),
    ("ZM",      "z_pct",   Z_RPM_SCALE,   z_speed_mm_s,    "mm/s",
     Z_VEL_MAX_MM_S, None),
)
SPEED_PREVIEW_BY_KEY = {p[1]: p for p in SPEED_PREVIEW}

ACCEL_PREVIEW = (
    ("RM",      "rot_acc_pct", ROT_RPM_SCALE, rot_speed_deg_s, "°/s²",
     ROT_ACC_MAX_DEG_S2, None),
    ("A1M/A2M", "arm_acc_pct", ARM_RPM_SCALE, arm_motor_speed_deg_s, "motor °/s²",
     None, ARM_ACC_RPM_MAX),
    ("ZM",      "z_acc_pct",   Z_RPM_SCALE,   z_speed_mm_s,    "mm/s²",
     Z_ACC_MAX_MM_S2, None),
)
ACCEL_PREVIEW_BY_KEY = {p[1]: p for p in ACCEL_PREVIEW}

ARM_LINK_SUM_MM = A4_MM + A5_MM            
ARM_RADIAL_OFFSET_MM = A3_MM + A6_MM       

# MEASURED ON THE MACHINE. mophong_init.m is no longer the reference.
#
# The .m file's frame put HOME at th3_cad 60 deg and a 133.2 mm reach, with
# the arm straight at 613.2 mm. Neither figure survived contact with the
# machine: HOME measures 240 mm from the turntable axis and the arm goes
# straight at ~605 mm. Those two numbers are what A3..A6 above are solved
# from - see the note beside them.
#
# So HOME is frog-leg 0, not 60. The frog-leg opens through a full 180 deg
# over the travel, and the base link swings half of that, 0..90 deg,
# because the knee is geared 2:1 against the shoulder. That is what makes
# base = fold / 2 exact rather than an approximation.
#
# CONSEQUENCE: the MATLAB parity sweep in tests/python_check.py cannot pass
# any more. That is intended - the .m was found to be wrong in several
# places and dropped as ground truth. Do NOT adjust the numbers here to
# make that sweep go green; re-point the test at these measurements or
# retire it.
ARM_ZERO_CAD_DEG = 0.0

FOLD_ANGLE_HOME_DEG = 0.0
# The rated working reach, 575 mm, sits BELOW the 605 mm straight-arm pose
# on purpose: 180 deg is the singularity and no program should be planning
# up against it. 146.68 deg is the fold angle that puts the wafer centre at
# 575 mm under the measured geometry.
FOLD_ANGLE_SPEC_MAX_DEG = 146.68
FOLD_ANGLE_MIN_DEG = FOLD_ANGLE_HOME_DEG
FOLD_ANGLE_MAX_DEG = 180.0

# Straight out is the singularity, so this keeps the same 10 deg of warning
# before it that the old 110-of-120 gave.
FOLD_ANGLE_SINGULARITY_WARN_DEG = 170.0


def _reach_at(fold_deg):
    """Reach for an angle in the from-home frame. The +ARM_ZERO_CAD_DEG
    is the one place the CAD frame is reintroduced."""
    return ARM_RADIAL_OFFSET_MM - ARM_LINK_SUM_MM * math.cos(
        math.radians(fold_deg + ARM_ZERO_CAD_DEG))


ARM_MIN_REACH_MM = _reach_at(FOLD_ANGLE_MIN_DEG)   
ARM_MAX_REACH_MM = _reach_at(FOLD_ANGLE_MAX_DEG)   

ARM_LINK_MM = ARM_LINK_SUM_MM / 2.0

Z_STROKE_MM = 285.0
D1_MIN_MM, D1_MAX_MM = 0.0, Z_STROKE_MM

Z_INPUT_MIN_MM = D1_MIN_MM                     
Z_INPUT_MAX_MM = D1_MAX_MM                     
Z_HOME_ABS_MM = Z_OFFSET_ARM1_MM               

Z_MIN_MM = Z_OFFSET_ARM1_MM                    
Z_MAX_MM = Z_OFFSET_ARM1_MM + Z_STROKE_MM      

ROT_MIN_DEG, ROT_MAX_DEG = 0.0, 340.0

ROT_SIM_MIN_DEG, ROT_SIM_MAX_DEG = ROT_MIN_DEG, ROT_MAX_DEG
Z_SIM_MIN_MM, Z_SIM_MAX_MM = D1_MIN_MM, D1_MAX_MM
ARM_MOTOR_MIN_DEG = FOLD_ANGLE_MIN_DEG * ARM_GEAR_RATIO    
ARM_MOTOR_MAX_DEG = FOLD_ANGLE_MAX_DEG * ARM_GEAR_RATIO    
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
    "Z": 5.0,      
    "ROT": 5.0,    
    "A1": 10.0,    
    "A2": 10.0,
}

_Z_M = LIMIT_SAFETY_MARGIN["Z"]
_R_M = LIMIT_SAFETY_MARGIN["ROT"]
_A_M = LIMIT_SAFETY_MARGIN["A1"]

DEFAULT_LIM_Z_MIN, DEFAULT_LIM_Z_MAX = D1_MIN_MM, D1_MAX_MM - _Z_M
DEFAULT_LIM_ROT_MIN, DEFAULT_LIM_ROT_MAX = ROT_MIN_DEG, ROT_MAX_DEG - _R_M
DEFAULT_LIM_A_MIN = ARM_MOTOR_MIN_DEG
DEFAULT_LIM_A_MAX = ARM_MOTOR_MAX_DEG - _A_M

LIMIT_ENFORCE_BY_AXIS = {
    "Z": "lim_z_enforced",
    "ROT": "lim_rot_enforced",
    "A1": "lim_a1_enforced",
    "A2": "lim_a2_enforced",
}
LIMIT_ENFORCE_KEYS = tuple(LIMIT_ENFORCE_BY_AXIS[a] for a in ("Z", "ROT", "A1", "A2"))
DEFAULT_LIMIT_ENFORCED = True

LIMITS_ENABLED_KEY = "limits_enabled"
DEFAULT_LIMITS_ENABLED = True

PLC_LINK_ENABLED_KEY = "plc_link_enabled"
DEFAULT_PLC_LINK_ENABLED = True

LIMIT_FIELDS = {
    "lim_z_min":   ("Lower limit",     "Z",   "MIN", "mm",
                    D1_MIN_MM, D1_MAX_MM, DEFAULT_LIM_Z_MIN, 2),
    "lim_z_max":   ("Upper limit",     "Z",   "MAX", "mm",
                    D1_MIN_MM, D1_MAX_MM, DEFAULT_LIM_Z_MAX, 2),
    "lim_rot_min": ("CCW limit",       "ROT", "MIN", "°",
                    ROT_MIN_DEG, ROT_MAX_DEG, DEFAULT_LIM_ROT_MIN, 2),
    "lim_rot_max": ("CW limit",        "ROT", "MAX", "°",
                    ROT_MIN_DEG, ROT_MAX_DEG, DEFAULT_LIM_ROT_MAX, 2),
    "lim_a1_min":  ("Taught limit A",  "A1",  "MIN", "motor °",
                    None, None, DEFAULT_LIM_A_MIN, 2),
    "lim_a1_max":  ("Taught limit B",  "A1",  "MAX", "motor °",
                    None, None, DEFAULT_LIM_A_MAX, 2),
    "lim_a2_min":  ("Taught limit A",  "A2",  "MIN", "motor °",
                    None, None, DEFAULT_LIM_A_MIN, 2),
    "lim_a2_max":  ("Taught limit B",  "A2",  "MAX", "motor °",
                    None, None, DEFAULT_LIM_A_MAX, 2),
}

LIMIT_CAPTURE_ONLY = frozenset({
    "lim_a1_min", "lim_a1_max", "lim_a2_min", "lim_a2_max",
})
LIMIT_KEYS = tuple(LIMIT_FIELDS)

LIMIT_GROUPS = (
    ("ZM — lift",         "lim_z_min",   "lim_z_max",   1.0),
    ("RM — turntable",    "lim_rot_min", "lim_rot_max", 1.0),
    ("A1M — arm 1 elbow (motor °)", "lim_a1_min",  "lim_a1_max",  None),
    ("A2M — arm 2 elbow (motor °)", "lim_a2_min",  "lim_a2_max",  None),
)

PID_FIELDS = (
    ("kp", "Kp — proportional", "kp_locked"),
    ("ki", "Ki — integral", "ki_locked"),
    ("kd", "Kd — derivative", "kd_locked"),
    ("n",  "N — derivative filter", "n_locked"),
)
PID_LOCK_KEYS = tuple(f[2] for f in PID_FIELDS)

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
PLC_POLL_IDLE_MS = 5000
PLC_POLL_HOMING_MS = 200
PLC_POLL_MS = PLC_POLL_IDLE_MS

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

PLC_SENSOR_JOINT_INDEX = {"Z": 0, "ROT": 1, "A1": 2, "A2": 3}

PLC_SENSOR_UNKNOWN_TEXT = "NO DATA"

PLC_SENSOR_STALE_MS = 15000

PLC_SENSOR_BROKEN_TEXT = "NO SENSOR"

PLC_HOME_STATE_ON_BITS = ("M5", "M6")
PLC_HOME_STATE_CLEAR_BITS = ("M7", "M8")

PLC_STATUS_POLL_MS = HEARTBEAT_INTERVAL_MS

PLC_LED_STATES = {
    "unknown":     ("NO LINK",     "TEXT_MUTED"),
    "connected":   ("CONNECTED",   "ACCENT_GREEN"),
    "no_reply":    ("NO REPLY",    "ACCENT_ORANGE"),
    "unreachable": ("UNREACHABLE", "ACCENT_RED"),
}

PLC_HOME_REQUEST_DEVICE = "X0"
PLC_HOME_REQUEST_SOURCE = "ClearCore IO-0 (hard-wired)"
PLC_HOME_DONE_DEVICE = "M1"
PLC_LINK_IS_READ_ONLY = True

import os

from . import paths

_HERE = paths.user_data_dir()
SETTINGS_FILE = os.path.join(_HERE, "machine_settings.json")

SETTINGS_SCHEMA = 4
SETTINGS_SCHEMA_KEY = "_schema"

ARM_FRAME_V2_RESET_KEYS = ("lim_a1_min", "lim_a1_max",
                           "lim_a2_min", "lim_a2_max")

ROT_FRAME_V4_RESET_KEYS = ("lim_rot_min", "lim_rot_max")

LIMIT_PRESETS_FILE = os.path.join(_HERE, "limit_presets.json")
LIMIT_PRESET_NAME_MAX = 40

ROT_HOME_DEG = 0.0         
ARM_HOME_DEG = FOLD_ANGLE_HOME_DEG * ARM_GEAR_RATIO    
Z_HOME_MM = D1_MIN_MM

DEFAULT_POINT_A = (240.0, 0.0, 45.0)
DEFAULT_POINT_B = (250.0, 250.0, 135.0)

ARM_CONFIGS = ("A1M", "A2M", "BOTH")
ELBOW_CONFIGS = ARM_CONFIGS 

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

JOG_ARM_AXES = {
    "A1_FWD":  (("A1M",), +1),
    "A1_BACK": (("A1M",), -1),
    "A2_FWD":  (("A2M",), +1),
    "A2_BACK": (("A2M",), -1),
    "ARM_FWD":  (("A1M", "A2M"), +1),
    "ARM_BACK": (("A1M", "A2M"), -1),
}

JOG_LINK_PROMOTION = {
    "A1_FWD": "ARM_FWD", "A2_FWD": "ARM_FWD",
    "A1_BACK": "ARM_BACK", "A2_BACK": "ARM_BACK",
}

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

LOG_MAX_LINES = 800

WINDOW_TITLE = "Robot Motion Controller — P2P + Joystick (v5)"
WINDOW_GEOMETRY = "1400x900"
WINDOW_MIN_SIZE = (900, 500)

SETTINGS_GEOMETRY = "780x650"
SETTINGS_MIN_SIZE = (700, 560)
