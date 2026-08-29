"""Tuning constants, machine geometry, protocol settings.

Plain values only — no tkinter, no logic — so any module can import w/o pulling in GUI.
"""

DEFAULT_COM_PORT = "COM7"
DEFAULT_BAUD_RATE = "115200"
BAUD_CHOICES = ["9600", "19200", "38400", "57600", "115200", "230400"]

PING_TIMEOUT_MS = 2000
HEARTBEAT_INTERVAL_MS = 3000
MISSED_BEAT_LIMIT = 3
SERIAL_POLL_MS = 50

JOG_SIM_TICK_MS = 50

BOOST_LEVELS = [1.0, 1.5, 2.0]

JOG_HEARTBEAT_MS = 150

# PID: ONE preset, from Stepper MATLAB report Table 2 ("normalised controller params as configured in Simulink").
# Plant from report (open-loop TF, ClearCore->TB6600->stepper->1:50 gearbox):
#   G(s) = 12.5 / (s * (s + 12.5))   [rad per STEP pulse]
# Pole placement for POT < 5% (zeta = 0.7071), ts ≈ 0.57 s.
#
# P/PI/PD alternatives + PARALLEL/I-PD/PREFILTER form selector REMOVED v9.1: only this row
# ever used on machine, form selector configured a structure that doesn't exist on open-loop
# board (changed nothing, could still be set wrong). PID_ENABLED replaces it: one switch, gains
# in play or not.
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

# SPEED: one universal RPM, one percentage per motor.
#   axisMotorRpm = master_rpm * (axis_pct / 100) * AXIS_RPM_SCALE
#
# AXIS_RPM_SCALE = calibration, not user setting. 3 axes geared differently, raw pct of one
# shared RPM meaningless:
#   RM 28.4375:1 -> 140 motor RPM = 29.5°/s   scale 1.000
#   ZM 20mm/rev  -> 105 motor RPM = 35.0mm/s  scale 0.750
#   AM ratio UNMEASURED -> runs at master RPM  scale 1.000
#
# AM's scale was 0.030 (from "25°/s = 4.17 motor RPM"), only valid if ARM_GEAR_RATIO were 1.0.
# On machine 100°/s still visibly slow -> proof elbow has real reduction, motor throttled to
# ~17 RPM while other axes ran 100+. Scale now 1.0: arm pct maps straight to master motor RPM
# like RM does, old 100°/s ceiling gone.
#
# RM/ZM still clamped to real engineering ceiling (gearing known). Arm bounded in MOTOR RPM
# instead — only unit on that axis that means anything, guards real hazard: open-loop stepper
# skipping steps at high RPM, no encoder to notice.
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

# SET ON THE MACHINE. This combination is the one that ran stably; it is a
# bench result, not a calculation, so do not re-derive it from anything.
DEFAULT_ARM_PCT = 62.5
DEFAULT_ROT_PCT = 50.0
DEFAULT_Z_PCT = 200.0

AXIS_PCT_MIN = 1.0

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
# the ramp can be tuned without also changing cruise speed.
#
# THESE ARE NOT EQUAL TO THE SPEED PERCENTAGES, and the difference is the
# point. Acceleration is what decides how far an axis carries on after the
# key is released — coast = v² / 2a — so an axis tuned for speed alone
# overshoots. The arm showed it worst: sharing a 125% speed figure it ramped
# for 0.40 s and coasted 225 MOTOR degrees, most of its taught band, on
# every release.
#
# SET ON THE MACHINE, alongside the speed percentages above, as the
# combination that ran stably. A bench result — do not re-derive it, and do
# not "tidy" these back to equal the speed percentages.
# ------------------------------------------------------------------
DEFAULT_ROT_ACC_PCT = 100.0
DEFAULT_ARM_ACC_PCT = 70.0
DEFAULT_Z_ACC_PCT = 200.0

ACCEL_FIELDS = {
    "rot_acc_pct": ("RM — rotation accel", "%", DEFAULT_ROT_ACC_PCT, AXIS_PCT_MIN, None),
    "arm_acc_pct": ("A1M / A2M — arms accel", "%", DEFAULT_ARM_ACC_PCT, AXIS_PCT_MIN, None),
    "z_acc_pct":   ("ZM — lift accel", "%", DEFAULT_Z_ACC_PCT, AXIS_PCT_MIN, None),
}
ACCEL_KEYS = ("rot_acc_pct", "arm_acc_pct", "z_acc_pct")

ACCEL_WIRE_KEYS = ("rot_acc_pct", "arm_acc_pct", "z_acc_pct")

ROT_ACC_MAX_DEG_S2 = 400.0
Z_ACC_MAX_MM_S2 = 400.0
ARM_ACC_RPM_MAX = 2000.0

ROT_VEL_MAX_DEG_S = 120.0
Z_VEL_MAX_MM_S = 140.0
ARM_MOTOR_RPM_MAX = 400.0

# MACHINE GEOMETRY — STCR4000S twin frog-leg arm.
# SOURCE OF TRUTH: MATLAB_v4_final/mophong_init.m (Simscape model driven by SolidWorks
# assembly). Symbol names mirror that file for eyeball diffing.
#
# Cross-checked vs JEL drawing MTCR4160-300-AM (related model, same line):
#   "340degree (Rotation angle)"  -> ROT span        AGREES
#   "160 160"                     -> A4_MM/A5_MM     AGREES
#   "575 (robot centre->wafer centre)" -> see REACH below
#   "315 (robot centre->3rd joint)"    -> see REACH below
#   "674.5/662.5 end-effector levels"  -> 12mm arm gap vs 9mm in MATLAB model. MATLAB wins:
#       sim + firmware must agree w/ CAD that generates them; drawing is different model no.
#       CONFIRM ON BENCH.
#
# Prior revision modelled arm as reach = 2*157.5*cos(theta), theta 0..90, reach 0..315 — wrong
# 3 ways: links are 160mm not 157.5, reach missing fixed +293.2mm offset (A3+A6), so reach never
# approached 0. See audit note below.
# ------------------------------------------------------------------
import math

# SOLVED FROM TWO BENCH MEASUREMENTS, not mophong_init.m.
# Only the two SUMS below measured; split within each pair not, since reach curve only uses sums:
#   R = (a3+a6) - (a4+a5) * cos(frog-leg angle)
#   HOME, frog-leg 0deg:   (a3+a6)-(a4+a5) = 240mm  measured
#   straight, 180deg:      (a3+a6)+(a4+a5) = 605mm  measured
#   -> a3+a6 = 422.5, a4+a5 = 182.5
#
# a3 keeps old 45.0 (base radius unchanged), a6 carries correction; a4/a5 stay equal. If links
# measured individually later, only sums must be preserved.
#
# .m file still says 45/160/160/248.2 -> 133.2..613.2mm. mophong_init.m NOT reference for this
# machine — wrong in several places, abandoned as ground truth. Bench measurements above are.
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

# ELBOW: MOTOR DEGREES vs FROG-LEG DEGREES — two different numbers. Board MEASURES motor shaft
# rotation only (counts step pulses). Frog-leg link angle DERIVED from it via this ratio.
#
#   fold_deg  = motor_deg / ARM_GEAR_RATIO
#   motor_deg = fold_deg  * ARM_GEAR_RATIO
#
# MEASURED ON THE MACHINE, 7.80. Arm at full extension reaches 575 mm; the previous 10.0
# put that same motor position at 498 mm, so 10.0 * fold(498) / fold(575) = 10.0 * 114.45 /
# 146.68 = 7.80. A reach measurement, not an angle one — reach is what a tape measure can
# actually read on this machine.
#
# THE MODEL SAID 2, AND THE MODEL WAS WRONG. mophongv2.slx drives each arm's two revolutes
# from one AM signal (shoulder x-1, knee x-2), and mophong_init.m's FK agrees in closed form
# — links at th2 ± th3_math, symmetric about the radial line — so on paper one fold degree
# costs two motor degrees. The Simscape diagram models the LINKAGE and not the gearbox in
# front of it; the bench says the real train is 7.80. Do not "restore" 2 from the .m: the
# geometry there is right, the drive ratio is not in it.
ARM_GEAR_RATIO = 7.80


I_ARM_TOTAL = ARM_GEAR_RATIO

# ZM LEAD — carriage travel per motor rev. CONFIRMED ON THE MACHINE: commanded mm are real
# mm, so 20 stands. It was carried as an assumption for a long time and is not one any more.
#
# Kept settable anyway (SET_Z_LEAD, no re-flash), because a changed lead screw or pulley
# would move where every ZM soft limit physically is — those are stored in millimetres. If
# the carriage ever travels FURTHER than commanded, the true lead is larger in exact
# proportion: a 10 mm command moving 30 mm means 3*20 = 60 mm/rev. A non-power-of-2 factor
# points at the mechanics; the driver's microstep switches can only ever err by powers of 2.
# Measure over 100 mm, not 10.
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

# MEASURED ON MACHINE. mophong_init.m no longer reference.
# .m frame put HOME at th3_cad 60deg, 133.2mm reach, arm straight at 613.2mm. Neither survived
# contact w/ machine: HOME measures 240mm from turntable axis, arm straight at ~605mm. Those
# two numbers are what A3..A6 above solved from — see note beside them.
#
# So HOME is frog-leg 0, not 60. Frog-leg opens full 180deg over travel, base link swings half
# (0..90deg) since knee geared 2:1 vs shoulder — makes base = fold/2 exact, not approximation.
#
# CONSEQUENCE: MATLAB parity sweep in tests/python_check.py can't pass anymore. Intended — .m
# found wrong in several places, dropped as ground truth. Do NOT adjust numbers here to make
# sweep green; re-point test at these measurements or retire it.
ARM_ZERO_CAD_DEG = 0.0

FOLD_ANGLE_HOME_DEG = 0.0
# Rated working reach 575mm sits BELOW 605mm straight-arm pose on purpose: 180deg is
# singularity, no program should plan up against it. 146.68deg = fold angle putting wafer
# centre at 575mm under measured geometry.
FOLD_ANGLE_SPEC_MAX_DEG = 146.68
FOLD_ANGLE_MIN_DEG = FOLD_ANGLE_HOME_DEG
FOLD_ANGLE_MAX_DEG = 180.0

# Straight out = singularity; keeps same 10deg warning before it that old 110-of-120 gave.
FOLD_ANGLE_SINGULARITY_WARN_DEG = 170.0


def _reach_at(fold_deg):
    """Reach for angle in from-home frame. +ARM_ZERO_CAD_DEG is the one place CAD frame
    reintroduced."""
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

# OPERATOR-DEFINED WORKING LIMITS. Everything above = FACTORY envelope (what structure allows).
# What machine may actually use is narrower, depends on install around it (cassette, chamber
# port, cable loop). Limits belong to operator: editable in Settings, capturable from current
# position ("set here as lower/upper limit").
#
# Each arm has OWN pair — sharing one arm limit is how v8 let A2M drive past its stop while
# A1M's angle was the one checked.
#
# Board holds these in RAM only; GUI is system of record, writes to JSON, re-sends every connect.
#
#   key -> (label, firmware axis, end, unit, factory floor, factory ceil, default, decimals)
# How far a TAUGHT elbow boundary may sit. Deliberately far wider than CAD envelope: board's
# reported elbow number scaled by unmeasured ARM_GEAR_RATIO, so captured position can land well
# outside 60-180deg. Narrowing would reject the teaching it exists to support.
# Kept only so older saved files/firmware constant names resolve. Nothing validates against
# these anymore — see LIMIT_FIELDS.
ARM_LIMIT_FLOOR_DEG = None
ARM_LIMIT_CEIL_DEG = None

# DEFAULT BOUNDARIES SIT INSIDE FACTORY ENVELOPE. Defaults used to BE factory envelope, meaning
# untaught machine would drive axis to mechanical end stop — soft limit == hard stop, protected
# nothing. First real test = collision, not warning.
#
# Each default now inset by margin below. Factory envelope unchanged, still outer bound; these
# are just numbers before anyone teaches better ones, deliberately timid.
#
# Widen by teaching (SET HERE) once real stops known — that's the workflow these exist for.
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

# Per-sensor boundary switch for M30/M31/M32 — separate from the taught
# soft limits above and from the master PLC_LINK_ENABLED_KEY (which stops
# the whole PLC socket, HOME included). This is for one BROKEN switch: a
# stuck or noisy sensor should not have to take HOME and the other two
# axes' protection down with it. Disabled -> the axis is never stopped by
# that switch, AND the switch stops counting toward the M30+M31+M32 HOME
# condition (a broken switch would otherwise block HOME forever).
PLC_SENSOR_ENFORCE_BY_AXIS = {
    "Z":   "plc_sensor_z_enforced",
    "ROT": "plc_sensor_rot_enforced",
    "A2":  "plc_sensor_a2_enforced",
}
PLC_SENSOR_ENFORCE_KEYS = tuple(PLC_SENSOR_ENFORCE_BY_AXIS[a] for a in ("Z", "ROT", "A2"))
DEFAULT_PLC_SENSOR_ENFORCED = True

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

# PLC LINK — MELSEC MC Protocol 3E, ASCII, TCP. Mirrors firmware's PLC section; nothing here
# opens a socket — GUI talks ClearCore over serial, ClearCore is the MC protocol client.
# Constants exist so console can SAY what board talks to, wrong address visible in one place
# not buried in a .ino nobody has open.
#
# Change one -> change matching #define in RobotMotionController_v9_ClearCore.ino, re-flash.
# Two copies on purpose (board must work from bare terminal, no GUI) but must not disagree.
PLC_IP = "192.168.3.101"
PLC_PORT = 1025
PLC_CLEARCORE_IP = "192.168.3.200"
PLC_POLL_IDLE_MS = 20
PLC_POLL_HOMING_MS = 10
PLC_POLL_MS = PLC_POLL_IDLE_MS

PLC_DEVICE_MAP = (
    ("X0",  "HOME request (wired from ClearCore IO-0)", "wire"),
    ("M0",  "RUN",           "plc"),
    ("M2",  "rHOME",         "plc"),
    ("M3",  "STOP",          "plc"),
    ("M4",  "rJOG",          "plc"),
    ("M30", "ZM travel limit",  "read"),
    ("M31", "RM travel limit",  "read"),
    ("M32", "A2M travel limit", "read"),
    ("M20", "AUTO",          "plc"),
    ("M21", "HOME",          "plc"),
    ("M23", "sHOME",         "plc"),
)


# M30..M32 TRAVEL LIMITS — the ONLY PLC devices read, shown in BOTH motion modes.
# (bit, axis label, firmware axis token, jog command that drives INTO it, which end: -1 min,
# +1 max)
#
# The panel used to show M5..M8, the home sensors. They were read, lit a lamp, and decided
# NOTHING — while M30 was the bit actually refusing a jog and had no lamp anywhere. An
# operator watching "M5 ZM lift = CLEAR" while ZM refused to move down was reading a device
# that has no say in it. M1 (DONE) and M10..M13 (run) went the same way: HOME completes on
# M30..M32, so nothing else needed reading.
#
# While covered, an axis may not drive FURTHER INTO its switch. The opposite direction stays
# available — blocking both would pin the machine on its own limit with no way off.
# ZM AND A2M ARE SWAPPED FROM THE OBVIOUS ORDER, measured on the machine: M32 tracks ZM
# (covered at the bottom of the stroke, clearing as Z rises), M30 is A2M's. The tidy
# M30=ZM order was an assumption, and it made ZM watch a bit that sits at 1 — so every
# Z_DOWN was refused wherever the carriage really was. Keep this in step with
# PLC_M_LIMIT_* in the firmware; python_check.py asserts the two agree.
PLC_SENSOR_PANEL = (
    ("M32", "ZM  lift",      "Z",   "Z_DOWN",  -1),
    ("M31", "RM  turntable", "ROT", "ROT_CW",  +1),
    ("M30", "A2M arm 2",     "A2",  "A2_BACK", -1),
)

# A2M's switch is wired at BOTH ends of its travel: one PLC device, two
# physical switches. The bit alone cannot say which end tripped it, so the
# BOARD works it out from the direction the axis was travelling on the
# rising edge and reports the answer in the "end Z/R/A2=" field. The GUI
# never derives this itself -- it would have to guess between polls, and a
# wrong guess refuses the one direction that comes off the switch.
#
# The end named in PLC_SENSOR_PANEL above stays the HOME-side end: the
# fallback until something is latched, and the only end that may count
# toward the home state.
PLC_SENSOR_BOTH_ENDS = frozenset({"M30"})

# Which jog command drives INTO a switch sitting at that end. Two entries
# per axis now, because a both-ends switch refuses whichever end it caught.
PLC_SENSOR_JOG_CMD = {
    ("Z",   -1): "Z_DOWN",
    ("Z",   +1): "Z_UP",
    ("ROT", +1): "ROT_CW",
    ("ROT", -1): "ROT_CCW",
    ("A2",  -1): "A2_BACK",
    ("A2",  +1): "A2_FWD",
}

PLC_SENSOR_JOINT_INDEX = {"Z": 0, "ROT": 1, "A1": 2, "A2": 3}

PLC_SENSOR_UNKNOWN_TEXT = "NO DATA"

PLC_SENSOR_STALE_MS = 15000

PLC_SENSOR_BROKEN_TEXT = "NO SENSOR"

# HOME STATE = all three limits true. Same condition the board homes on.
PLC_HOME_STATE_ON_BITS = ("M30", "M31", "M32")
PLC_HOME_STATE_CLEAR_BITS = ()

PLC_STATUS_POLL_MS = HEARTBEAT_INTERVAL_MS

PLC_LED_STATES = {
    "unknown":     ("NO LINK",     "TEXT_MUTED"),
    "connected":   ("CONNECTED",   "ACCENT_GREEN"),
    "no_reply":    ("NO REPLY",    "ACCENT_ORANGE"),
    "unreachable": ("UNREACHABLE", "ACCENT_RED"),
    "disabled":    ("DISABLED",    "TEXT_MUTED"),
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

# Jog command vocabulary, single source of truth — previously 3 copies in app class, drifted
# apart. A1M/A2M separate motors on separate frog-leg linkages, each own jog axis. ARM_FWD/
# ARM_BACK kept on wire as "both arms together" gesture tested on real hardware, sent by LINK
# toggle.
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

# Keyboard bindings no longer hard-coded — live in keybinds.py, editable Settings->Controls,
# persist to keybinds.json. Names below kept since rest of app reads them, but DERIVED — needs
# live layout after rebind must call keybinds.active_map(), not capture these at import.
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

# ══════════════════════════════════════════════════════════════════════
# SCAN — the third control mode
#
# RM sweeps, a distance sensor reads, ZM steps up, RM sweeps back. Same
# board, same firmware, same link as P2P and JOG: the sweep runs through
# the jog primitives, so soft limits, PLC switches and E-STOP apply to a
# scan exactly as to a held key.
#
# Wire strings mirror the .ino and the stand-alone Scan/ app. Three copies
# because the board must work from a bare terminal and Scan/ must keep
# running; what matters is they cannot DISAGREE — python_check.py reads
# the .ino and asserts it.
# ══════════════════════════════════════════════════════════════════════

# THE OPERATOR'S FOUR NUMBERS. No angular step, no speed asked for:
#   t = P / f          seconds a slice        (P points at f Hz)
#   w = sweep / t      deg/s RM must turn at
#   step = sweep / P   degrees between samples
# 50 Hz, 50 points, 330 deg sweep = 330 deg in 1 s. 100 points = 2 s.
DEFAULT_SCAN_SAMPLE_HZ = 50.0
# 250, not the 50 of the worked example: at 50 Hz over 340 deg, 50 points
# would demand 340 deg/s and RM does 69. A default that always warns is
# noise. 250 asks for 68 deg/s, just inside it.
DEFAULT_SCAN_POINTS_PER_SLICE = 250
DEFAULT_SCAN_SLICES = 10
DEFAULT_SCAN_SLICE_GAP_MM = 5.0
DEFAULT_SCAN_SWEEP_DEG = 340.0
DEFAULT_SCAN_SENSOR = "ULTRASONIC"

SCAN_SENSOR_KINDS = ("ULTRASONIC", "ANALOG")

# Mirrors the board's own limits: a bad number is caught at the panel, not
# returned as an [ERROR] with the machine standing still.
SCAN_SAMPLE_HZ_MIN = 0.10
SCAN_SAMPLE_HZ_MAX = 1000.0
SCAN_POINTS_MIN = 2
SCAN_POINTS_MAX = 3400
SCAN_SLICES_MIN = 1
SCAN_SLICES_MAX = 500
SCAN_SLICE_GAP_MIN_MM = 0.10
SCAN_DEG_STEP_MIN = 0.10
SCAN_DEG_STEP_MAX = 90.0
SCAN_SWEEP_MIN_DEG = 1.0
SCAN_SWEEP_MAX_DEG = 340.0

# THE ZM TRAVEL CEILING — a setting, not a constant. Lift used is
# gap * (slices - 1). Past it the panel WARNS and asks, never refuses:
# D1_MAX_MM on the board is the hard stop, this is the operator's own
# working ceiling, normally well inside it.
SCAN_MAX_Z_KEY = "scan_max_z_mm"
DEFAULT_SCAN_MAX_Z_MM = 180.0
SCAN_MAX_Z_MIN_MM = 1.0

SCAN_SETTING_FIELDS = {
    SCAN_MAX_Z_KEY: ("Maximum ZM travel per scan", "mm", DEFAULT_SCAN_MAX_Z_MM,
                     SCAN_MAX_Z_MIN_MM, D1_MAX_MM),
}

# A failed reading comes back NEGATIVE, never 0 — 0 mm is a real distance
# and "no echo" is not. Gap on the plot, marked in the CSV.
SCAN_MISS_MM = -1.0

# Sensor lamp judges health over this many recent readings: one lost echo
# is not a fault, a wall of them is.
SCAN_RECENT_WINDOW = 50
SCAN_MISS_WARN_FRACTION = 0.25

# Radial scale is taken from layer 1 and FROZEN: a plot that rescales
# cannot be compared with the layer above it by eye.
SCAN_PLOT_MIN_RANGE_MM = 200.0
SCAN_PLOT_RINGS = 4
SCAN_PLOT_SIZE = 360
# Repaint on a timer, not per point: 341 points a layer.
SCAN_REDRAW_MS = 250


def cmd_scan_start(z_step_mm, deg_step, layers, sweep_deg=DEFAULT_SCAN_SWEEP_DEG,
                   rot_deg_s=None):
    """SCAN_START:zStep,degStep,layers[,sweep[,rotDegS]]

    Speed is LAST and optional, so a board flashed before it reads four
    fields and ignores the fifth — falling back to its own scan scale.
    """
    line = (f"SCAN_START:{z_step_mm:.3f},{deg_step:.3f},"
            f"{int(layers)},{sweep_deg:.2f}")
    if rot_deg_s is not None:
        line += f",{rot_deg_s:.3f}"
    return line


def cmd_scan_sensor(kind):
    return f"SET_SCAN_SENSOR:{kind}"


def cmd_scan_cal(mm_per_count, offset_mm):
    return f"SET_SCAN_CAL:{mm_per_count:.6f},{offset_mm:.3f}"


SCAN_TAG_POINT = "[SCAN_PT]"
SCAN_TAG_BEGIN = "[SCAN_BEGIN]"
SCAN_TAG_LAYER = "[SCAN_LAYER]"
SCAN_TAG_DONE = "[SCAN_DONE]"
SCAN_TAG_ABORT = "[SCAN_ABORT]"
SCAN_TAG_SEEK = "[SCAN_SEEK]"
SCAN_TAG_REF = "[SCAN_REF]"
SCAN_TAG_READ = "[SCAN_READ]"
SCAN_TAG_STATUS = "[SCAN_STATUS]"
