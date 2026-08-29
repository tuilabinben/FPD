"""Constants and the wire strings the scanner shares with the board.

Kept in one file so the protocol has a single spelling. Every string here
has a counterpart in RobotMotionController_v9_ClearCore.ino -- the scan is
a set of commands added to the SAME firmware the main console drives, not
a second sketch, so the board applies its soft limits, its PLC travel
switches and its E-STOP to a sweep exactly as it does to a jog.
"""

APP_TITLE = "340° Scanner"
WINDOW_MIN = (980, 660)

DEFAULT_BAUD = "115200"
BAUD_CHOICES = ["9600", "19200", "38400", "57600", "115200", "230400"]

# ---------------------------------------------------------------------
# Scan parameters
# ---------------------------------------------------------------------
# The Z step is THE input of this application -- how far the lift rises
# between one 340 degree sweep and the next. Everything else has a default
# that works.
DEFAULT_Z_STEP_MM = 5.0
DEFAULT_LAYERS = 10
DEFAULT_DEG_STEP = 1.0
DEFAULT_SWEEP_DEG = 340.0

# Mirrors of the board's own limits, so a bad number is caught before it is
# sent rather than coming back as an [ERROR] the operator has to read.
Z_STEP_MIN_MM = 0.10
DEG_STEP_MIN = 0.10
DEG_STEP_MAX = 90.0
LAYERS_MIN = 1
LAYERS_MAX = 500
Z_STROKE_MM = 285.0
# The sweep may be SHORTER than the travel -- scanning one wall is a real
# job. It may not be longer: the turntable cannot reach past its own stop,
# so the extra degrees would be spent driving into the RM soft limit.
SWEEP_MIN_DEG = 1.0
SWEEP_MAX_DEG = 340.0

SENSOR_KINDS = ("ULTRASONIC", "ANALOG")
DEFAULT_SENSOR = "ULTRASONIC"

# A reading the board could not take comes back NEGATIVE, never 0 -- 0 mm
# is a real distance and "the echo never returned" is not. Plotted as a
# gap, counted as a miss.
MISS_MM = -1.0

# ---------------------------------------------------------------------
# Commands out
# ---------------------------------------------------------------------
CMD_PING = "PING"
CMD_BYE = "BYE"
CMD_ESTOP = "ESTOP"
CMD_SCAN_STOP = "SCAN_STOP"
CMD_SCAN_STATUS = "SCAN_STATUS"
CMD_SCAN_READ = "SCAN_READ"
CMD_PLC_STATUS = "PLC_STATUS"
CMD_HOME = "HOME"

# The board PUSHES [PLC_STATE] whenever the status word changes, so this is
# a backstop, not the mechanism: it covers the first reply after connecting
# and the case where nothing has changed for a long time and the operator
# wants to know the link is still alive. Slow on purpose -- the board's own
# idle poll of the PLC is 5 s, so asking faster than that cannot learn
# anything new.
PLC_POLL_MS = 5000


def cmd_scan_start(z_step_mm, deg_step, layers, sweep_deg=DEFAULT_SWEEP_DEG):
    return (f"SCAN_START:{z_step_mm:.3f},{deg_step:.3f},"
            f"{int(layers)},{sweep_deg:.2f}")


def cmd_sensor(kind):
    return f"SET_SCAN_SENSOR:{kind}"


def cmd_cal(mm_per_count, offset_mm):
    return f"SET_SCAN_CAL:{mm_per_count:.6f},{offset_mm:.3f}"


# ---------------------------------------------------------------------
# Replies in
# ---------------------------------------------------------------------
TAG_POINT = "[SCAN_PT]"
TAG_BEGIN = "[SCAN_BEGIN]"
TAG_LAYER = "[SCAN_LAYER]"
TAG_DONE = "[SCAN_DONE]"
TAG_ABORT = "[SCAN_ABORT]"
TAG_SEEK = "[SCAN_SEEK]"
TAG_REF = "[SCAN_REF]"
TAG_READ = "[SCAN_READ]"
TAG_SENSOR = "[SCAN_SENSOR]"
TAG_PLC_STATE = "[PLC_STATE]"
TAG_HOME = "[HOME]"
TAG_PLC_HOME = "[PLC_HOME]"
TAG_COORD_RESET = "[COORD_RESET]"

# What the board says when a home cycle ends. Matched on these rather than
# on "[HOME]" alone, because every step of the cycle reports under the same
# tag and only these two mean it is over.
HOME_DONE_TEXT = "HOMING COMPLETE"
HOME_FAILED_TEXT = "FAILED"
TAG_STATUS = "[SCAN_STATUS]"
TAG_ERROR = "[ERROR]"
TAG_WARN = "[WARN]"

# ---------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------
# The radial scale is chosen from the first layer and then FROZEN for the
# rest of the scan. A plot that rescales itself every layer cannot be
# compared with the one above it by eye, which is the whole point of
# stacking them.
PLOT_MIN_RANGE_MM = 200.0
PLOT_RINGS = 4

# ---------------------------------------------------------------------
# Colours -- flat constants, no theme engine. This is a one-window tool.
# ---------------------------------------------------------------------
BG = "#16181c"
PANEL = "#1d2025"
FIELD = "#111318"
INK = "#e8eaed"
MUTED = "#8b9099"
ACCENT = "#4fc3f7"
OK = "#5dd39e"
WARN = "#f2b134"
BAD = "#ef5b5b"
GRID = "#2b3038"

# ---------------------------------------------------------------------
# The PLC link, and RM's switch
# ---------------------------------------------------------------------
# The scan is REFUSED without both of these, so they belong on the panel
# rather than in an [ERROR] after START: no device data means RM's switch
# cannot be seen, and that switch is the frame every layer is referenced
# to. Below the colours because they name them.
#
# THE LAMP REPORTS DEVICE DATA, NOT THE SOCKET. A lamp that followed the
# socket flapped every few seconds on the machine -- the board drops and
# reopens the socket on every reply timeout, which is a resynchronisation,
# not a fault. `data=` is what the scan actually depends on. Mirrors the
# main console's PLC_LED_STATES, deliberately: two panels disagreeing about
# whether the PLC is up is worse than either being wrong on its own.
PLC_STATE_LABELS = {
    "unknown":     ("PLC: NO LINK", MUTED),
    "connected":   ("PLC: CONNECTED", OK),
    "no_reply":    ("PLC: NO REPLY", WARN),
    "unreachable": ("PLC: UNREACHABLE", BAD),
    "disabled":    ("PLC: DISABLED", MUTED),
}

# RM's switch. FOUR states, and `?` is not CLEAR: a dead link showing a
# clear lamp reads as good news on a safety display while the switch may be
# physically covered. That was a real field bug on the main console.
#
# DISABLED is red rather than grey because it is the one state that stops a
# scan outright -- SET_PLC_SENSOR_ENFORCE:ROT,0 leaves the bit readable and
# cosmetic, and the board then refuses START.
RM_STATE_LABELS = {
    "unknown":  ("RM SWITCH: ?", MUTED),
    "covered":  ("RM SWITCH: ON", OK),
    "clear":    ("RM SWITCH: CLEAR", INK),
    "disabled": ("RM SWITCH: OFF", BAD),
}

FONT = ("Segoe UI", 10)
FONT_BOLD = ("Segoe UI", 10, "bold")
FONT_BIG = ("Segoe UI", 22, "bold")
FONT_MONO = ("Consolas", 10)
FONT_CAPTION = ("Segoe UI", 8)
