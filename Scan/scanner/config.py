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
TAG_READ = "[SCAN_READ]"
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

FONT = ("Segoe UI", 10)
FONT_BOLD = ("Segoe UI", 10, "bold")
FONT_BIG = ("Segoe UI", 22, "bold")
FONT_MONO = ("Consolas", 10)
FONT_CAPTION = ("Segoe UI", 8)
