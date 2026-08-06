import datetime
import math
import re
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

# ------------------------------------------------------------------
# Config
# ------------------------------------------------------------------
DEFAULT_COM_PORT = "COM7"
DEFAULT_BAUD_RATE = "115200"
PING_TIMEOUT_MS = 2000
HEARTBEAT_INTERVAL_MS = 3000
MISSED_BEAT_LIMIT = 3

JOG_SIM_TICK_MS = 50            # software-only jog simulation tick, no hardware needed
ROT_SPEED_DEG_PER_SEC = 30.0
ARM_SPEED_DEG_PER_SEC = 20.0    # A1M+A2M jog now moves in degrees (revolute joints), not mm
JZ_SPEED_MM_PER_SEC = 15.0

# ------------------------------------------------------------------
# SETTINGS — PID + velocity/acceleration defaults (Section: Settings)
# ------------------------------------------------------------------
DEFAULT_KP, DEFAULT_KI, DEFAULT_KD = 1.0, 0.0, 0.0
DEFAULT_VEL_RPM = 300.0
DEFAULT_ACCEL_RPM_S = 10000.0

# Jog "Boost" cycles through these multipliers (x1 -> x1.5 -> x2 -> x1 ...)
BOOST_LEVELS = [1.0, 1.5, 2.0]

# ------------------------------------------------------------------
# ARM GEOMETRY / INVERSE KINEMATICS — STCR4000S twin-arm "frog-leg"
# model, corrected from JEL's own spec sheet + a manufacturer CAD
# drawing (MTCR4160-300-AM, a closely related model in the same
# product line) that the user provided directly.
#
# CONFIRMED:
#   - RM rotation range is 340° total (spec sheet + CAD drawing agree;
#     earlier turns wrongly used 330°).
#   - Arm max reach is 315mm.
#   - Each arm (A1M, A2M) is its OWN single-motor "frog-leg" double-
#     parallelogram linkage — NOT a bending shoulder+elbow pair like an
#     earlier revision of this file assumed. One motor drives the whole
#     arm's straight-line radial reach:
#         reach = 2 * ARM_LINK_MM * cos(theta)   [theta=0 -> full reach]
#         theta = arccos(reach / (2 * ARM_LINK_MM))  [theta=90 -> folded]
#     ARM_LINK_MM = 157.5 is back-calculated from the spec's exact
#     315mm max reach (315 / 2). The drawing's own link dimensions read
#     ~160mm (matching JEL's "160mm arm" naming for this variant,
#     e.g. STCR4160S) — 157.5 vs 160 is normal mechanical-offset noise;
#     157.5 was used since it reproduces the spec's exact max reach.
#   - The "A1M or A2M" P2P selector means exactly what it says: which
#     of the two independent arms to use for a move.
#   - d1 (Z) is IDENTICAL for both arms — they both ride the same
#     turntable/lift, there is no per-arm Z offset.
#   - Both arms share the EXACT SAME rotation angle (theta2) — their
#     tips can reach and overlap at the same point. "Mirrored" only
#     describes the internal linkage handedness, NOT an azimuthal
#     offset. (Two earlier revisions wrongly assumed Arm 2 was mounted
#     180° away and needed a Z offset — both are removed now.)
# ------------------------------------------------------------------
ARM_LINK_MM = 157.5                      # frog-leg link length (see above)
ARM_MAX_REACH_MM = 2 * ARM_LINK_MM       # 315mm, matches spec exactly
ARM_MIN_REACH_MM = 0.0                   # fully folded

FOLD_ANGLE_MIN_DEG, FOLD_ANGLE_MAX_DEG = 0.0, 90.0   # theta range for EITHER arm's motor

Z_MIN_MM, Z_MAX_MM = 0.0, 200.0

# RM (rotation) stops at real optical-sensor limits: 340° total span,
# centered on 0 as -170..+170 — ASSUMPTION: adjust the split if the
# real limits aren't centered on 0. Used both by solve_ik() (so LOAD
# rejects an unreachable rotation instead of sending a target that
# would just hit the sensor mid-RUN) and by the jog software-simulation
# fallback below.
ROT_MIN_DEG, ROT_MAX_DEG = -170.0, 170.0
ROT_SIM_MIN_DEG, ROT_SIM_MAX_DEG = ROT_MIN_DEG, ROT_MAX_DEG
Z_SIM_MIN_MM, Z_SIM_MAX_MM = Z_MIN_MM, Z_MAX_MM

# Which arm to use for a P2P move — literally "arm 1" or "arm 2", or
# BOTH for a simultaneous move (same rotation, independent reach).
ELBOW_CONFIGS = ("A1M", "A2M", "BOTH")  # kept the old name to avoid touching every call site


# ------------------------------------------------------------------
# Palette (dark theme, shared across both modes)
# ------------------------------------------------------------------
BG = "#12121c"
PANEL_BG = "#1b1b29"
BORDER = "#33334a"
ENTRY_BG = "#252538"
LED_BG = "#0a0a12"

ACCENT_CYAN = "#00e5ff"
ACCENT_GREEN = "#00c853"
ACCENT_RED = "#ff1744"
ACCENT_ORANGE = "#ff9100"
ACCENT_PURPLE = "#7c4dff"

AXIS_X_COLOR = "#ff5252"
AXIS_Y_COLOR = "#69f0ae"
AXIS_Z_COLOR = "#40c4ff"

ROT_COLOR = "#ffab40"
ARM_COLOR = "#7c4dff"
JZ_COLOR = "#1de9b6"

TEXT_LIGHT = "#e6e6f0"
TEXT_MUTED = "#8a8aa3"


def _normalize_angle(deg):
    """Wraps an angle into (-180, 180]."""
    a = (deg + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


def solve_ik(x, y, z, arm_choice="A1M", reference_deg=0.0):
    """Inverse kinematics for the STCR4000S's twin frog-leg arms.

    Model: prismatic Z lift (d1) + rotating base (theta2, RM) + ONE of
    two independent single-motor "frog-leg" arms (A1M or A2M). Each arm
    reaches purely radially outward from the rotation center:
        reach = 2 * ARM_LINK_MM * cos(theta)   [theta=0 -> full reach]
    `arm_choice` selects which arm makes this move; the OTHER arm's
    joint is returned fully folded (theta=90°, reach=0) so it stays
    clear of the workspace. Both arms share the exact same rotation
    (theta2) — CONFIRMED: their tips can reach and overlap at the same
    point, so "mirrored" only describes the internal linkage handedness,
    not an azimuthal offset (an earlier revision wrongly assumed Arm 2
    was mounted 180° away — that's removed now).

    Returns (d1_mm, theta2_deg, theta_a1m_deg, theta_a2m_deg).
    Raises ValueError if unreachable or a joint/rotation limit is hit.
    """
    r = math.hypot(x, y)
    theta2 = reference_deg if r < 1e-6 else math.degrees(math.atan2(y, x))

    if not (ROT_MIN_DEG - 1e-6 <= theta2 <= ROT_MAX_DEG + 1e-6):
        raise ValueError(f"Góc quay θ2 (RM)={theta2:.1f}° ngoài tầm quay thực tế "
                         f"[{ROT_MIN_DEG:.0f}°, {ROT_MAX_DEG:.0f}°] — RM chỉ quay được 340°, "
                         f"không phải bàn xoay liên tục.")

    if r > ARM_MAX_REACH_MM + 1e-6 or r < ARM_MIN_REACH_MM - 1e-6:
        raise ValueError(f"Điểm ({x:.1f},{y:.1f}) ngoài tầm với: r={r:.1f} mm "
                         f"(tầm với hợp lệ [{ARM_MIN_REACH_MM:.1f}, {ARM_MAX_REACH_MM:.1f}] mm)")

    cos_theta = max(-1.0, min(1.0, r / (2 * ARM_LINK_MM)))
    theta_active = math.degrees(math.acos(cos_theta))   # 0°=full reach, 90°=folded

    theta_a1m = theta_active if arm_choice == "A1M" else FOLD_ANGLE_MAX_DEG
    theta_a2m = theta_active if arm_choice == "A2M" else FOLD_ANGLE_MAX_DEG

    # Both arms ride the same turntable/lift — no per-arm Z offset.
    d1 = z
    if not (Z_MIN_MM - 1e-6 <= d1 <= Z_MAX_MM + 1e-6):
        raise ValueError(f"Z={z:.1f} mm ngoài giới hạn [{Z_MIN_MM:.1f}, {Z_MAX_MM:.1f}] mm")

    return d1, theta2, theta_a1m, theta_a2m


def forward_kinematics(d1, theta2, theta_a1m, theta_a2m):
    """Joint angles -> Cartesian (x, y, z) mm, for display only. Infers
    which arm is active as whichever is closer to fully extended (the
    other should be sitting folded at 90°)."""
    active_theta = theta_a1m if theta_a1m <= theta_a2m else theta_a2m
    r = 2 * ARM_LINK_MM * math.cos(math.radians(active_theta))
    rad = math.radians(theta2)
    return r * math.cos(rad), r * math.sin(rad), d1


def solve_ik_both(x0, y0, z0, x1, y1, z1, reference_deg=0.0):
    """Inverse kinematics for a SIMULTANEOUS dual-arm move: Arm 1 (A1M)
    reaches (x0,y0,z0) while Arm 2 (A2M) reaches (x1,y1,z1) at the same
    time. This is physically possible because RM and Z are shared by
    both arms while each arm's reach has its own motor — but that also
    means the two points MUST lie in the SAME direction from the center
    (only the radius/reach may differ, since both arms share theta2),
    and at the same Z height, since there's only one rotation motor and
    one lift motor between them.

    Returns (d1_mm, theta2_deg, theta_a1m_deg, theta_a2m_deg).
    Raises ValueError if the two points aren't compatible with a single
    simultaneous move, or if either is unreachable.
    """
    if abs(z0 - z1) > 0.5:
        raise ValueError(f"Hai điểm phải cùng độ cao Z (dùng chung trục ZM): "
                         f"Z0={z0:.1f} mm khác Z1={z1:.1f} mm.")

    r0, r1 = math.hypot(x0, y0), math.hypot(x1, y1)

    if r0 < 1e-6 and r1 < 1e-6:
        rot = reference_deg
    elif r0 < 1e-6:
        rot = math.degrees(math.atan2(y1, x1))
    elif r1 < 1e-6:
        rot = math.degrees(math.atan2(y0, x0))
    else:
        principal0 = math.degrees(math.atan2(y0, x0))
        principal1 = math.degrees(math.atan2(y1, x1))
        diff = abs(_normalize_angle(principal1 - principal0))
        if diff > 1.0:
            raise ValueError(
                f"Hai điểm không cùng hướng qua tâm (2 tay dùng chung RM nên phải "
                f"cùng góc, chỉ khác bán kính): điểm 1 ở {principal0:.1f}°, "
                f"điểm 2 ở {principal1:.1f}° (lệch {diff:.1f}°, cần 0°).")
        rot = principal0

    if not (ROT_MIN_DEG - 1e-6 <= rot <= ROT_MAX_DEG + 1e-6):
        raise ValueError(f"Góc quay θ2 (RM)={rot:.1f}° ngoài tầm quay thực tế "
                         f"[{ROT_MIN_DEG:.0f}°, {ROT_MAX_DEG:.0f}°].")

    for r, label in ((r0, "Điểm 1 (A1M)"), (r1, "Điểm 2 (A2M)")):
        if r > ARM_MAX_REACH_MM + 1e-6 or r < ARM_MIN_REACH_MM - 1e-6:
            raise ValueError(f"{label} ngoài tầm với: r={r:.1f} mm "
                             f"(hợp lệ [{ARM_MIN_REACH_MM:.1f}, {ARM_MAX_REACH_MM:.1f}] mm)")

    theta_a1m = math.degrees(math.acos(max(-1.0, min(1.0, r0 / (2 * ARM_LINK_MM)))))
    theta_a2m = math.degrees(math.acos(max(-1.0, min(1.0, r1 / (2 * ARM_LINK_MM)))))

    d1 = z0  # z0 == z1 already checked above
    if not (Z_MIN_MM - 1e-6 <= d1 <= Z_MAX_MM + 1e-6):
        raise ValueError(f"Z={d1:.1f} mm ngoài giới hạn [{Z_MIN_MM:.1f}, {Z_MAX_MM:.1f}] mm")

    return d1, rot, theta_a1m, theta_a2m


def _round_rect_points(x1, y1, x2, y2, r):
    return [x1 + r, y1, x2 - r, y1, x2, y1, x2, y1 + r,
            x2, y2 - r, x2, y2, x2 - r, y2, x1 + r, y2,
            x1, y2, x1, y2 - r, x1, y1 + r, x1, y1]


def _shade(hex_color, factor):
    """factor < 1 darkens, factor > 1 lightens (clamped 0..255/channel)."""
    hex_color = hex_color.lstrip("#")
    r, g, b = (int(hex_color[i:i + 2], 16) for i in (0, 2, 4))
    r, g, b = (max(0, min(255, int(c * factor))) for c in (r, g, b))
    return f"#{r:02x}{g:02x}{b:02x}"


def _draw_bevel_rect(canvas, x1, y1, x2, y2, r, color):
    """Rounded rect with a raised 3D bevel: a darker sunken rim peeking
    out bottom-right, a lighter highlight line along the top-left edge —
    the 'rounder with a bevel side, not a flat/sharp rectangle' look."""
    dark = _shade(color, 0.55)
    light = _shade(color, 1.55)
    canvas.create_polygon(_round_rect_points(x1 + 2, y1 + 2, x2, y2, r),
                          smooth=True, fill=dark, outline="")
    canvas.create_polygon(_round_rect_points(x1, y1, x2 - 2, y2 - 2, r),
                          smooth=True, fill=color, outline="")
    canvas.create_line(x1 + r, y1 + 1, x2 - 2 - r, y1 + 1, fill=light, width=2, capstyle="round")
    canvas.create_line(x1 + 1, y1 + r, x1 + 1, y2 - 2 - r, fill=light, width=2, capstyle="round")


class RoundedButton(tk.Canvas):
    def __init__(self, parent, text, command=None, icon="", radius=20,
                 bg_color=ACCENT_GREEN, fg_color="#0a0a12", hover_color=None,
                 width=160, height=38, font=("Segoe UI", 10, "bold")):
        super().__init__(parent, width=width, height=height, bg=parent["bg"],
                         highlightthickness=0, bd=0, cursor="hand2")
        self.command = command
        self.base_color = bg_color
        self.hover_color = hover_color or _shade(bg_color, 1.15)
        self.radius = radius
        self.w, self.h = width, height
        self.text_str = text
        self.icon_str = icon
        self.fg_color = fg_color
        self.font = font
        self.enabled = True

        self._draw(self.base_color)
        self.bind("<Enter>", lambda e: self.enabled and self._draw(self.hover_color))
        self.bind("<Leave>", lambda e: self.enabled and self._draw(self.base_color))
        self.bind("<Button-1>", self._on_click)

    def _draw(self, color):
        self.delete("all")
        face = color if self.enabled else BORDER
        _draw_bevel_rect(self, 1, 1, self.w - 1, self.h - 1, self.radius, face)
        text_color = self.fg_color if self.enabled else TEXT_MUTED
        display_text = f"{self.icon_str}  {self.text_str}" if self.icon_str else self.text_str
        self.create_text(self.w // 2, self.h // 2, text=display_text,
                         fill=text_color, font=self.font)

    def set_config(self, text, bg_color, icon="", fg_color=None):
        self.text_str = text
        self.icon_str = icon
        self.base_color = bg_color
        self.hover_color = _shade(bg_color, 1.15)
        if fg_color is not None:
            self.fg_color = fg_color
        self._draw(self.base_color)

    def set_enabled(self, enabled):
        self.enabled = enabled
        self.configure(cursor="hand2" if enabled else "arrow")
        self._draw(self.base_color)

    def _on_click(self, _event):
        if self.enabled and self.command:
            self.command()


class JogPad(tk.Canvas):
    """Canvas-drawn jog button: rounded rect, hover/active states, keycap hint.
    Adapted from stcr4000s_joystick_control.py, re-themed to the dark palette
    already used throughout the rest of this app."""

    def __init__(self, parent, w, h, glyph, label, keycap,
                 base_color, hi_color, on_press, on_release):
        super().__init__(parent, width=w, height=h, bg=PANEL_BG,
                         highlightthickness=0, bd=0, cursor="hand2")
        self.w, self.h = w, h
        self.glyph = glyph
        self.label = label
        self.keycap = keycap
        self.base_color = base_color
        self.hi_color = hi_color
        self.on_press = on_press
        self.on_release = on_release
        self.state = "idle"
        self.enabled = True

        self._draw()
        self.bind("<ButtonPress-1>", self._press)
        self.bind("<ButtonRelease-1>", self._release)
        self.bind("<Enter>", self._enter)
        self.bind("<Leave>", self._leave)

    def _rounded_rect(self, x1, y1, x2, y2, r, **kwargs):
        pts = [x1 + r, y1, x2 - r, y1, x2, y1, x2, y1 + r, x2, y2 - r, x2, y2,
               x2 - r, y2, x1 + r, y2, x1, y2, x1, y2 - r, x1, y1 + r, x1, y1]
        return self.create_polygon(pts, smooth=True, **kwargs)

    def _draw(self):
        self.delete("all")
        if not self.enabled:
            fill, outline = ENTRY_BG, BORDER
            glyph_fill, label_fill = TEXT_MUTED, TEXT_MUTED
        elif self.state == "active":
            fill, outline = self.hi_color, self.hi_color
            glyph_fill, label_fill = "#0a0a12", "#0a0a12"
        elif self.state == "hover":
            fill, outline = ENTRY_BG, self.base_color
            glyph_fill, label_fill = self.hi_color, TEXT_LIGHT
        else:
            fill, outline = ENTRY_BG, BORDER
            glyph_fill, label_fill = self.base_color, TEXT_MUTED

        if self.enabled and self.state != "active":
            # idle/hover: raised bevel look
            _draw_bevel_rect(self, 2, 2, self.w - 2, self.h - 2, 14, fill)
            self._rounded_rect(2, 2, self.w - 2, self.h - 2, 14, fill="", outline=outline, width=1.5)
        else:
            # pressed-in (active) or disabled: flat, no raised edge
            self._rounded_rect(2, 2, self.w - 2, self.h - 2, 14, fill=fill, outline=outline, width=1.5)

        cx = self.w / 2
        self.create_text(cx, self.h * 0.34, text=self.glyph, font=("Segoe UI", 17, "bold"), fill=glyph_fill)
        self.create_text(cx, self.h * 0.62, text=self.label, font=("Segoe UI", 8, "bold"), fill=label_fill)

        kc_w, kc_h = 20, 15
        kx, ky = cx, self.h * 0.83
        self._rounded_rect(kx - kc_w / 2, ky - kc_h / 2, kx + kc_w / 2, ky + kc_h / 2, 4,
                           fill=BG if self.state != "active" else self.base_color,
                           outline=BORDER if self.state != "active" else self.hi_color,
                           width=1)
        self.create_text(kx, ky, text=self.keycap, font=("Consolas", 8, "bold"),
                         fill=TEXT_MUTED if self.state != "active" else "#0a0a12")

    def set_state(self, state):
        if self.state != state:
            self.state = state
            self._draw()

    def set_enabled(self, enabled):
        self.enabled = enabled
        self.configure(cursor="hand2" if enabled else "arrow")
        if not enabled:
            self.state = "idle"
        self._draw()

    def _press(self, _=None):
        if not self.enabled:
            return
        self.set_state("active")
        self.on_press()

    def _release(self, _=None):
        if not self.enabled:
            return
        self.set_state("hover")
        self.on_release()

    def _enter(self, _=None):
        if not self.enabled:
            return
        if self.state != "active":
            self.set_state("hover")

    def _leave(self, _=None):
        if not self.enabled:
            return
        if self.state == "active":
            self.on_release()
        self.set_state("idle")

    def key_activate(self):
        if self.enabled:
            self.set_state("active")

    def key_deactivate(self):
        self.set_state("idle")


class HomeButton(tk.Canvas):
    """Circular HOME button with the same raised-bevel look as the other
    controls. Used in both the Jog panel and the P2P panel — pressing
    either one sends the same HOME command."""

    def __init__(self, parent, command, size=78):
        super().__init__(parent, width=size, height=size, bg=PANEL_BG,
                         highlightthickness=0, cursor="hand2")
        self.size = size
        self.command = command
        self.state = "idle"
        self.enabled = True
        self._draw()
        self.bind("<ButtonPress-1>", lambda e: self._on_press())
        self.bind("<ButtonRelease-1>", lambda e: self._on_release())
        self.bind("<Enter>", lambda e: self._on_enter())
        self.bind("<Leave>", lambda e: self._on_leave())

    def _draw(self):
        self.delete("all")
        s = self.size
        if not self.enabled:
            fill, outline, glyph_fill, text_fill = PANEL_BG, BORDER, TEXT_MUTED, TEXT_MUTED
        elif self.state == "active":
            fill, outline, glyph_fill, text_fill = "#58c9ff", "#58c9ff", "#0a0a12", "#0a0a12"
        elif self.state == "hover":
            fill, outline, glyph_fill, text_fill = PANEL_BG, ACCENT_CYAN, ACCENT_CYAN, TEXT_MUTED
        else:
            fill, outline, glyph_fill, text_fill = PANEL_BG, BORDER, TEXT_MUTED, TEXT_MUTED

        if self.enabled and self.state != "active":
            dark = BORDER if fill == PANEL_BG else _shade(fill, 0.6)
            light = "#3a3a52" if fill == PANEL_BG else _shade(fill, 1.6)
            self.create_oval(8, 8, s - 6, s - 6, fill=dark, outline="")
            self.create_oval(6, 6, s - 8, s - 8, fill=fill, outline=outline, width=1.5)
            self.create_arc(6, 6, s - 8, s - 8, start=55, extent=150, style="arc",
                            outline=light, width=2)
        else:
            self.create_oval(6, 6, s - 6, s - 6, fill=fill, outline=outline, width=1.5)

        cx = s / 2
        self.create_text(cx, s * 0.42, text="\u2302", font=("Segoe UI", 20), fill=glyph_fill)
        self.create_text(cx, s * 0.70, text="HOME", font=("Segoe UI", 8, "bold"), fill=text_fill)

    def set_enabled(self, enabled):
        self.enabled = enabled
        self.configure(cursor="hand2" if enabled else "arrow")
        self.state = "idle"
        self._draw()

    def _on_press(self):
        if self.enabled:
            self.state = "active"
            self._draw()

    def _on_release(self):
        if not self.enabled:
            return
        self.state = "hover"
        self._draw()
        if self.command:
            self.command()

    def _on_enter(self):
        if self.enabled and self.state != "active":
            self.state = "hover"
            self._draw()

    def _on_leave(self):
        if not self.enabled:
            return
        self.state = "idle"
        self._draw()


class RobotControlApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Robot Motion Controller — P2P + Joystick (v5)")
        self.root.configure(bg=BG)
        self.root.geometry("1260x900")
        self.root.minsize(900, 500)

        # Serial / connection
        self.ser = None
        self.is_connected = False
        self.hw_confirmed = False
        self._ping_timeout_job = None
        self._heartbeat_job = None
        self._missed_beats = 0

        # Settings (PID + velocity/acceleration) — sent to the board via
        # SET_PARAMS, applied locally to the jog-simulation speeds too.
        self.settings = {
            "kp": DEFAULT_KP, "ki": DEFAULT_KI, "kd": DEFAULT_KD,
            "vel_rpm": DEFAULT_VEL_RPM, "accel_rpm_s": DEFAULT_ACCEL_RPM_S,
        }

        # P2P motion — now a 2-point (A -> B) program computed via inverse
        # kinematics, loaded onto the board, then run as a separate step.
        self.is_running = False
        self.is_homing = False
        self.anim_job = None
        self.loaded_program = None       # (d1A,rot A,a1A,a2A, d1B,rotB,a1B,a2B) once LOAD succeeds
        self.elbow_config = ELBOW_CONFIGS[0]
        self.elbow_buttons = {}
        self.motion_lock_widgets = []     # buttons/entries disabled during RUN/HOME (except STOP/ESTOP)

        # Jog motion
        self.jog_active = set()
        self.jog_pads = {}
        self._jog_sim_job = None
        self.sim_rot = 0.0
        self.sim_arm = 0.0
        self.sim_z = 0.0
        self.boost_index = 0             # BOOST_LEVELS[0] == 1.0x (off)
        self.rot_limit = {"ROT_CW": False, "ROT_CCW": False}
        self.z_limit = {"Z_UP": False, "Z_DOWN": False}

        # Mode
        self.mode = "P2P"

        self._configure_styles()
        self._build_ui()
        self._bind_keys()
        self.refresh_com_ports()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ══════════════════════════════════════════════════════════════════
    # STYLES
    # ══════════════════════════════════════════════════════════════════
    def _configure_styles(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(
            "Cyan.Horizontal.TProgressbar",
            troughcolor=ENTRY_BG, background=ACCENT_CYAN,
            bordercolor=PANEL_BG, lightcolor=ACCENT_CYAN,
            darkcolor=ACCENT_CYAN, thickness=14,
        )
        style.configure(
            "TCombobox",
            fieldbackground=ENTRY_BG, background=PANEL_BG,
            foreground=TEXT_LIGHT, bordercolor=BORDER,
            arrowcolor=ACCENT_CYAN, padding=4
        )
        style.map("TCombobox",
                  fieldbackground=[("readonly", ENTRY_BG)],
                  foreground=[("readonly", TEXT_LIGHT)])

    # ══════════════════════════════════════════════════════════════════
    # UI BUILD
    # ══════════════════════════════════════════════════════════════════
    def _build_ui(self):
        self.status_var = tk.StringVar(value="READY — Hardware Offline")
        status_bar = tk.Label(self.root, textvariable=self.status_var, anchor="w",
                              bg="#08080f", fg=ACCENT_CYAN, font=("Consolas", 10), padx=12, pady=6)
        status_bar.pack(side="bottom", fill="x")

        # Everything else lives inside a scrollable canvas. The window's
        # content has grown (Settings, IK readouts, extra P2P/jog
        # controls) and different screens have different usable heights
        # — scrolling guarantees Section 4 (Event Log) is always
        # reachable instead of silently overflowing past the bottom of
        # a shorter screen.
        scroll_container = tk.Frame(self.root, bg=BG)
        scroll_container.pack(side="top", fill="both", expand=True)

        self._main_canvas = tk.Canvas(scroll_container, bg=BG, highlightthickness=0)
        vscroll = ttk.Scrollbar(scroll_container, orient="vertical", command=self._main_canvas.yview)
        self._main_canvas.configure(yscrollcommand=vscroll.set)
        self._main_canvas.pack(side="left", fill="both", expand=True)
        vscroll.pack(side="right", fill="y")

        main = tk.Frame(self._main_canvas, bg=BG)
        main_win_id = self._main_canvas.create_window((0, 0), window=main, anchor="nw")

        def _sync_scrollregion(_event=None):
            self._main_canvas.configure(scrollregion=self._main_canvas.bbox("all"))

        def _sync_main_width(event):
            self._main_canvas.itemconfig(main_win_id, width=event.width)

        main.bind("<Configure>", _sync_scrollregion)
        self._main_canvas.bind("<Configure>", _sync_main_width)

        def _on_mousewheel(event):
            self._main_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        self._main_canvas.bind_all("<MouseWheel>", _on_mousewheel)     # Windows / macOS
        self._main_canvas.bind_all("<Button-4>", lambda e: self._main_canvas.yview_scroll(-3, "units"))  # Linux
        self._main_canvas.bind_all("<Button-5>", lambda e: self._main_canvas.yview_scroll(3, "units"))   # Linux

        # ── SECTION 1: HARDWARE CONNECTION ──────────────────────────────
        s0, _ = self._make_section(main, "1. HARDWARE CONNECTION (SERIAL PORT)", 0)
        conn_frame = tk.Frame(s0, bg=PANEL_BG)
        conn_frame.pack(fill="x", pady=2)

        tk.Label(conn_frame, text="PORT:", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=("Segoe UI", 9, "bold")).pack(side="left", padx=(0, 5))
        self.com_var = tk.StringVar()
        self.com_combo = ttk.Combobox(conn_frame, textvariable=self.com_var, width=12, state="readonly")
        self.com_combo.pack(side="left", padx=(0, 10))

        RoundedButton(conn_frame, text="REFRESH", icon="🔄", bg_color="#3a3a52", fg_color=TEXT_LIGHT,
                      width=100, height=34, command=self.refresh_com_ports).pack(side="left", padx=(0, 16))

        tk.Label(conn_frame, text="BAUD:", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=("Segoe UI", 9, "bold")).pack(side="left", padx=(0, 5))
        self.baud_var = tk.StringVar(value=DEFAULT_BAUD_RATE)
        self.baud_combo = ttk.Combobox(conn_frame, textvariable=self.baud_var, width=9, state="readonly",
                                       values=["9600", "19200", "38400", "57600", "115200", "230400"])
        self.baud_combo.pack(side="left", padx=(0, 16))

        self.btn_connect = RoundedButton(conn_frame, text="CONNECT", icon="🔌", bg_color=ACCENT_GREEN,
                                         width=130, height=34, command=self.toggle_connection)
        self.btn_connect.pack(side="left", padx=(0, 10))

        RoundedButton(conn_frame, text="PING", icon="📡", bg_color=ACCENT_PURPLE, fg_color=TEXT_LIGHT,
                      width=90, height=34, command=self.manual_ping).pack(side="left", padx=(0, 10))

        RoundedButton(conn_frame, text="SETTINGS", icon="⚙", bg_color="#3a3a52", fg_color=TEXT_LIGHT,
                      width=130, height=34, command=self.open_settings_dialog).pack(side="left", padx=(0, 16))

        handshake_frame = tk.Frame(conn_frame, bg=PANEL_BG)
        handshake_frame.pack(side="right", padx=5)
        self.com_led_card = self._make_status_led(handshake_frame, "COM PORT", "CLOSED", ACCENT_RED)
        self.com_led_card["frame"].pack(side="left", padx=(0, 8))
        self.hw_led_card = self._make_status_led(handshake_frame, "CLEARCORE IO0", "NO SIGNAL", ACCENT_RED)
        self.hw_led_card["frame"].pack(side="left", padx=(0, 8))
        self.hb_led_card = self._make_status_led(handshake_frame, "HEARTBEAT (3s)", "IDLE", TEXT_MUTED)
        self.hb_led_card["frame"].pack(side="left")

        # ── SECTION 2: CONTROL MODE ──────────────────────────────────────
        s_mode, _ = self._make_section(main, "2. CONTROL MODE", 1)
        mode_row = tk.Frame(s_mode, bg=PANEL_BG)
        mode_row.pack(fill="x")
        self.btn_mode_p2p = RoundedButton(mode_row, text="POINT TO POINT", icon="🎯",
                                          bg_color=ACCENT_CYAN, fg_color="#0a0a12",
                                          width=250, height=42, command=lambda: self.set_mode("P2P"))
        self.btn_mode_p2p.pack(side="left", padx=(0, 12))
        self.btn_mode_jog = RoundedButton(mode_row, text="JOYSTICK", icon="🕹",
                                          bg_color="#3a3a52", fg_color=TEXT_LIGHT,
                                          width=250, height=42, command=lambda: self.set_mode("JOG"))
        self.btn_mode_jog.pack(side="left")
        tk.Label(mode_row, text="  Switching mode auto-stops all motion first.",
                 bg=PANEL_BG, fg=TEXT_MUTED, font=("Segoe UI", 8, "italic")).pack(side="left", padx=12)

        # ── SECTION 3: MOTION CONTROL (dynamic content) ──────────────────
        s_motion, self.motion_title_label = self._make_section(
            main, "3. MOTION CONTROL — POINT TO POINT", 2)
        self.p2p_frame = tk.Frame(s_motion, bg=PANEL_BG)
        self.jog_frame = tk.Frame(s_motion, bg=PANEL_BG)
        self._build_p2p_panel(self.p2p_frame)
        self._build_jog_panel(self.jog_frame)
        self.p2p_frame.pack(fill="both", expand=True)
        self.motion_lock_widgets += [self.btn_mode_p2p, self.btn_mode_jog]

        # ── SECTION 4: EVENT LOG ──────────────────────────────────────────
        log_section, _ = self._make_section(main, "4. EVENT LOG", 3, expand_vertically=True)
        log_frame = tk.Frame(log_section, bg=PANEL_BG)
        log_frame.pack(fill="both", expand=True)
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical")
        self.log_text = tk.Text(log_frame, height=12, bg=LED_BG, fg=TEXT_LIGHT,
                                insertbackground=TEXT_LIGHT, relief="flat",
                                font=("Consolas", 10), wrap="word",
                                yscrollcommand=scrollbar.set, state="disabled")
        scrollbar.configure(command=self.log_text.yview)
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        self.log_text.tag_config("tx", foreground=ACCENT_CYAN)
        self.log_text.tag_config("rx", foreground=ACCENT_GREEN)
        self.log_text.tag_config("warn", foreground=ACCENT_ORANGE)
        self.log_text.tag_config("error", foreground=ACCENT_RED)
        self.log_text.tag_config("default", foreground=TEXT_LIGHT)

        self._log("System initialized. Mode: POINT TO POINT.")
        self._log(f"Heartbeat: PING every {HEARTBEAT_INTERVAL_MS // 1000}s | "
                 f"Timeout {PING_TIMEOUT_MS // 1000}s | Lost after {MISSED_BEAT_LIMIT} misses")

    # ── UI HELPERS ────────────────────────────────────────────────────────
    def _make_section(self, parent, title, row, expand_vertically=False):
        outer = tk.Frame(parent, bg=PANEL_BG, highlightbackground=BORDER, highlightthickness=1)
        if expand_vertically:
            outer.grid(row=row, column=0, sticky="nsew", padx=14, pady=6)
            parent.grid_rowconfigure(row, weight=1)
        else:
            outer.grid(row=row, column=0, sticky="ew", padx=14, pady=4)
        parent.grid_columnconfigure(0, weight=1)
        title_label = tk.Label(outer, text=title, bg=PANEL_BG, fg=ACCENT_CYAN,
                               font=("Segoe UI", 10, "bold"), anchor="w")
        title_label.pack(fill="x", padx=12, pady=(6, 4))
        tk.Frame(outer, bg=BORDER, height=1).pack(fill="x", padx=12)
        content = tk.Frame(outer, bg=PANEL_BG)
        content.pack(fill="both", expand=True, padx=12, pady=6)
        return content, title_label

    def _make_status_led(self, parent, label, initial_text, initial_color):
        outer = tk.Frame(parent, bg=PANEL_BG)
        tk.Label(outer, text=label, bg=PANEL_BG, fg=TEXT_MUTED,
                 font=("Segoe UI", 7, "bold")).pack()
        card = tk.Frame(outer, bg=LED_BG, highlightbackground=initial_color, highlightthickness=2)
        card.pack()
        text_var = tk.StringVar(value=initial_text)
        led_label = tk.Label(card, textvariable=text_var, bg=LED_BG, fg=initial_color,
                             font=("Consolas", 10, "bold"), padx=14, pady=5)
        led_label.pack()
        return {"frame": outer, "card": card, "label": led_label, "var": text_var}

    def _set_led(self, led_dict, text, color):
        led_dict["var"].set(text)
        led_dict["card"].config(highlightbackground=color)
        led_dict["label"].config(fg=color)

    def _make_coord_card(self, parent, col, badge_text, badge_color, var):
        parent.grid_columnconfigure(col, weight=1)
        card = tk.Frame(parent, bg=PANEL_BG, highlightbackground=badge_color, highlightthickness=1)
        card.grid(row=0, column=col, padx=6, pady=2, sticky="ew")
        inner = tk.Frame(card, bg=PANEL_BG)
        inner.pack(padx=8, pady=4, fill="x")
        tk.Label(inner, text=badge_text, bg=badge_color, fg="#0a0a12",
                 font=("Segoe UI", 9, "bold"), width=3).pack(side="left", padx=(0, 6))
        entry = tk.Entry(inner, textvariable=var, bg=ENTRY_BG, fg=TEXT_LIGHT, relief="flat",
                         font=("Segoe UI", 11), justify="center", width=7)
        entry.pack(side="left", fill="x", expand=True)
        tk.Label(inner, text="mm", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=("Segoe UI", 9)).pack(side="left", padx=(4, 0))
        return entry

    def _make_led_card(self, parent, col, title, var, color):
        parent.grid_columnconfigure(col, weight=1)
        card = tk.Frame(parent, bg=LED_BG, highlightbackground=color, highlightthickness=1)
        card.grid(row=0, column=col, padx=4, sticky="ew")
        tk.Label(card, text=title, bg=LED_BG, fg=TEXT_MUTED,
                 font=("Segoe UI", 8, "bold")).pack(pady=(4, 0))
        tk.Label(card, textvariable=var, bg=LED_BG, fg=color,
                 font=("Consolas", 14, "bold")).pack(pady=(0, 4))

    # ══════════════════════════════════════════════════════════════════
    # SETTINGS DIALOG (PID + velocity/acceleration)
    # ══════════════════════════════════════════════════════════════════
    def open_settings_dialog(self):
        dlg = tk.Toplevel(self.root)
        dlg.title("Settings")
        dlg.configure(bg=PANEL_BG)
        dlg.resizable(False, False)
        dlg.transient(self.root)
        dlg.grab_set()

        tk.Label(dlg, text="⚙  SETTINGS", bg=PANEL_BG, fg=ACCENT_CYAN,
                 font=("Segoe UI", 13, "bold")).pack(pady=(16, 2), padx=24)
        tk.Label(dlg, text="Cài đặt PID và tốc độ / gia tốc động cơ",
                 bg=PANEL_BG, fg=TEXT_MUTED, font=("Segoe UI", 8, "italic")).pack(pady=(0, 14))

        form = tk.Frame(dlg, bg=PANEL_BG)
        form.pack(padx=24, pady=(0, 6))

        self._set_kp_v = tk.StringVar(value=str(self.settings["kp"]))
        self._set_ki_v = tk.StringVar(value=str(self.settings["ki"]))
        self._set_kd_v = tk.StringVar(value=str(self.settings["kd"]))
        self._set_vel_v = tk.StringVar(value=str(self.settings["vel_rpm"]))
        self._set_accel_v = tk.StringVar(value=str(self.settings["accel_rpm_s"]))

        def add_row(r, label, var, unit):
            tk.Label(form, text=label, bg=PANEL_BG, fg=TEXT_LIGHT,
                     font=("Segoe UI", 10)).grid(row=r, column=0, sticky="w", pady=6, padx=(0, 12))
            e = tk.Entry(form, textvariable=var, bg=ENTRY_BG, fg=TEXT_LIGHT, relief="flat",
                        font=("Consolas", 11), width=12, justify="right")
            e.grid(row=r, column=1, ipady=3)
            tk.Label(form, text=unit, bg=PANEL_BG, fg=TEXT_MUTED, width=7, anchor="w",
                     font=("Segoe UI", 9)).grid(row=r, column=2, sticky="w", padx=(8, 0))

        add_row(0, "Kp (PID)", self._set_kp_v, "")
        add_row(1, "Ki (PID)", self._set_ki_v, "")
        add_row(2, "Kd (PID)", self._set_kd_v, "")
        tk.Frame(form, bg=BORDER, height=1).grid(row=3, column=0, columnspan=3, sticky="ew", pady=8)
        add_row(4, "Vận tốc / Velocity", self._set_vel_v, "RPM")
        add_row(5, "Gia tốc / Accel.", self._set_accel_v, "RPM/s")

        note = ("Ghi chú: nếu driver là step/dir hở (không hồi tiếp), Kp/Ki/Kd\n"
               "chỉ được lưu & gửi xuống board, chưa chắc điều khiển vòng kín\n"
               "thực tế — phụ thuộc vào driver motor thật của bạn.")
        tk.Label(dlg, text=note, bg=PANEL_BG, fg=TEXT_MUTED, font=("Segoe UI", 7, "italic"),
                 justify="left").pack(padx=24, pady=(4, 10), anchor="w")

        btn_row = tk.Frame(dlg, bg=PANEL_BG)
        btn_row.pack(pady=(4, 18))
        RoundedButton(btn_row, text="RESET", icon="↺", bg_color="#3a3a52", fg_color=TEXT_LIGHT,
                      width=110, height=36, command=self._settings_reset_form).pack(side="left", padx=6)
        RoundedButton(btn_row, text="APPLY", icon="✔", bg_color=ACCENT_GREEN,
                      width=110, height=36, command=lambda: self._settings_apply(dlg)).pack(side="left", padx=6)
        RoundedButton(btn_row, text="CLOSE", bg_color=ACCENT_RED,
                      width=110, height=36, command=dlg.destroy).pack(side="left", padx=6)

    def _settings_reset_form(self):
        """Resets the FORM fields to factory defaults. Does not take
        effect on the robot until APPLY is pressed."""
        self._set_kp_v.set(str(DEFAULT_KP))
        self._set_ki_v.set(str(DEFAULT_KI))
        self._set_kd_v.set(str(DEFAULT_KD))
        self._set_vel_v.set(str(DEFAULT_VEL_RPM))
        self._set_accel_v.set(str(DEFAULT_ACCEL_RPM_S))

    def _settings_apply(self, dlg):
        try:
            kp = float(self._set_kp_v.get())
            ki = float(self._set_ki_v.get())
            kd = float(self._set_kd_v.get())
            vel_rpm = float(self._set_vel_v.get())
            accel_rpm_s = float(self._set_accel_v.get())
            if vel_rpm <= 0 or accel_rpm_s <= 0:
                raise ValueError("Vận tốc và gia tốc phải > 0")
        except ValueError as e:
            messagebox.showerror("Lỗi", f"Giá trị không hợp lệ: {e}")
            return

        self.settings.update(kp=kp, ki=ki, kd=kd, vel_rpm=vel_rpm, accel_rpm_s=accel_rpm_s)
        self._send_serial(f"SET_PARAMS:{kp},{ki},{kd},{vel_rpm},{accel_rpm_s}")
        self._log(f"Settings applied: Kp={kp} Ki={ki} Kd={kd} "
                 f"Vel={vel_rpm} RPM Accel={accel_rpm_s} RPM/s")
        dlg.destroy()

    # ══════════════════════════════════════════════════════════════════
    # P2P PANEL
    # ══════════════════════════════════════════════════════════════════
    def _build_p2p_panel(self, parent):
        coord_row = tk.Frame(parent, bg=PANEL_BG)
        coord_row.pack(fill="x")

        start_col = tk.Frame(coord_row, bg=PANEL_BG)
        start_col.pack(side="left", fill="both", expand=True, padx=(0, 8))
        tk.Label(start_col, text="POINT A (mm, Cartesian)", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=("Segoe UI", 8, "bold")).pack(anchor="w")
        start_grid = tk.Frame(start_col, bg=PANEL_BG)
        start_grid.pack(fill="x", pady=(4, 0))
        self.x0_v = tk.StringVar(value="0")
        self.y0_v = tk.StringVar(value="0")
        self.z0_v = tk.StringVar(value="0")
        self._make_coord_card(start_grid, 0, "X0", AXIS_X_COLOR, self.x0_v)
        self._make_coord_card(start_grid, 1, "Y0", AXIS_Y_COLOR, self.y0_v)
        self._make_coord_card(start_grid, 2, "Z0", AXIS_Z_COLOR, self.z0_v)

        target_col = tk.Frame(coord_row, bg=PANEL_BG)
        target_col.pack(side="left", fill="both", expand=True, padx=(8, 0))
        tk.Label(target_col, text="POINT B (mm, Cartesian)", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=("Segoe UI", 8, "bold")).pack(anchor="w")
        target_grid = tk.Frame(target_col, bg=PANEL_BG)
        target_grid.pack(fill="x", pady=(4, 0))
        self.x1_v = tk.StringVar(value="100")
        self.y1_v = tk.StringVar(value="100")
        self.z1_v = tk.StringVar(value="100")
        self._make_coord_card(target_grid, 0, "X1", AXIS_X_COLOR, self.x1_v)
        self._make_coord_card(target_grid, 1, "Y1", AXIS_Y_COLOR, self.y1_v)
        self.z1_entry = self._make_coord_card(target_grid, 2, "Z1", AXIS_Z_COLOR, self.z1_v)
        self.z0_v.trace_add("write", self._sync_z_for_both_mode)

        # ── Arm/elbow configuration selector ────────────────────────────
        elbow_row = tk.Frame(parent, bg=PANEL_BG)
        elbow_row.pack(fill="x", pady=(14, 0))
        tk.Label(elbow_row, text="CHỌN CÁNH TAY / ARM SELECT:", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=("Segoe UI", 8, "bold")).pack(side="left", padx=(0, 10))
        for cfg in ELBOW_CONFIGS:
            b = RoundedButton(elbow_row, text=cfg, bg_color=(ACCENT_CYAN if cfg == self.elbow_config else "#3a3a52"),
                              fg_color=("#0a0a12" if cfg == self.elbow_config else TEXT_LIGHT),
                              width=90, height=32, command=lambda c=cfg: self._set_elbow_config(c))
            b.pack(side="left", padx=4)
            self.elbow_buttons[cfg] = b
        tk.Label(elbow_row, text="  (A1M/A2M = dùng 1 tay · BOTH = 2 tay đồng thời, 2 điểm phải cùng hướng qua tâm, khác bán kính)",
                 bg=PANEL_BG, fg=TEXT_MUTED, font=("Segoe UI", 8, "italic")).pack(side="left")

        # ── Computed inverse-kinematics targets (what will be LOADed) ──
        tk.Label(parent, text="COMPUTED JOINT TARGETS (d1 / θ2 RM / θ3 A1M / θ4 A2M)",
                 bg=PANEL_BG, fg=TEXT_MUTED, font=("Segoe UI", 8, "bold")).pack(anchor="w", pady=(14, 0))

        calc_a_row = tk.Frame(parent, bg=PANEL_BG)
        calc_a_row.pack(fill="x", pady=(4, 0))
        tk.Label(calc_a_row, text="A", bg=PANEL_BG, fg=AXIS_X_COLOR, width=2,
                 font=("Consolas", 11, "bold")).pack(side="left")
        self.calc_a_d1_v = tk.StringVar(value="—")
        self.calc_a_rot_v = tk.StringVar(value="—")
        self.calc_a_a1_v = tk.StringVar(value="—")
        self.calc_a_a2_v = tk.StringVar(value="—")
        calc_a_cards = tk.Frame(calc_a_row, bg=PANEL_BG)
        calc_a_cards.pack(side="left", fill="x", expand=True)
        self._make_led_card(calc_a_cards, 0, "D1", self.calc_a_d1_v, AXIS_Z_COLOR)
        self._make_led_card(calc_a_cards, 1, "ROT", self.calc_a_rot_v, ROT_COLOR)
        self._make_led_card(calc_a_cards, 2, "A1M", self.calc_a_a1_v, ARM_COLOR)
        self._make_led_card(calc_a_cards, 3, "A2M", self.calc_a_a2_v, ACCENT_PURPLE)

        calc_b_row = tk.Frame(parent, bg=PANEL_BG)
        calc_b_row.pack(fill="x", pady=(4, 0))
        tk.Label(calc_b_row, text="B", bg=PANEL_BG, fg=AXIS_X_COLOR, width=2,
                 font=("Consolas", 11, "bold")).pack(side="left")
        self.calc_b_d1_v = tk.StringVar(value="—")
        self.calc_b_rot_v = tk.StringVar(value="—")
        self.calc_b_a1_v = tk.StringVar(value="—")
        self.calc_b_a2_v = tk.StringVar(value="—")
        calc_b_cards = tk.Frame(calc_b_row, bg=PANEL_BG)
        calc_b_cards.pack(side="left", fill="x", expand=True)
        self._make_led_card(calc_b_cards, 0, "D1", self.calc_b_d1_v, AXIS_Z_COLOR)
        self._make_led_card(calc_b_cards, 1, "ROT", self.calc_b_rot_v, ROT_COLOR)
        self._make_led_card(calc_b_cards, 2, "A1M", self.calc_b_a1_v, ARM_COLOR)
        self._make_led_card(calc_b_cards, 3, "A2M", self.calc_b_a2_v, ACCENT_PURPLE)

        # ── Live telemetry (real joint feedback from the board) ─────────
        tk.Label(parent, text="LIVE TELEMETRY (from ClearCore)", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=("Segoe UI", 8, "bold")).pack(anchor="w", pady=(16, 0))
        self.curr_d1_v = tk.StringVar(value="0.00 mm")
        self.curr_rot_v = tk.StringVar(value="0.00 deg")
        self.curr_a1_v = tk.StringVar(value="0.00 deg")
        self.curr_a2_v = tk.StringVar(value="0.00 deg")
        led_row = tk.Frame(parent, bg=PANEL_BG)
        led_row.pack(fill="x", pady=(4, 0))
        self._make_led_card(led_row, 0, "D1_CURR", self.curr_d1_v, AXIS_Z_COLOR)
        self._make_led_card(led_row, 1, "ROT_CURR", self.curr_rot_v, ROT_COLOR)
        self._make_led_card(led_row, 2, "A1M_CURR", self.curr_a1_v, ARM_COLOR)
        self._make_led_card(led_row, 3, "A2M_CURR", self.curr_a2_v, ACCENT_PURPLE)

        self.calc_xyz_v = tk.StringVar(value="Cartesian (tính từ động học thuận): X=0.00  Y=0.00  Z=0.00 mm")
        tk.Label(parent, textvariable=self.calc_xyz_v, bg=PANEL_BG, fg=TEXT_MUTED,
                 font=("Consolas", 9)).pack(anchor="w", pady=(4, 0))

        progress_block = tk.Frame(parent, bg=PANEL_BG)
        progress_block.pack(fill="x", pady=(10, 0))
        tk.Label(progress_block, text="PROGRESS", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=("Segoe UI", 8, "bold")).pack(anchor="w")
        bar_row = tk.Frame(progress_block, bg=PANEL_BG)
        bar_row.pack(fill="x", pady=(4, 0))
        self.progress_var = tk.DoubleVar(value=0.0)
        self.progress_bar = ttk.Progressbar(bar_row, orient="horizontal", mode="determinate",
                                            style="Cyan.Horizontal.TProgressbar",
                                            variable=self.progress_var, maximum=100)
        self.progress_bar.pack(side="left", fill="x", expand=True)
        self.progress_pct_var = tk.StringVar(value="0%")
        tk.Label(bar_row, textvariable=self.progress_pct_var, bg=PANEL_BG, fg=ACCENT_CYAN,
                 font=("Consolas", 10, "bold"), width=5).pack(side="left", padx=(8, 0))

        btn_row1 = tk.Frame(parent, bg=PANEL_BG)
        btn_row1.pack(pady=(18, 6))
        self.btn_p2p_load = RoundedButton(btn_row1, text="LOAD PARAMETERS", icon="⇩", bg_color=ACCENT_PURPLE,
                                          fg_color=TEXT_LIGHT, width=210, height=44,
                                          command=self.p2p_load_parameters)
        self.btn_p2p_load.pack(side="left", padx=6)
        self.btn_p2p_run = RoundedButton(btn_row1, text="RUN PROGRAM", icon="▶", bg_color=ACCENT_GREEN,
                                         width=190, height=44, command=self.p2p_run_program)
        self.btn_p2p_run.pack(side="left", padx=6)
        self.p2p_home_btn = HomeButton(btn_row1, command=self._home_clicked, size=56)
        self.p2p_home_btn.pack(side="left", padx=6)

        btn_row2 = tk.Frame(parent, bg=PANEL_BG)
        btn_row2.pack(pady=(0, 4))
        RoundedButton(btn_row2, text="STOP", icon="■", bg_color=ACCENT_ORANGE,
                      width=190, height=44, command=self.p2p_stop).pack(side="left", padx=6)
        RoundedButton(btn_row2, text="EMERGENCY STOP", icon="⬛", bg_color=ACCENT_RED,
                      width=210, height=44, command=self.emergency_stop_all).pack(side="left", padx=6)

        self.motion_lock_widgets += [
            self.btn_p2p_load, self.btn_p2p_run, self.p2p_home_btn,
            self.elbow_buttons["A1M"], self.elbow_buttons["A2M"],
        ]

    # ══════════════════════════════════════════════════════════════════
    # JOYSTICK PANEL
    # ══════════════════════════════════════════════════════════════════
    def _build_jog_panel(self, parent):
        top_row = tk.Frame(parent, bg=PANEL_BG)
        top_row.pack()

        motion_card = tk.Frame(top_row, bg=PANEL_BG, highlightbackground=BORDER, highlightthickness=1)
        motion_card.pack(side="left", padx=(0, 12))
        tk.Label(motion_card, text="ROTATION + ARM  (A/D · W/S)", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=("Segoe UI", 8, "bold")).pack(anchor="w", padx=12, pady=(8, 0))
        grid = tk.Frame(motion_card, bg=PANEL_BG)
        grid.pack(padx=14, pady=(6, 14))

        self._jog_pad(grid, 0, 1, "\u25b2", "ARM FWD", "W", "ARM_FWD", "ARM_STOP", ACCENT_CYAN, "#58c9ff")
        self._jog_pad(grid, 1, 0, "\u25c0", "ROT CCW", "A", "ROT_CCW", "ROT_STOP", ACCENT_CYAN, "#58c9ff")
        self._jog_pad(grid, 1, 2, "\u25b6", "ROT CW", "D", "ROT_CW", "ROT_STOP", ACCENT_CYAN, "#58c9ff")
        self._jog_pad(grid, 2, 1, "\u25bc", "ARM BACK", "S", "ARM_BACK", "ARM_STOP", ACCENT_CYAN, "#58c9ff")

        self.home_btn = HomeButton(grid, command=self._home_clicked, size=78)
        self.home_btn.grid(row=1, column=1, padx=4, pady=4)

        z_card = tk.Frame(top_row, bg=PANEL_BG, highlightbackground=BORDER, highlightthickness=1)
        z_card.pack(side="left", fill="y")
        tk.Label(z_card, text="Z AXIS  (I/K)", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=("Segoe UI", 8, "bold")).pack(anchor="w", padx=12, pady=(8, 0))
        zgrid = tk.Frame(z_card, bg=PANEL_BG)
        zgrid.pack(padx=14, pady=(6, 14))
        self._jog_pad(zgrid, 0, 0, "\u25b2", "Z UP", "I", "Z_UP", "Z_STOP", ACCENT_GREEN, "#4ee08a")
        tk.Frame(zgrid, bg=PANEL_BG, height=8).grid(row=1, column=0)
        self._jog_pad(zgrid, 2, 0, "\u25bc", "Z DOWN", "K", "Z_DOWN", "Z_STOP", ACCENT_GREEN, "#4ee08a")

        tk.Label(parent, text="LIVE JOG POSITION", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=("Segoe UI", 8, "bold")).pack(anchor="w", pady=(16, 0))
        self.rot_pos_v = tk.StringVar(value="0.00 deg")
        self.arm_pos_v = tk.StringVar(value="0.00 deg")
        self.jz_pos_v = tk.StringVar(value="0.00 mm")
        jog_led_row = tk.Frame(parent, bg=PANEL_BG)
        jog_led_row.pack(fill="x", pady=(4, 0))
        self._make_led_card(jog_led_row, 0, "ROT_POS", self.rot_pos_v, ROT_COLOR)
        self._make_led_card(jog_led_row, 1, "ARM_POS", self.arm_pos_v, ARM_COLOR)
        self._make_led_card(jog_led_row, 2, "Z_POS", self.jz_pos_v, JZ_COLOR)

        status_row = tk.Frame(parent, bg=PANEL_BG)
        status_row.pack(fill="x", pady=(14, 0))
        self.jog_dot = tk.Canvas(status_row, width=10, height=10, bg=PANEL_BG, highlightthickness=0)
        self.jog_dot.pack(side="left", padx=(0, 6))
        self._jog_dot_id = self.jog_dot.create_oval(1, 1, 9, 9, fill=TEXT_MUTED, outline="")
        self.jog_status_var = tk.StringVar(value="Idle — hold a key or button to jog")
        tk.Label(status_row, textvariable=self.jog_status_var, bg=PANEL_BG, fg=TEXT_MUTED,
                 font=("Consolas", 9)).pack(side="left")

        self.boost_btn = RoundedButton(status_row, text="BOOST: OFF", icon="⚡", bg_color="#3a3a52",
                                       fg_color=TEXT_LIGHT, width=140, height=30, command=self._cycle_boost)
        self.boost_btn.pack(side="right")
        tk.Label(status_row, text="Jog quá chậm? Nhấn BOOST để x1.5 / x2:",
                 bg=PANEL_BG, fg=TEXT_MUTED, font=("Segoe UI", 8, "italic")).pack(side="right", padx=(0, 8))

        RoundedButton(parent, text="EMERGENCY STOP (SPACE)", icon="⬛", bg_color=ACCENT_RED,
                      width=440, height=46, command=self.emergency_stop_all).pack(pady=(18, 4))
        tk.Label(parent, text="Hold a direction to move · release to stop · dead-man-switch (no latching)",
                 bg=PANEL_BG, fg=TEXT_MUTED, font=("Segoe UI", 8, "italic")).pack(pady=(2, 0))

        self.motion_lock_widgets += [self.home_btn]

    def _jog_pad(self, parent, row, col, glyph, label, keycap, start_cmd, stop_cmd, base, hi):
        pad = JogPad(parent, 78, 78, glyph, label, keycap, base, hi,
                     on_press=lambda: self.jog_start(start_cmd),
                     on_release=lambda: self.jog_stop(start_cmd, stop_cmd))
        pad.grid(row=row, column=col, padx=4, pady=4)
        self.jog_pads[start_cmd] = pad
        return pad

    def _home_clicked(self):
        stop_map = {
            "ROT_CW": "ROT_STOP", "ROT_CCW": "ROT_STOP",
            "ARM_FWD": "ARM_STOP", "ARM_BACK": "ARM_STOP",
            "Z_UP": "Z_STOP", "Z_DOWN": "Z_STOP",
        }
        for cmd in list(self.jog_active):
            self._send_serial(stop_map.get(cmd, cmd))
        self.jog_active.clear()
        for pad in self.jog_pads.values():
            pad.key_deactivate()
        self._refresh_jog_status()
        if self._jog_sim_job:
            self.root.after_cancel(self._jog_sim_job)
            self._jog_sim_job = None

        self.is_homing = True
        self._set_motion_locked(True)
        self._send_serial("HOME")

        self.jog_dot.itemconfig(self._jog_dot_id, fill=ACCENT_GREEN)
        self.jog_status_var.set("Homing...")
        self.status_var.set("HOMING — các nút khác đã bị khóa (trừ STOP/ESTOP)...")
        self._log("HOME command sent. Other motion controls locked until homing completes.")

        if not (self.is_connected and self.hw_confirmed):
            self.root.after(800, self._simulate_home_complete)

    def _simulate_home_complete(self):
        self.sim_rot = self.sim_arm = self.sim_z = 0.0
        self.rot_limit = {k: False for k in self.rot_limit}
        self.z_limit = {k: False for k in self.z_limit}
        self.rot_pos_v.set("0.00 deg")
        self.arm_pos_v.set("0.00 deg")
        self.jz_pos_v.set("0.00 mm")
        self.is_homing = False
        self._set_motion_locked(False)
        self.jog_dot.itemconfig(self._jog_dot_id, fill=TEXT_MUTED)
        self.jog_status_var.set("Idle — hold a key or button to jog")
        self.status_var.set("READY — Home position reached (simulated).")
        self._log("Homing complete (simulated).")

    # ══════════════════════════════════════════════════════════════════
    # MODE SWITCH
    # ══════════════════════════════════════════════════════════════════
    def set_mode(self, mode):
        if mode == self.mode:
            return
        self._stop_all_motion_for_mode_switch()
        self.mode = mode
        if mode == "P2P":
            self.jog_frame.pack_forget()
            self.p2p_frame.pack(fill="both", expand=True)
            self.motion_title_label.config(text="3. MOTION CONTROL — POINT TO POINT")
            self.btn_mode_p2p.set_config("POINT TO POINT", ACCENT_CYAN, icon="🎯", fg_color="#0a0a12")
            self.btn_mode_jog.set_config("JOYSTICK", "#3a3a52", icon="🕹", fg_color=TEXT_LIGHT)
        else:
            self.p2p_frame.pack_forget()
            self.jog_frame.pack(fill="both", expand=True)
            self.motion_title_label.config(text="3. MOTION CONTROL — JOYSTICK")
            self.btn_mode_p2p.set_config("POINT TO POINT", "#3a3a52", icon="🎯", fg_color=TEXT_LIGHT)
            self.btn_mode_jog.set_config("JOYSTICK", ACCENT_CYAN, icon="🕹", fg_color="#0a0a12")
        self.root.focus_set()
        self._log(f"Mode switched to {mode}.")

    def _stop_all_motion_for_mode_switch(self):
        if self.is_running:
            self.is_running = False
            if self.anim_job:
                self.root.after_cancel(self.anim_job)
                self.anim_job = None
            self._send_serial("STOP")

        if self.jog_active:
            stop_map = {
                "ROT_CW": "ROT_STOP", "ROT_CCW": "ROT_STOP",
                "ARM_FWD": "ARM_STOP", "ARM_BACK": "ARM_STOP",
                "Z_UP": "Z_STOP", "Z_DOWN": "Z_STOP",
            }
            for cmd in list(self.jog_active):
                self._send_serial(stop_map.get(cmd, cmd))
            self.jog_active.clear()
            for pad in self.jog_pads.values():
                pad.key_deactivate()
            self._refresh_jog_status()

        if self._jog_sim_job:
            self.root.after_cancel(self._jog_sim_job)
            self._jog_sim_job = None

    # ══════════════════════════════════════════════════════════════════
    # KEYBOARD
    # ══════════════════════════════════════════════════════════════════
    def _bind_keys(self):
        keymap = {
            "a": ("ROT_CCW", "ROT_STOP"),
            "d": ("ROT_CW", "ROT_STOP"),
            "w": ("ARM_FWD", "ARM_STOP"),
            "s": ("ARM_BACK", "ARM_STOP"),
            "i": ("Z_UP", "Z_STOP"),
            "k": ("Z_DOWN", "Z_STOP"),
            "Left": ("ROT_CCW", "ROT_STOP"),
            "Right": ("ROT_CW", "ROT_STOP"),
            "Up": ("ARM_FWD", "ARM_STOP"),
            "Down": ("ARM_BACK", "ARM_STOP"),
        }
        for key, (start_cmd, stop_cmd) in keymap.items():
            keys = [key, key.upper()] if len(key) == 1 else [key]
            for k in keys:
                self.root.bind_all(f"<KeyPress-{k}>", lambda e, s=start_cmd: self._key_press(s))
                self.root.bind_all(f"<KeyRelease-{k}>", lambda e, s=start_cmd, t=stop_cmd: self._key_release(s, t))

        self.root.bind_all("<KeyPress-space>", lambda e: self._space_pressed())
        self.root.bind_all("<KeyPress-h>", lambda e: self._h_pressed())
        self.root.bind_all("<KeyPress-H>", lambda e: self._h_pressed())

    def _jog_keys_enabled(self):
        if self.mode != "JOG":
            return False
        focused = self.root.focus_get()
        if isinstance(focused, (tk.Entry, ttk.Combobox, tk.Text)):
            return False
        return True

    def _key_press(self, start_cmd):
        if not self._jog_keys_enabled():
            return
        if start_cmd in self.jog_pads:
            self.jog_pads[start_cmd].key_activate()
        self.jog_start(start_cmd)

    def _key_release(self, start_cmd, stop_cmd):
        if start_cmd not in self.jog_active:
            return
        if start_cmd in self.jog_pads:
            self.jog_pads[start_cmd].key_deactivate()
        self.jog_stop(start_cmd, stop_cmd)

    def _space_pressed(self):
        # Universal panic key: always works, regardless of mode or focus.
        self.emergency_stop_all()

    def _h_pressed(self):
        if not self._jog_keys_enabled():
            return
        self._home_clicked()

    # ══════════════════════════════════════════════════════════════════
    # SERIAL PORT MANAGEMENT
    # ══════════════════════════════════════════════════════════════════
    def refresh_com_ports(self):
        if not HAS_SERIAL:
            self._log("CẢNH BÁO: Thư viện 'pyserial' chưa được cài đặt. Chạy: pip install pyserial", tag="error")
            return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if ports:
            self.com_combo['values'] = ports
            self.com_combo.set(DEFAULT_COM_PORT if DEFAULT_COM_PORT in ports else ports[0])
            self._log(f"Tìm thấy {len(ports)} cổng COM: {', '.join(ports)}")
        else:
            self.com_combo['values'] = ["Không có COM"]
            self.com_combo.set("Không có COM")
            self._log("Không tìm thấy cổng COM nào.")

    def toggle_connection(self):
        if self.is_connected:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        if not HAS_SERIAL:
            messagebox.showerror("Thiếu thư viện",
                                 "Thư viện 'pyserial' chưa được cài đặt.\nChạy: pip install pyserial")
            return
        port = self.com_var.get()
        baud = self.baud_var.get()
        if not port or port == "Không có COM":
            messagebox.showwarning("Cảnh báo", "Vui lòng chọn một cổng COM hợp lệ!")
            return
        try:
            self.ser = serial.Serial(port=port, baudrate=int(baud), timeout=0.05)
            self.is_connected = True
            self._missed_beats = 0

            self._set_led(self.com_led_card, "OPEN", ACCENT_GREEN)
            self._set_led(self.hw_led_card, "WAITING...", ACCENT_ORANGE)
            self._set_led(self.hb_led_card, "WAITING...", ACCENT_ORANGE)

            self.btn_connect.set_config("DISCONNECT", ACCENT_RED, icon="❌")
            self.status_var.set(f"COM OPEN — Waiting for ClearCore handshake on {port}...")
            self._log(f"Đã mở cổng {port} ({baud} bps). Đang gửi PING tới ClearCore...")

            self._listen_hardware_response()
            self._send_serial("PING")

            self._ping_timeout_job = self.root.after(PING_TIMEOUT_MS, self._on_ping_timeout)
            self._schedule_next_heartbeat()

        except serial.SerialException as e:
            self._log(f"Lỗi kết nối tới cổng {port}: {e}", tag="error")
            messagebox.showerror("Lỗi kết nối", f"Không thể mở cổng {port}.\n"
                                 "Có thể cổng đang bị phần mềm khác chiếm giữ!")

    def disconnect_serial(self):
        self._send_serial("BYE")
        self.root.after(100, self._do_disconnect)

    def _do_disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.is_connected = False
        self.hw_confirmed = False
        self._missed_beats = 0

        for job_attr in ('_ping_timeout_job', '_heartbeat_job'):
            job = getattr(self, job_attr)
            if job:
                self.root.after_cancel(job)
                setattr(self, job_attr, None)

        # Safety: treat any disconnect as an all-stop
        self.is_running = False
        self.is_homing = False
        if self.anim_job:
            self.root.after_cancel(self.anim_job)
            self.anim_job = None
        self.jog_active.clear()
        for pad in self.jog_pads.values():
            pad.key_deactivate()
        self._refresh_jog_status()
        if self._jog_sim_job:
            self.root.after_cancel(self._jog_sim_job)
            self._jog_sim_job = None
        self._set_motion_locked(False)

        self._set_led(self.com_led_card, "CLOSED", ACCENT_RED)
        self._set_led(self.hw_led_card, "NO SIGNAL", ACCENT_RED)
        self._set_led(self.hb_led_card, "IDLE", TEXT_MUTED)

        self.btn_connect.set_config("CONNECT", ACCENT_GREEN, icon="🔌")
        self.status_var.set("DISCONNECTED — Hardware Offline")
        self._log("Đã gửi BYE và ngắt kết nối. Tất cả chuyển động đã dừng.")

    def manual_ping(self):
        if not self.is_connected:
            messagebox.showwarning("Cảnh báo", "Chưa kết nối COM port!")
            return
        self._missed_beats = 0
        self._set_led(self.hb_led_card, "PINGING...", ACCENT_ORANGE)
        self._send_serial("PING")
        self._log("Manual PING sent → chờ PONG từ ClearCore...")
        if self._ping_timeout_job:
            self.root.after_cancel(self._ping_timeout_job)
        self._ping_timeout_job = self.root.after(PING_TIMEOUT_MS, self._on_ping_timeout)

    # ══════════════════════════════════════════════════════════════════
    # HEARTBEAT
    # ══════════════════════════════════════════════════════════════════
    def _schedule_next_heartbeat(self):
        if self.is_connected:
            self._heartbeat_job = self.root.after(HEARTBEAT_INTERVAL_MS, self._send_heartbeat_ping)

    def _send_heartbeat_ping(self):
        self._heartbeat_job = None
        if not self.is_connected:
            return
        self._send_serial("PING", log_tx=False)  # silent — the RX "<< PONG" is the visible confirmation
        if self._ping_timeout_job:
            self.root.after_cancel(self._ping_timeout_job)
        self._ping_timeout_job = self.root.after(PING_TIMEOUT_MS, self._on_ping_timeout)
        self._schedule_next_heartbeat()

    def _on_heartbeat_pong(self):
        if self._ping_timeout_job:
            self.root.after_cancel(self._ping_timeout_job)
            self._ping_timeout_job = None
        self._missed_beats = 0

        if not self.hw_confirmed:
            self.hw_confirmed = True
            self._set_led(self.hw_led_card, "IO0 ON ✓", ACCENT_GREEN)
            self.status_var.set("CONNECTED — ClearCore IO0 ACTIVE | Heartbeat OK")
            self._log("✓ PONG nhận được! ClearCore IO0 đang sáng. Handshake thành công.")

        self._blink_heartbeat_led()

    def _blink_heartbeat_led(self):
        self._set_led(self.hb_led_card, "♥ ALIVE", ACCENT_GREEN)
        self.root.after(400, lambda: self._set_led(self.hb_led_card, "OK", ACCENT_CYAN)
                        if self.is_connected else None)

    def _on_ping_timeout(self):
        self._ping_timeout_job = None
        self._missed_beats += 1
        remaining = MISSED_BEAT_LIMIT - self._missed_beats

        if self._missed_beats < MISSED_BEAT_LIMIT:
            self._set_led(self.hb_led_card, f"MISS {self._missed_beats}/{MISSED_BEAT_LIMIT}", ACCENT_ORANGE)
            self.status_var.set(f"WARNING — PONG miss #{self._missed_beats} ({remaining} more until LOST)")
            self._log(f"⚠ PONG timeout #{self._missed_beats}/{MISSED_BEAT_LIMIT}.", tag="warn")
        else:
            self._set_led(self.hw_led_card, "LOST", ACCENT_RED)
            self._set_led(self.hb_led_card, "LOST", ACCENT_RED)
            self.hw_confirmed = False
            self.status_var.set("⛔ CONNECTION LOST — ClearCore không phản hồi!")
            self._log(f"✗ CONNECTION LOST sau {MISSED_BEAT_LIMIT} PONG miss liên tiếp!", tag="error")
            self._log("→ Kiểm tra nguồn board, cáp USB, và firmware ClearCore.", tag="error")
            if self._heartbeat_job:
                self.root.after_cancel(self._heartbeat_job)
                self._heartbeat_job = None

    # ══════════════════════════════════════════════════════════════════
    # SERIAL SEND / RECEIVE
    # ══════════════════════════════════════════════════════════════════
    def _send_serial(self, msg: str, log_tx=True):
        line = msg.strip()
        if not line:
            return
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((line + "\n").encode('utf-8'))
                if log_tx:
                    self._log(f">> {line}", tag="tx")
            except Exception as e:
                self._log(f"Lỗi gửi Serial: {e}", tag="error")

    def _listen_hardware_response(self):
        if self.is_connected and self.ser and self.ser.is_open:
            try:
                while self.ser.in_waiting > 0:
                    response = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if response:
                        tag = "rx"
                        if response.startswith("[ERROR]"):
                            tag = "error"
                        elif response.startswith("[WARN]"):
                            tag = "warn"
                        self._log(f"<< {response}", tag=tag)
                        self._parse_hardware_response(response)
            except Exception:
                pass
            self.root.after(50, self._listen_hardware_response)

    def _parse_hardware_response(self, response: str):
        resp_upper = response.strip().upper()

        if resp_upper == "PONG":
            self._on_heartbeat_pong()
            return

        if resp_upper.startswith("[ALIVE]"):
            self._missed_beats = 0
            self._blink_heartbeat_led()
            return

        if "CLEARCORE POS" in resp_upper:
            # Joint-space telemetry now — the board no longer does IK/FK,
            # it just reports D1/ROT/A1M/A2M directly.
            pos_match = re.search(
                r"D1:\s*([\d\.-]+)\s*mm\s*\|\s*ROT:\s*([\d\.-]+)\s*deg\s*\|\s*"
                r"A1M:\s*([\d\.-]+)\s*deg\s*\|\s*A2M:\s*([\d\.-]+)\s*deg\s*\((\d+)%\)",
                response
            )
            if pos_match:
                d1, rot, a1, a2, pct = pos_match.groups()
                self._update_p2p_telemetry(float(d1), float(rot), float(a1), float(a2), int(pct))
            return

        if "JOG POS" in resp_upper:
            jog_match = re.search(
                r"ROT:\s*([\d\.-]+)\s*deg\s*\|\s*ARM:\s*([\d\.-]+)\s*deg\s*\|\s*Z:\s*([\d\.-]+)\s*mm",
                response
            )
            if jog_match:
                rot, arm, z = jog_match.groups()
                self.rot_pos_v.set(f"{float(rot):.2f} deg")
                self.arm_pos_v.set(f"{float(arm):.2f} deg")
                self.jz_pos_v.set(f"{float(z):.2f} mm")
            return

        if resp_upper.startswith("[LOADED]"):
            self.status_var.set("LOADED — ClearCore acknowledged Point A/B.")
            return

        if resp_upper.startswith("[RUN]"):
            self.status_var.set(f"RUNNING — {response.split(']', 1)[-1].strip()}")
            return

        if resp_upper.startswith("[PARAMS_OK]"):
            self._log("ClearCore đã xác nhận thông số PID/vận tốc/gia tốc mới.")
            return

        if resp_upper.startswith("[LIMIT]"):
            # e.g. "[LIMIT] ROT_CW" / "[LIMIT] ROT_CCW" / "[LIMIT] Z_UP" / "[LIMIT] Z_DOWN"
            axis_dir = response.split("]", 1)[-1].strip()
            self._on_limit_triggered(axis_dir)
            return

        if "DA DEN DIEM DICH THANH CONG" in response:
            self.is_running = False
            self._set_motion_locked(False)
            self.status_var.set("READY — Target point (B) reached.")
            return

        if "DUNG KHAN CAP" in response:
            self.is_running = False
            self.jog_active.clear()
            for pad in self.jog_pads.values():
                pad.key_deactivate()
            self._refresh_jog_status()
            self._set_motion_locked(False)
            self.status_var.set("STOPPED — Emergency Stop Triggered!")
            return

        if resp_upper.startswith("[HOME]") and "COMPLETE" in resp_upper:
            self.is_homing = False
            self._set_motion_locked(False)
            self.jog_dot.itemconfig(self._jog_dot_id, fill=TEXT_MUTED)
            self.jog_status_var.set("Idle — hold a key or button to jog")
            self.status_var.set("READY — Home position reached.")
            return

    # ══════════════════════════════════════════════════════════════════
    # LOG
    # ══════════════════════════════════════════════════════════════════
    def _log(self, message: str, tag: str = "default"):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{timestamp}] {message}\n", tag)
        self.log_text.see("end")
        if float(self.log_text.index("end-1c").split(".")[0]) > 800:
            self.log_text.delete("1.0", "2.0")
        self.log_text.configure(state="disabled")

    # ══════════════════════════════════════════════════════════════════
    # JOG MOTION
    # ══════════════════════════════════════════════════════════════════
    LIMIT_BLOCKED_DIRS = {"ROT_CW", "ROT_CCW", "Z_UP", "Z_DOWN"}
    _LIMIT_OPPOSITE = {"ROT_CW": "ROT_CCW", "ROT_CCW": "ROT_CW", "Z_UP": "Z_DOWN", "Z_DOWN": "Z_UP"}

    def jog_start(self, command):
        if command in self.jog_active:
            return
        if self._is_limited(command):
            self._log(f"{command} bị khóa — đã tới giới hạn cảm biến quang. "
                     f"Nhấn chiều ngược lại để rời khỏi giới hạn.", tag="warn")
            return
        self._clear_limit_if_opposite(command)
        self.jog_active.add(command)
        self._send_serial(command)
        self._refresh_jog_status()
        if not (self.is_connected and self.hw_confirmed) and self._jog_sim_job is None:
            self._jog_sim_job = self.root.after(JOG_SIM_TICK_MS, self._jog_sim_tick)

    def jog_stop(self, start_cmd, stop_cmd):
        if start_cmd not in self.jog_active:
            return
        self.jog_active.discard(start_cmd)
        self._send_serial(stop_cmd)
        self._refresh_jog_status()

    def _refresh_jog_status(self):
        if self.jog_active:
            self.jog_dot.itemconfig(self._jog_dot_id, fill=ACCENT_CYAN)
            self.jog_status_var.set("  ".join(sorted(self.jog_active)))
        else:
            self.jog_dot.itemconfig(self._jog_dot_id, fill=TEXT_MUTED)
            self.jog_status_var.set("Idle — hold a key or button to jog")

    # ── ROT/Z limit-sensor lock (real sensors on hardware; simulated bounds otherwise) ──
    def _is_limited(self, direction):
        if direction in self.rot_limit and self.rot_limit[direction]:
            return True
        if direction in self.z_limit and self.z_limit[direction]:
            return True
        return False

    def _on_limit_triggered(self, direction):
        """Called when the board reports [LIMIT] <direction>, or when the
        software simulation reaches its own simulated bound."""
        if direction in self.rot_limit:
            self.rot_limit[direction] = True
        elif direction in self.z_limit:
            self.z_limit[direction] = True
        stop_map = {"ROT_CW": "ROT_STOP", "ROT_CCW": "ROT_STOP", "Z_UP": "Z_STOP", "Z_DOWN": "Z_STOP"}
        self.jog_active.discard(direction)
        if direction in self.jog_pads:
            self.jog_pads[direction].key_deactivate()
        self._refresh_jog_status()
        self._log(f"[LIMIT] {direction} — cảm biến quang báo đã tới giới hạn, khóa chiều này.", tag="warn")

    def _clear_limit_if_opposite(self, direction):
        opp = self._LIMIT_OPPOSITE.get(direction)
        if opp is None:
            return
        if opp in self.rot_limit:
            self.rot_limit[opp] = False
        elif opp in self.z_limit:
            self.z_limit[opp] = False

    # ── Boost (x1 / x1.5 / x2, cycles on each click) ────────────────────
    def _cycle_boost(self):
        self.boost_index = (self.boost_index + 1) % len(BOOST_LEVELS)
        mult = BOOST_LEVELS[self.boost_index]
        self._send_serial(f"SET_BOOST:{mult}")
        label = "OFF" if mult == 1.0 else f"x{mult:g}"
        self.boost_btn.set_config(f"BOOST: {label}",
                                  ACCENT_ORANGE if mult != 1.0 else "#3a3a52",
                                  icon="⚡", fg_color=("#0a0a12" if mult != 1.0 else TEXT_LIGHT))
        self._log(f"Boost set to x{mult:g}.")

    def _speed_scale(self):
        """Combines the Settings velocity (relative to its default) with
        the active Boost multiplier — used only for the software-only jog
        simulation; real hardware speed comes from SET_PARAMS/SET_BOOST."""
        base = self.settings["vel_rpm"] / DEFAULT_VEL_RPM
        return base * BOOST_LEVELS[self.boost_index]

    def _jog_sim_tick(self):
        """Software-only jog animation, used only while no hardware is
        confirmed, so both modes stay demoable without anything plugged in."""
        self._jog_sim_job = None
        if not self.jog_active or (self.is_connected and self.hw_confirmed):
            return
        dt = JOG_SIM_TICK_MS / 1000.0
        scale = self._speed_scale()
        if "ROT_CW" in self.jog_active:
            self.sim_rot += ROT_SPEED_DEG_PER_SEC * dt * scale
        if "ROT_CCW" in self.jog_active:
            self.sim_rot -= ROT_SPEED_DEG_PER_SEC * dt * scale
        if "ARM_FWD" in self.jog_active:
            self.sim_arm += ARM_SPEED_DEG_PER_SEC * dt * scale
        if "ARM_BACK" in self.jog_active:
            self.sim_arm -= ARM_SPEED_DEG_PER_SEC * dt * scale
        if "Z_UP" in self.jog_active:
            self.sim_z += JZ_SPEED_MM_PER_SEC * dt * scale
        if "Z_DOWN" in self.jog_active:
            self.sim_z -= JZ_SPEED_MM_PER_SEC * dt * scale

        if self.sim_rot >= ROT_SIM_MAX_DEG:
            self.sim_rot = ROT_SIM_MAX_DEG
            self._on_limit_triggered("ROT_CW")
        elif self.sim_rot <= ROT_SIM_MIN_DEG:
            self.sim_rot = ROT_SIM_MIN_DEG
            self._on_limit_triggered("ROT_CCW")
        if self.sim_z >= Z_SIM_MAX_MM:
            self.sim_z = Z_SIM_MAX_MM
            self._on_limit_triggered("Z_UP")
        elif self.sim_z <= Z_SIM_MIN_MM:
            self.sim_z = Z_SIM_MIN_MM
            self._on_limit_triggered("Z_DOWN")

        self.rot_pos_v.set(f"{self.sim_rot:.2f} deg")
        self.arm_pos_v.set(f"{self.sim_arm:.2f} deg")
        self.jz_pos_v.set(f"{self.sim_z:.2f} mm")

        if self.jog_active:
            self._jog_sim_job = self.root.after(JOG_SIM_TICK_MS, self._jog_sim_tick)

    # ══════════════════════════════════════════════════════════════════
    # UNIVERSAL EMERGENCY STOP — the only estop code path in the app
    # ══════════════════════════════════════════════════════════════════
    def emergency_stop_all(self):
        for _ in range(3):
            self._send_serial("ESTOP")

        self.jog_active.clear()
        for pad in self.jog_pads.values():
            pad.key_deactivate()
        self._refresh_jog_status()
        if self._jog_sim_job:
            self.root.after_cancel(self._jog_sim_job)
            self._jog_sim_job = None

        self.is_running = False
        self.is_homing = False
        if self.anim_job:
            self.root.after_cancel(self.anim_job)
            self.anim_job = None
        self._set_motion_locked(False)

        self.jog_dot.itemconfig(self._jog_dot_id, fill=ACCENT_RED)
        self.jog_status_var.set("EMERGENCY STOP SENT")
        self.status_var.set("STOPPED — Emergency Stop Triggered!")
        self._log("EMERGENCY STOP (ESTOP x3) — all motion halted.", tag="warn")

    # ══════════════════════════════════════════════════════════════════
    # MOTION LOCK — disables everything except STOP/ESTOP during RUN/HOME
    # ══════════════════════════════════════════════════════════════════
    def _set_motion_locked(self, locked):
        for w in self.motion_lock_widgets:
            w.set_enabled(not locked)
        for pad in self.jog_pads.values():
            pad.set_enabled(not locked)

    # ══════════════════════════════════════════════════════════════════
    # P2P — ELBOW CONFIG SELECTOR
    # ══════════════════════════════════════════════════════════════════
    def _set_elbow_config(self, cfg):
        self.elbow_config = cfg
        self._sync_z_for_both_mode()
        for c, btn in self.elbow_buttons.items():
            if c == cfg:
                btn.set_config(c, ACCENT_CYAN, fg_color="#0a0a12")
            else:
                btn.set_config(c, "#3a3a52", fg_color=TEXT_LIGHT)
        self._log(f"Elbow/arm config solution set to {cfg}.")

    def _sync_z_for_both_mode(self, *args):
        """BOTH mode moves both arms on the SAME shared Z axis, so Z1
        always mirrors Z0 and is disabled (not independently editable)
        — there's only one real Z value to control in this mode, and a
        second editable-but-overwritten field was just confusing."""
        if self.elbow_config == "BOTH":
            self.z1_v.set(self.z0_v.get())
            self.z1_entry.config(state="disabled")
        else:
            self.z1_entry.config(state="normal")

    # ══════════════════════════════════════════════════════════════════
    # P2P MOTION — LOAD (compute IK) / RUN (A -> B) / STOP
    # ══════════════════════════════════════════════════════════════════
    def p2p_load_parameters(self):
        if self.elbow_config == "BOTH":
            self.z1_v.set(self.z0_v.get())
        try:
            x0 = float(self.x0_v.get()); y0 = float(self.y0_v.get()); z0 = float(self.z0_v.get())
            x1 = float(self.x1_v.get()); y1 = float(self.y1_v.get()); z1 = float(self.z1_v.get())
        except ValueError:
            messagebox.showerror("Lỗi", "Tọa độ nhập vào phải là số!")
            return

        if self.elbow_config == "BOTH":
            try:
                d1, rot, a1, a2 = solve_ik_both(x0, y0, z0, x1, y1, z1)
            except ValueError as e:
                messagebox.showerror("Không thể tính động học nghịch (IK)", str(e))
                self._log(f"IK error: {e}", tag="error")
                return

            self.loaded_program = {"mode": "both", "target": (d1, rot, a1, a2)}
            self.calc_a_d1_v.set(f"{d1:.2f} mm"); self.calc_a_rot_v.set(f"{rot:.2f} deg")
            self.calc_a_a1_v.set(f"{a1:.2f} deg"); self.calc_a_a2_v.set("—")
            self.calc_b_d1_v.set(f"{d1:.2f} mm"); self.calc_b_rot_v.set(f"{rot:.2f} deg")
            self.calc_b_a1_v.set("—"); self.calc_b_a2_v.set(f"{a2:.2f} deg")

            self._send_serial(f"LOAD_BOTH:{d1:.3f},{rot:.3f},{a1:.3f},{a2:.3f}")
            self._log(f"Đã tính IK (2 tay đồng thời) & nạp thông số: "
                     f"d1={d1:.1f} rot={rot:.1f} a1m={a1:.1f} a2m={a2:.1f}")
            self.status_var.set("LOADED — Simultaneous dual-arm move ready. Press RUN PROGRAM.")
            return

        try:
            d1a, rota, a1a, a2a = solve_ik(x0, y0, z0, self.elbow_config, reference_deg=0.0)
            d1b, rotb, a1b, a2b = solve_ik(x1, y1, z1, self.elbow_config, reference_deg=0.0)
        except ValueError as e:
            messagebox.showerror("Không thể tính động học nghịch (IK)", str(e))
            self._log(f"IK error: {e}", tag="error")
            return

        self.loaded_program = {"mode": "single", "waypoints": (d1a, rota, a1a, a2a, d1b, rotb, a1b, a2b)}
        self.calc_a_d1_v.set(f"{d1a:.2f} mm"); self.calc_a_rot_v.set(f"{rota:.2f} deg")
        self.calc_a_a1_v.set(f"{a1a:.2f} deg"); self.calc_a_a2_v.set(f"{a2a:.2f} deg")
        self.calc_b_d1_v.set(f"{d1b:.2f} mm"); self.calc_b_rot_v.set(f"{rotb:.2f} deg")
        self.calc_b_a1_v.set(f"{a1b:.2f} deg"); self.calc_b_a2_v.set(f"{a2b:.2f} deg")

        self._send_serial(f"LOAD:{d1a:.3f},{rota:.3f},{a1a:.3f},{a2a:.3f},"
                          f"{d1b:.3f},{rotb:.3f},{a1b:.3f},{a2b:.3f}")
        self._log(f"Đã tính IK & nạp thông số. A=(d1={d1a:.1f},rot={rota:.1f},a1m={a1a:.1f},a2m={a2a:.1f}) "
                 f"B=(d1={d1b:.1f},rot={rotb:.1f},a1m={a1b:.1f},a2m={a2b:.1f})")
        self.status_var.set("LOADED — Program A → B ready. Press RUN PROGRAM.")

    def p2p_run_program(self):
        if self.loaded_program is None:
            messagebox.showwarning("Cảnh báo", "Hãy nạp thông số (LOAD PARAMETERS) trước khi chạy!")
            return

        if self.anim_job:
            self.root.after_cancel(self.anim_job)

        self.is_running = True
        self._set_motion_locked(True)
        if self.loaded_program["mode"] == "both":
            self.status_var.set("RUNNING — Simultaneous dual-arm move executing...")
            self._log("RUN PROGRAM — di chuyển đồng thời cả 2 tay (A1M + A2M).")
        else:
            self.status_var.set("RUNNING — Program executing A → B...")
            self._log("RUN PROGRAM — di chuyển A → B.")

        if self.is_connected and self.hw_confirmed and self.ser and self.ser.is_open:
            self._send_serial("RUN")
        else:
            if self.is_connected and not self.hw_confirmed:
                self._log("CẢNH BÁO: Gửi lệnh nhưng ClearCore chưa xác nhận handshake!", tag="warn")
            self._simulate_p2p_run()

    def p2p_stop(self):
        self._send_serial("STOP")
        self.is_running = False
        if self.anim_job:
            self.root.after_cancel(self.anim_job)
            self.anim_job = None
        self._set_motion_locked(False)
        self.status_var.set("STOPPED — Program halted.")
        self._log("STOP — chương trình P2P đã dừng.", tag="warn")

    def _read_current_joint_telemetry(self):
        def _num(var):
            try:
                return float(var.get().split()[0])
            except (ValueError, IndexError):
                return 0.0
        return (_num(self.curr_d1_v), _num(self.curr_rot_v), _num(self.curr_a1_v), _num(self.curr_a2_v))

    def _simulate_p2p_run(self):
        """Software-only fallback: animates the loaded program, since no
        hardware is confirmed."""
        prog = self.loaded_program
        if prog["mode"] == "both":
            target_j = prog["target"]
            start_j = self._read_current_joint_telemetry()
            self._animate_joint_motion(step=0, total_steps=25, start_j=start_j, target_j=target_j,
                                       done_message="Simultaneous dual-arm move complete.")
        else:
            d1a, rota, a1a, a2a, d1b, rotb, a1b, a2b = prog["waypoints"]
            self._animate_joint_motion(step=0, total_steps=25,
                                       start_j=(d1a, rota, a1a, a2a),
                                       target_j=(d1b, rotb, a1b, a2b),
                                       done_message="Target point (B) reached.")

    def _animate_joint_motion(self, step, total_steps, start_j, target_j,
                              done_message="Target point reached."):
        if not self.is_running:
            return
        progress = step / total_steps
        pct = int(progress * 100)
        d1 = start_j[0] + (target_j[0] - start_j[0]) * progress
        rot = start_j[1] + (target_j[1] - start_j[1]) * progress
        a1 = start_j[2] + (target_j[2] - start_j[2]) * progress
        a2 = start_j[3] + (target_j[3] - start_j[3]) * progress
        self._update_p2p_telemetry(d1, rot, a1, a2, pct)
        if step < total_steps:
            self.anim_job = self.root.after(150, self._animate_joint_motion,
                                            step + 1, total_steps, start_j, target_j, done_message)
        else:
            self.is_running = False
            self._set_motion_locked(False)
            self.status_var.set(f"READY — {done_message}")
            self._log(f"Simulation complete. {done_message}")

    def _update_p2p_telemetry(self, d1, rot, a1, a2, pct):
        self.curr_d1_v.set(f"{d1:.2f} mm")
        self.curr_rot_v.set(f"{rot:.2f} deg")
        self.curr_a1_v.set(f"{a1:.2f} deg")
        self.curr_a2_v.set(f"{a2:.2f} deg")
        self.progress_var.set(pct)
        self.progress_pct_var.set(f"{pct}%")
        x, y, z = forward_kinematics(d1, rot, a1, a2)
        self.calc_xyz_v.set(f"Cartesian (tính từ động học thuận): X={x:.2f}  Y={y:.2f}  Z={z:.2f} mm")

    # ══════════════════════════════════════════════════════════════════
    # SHUTDOWN
    # ══════════════════════════════════════════════════════════════════
    def _on_close(self):
        self._send_serial("BYE", log_tx=False)
        for job_attr in ('_ping_timeout_job', '_heartbeat_job', 'anim_job', '_jog_sim_job'):
            job = getattr(self, job_attr, None)
            if job:
                self.root.after_cancel(job)
        self.root.after(120, lambda: (
            self.ser.close() if self.ser and self.ser.is_open else None,
            self.root.destroy()
        ))


if __name__ == "__main__":
    root = tk.Tk()
    RobotControlApp(root)
    root.mainloop()