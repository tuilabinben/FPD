"""
Robot Motion Controller — P2P + Joystick (v4)
=============================================================================
Merges two control modes on one shared connection layer:
  - POINT TO POINT : absolute X/Y/Z target motion (from v3)
  - JOYSTICK        : held-key jog on ROT/ARM/Z axes (from
                       stcr4000s_joystick_control.py), dead-man-switch,
                       HOME, and a universal ESTOP

SHARED ACROSS BOTH MODES:
  - One connection panel + 3-LED status (COM PORT / CLEARCORE IO0 / HEARTBEAT)
  - PING/PONG heartbeat every HEARTBEAT_INTERVAL_MS with missed-beat tracking
  - One unified Event Log (>> sent / << received, color-tagged)
  - A software-only simulation fallback when no hardware is confirmed, so
    both modes remain fully testable/demoable with nothing plugged in

SAFETY DESIGN (defense in depth — enforced in BOTH layers independently):
  - Switching mode in the GUI auto-stops any active jog axis + any P2P move
  - Firmware also refuses to run a P2P move while a jog axis is held (and
    vice versa) — a backstop in case the GUI and firmware ever desync
  - There is exactly ONE emergency-stop code path (emergency_stop_all):
    the P2P panel's button, the Joystick panel's button, and the Space key
    all call it. One audited function beats three similar ones.
  - PING always gets a PONG reply immediately, even mid-motion, so the
    heartbeat never times out just because the board is busy

ASSUMPTION TO VERIFY BEFORE REAL HARDWARE USE:
  ROT/ARM/Z (jog, cylindrical-style) and X/Y/Z (P2P, Cartesian) are kept as
  TWO SEPARATE simulated axis sets, both here and in the firmware. If your
  real STCR4000S kinematics mean these are the same physical joints, you
  need to add the forward/inverse kinematics transform yourself — seeding
  that transform without knowing the real chain would silently misreport
  position. See RobotMotionController_v4.ino header for the matching note.

PROTOCOL (Python -> ClearCore):
  PING / BYE / START:X0,Y0,Z0,X1,Y1,Z1 / STOP
  ROT_CW / ROT_CCW / ROT_STOP
  ARM_FWD / ARM_BACK / ARM_STOP
  Z_UP / Z_DOWN / Z_STOP
  HOME / ESTOP

PROTOCOL (ClearCore -> Python):
  PONG | [ALIVE] uptime: Xs
  [CLEARCORE POS] Vi tri hien tai -> X: F mm | Y: F mm | Z: F mm (P%)
  [JOG POS] ROT: F deg | ARM: F mm | Z: F mm
  DA DEN DIEM DICH THANH CONG | DUNG KHAN CAP
  [HOME] Homing started. | [HOME] Homing complete. ROT=0 ARM=0 Z=0
"""

import datetime
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
ARM_SPEED_MM_PER_SEC = 20.0
JZ_SPEED_MM_PER_SEC = 15.0

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


class RoundedButton(tk.Canvas):
    def __init__(self, parent, text, command=None, icon="", radius=18,
                 bg_color=ACCENT_GREEN, fg_color="#0a0a12", hover_color=None,
                 width=160, height=38, font=("Segoe UI", 10, "bold")):
        super().__init__(parent, width=width, height=height, bg=parent["bg"],
                         highlightthickness=0, bd=0, cursor="hand2")
        self.command = command
        self.base_color = bg_color
        self.hover_color = hover_color or self._lighten(bg_color)
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

    @staticmethod
    def _lighten(hex_color, factor=1.15):
        hex_color = hex_color.lstrip("#")
        r, g, b = (int(hex_color[i:i + 2], 16) for i in (0, 2, 4))
        r, g, b = (min(255, int(c * factor)) for c in (r, g, b))
        return f"#{r:02x}{g:02x}{b:02x}"

    def _round_rect(self, x1, y1, x2, y2, r, **kwargs):
        pts = [x1 + r, y1, x2 - r, y1, x2, y1, x2, y1 + r,
               x2, y2 - r, x2, y2, x2 - r, y2, x1 + r, y2,
               x1, y2, x1, y2 - r, x1, y1 + r, x1, y1]
        return self.create_polygon(pts, smooth=True, **kwargs)

    def _draw(self, color):
        self.delete("all")
        self._round_rect(2, 2, self.w - 2, self.h - 2, self.radius, fill=color, outline="")
        text_color = self.fg_color if self.enabled else TEXT_MUTED
        display_text = f"{self.icon_str}  {self.text_str}" if self.icon_str else self.text_str
        self.create_text(self.w // 2, self.h // 2, text=display_text,
                         fill=text_color, font=self.font)

    def set_config(self, text, bg_color, icon="", fg_color=None):
        self.text_str = text
        self.icon_str = icon
        self.base_color = bg_color
        self.hover_color = self._lighten(bg_color)
        if fg_color is not None:
            self.fg_color = fg_color
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
        if self.state == "active":
            fill, outline = self.hi_color, self.hi_color
            glyph_fill, label_fill = "#0a0a12", "#0a0a12"
        elif self.state == "hover":
            fill, outline = ENTRY_BG, self.base_color
            glyph_fill, label_fill = self.hi_color, TEXT_LIGHT
        else:
            fill, outline = ENTRY_BG, BORDER
            glyph_fill, label_fill = self.base_color, TEXT_MUTED

        self._rounded_rect(2, 2, self.w - 2, self.h - 2, 12, fill=fill, outline=outline, width=1.5)

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

    def _press(self, _=None):
        self.set_state("active")
        self.on_press()

    def _release(self, _=None):
        self.set_state("hover")
        self.on_release()

    def _enter(self, _=None):
        if self.state != "active":
            self.set_state("hover")

    def _leave(self, _=None):
        if self.state == "active":
            self.on_release()
        self.set_state("idle")

    def key_activate(self):
        self.set_state("active")

    def key_deactivate(self):
        self.set_state("idle")


class RobotControlApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Robot Motion Controller — P2P + Joystick (v4)")
        self.root.configure(bg=BG)
        self.root.geometry("1220x1060")

        # Serial / connection
        self.ser = None
        self.is_connected = False
        self.hw_confirmed = False
        self._ping_timeout_job = None
        self._heartbeat_job = None
        self._missed_beats = 0

        # P2P motion
        self.is_running = False
        self.anim_job = None

        # Jog motion
        self.jog_active = set()
        self.jog_pads = {}
        self._jog_sim_job = None
        self.sim_rot = 0.0
        self.sim_arm = 0.0
        self.sim_z = 0.0

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

        main = tk.Frame(self.root, bg=BG)
        main.pack(side="top", fill="both", expand=True)

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
                      width=90, height=34, command=self.manual_ping).pack(side="left", padx=(0, 16))

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
        tk.Entry(inner, textvariable=var, bg=ENTRY_BG, fg=TEXT_LIGHT, relief="flat",
                 font=("Segoe UI", 11), justify="center", width=7).pack(side="left", fill="x", expand=True)
        tk.Label(inner, text="mm", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=("Segoe UI", 9)).pack(side="left", padx=(4, 0))

    def _make_led_card(self, parent, col, title, var, color):
        parent.grid_columnconfigure(col, weight=1)
        card = tk.Frame(parent, bg=LED_BG, highlightbackground=color, highlightthickness=1)
        card.grid(row=0, column=col, padx=4, sticky="ew")
        tk.Label(card, text=title, bg=LED_BG, fg=TEXT_MUTED,
                 font=("Segoe UI", 8, "bold")).pack(pady=(4, 0))
        tk.Label(card, textvariable=var, bg=LED_BG, fg=color,
                 font=("Consolas", 14, "bold")).pack(pady=(0, 4))

    # ══════════════════════════════════════════════════════════════════
    # P2P PANEL
    # ══════════════════════════════════════════════════════════════════
    def _build_p2p_panel(self, parent):
        coord_row = tk.Frame(parent, bg=PANEL_BG)
        coord_row.pack(fill="x")

        start_col = tk.Frame(coord_row, bg=PANEL_BG)
        start_col.pack(side="left", fill="both", expand=True, padx=(0, 8))
        tk.Label(start_col, text="START POINT (mm)", bg=PANEL_BG, fg=TEXT_MUTED,
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
        tk.Label(target_col, text="TARGET POINT (mm)", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=("Segoe UI", 8, "bold")).pack(anchor="w")
        target_grid = tk.Frame(target_col, bg=PANEL_BG)
        target_grid.pack(fill="x", pady=(4, 0))
        self.x1_v = tk.StringVar(value="100")
        self.y1_v = tk.StringVar(value="100")
        self.z1_v = tk.StringVar(value="100")
        self._make_coord_card(target_grid, 0, "X1", AXIS_X_COLOR, self.x1_v)
        self._make_coord_card(target_grid, 1, "Y1", AXIS_Y_COLOR, self.y1_v)
        self._make_coord_card(target_grid, 2, "Z1", AXIS_Z_COLOR, self.z1_v)

        tk.Label(parent, text="LIVE TELEMETRY", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=("Segoe UI", 8, "bold")).pack(anchor="w", pady=(16, 0))
        self.curr_x_v = tk.StringVar(value="0.00 mm")
        self.curr_y_v = tk.StringVar(value="0.00 mm")
        self.curr_z_v = tk.StringVar(value="0.00 mm")
        led_row = tk.Frame(parent, bg=PANEL_BG)
        led_row.pack(fill="x", pady=(4, 0))
        self._make_led_card(led_row, 0, "X_CURR", self.curr_x_v, AXIS_X_COLOR)
        self._make_led_card(led_row, 1, "Y_CURR", self.curr_y_v, AXIS_Y_COLOR)
        self._make_led_card(led_row, 2, "Z_CURR", self.curr_z_v, AXIS_Z_COLOR)

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

        btn_row = tk.Frame(parent, bg=PANEL_BG)
        btn_row.pack(pady=(18, 4))
        RoundedButton(btn_row, text="START SIMULATION", icon="▶", bg_color=ACCENT_GREEN,
                      width=220, height=46, command=self.on_start).pack(side="left", padx=8)
        RoundedButton(btn_row, text="EMERGENCY STOP", icon="⬛", bg_color=ACCENT_RED,
                      width=220, height=46, command=self.emergency_stop_all).pack(side="left", padx=8)

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

        self.home_btn = tk.Canvas(grid, width=78, height=78, bg=PANEL_BG, highlightthickness=0, cursor="hand2")
        self.home_btn.grid(row=1, column=1, padx=4, pady=4)
        self._draw_home_button("idle")
        self.home_btn.bind("<ButtonPress-1>", lambda e: self._draw_home_button("active"))
        self.home_btn.bind("<ButtonRelease-1>", lambda e: self._home_clicked())
        self.home_btn.bind("<Enter>", lambda e: self._draw_home_button("hover"))
        self.home_btn.bind("<Leave>", lambda e: self._draw_home_button("idle"))

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
        self.arm_pos_v = tk.StringVar(value="0.00 mm")
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

        RoundedButton(parent, text="EMERGENCY STOP (SPACE)", icon="⬛", bg_color=ACCENT_RED,
                      width=440, height=46, command=self.emergency_stop_all).pack(pady=(18, 4))
        tk.Label(parent, text="Hold a direction to move · release to stop · dead-man-switch (no latching)",
                 bg=PANEL_BG, fg=TEXT_MUTED, font=("Segoe UI", 8, "italic")).pack(pady=(2, 0))

    def _jog_pad(self, parent, row, col, glyph, label, keycap, start_cmd, stop_cmd, base, hi):
        pad = JogPad(parent, 78, 78, glyph, label, keycap, base, hi,
                     on_press=lambda: self.jog_start(start_cmd),
                     on_release=lambda: self.jog_stop(start_cmd, stop_cmd))
        pad.grid(row=row, column=col, padx=4, pady=4)
        self.jog_pads[start_cmd] = pad
        return pad

    def _draw_home_button(self, state):
        c = self.home_btn
        c.delete("all")
        if state == "active":
            fill, outline, glyph_fill, text_fill = "#58c9ff", "#58c9ff", "#0a0a12", "#0a0a12"
        elif state == "hover":
            fill, outline, glyph_fill, text_fill = PANEL_BG, ACCENT_CYAN, ACCENT_CYAN, TEXT_MUTED
        else:
            fill, outline, glyph_fill, text_fill = PANEL_BG, BORDER, TEXT_MUTED, TEXT_MUTED
        c.create_oval(6, 6, 72, 72, fill=fill, outline=outline, width=1.5)
        c.create_text(39, 33, text="\u2302", font=("Segoe UI", 20), fill=glyph_fill)
        c.create_text(39, 55, text="HOME", font=("Segoe UI", 8, "bold"), fill=text_fill)

    def _home_clicked(self):
        self._draw_home_button("hover")
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

        self._send_serial("HOME")

        if not (self.is_connected and self.hw_confirmed):
            self.sim_rot = self.sim_arm = self.sim_z = 0.0
            self.rot_pos_v.set("0.00 deg")
            self.arm_pos_v.set("0.00 mm")
            self.jz_pos_v.set("0.00 mm")
            self.status_var.set("READY — Home position reached (simulated).")

        self.jog_dot.itemconfig(self._jog_dot_id, fill=ACCENT_GREEN)
        self.jog_status_var.set("Homing...")
        self._log("HOME command sent.")

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
                self.root.bind(f"<KeyPress-{k}>", lambda e, s=start_cmd: self._key_press(s))
                self.root.bind(f"<KeyRelease-{k}>", lambda e, s=start_cmd, t=stop_cmd: self._key_release(s, t))

        self.root.bind("<KeyPress-space>", lambda e: self._space_pressed())
        self.root.bind("<KeyPress-h>", lambda e: self._h_pressed())
        self.root.bind("<KeyPress-H>", lambda e: self._h_pressed())

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
            pos_match = re.search(
                r"X:\s*([\d\.-]+)\s*mm\s*\|\s*Y:\s*([\d\.-]+)\s*mm\s*\|\s*Z:\s*([\d\.-]+)\s*mm\s*\((\d+)%\)",
                response
            )
            if pos_match:
                x, y, z, pct = pos_match.groups()
                self.curr_x_v.set(f"{float(x):.2f} mm")
                self.curr_y_v.set(f"{float(y):.2f} mm")
                self.curr_z_v.set(f"{float(z):.2f} mm")
                self.progress_var.set(float(pct))
                self.progress_pct_var.set(f"{pct}%")
            return

        if "JOG POS" in resp_upper:
            jog_match = re.search(
                r"ROT:\s*([\d\.-]+)\s*deg\s*\|\s*ARM:\s*([\d\.-]+)\s*mm\s*\|\s*Z:\s*([\d\.-]+)\s*mm",
                response
            )
            if jog_match:
                rot, arm, z = jog_match.groups()
                self.rot_pos_v.set(f"{float(rot):.2f} deg")
                self.arm_pos_v.set(f"{float(arm):.2f} mm")
                self.jz_pos_v.set(f"{float(z):.2f} mm")
            return

        if "DA DEN DIEM DICH THANH CONG" in response:
            self.is_running = False
            self.status_var.set("READY — Target point reached.")
            return

        if "DUNG KHAN CAP" in response:
            self.is_running = False
            self.jog_active.clear()
            for pad in self.jog_pads.values():
                pad.key_deactivate()
            self._refresh_jog_status()
            self.status_var.set("STOPPED — Emergency Stop Triggered!")
            return

        if resp_upper.startswith("[HOME]") and "COMPLETE" in resp_upper:
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
    def jog_start(self, command):
        if command in self.jog_active:
            return
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

    def _jog_sim_tick(self):
        """Software-only jog animation, used only while no hardware is
        confirmed, so both modes stay demoable without anything plugged in."""
        self._jog_sim_job = None
        if not self.jog_active or (self.is_connected and self.hw_confirmed):
            return
        dt = JOG_SIM_TICK_MS / 1000.0
        if "ROT_CW" in self.jog_active:
            self.sim_rot += ROT_SPEED_DEG_PER_SEC * dt
        if "ROT_CCW" in self.jog_active:
            self.sim_rot -= ROT_SPEED_DEG_PER_SEC * dt
        if "ARM_FWD" in self.jog_active:
            self.sim_arm += ARM_SPEED_MM_PER_SEC * dt
        if "ARM_BACK" in self.jog_active:
            self.sim_arm -= ARM_SPEED_MM_PER_SEC * dt
        if "Z_UP" in self.jog_active:
            self.sim_z += JZ_SPEED_MM_PER_SEC * dt
        if "Z_DOWN" in self.jog_active:
            self.sim_z -= JZ_SPEED_MM_PER_SEC * dt

        self.rot_pos_v.set(f"{self.sim_rot:.2f} deg")
        self.arm_pos_v.set(f"{self.sim_arm:.2f} mm")
        self.jz_pos_v.set(f"{self.sim_z:.2f} mm")

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
        if self.anim_job:
            self.root.after_cancel(self.anim_job)
            self.anim_job = None

        self.jog_dot.itemconfig(self._jog_dot_id, fill=ACCENT_RED)
        self.jog_status_var.set("EMERGENCY STOP SENT")
        self.status_var.set("STOPPED — Emergency Stop Triggered!")
        self._log("EMERGENCY STOP (ESTOP x3) — all motion halted.", tag="warn")

    # ══════════════════════════════════════════════════════════════════
    # P2P MOTION
    # ══════════════════════════════════════════════════════════════════
    def on_start(self):
        try:
            x0 = float(self.x0_v.get()); y0 = float(self.y0_v.get()); z0 = float(self.z0_v.get())
            x1 = float(self.x1_v.get()); y1 = float(self.y1_v.get()); z1 = float(self.z1_v.get())
        except ValueError:
            messagebox.showerror("Lỗi", "Tọa độ nhập vào phải là số!")
            return

        if self.anim_job:
            self.root.after_cancel(self.anim_job)

        self.is_running = True
        self.status_var.set("RUNNING — Robot moving from Start to Target...")
        self._log(f"Simulation started: ({x0},{y0},{z0}) → ({x1},{y1},{z1})")

        if self.is_connected and self.hw_confirmed and self.ser and self.ser.is_open:
            self._send_serial(f"START:{x0},{y0},{z0},{x1},{y1},{z1}")
        else:
            if self.is_connected and not self.hw_confirmed:
                self._log("CẢNH BÁO: Gửi lệnh nhưng ClearCore chưa xác nhận handshake!", tag="warn")
            self._animate_motion(step=0, total_steps=15, start_p=(x0, y0, z0), target_p=(x1, y1, z1))

    def _animate_motion(self, step, total_steps, start_p, target_p):
        if not self.is_running:
            return
        progress = step / total_steps
        pct = int(progress * 100)
        curr_x = start_p[0] + (target_p[0] - start_p[0]) * progress
        curr_y = start_p[1] + (target_p[1] - start_p[1]) * progress
        curr_z = start_p[2] + (target_p[2] - start_p[2]) * progress
        self.curr_x_v.set(f"{curr_x:.2f} mm")
        self.curr_y_v.set(f"{curr_y:.2f} mm")
        self.curr_z_v.set(f"{curr_z:.2f} mm")
        self.progress_var.set(pct)
        self.progress_pct_var.set(f"{pct}%")
        if 0 < step < total_steps:
            self._log(f"Moving... X={curr_x:.2f} Y={curr_y:.2f} Z={curr_z:.2f} ({pct}%)")
        if step < total_steps:
            self.anim_job = self.root.after(200, self._animate_motion, step + 1, total_steps, start_p, target_p)
        else:
            self.is_running = False
            self.status_var.set("READY — Target point reached.")
            self._log("Simulation complete. Target point reached.")

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