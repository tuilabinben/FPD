"""Window shell: ttk styles, the scrollable body, and sections 1, 2 and 4."""

import tkinter as tk
from tkinter import ttk

from ..config import (
    BAUD_CHOICES,
    DEFAULT_BAUD_RATE,
    HEARTBEAT_INTERVAL_MS,
    PLC_IP,
    PLC_LED_STATES,
    MISSED_BEAT_LIMIT,
    PING_TIMEOUT_MS,
)
from ..theme import (
    FONT_BUTTON,
    FONT_HINT,
    FONT_MONO,
    ACCENT_GREEN,
    ACCENT_MINT,
    ACCENT_ORANGE,
    ACCENT_PURPLE,
    ACCENT_RED,
    BG,
    BORDER,
    BORDER_SOFT,
    ENTRY_BG,
    INK_DARK,
    LED_BG,
    PANEL_BG,
    RADIUS_MD,
    SURFACE,
    TEXT_LIGHT,
    TEXT_MUTED,
)
from ..widgets import (RoundedButton, RoundedFrame, make_section,
                       make_status_led)


class LayoutMixin:
    # ── ttk styles ───────────────────────────────────────────────────
    def _configure_styles(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(
            "Neu.Horizontal.TProgressbar",
            troughcolor=LED_BG, background=ACCENT_MINT,
            bordercolor=PANEL_BG, lightcolor=ACCENT_MINT,
            darkcolor=ACCENT_MINT, thickness=10,
        )
        style.configure(
            "TCombobox",
            fieldbackground=ENTRY_BG, background=SURFACE,
            foreground=TEXT_LIGHT, bordercolor=BORDER,
            arrowcolor=ACCENT_MINT, padding=5, relief="flat",
        )
        style.map("TCombobox",
                  fieldbackground=[("readonly", ENTRY_BG)],
                  foreground=[("readonly", TEXT_LIGHT)],
                  bordercolor=[("focus", ACCENT_MINT)])
        style.configure("Vertical.TScrollbar", background=SURFACE,
                        troughcolor=BG, bordercolor=BG, arrowcolor=TEXT_MUTED,
                        relief="flat")

    # ── main build ───────────────────────────────────────────────────
    def _build_ui(self):
        self.status_var = tk.StringVar(value="READY — Hardware Offline")
        tk.Label(self.root, textvariable=self.status_var, anchor="w",
                 bg=LED_BG, fg=ACCENT_MINT, font=FONT_MONO,
                 padx=14, pady=7).pack(side="bottom", fill="x")

        main = self._build_scroll_body()

        self._build_connection_section(main)
        self._build_mode_section(main)
        self._build_motion_section(main)
        self._build_log_section(main)

        self.log("System initialized. Mode: POINT TO POINT.")
        self.log(f"Heartbeat: PING every {HEARTBEAT_INTERVAL_MS // 1000}s | "
                 f"Timeout {PING_TIMEOUT_MS // 1000}s | Lost after {MISSED_BEAT_LIMIT} misses")

    def _build_scroll_body(self):
        """The window content is taller than some screens, so everything
        below the status bar lives in a scrollable canvas — otherwise
        section 4 silently overflows off the bottom."""
        container = tk.Frame(self.root, bg=BG)
        container.pack(side="top", fill="both", expand=True)

        canvas = tk.Canvas(container, bg=BG, highlightthickness=0)
        vscroll = ttk.Scrollbar(container, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vscroll.set)
        canvas.pack(side="left", fill="both", expand=True)
        vscroll.pack(side="right", fill="y")
        self._main_canvas = canvas

        main = tk.Frame(canvas, bg=BG)
        window_id = canvas.create_window((0, 0), window=main, anchor="nw")

        main.bind("<Configure>",
                  lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>",
                    lambda e: canvas.itemconfig(window_id, width=e.width))

        # Scroll only while the pointer is over the body. The old version used
        # bind_all, which hijacked the wheel everywhere — including over the
        # combo boxes and the log — for the whole application.
        canvas.bind("<Enter>", lambda e: self._bind_mousewheel(True))
        canvas.bind("<Leave>", lambda e: self._bind_mousewheel(False))
        return main

    def _bind_mousewheel(self, active):
        canvas = self._main_canvas
        if active:
            canvas.bind_all("<MouseWheel>",
                            lambda e: canvas.yview_scroll(int(-e.delta / 120), "units"))
            canvas.bind_all("<Button-4>", lambda e: canvas.yview_scroll(-3, "units"))
            canvas.bind_all("<Button-5>", lambda e: canvas.yview_scroll(3, "units"))
        else:
            canvas.unbind_all("<MouseWheel>")
            canvas.unbind_all("<Button-4>")
            canvas.unbind_all("<Button-5>")

    # ── section 1 ────────────────────────────────────────────────────
    def _build_connection_section(self, main):
        s0, _ = make_section(main, "1. HARDWARE CONNECTION (SERIAL PORT)", 0)
        row = tk.Frame(s0, bg=PANEL_BG)
        row.pack(fill="x", pady=2)

        tk.Label(row, text="PORT:", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_BUTTON).pack(side="left", padx=(0, 5))
        self.com_var = tk.StringVar()
        self.com_combo = ttk.Combobox(row, textvariable=self.com_var, width=12, state="readonly")
        self.com_combo.pack(side="left", padx=(0, 10))

        RoundedButton(row, text="REFRESH", icon="🔄", bg_color=SURFACE, fg_color=TEXT_LIGHT,
                      width=100, height=34,
                      command=self.refresh_com_ports).pack(side="left", padx=(0, 16))

        tk.Label(row, text="BAUD:", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_BUTTON).pack(side="left", padx=(0, 5))
        self.baud_var = tk.StringVar(value=DEFAULT_BAUD_RATE)
        self.baud_combo = ttk.Combobox(row, textvariable=self.baud_var, width=9,
                                       state="readonly", values=BAUD_CHOICES)
        self.baud_combo.pack(side="left", padx=(0, 16))

        self.btn_connect = RoundedButton(row, text="CONNECT", icon="🔌", bg_color=ACCENT_GREEN, fg_color=INK_DARK,
                                         width=130, height=34, command=self.toggle_connection)
        self.btn_connect.pack(side="left", padx=(0, 10))

        RoundedButton(row, text="PING", icon="📡", bg_color=ACCENT_PURPLE, fg_color=INK_DARK,
                      width=90, height=34, command=self.manual_ping).pack(side="left", padx=(0, 10))

        # Same neutral surface colour as SETTINGS: neither is a motion
        # command, and colouring RESTART like one would put it in the same
        # visual class as CONNECT and PING, which do talk to the machine.
        RoundedButton(row, text="RESTART", icon="⟳", bg_color=SURFACE, fg_color=TEXT_LIGHT,
                      width=120, height=34,
                      command=self.restart_app).pack(side="left", padx=(0, 10))

        RoundedButton(row, text="SETTINGS  ESC", icon="⚙", bg_color=SURFACE, fg_color=TEXT_LIGHT,
                      width=150, height=34,
                      command=self.open_settings_dialog).pack(side="left", padx=(0, 16))

        leds = tk.Frame(row, bg=PANEL_BG)
        leds.pack(side="right", padx=5)
        self.com_led_card = make_status_led(leds, "COM PORT", "CLOSED", ACCENT_RED)
        self.com_led_card["frame"].pack(side="left", padx=(0, 8))
        self.hw_led_card = make_status_led(leds, "CLEARCORE IO0", "NO SIGNAL", ACCENT_RED)
        self.hw_led_card["frame"].pack(side="left", padx=(0, 8))
        # Was HEARTBEAT (3s). The heartbeat still runs — it is what detects
        # a dead board — but its state duplicates the two lamps to the left.
        # PLC reachability is not shown anywhere else and decides whether
        # HOME can work at all.
        self.plc_led_card = make_status_led(
            leds, f"PLC {PLC_IP}", PLC_LED_STATES["unknown"][0], TEXT_MUTED)
        self.plc_led_card["frame"].pack(side="left")

    # ── section 2 ────────────────────────────────────────────────────
    def _build_mode_section(self, main):
        s_mode, _ = make_section(main, "2. CONTROL MODE", 1)
        row = tk.Frame(s_mode, bg=PANEL_BG)
        row.pack(fill="x")

        self.btn_mode_p2p = RoundedButton(row, text="POINT TO POINT", icon="🎯",
                                          bg_color=ACCENT_MINT, fg_color=INK_DARK,
                                          width=250, height=42,
                                          command=lambda: self.set_mode("P2P"))
        self.btn_mode_p2p.pack(side="left", padx=(0, 12))
        self.btn_mode_jog = RoundedButton(row, text="JOYSTICK", icon="🕹",
                                          bg_color=SURFACE, fg_color=TEXT_LIGHT,
                                          width=250, height=42,
                                          command=lambda: self.set_mode("JOG"))
        self.btn_mode_jog.pack(side="left")
        tk.Label(row, text="  Switching mode auto-stops all motion first.",
                 bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_HINT).pack(side="left", padx=12)

    def _style_mode_buttons(self, p2p_active):
        if p2p_active:
            self.btn_mode_p2p.set_config("POINT TO POINT", ACCENT_MINT, icon="🎯", fg_color=INK_DARK)
            self.btn_mode_jog.set_config("JOYSTICK", SURFACE, icon="🕹", fg_color=TEXT_LIGHT)
        else:
            self.btn_mode_p2p.set_config("POINT TO POINT", SURFACE, icon="🎯", fg_color=TEXT_LIGHT)
            self.btn_mode_jog.set_config("JOYSTICK", ACCENT_MINT, icon="🕹", fg_color=INK_DARK)

    # ── section 3 ────────────────────────────────────────────────────
    def _build_motion_section(self, main):
        s_motion, self.motion_title_label = make_section(
            main, "3. MOTION CONTROL — POINT TO POINT", 2)
        self.p2p_frame = tk.Frame(s_motion, bg=PANEL_BG)
        self.jog_frame = tk.Frame(s_motion, bg=PANEL_BG)
        self._build_p2p_panel(self.p2p_frame)
        self._build_jog_panel(self.jog_frame)
        self.p2p_frame.pack(fill="both", expand=True)
        self.motion_lock_widgets += [self.btn_mode_p2p, self.btn_mode_jog]

    # ── section 4 ────────────────────────────────────────────────────
    def _build_log_section(self, main):
        section, _ = make_section(main, "4. EVENT LOG", 3, expand_vertically=True)
        wrap = RoundedFrame(section, bg=LED_BG, radius=RADIUS_MD,
                            border=BORDER_SOFT, border_w=1)
        wrap.pack(fill="both", expand=True)
        frame = wrap.body

        scrollbar = ttk.Scrollbar(frame, orient="vertical")
        self.log_text = tk.Text(frame, height=12, bg=LED_BG, fg=TEXT_LIGHT,
                                insertbackground=TEXT_LIGHT, relief="flat",
                                font=FONT_MONO, wrap="word",
                                yscrollcommand=scrollbar.set, state="disabled")
        scrollbar.configure(command=self.log_text.yview)
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self.log_text.tag_config("tx", foreground=ACCENT_MINT)
        self.log_text.tag_config("rx", foreground=ACCENT_GREEN)
        self.log_text.tag_config("warn", foreground=ACCENT_ORANGE)
        self.log_text.tag_config("error", foreground=ACCENT_RED)
        self.log_text.tag_config("default", foreground=TEXT_LIGHT)
