"""Section 3 — SCAN panel.

Left: the four numbers the operator knows, and the live derivation under
them. Right: the polar view, previous layer ghosted.

The derivation line is a READOUT, not a staged edit — it follows every
keystroke, like the P2P real-height line.
"""

import tkinter as tk
from tkinter import ttk

from ..config import (
    DEFAULT_SCAN_SAMPLE_HZ,
    DEFAULT_SCAN_POINTS_PER_SLICE,
    DEFAULT_SCAN_SENSOR,
    DEFAULT_SCAN_SLICES,
    DEFAULT_SCAN_SLICE_GAP_MM,
    DEFAULT_SCAN_SWEEP_DEG,
    SCAN_SENSOR_KINDS,
)
from ..theme import (
    ACCENT_GREEN,
    ACCENT_MINT,
    ACCENT_ORANGE,
    ACCENT_RED,
    FONT_CAPTION,
    FONT_HINT,
    FONT_MONO,
    INK_DARK,
    LED_BG,
    PANEL_BG,
    SURFACE,
    TEXT_LIGHT,
    TEXT_MUTED,
)
from ..widgets import RoundedButton, make_inset_entry, make_well
from .scan_plot import ScanPolarPlot


class ScanPanelMixin:
    def _build_scan_panel(self, parent):
        split = tk.Frame(parent, bg=PANEL_BG)
        split.pack(fill="x")
        split.grid_columnconfigure(0, weight=0)
        split.grid_columnconfigure(1, weight=1)
        left = tk.Frame(split, bg=PANEL_BG)
        left.grid(row=0, column=0, sticky="ns")
        right = tk.Frame(split, bg=PANEL_BG)
        right.grid(row=0, column=1, sticky="n")

        self._build_scan_inputs(left)
        self._build_scan_plot(right)
        self._build_scan_sensor_row(parent)
        self._build_scan_buttons(parent)
        self._build_plc_sensor_row(parent)
        self._build_coord_reset_row(parent)

        self._refresh_scan_hint()
        self._refresh_scan_buttons()
        self._refresh_scan_sensor_lamp()

    # ── the four numbers ─────────────────────────────────────────────
    def _scan_field(self, parent, row, label, var, unit, hint):
        tk.Label(parent, text=label, bg=PANEL_BG, fg=TEXT_LIGHT,
                 font=FONT_CAPTION, anchor="w").grid(
            row=row, column=0, sticky="w", pady=4, padx=(0, 12))
        wrap, _entry = make_inset_entry(parent, var, width=8)
        wrap.grid(row=row, column=1, sticky="e")
        tk.Label(parent, text=unit, bg=PANEL_BG, fg=TEXT_MUTED, width=6,
                 anchor="w", font=FONT_MONO).grid(row=row, column=2,
                                                  sticky="w", padx=(8, 0))
        tk.Label(parent, text=hint, bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_HINT, anchor="w").grid(row=row, column=3,
                                                  sticky="w", padx=(12, 0))
        var.trace_add("write", lambda *_a: self._refresh_scan_hint())

    def _build_scan_inputs(self, parent):
        tk.Label(parent, text="SCAN PARAMETERS", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(anchor="w")

        grid = tk.Frame(parent, bg=PANEL_BG)
        grid.pack(fill="x", pady=(6, 0))

        self.scan_hz_v = tk.StringVar(value=f"{DEFAULT_SCAN_SAMPLE_HZ:g}")
        self.scan_points_v = tk.StringVar(value=f"{DEFAULT_SCAN_POINTS_PER_SLICE:g}")
        self.scan_slices_v = tk.StringVar(value=f"{DEFAULT_SCAN_SLICES:g}")
        self.scan_gap_v = tk.StringVar(value=f"{DEFAULT_SCAN_SLICE_GAP_MM:g}")
        self.scan_sweep_v = tk.StringVar(value=f"{DEFAULT_SCAN_SWEEP_DEG:g}")

        self._scan_field(grid, 0, "SENSOR SAMPLE RATE", self.scan_hz_v, "Hz",
                         "what the sensor can deliver")
        self._scan_field(grid, 1, "POINTS PER SLICE", self.scan_points_v, "pts",
                         "how finely one slice is read")
        self._scan_field(grid, 2, "SLICES", self.scan_slices_v, "",
                         "how many heights")
        self._scan_field(grid, 3, "SLICE SPACING", self.scan_gap_v, "mm",
                         "how far ZM rises between them")
        self._scan_field(grid, 4, "SWEEP", self.scan_sweep_v, "°",
                         "how far round each slice goes, 1–340")

        # Speed is an OUTPUT: 50 points at 50 Hz is 1 s a slice, so a
        # 330° sweep is 330°/s. Operator knows points, not deg/s.
        well = make_well(parent)
        well.pack(fill="x", pady=(10, 0))
        self.scan_hint_v = tk.StringVar(value="")
        self.scan_hint_lbl = tk.Label(well.body, textvariable=self.scan_hint_v,
                                      bg=LED_BG, fg=ACCENT_MINT, font=FONT_MONO,
                                      justify="left", anchor="w")
        self.scan_hint_lbl.pack(fill="x", padx=10, pady=(8, 2))
        self.scan_warn_v = tk.StringVar(value="")
        self.scan_warn_lbl = tk.Label(well.body, textvariable=self.scan_warn_v,
                                      bg=LED_BG, fg=ACCENT_ORANGE,
                                      font=FONT_HINT, justify="left",
                                      anchor="w", wraplength=420)
        self.scan_warn_lbl.pack(fill="x", padx=10, pady=(0, 8))

    # ── sensor picker and lamp ───────────────────────────────────────
    def _build_scan_sensor_row(self, parent):
        row = tk.Frame(parent, bg=PANEL_BG)
        row.pack(fill="x", pady=(12, 0))

        tk.Label(row, text="SENSOR", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(side="left", padx=(0, 8))
        self.scan_sensor_v = tk.StringVar(value=DEFAULT_SCAN_SENSOR)
        combo = ttk.Combobox(row, textvariable=self.scan_sensor_v, width=13,
                             state="readonly", values=list(SCAN_SENSOR_KINDS))
        combo.pack(side="left", padx=(0, 12))
        self._wheel_scrolls_page(combo)

        # An unplugged sensor and an empty room look identical until a
        # whole layer has been swept. This says which, before the run.
        self.scan_sensor_lamp = tk.Label(row, text="● no reading yet",
                                         bg=PANEL_BG, fg=TEXT_MUTED,
                                         font=FONT_MONO)
        self.scan_sensor_lamp.pack(side="left")
        self.scan_sensor_hint = tk.Label(row, text="", bg=PANEL_BG,
                                         fg=TEXT_MUTED, font=FONT_HINT)
        self.scan_sensor_hint.pack(side="left", padx=(10, 0))

    # ── plot ─────────────────────────────────────────────────────────
    def _build_scan_plot(self, parent):
        self.scan_plot = ScanPolarPlot(parent)
        self.scan_plot.pack(side="top")
        self.scan_scale_v = tk.StringVar(
            value="0° is straight ahead at RM 0")
        tk.Label(parent, textvariable=self.scan_scale_v, bg=PANEL_BG,
                 fg=TEXT_MUTED, font=FONT_HINT).pack(anchor="center")
        self.scan_progress_v = tk.StringVar(value="Idle")
        tk.Label(parent, textvariable=self.scan_progress_v, bg=PANEL_BG,
                 fg=TEXT_LIGHT, font=FONT_MONO).pack(anchor="center")
        self.scan_counts_v = tk.StringVar(value="0 points · 0 missed")
        tk.Label(parent, textvariable=self.scan_counts_v, bg=PANEL_BG,
                 fg=TEXT_MUTED, font=FONT_HINT).pack(anchor="center")

    # ── buttons ──────────────────────────────────────────────────────
    def _build_scan_buttons(self, parent):
        row = tk.Frame(parent, bg=PANEL_BG)
        row.pack(fill="x", pady=(12, 0))

        self.btn_scan_start = RoundedButton(
            row, text="START SCAN", icon="◎", bg_color=ACCENT_GREEN,
            fg_color=INK_DARK, width=170, height=42, command=self.start_scan)
        self.btn_scan_start.pack(side="left", padx=(0, 10))

        # NOT in motion_lock_widgets: that list dies while moving, which
        # is when STOP has to work.
        self.btn_scan_stop = RoundedButton(
            row, text="STOP SCAN", icon="■", bg_color=SURFACE,
            fg_color=TEXT_LIGHT, width=150, height=42, command=self.stop_scan)
        self.btn_scan_stop.pack(side="left", padx=(0, 10))

        self.btn_scan_read = RoundedButton(
            row, text="TEST READ", icon="📏", bg_color=SURFACE,
            fg_color=TEXT_LIGHT, width=150, height=42,
            command=self.scan_test_read)
        self.btn_scan_read.pack(side="left", padx=(0, 10))

        self.btn_scan_save = RoundedButton(
            row, text="SAVE CSV…", icon="💾", bg_color=SURFACE,
            fg_color=TEXT_LIGHT, width=150, height=42,
            command=self.save_scan_csv)
        self.btn_scan_save.pack(side="left", padx=(0, 10))

        # Same audited path as the other panels: one stop, three buttons.
        RoundedButton(row, text="EMERGENCY STOP  SPACE", icon="⏹",
                      bg_color=ACCENT_RED, fg_color=TEXT_LIGHT, width=260,
                      height=42, command=self.emergency_stop_all).pack(
            side="right")

        self.motion_lock_widgets += [self.btn_scan_read]

    # ── live readouts ────────────────────────────────────────────────
    def _refresh_scan_hint(self, *_a):
        """Every keystroke: what the four numbers work out as."""
        plan, error = self.scan_current_plan()
        if error:
            self.scan_hint_v.set("—")
            self.scan_hint_lbl.config(fg=TEXT_MUTED)
            self.scan_warn_v.set(error)
            self.scan_warn_lbl.config(fg=ACCENT_RED)
            return
        self.scan_hint_v.set(
            f"{plan['deg_step']:.2f}° between points · "
            f"{plan['points_in_slice']} points a slice\n"
            f"RM {plan['rot_deg_s']:.1f}°/s · {plan['slice_seconds']:.2f} s a "
            f"slice · {plan['sweep_seconds_total']:.0f} s of sweeping\n"
            f"ZM {plan['z_travel_mm']:.1f} mm total, ceiling "
            f"{plan['z_travel_max_mm']:g} mm")
        self.scan_hint_lbl.config(
            fg=ACCENT_ORANGE if plan["z_travel_over"] else ACCENT_MINT)
        self.scan_warn_v.set("  ".join("⚠ " + w for w in plan["warnings"]))
        self.scan_warn_lbl.config(fg=ACCENT_ORANGE)

    def _refresh_scan_buttons(self):
        running = getattr(self, "scan_running", False)
        if not hasattr(self, "btn_scan_start"):
            return
        self.btn_scan_start.set_enabled(not running)
        self.btn_scan_stop.set_enabled(running)

    def _refresh_scan_sensor_lamp(self):
        if not hasattr(self, "scan_sensor_lamp"):
            return
        state, headline, detail = self.scan_sensor_health()
        colour = {"none": TEXT_MUTED, "ok": ACCENT_GREEN,
                  "bad": ACCENT_RED}[state]
        self.scan_sensor_lamp.config(text="● " + headline, fg=colour)
        self.scan_sensor_hint.config(
            text=detail, fg=ACCENT_RED if state == "bad" else TEXT_MUTED)
