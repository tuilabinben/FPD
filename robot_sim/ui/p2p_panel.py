"""Section 3 — Point-to-point panel."""

import tkinter as tk
from tkinter import ttk

# No reach constants here any more. Workspace line built from operator's
# taught boundaries by _refresh_workspace_hint() — only limits actually applied.
from ..config import ARM_CONFIGS, DEFAULT_POINT_A, DEFAULT_POINT_B
from ..theme import (
    FONT_CAPTION,
    FONT_HINT,
    FONT_MONO,
    FONT_READOUT,
    ACCENT_GREEN,
    ACCENT_MINT,
    ARM2_COLOR,
    ACCENT_RED,
    ARM_COLOR,
    AXIS_X_COLOR,
    AXIS_Y_COLOR,
    AXIS_Z_COLOR,
    SURFACE,
    INK_DARK,
    PANEL_BG,
    ROT_COLOR,
    TEXT_LIGHT,
    TEXT_MUTED,
)
from ..widgets import HomeButton, RoundedButton, make_coord_card, make_led_card


class P2PPanelMixin:
    def _build_p2p_panel(self, parent):
        # Left column: Point A stacked above Point B (+ HOME-frame caption /
        # workspace-hint). Right column: Oxy board, wide open. Used to be a
        # side column squeezed beside two side-by-side point blocks, which
        # clipped the right edge of the workspace circle — annulus must be
        # fully visible or plot is worse than no plot. Stacking A/B left
        # frees a dedicated column for it.
        # grid, not pack, for the split: grid column weights unambiguous —
        # column 0 at weight=0 gets EXACTLY its content's natural width,
        # however wide `split` stretches from the scrollable body above.
        # Nested pack fill="x" chains let left column balloon to container's
        # full width (coordinate cards' grid columns have weight=1 of their
        # own — see _build_coordinate_inputs — so widening their container
        # widened cards too), pushing board off right edge with no h-scroll
        # to reach it.
        split = tk.Frame(parent, bg=PANEL_BG)
        split.pack(fill="x")
        split.grid_columnconfigure(0, weight=0)
        split.grid_columnconfigure(1, weight=1)
        left_col = tk.Frame(split, bg=PANEL_BG)
        left_col.grid(row=0, column=0, sticky="ns")
        right_col = tk.Frame(split, bg=PANEL_BG)
        # sticky="n", NOT "nsew": column 1 weight=1 so its CELL absorbs all
        # leftover width (varies with window's own width on resize).
        # right_col stays exactly board-width, CENTRED in that cell — fixed
        # padding looked right at one window size, lopsided at another;
        # centring uses whatever space exists, both sides, any width.
        right_col.grid(row=0, column=1, sticky="n")

        # Inputs FIRST: board reads x0_v..y1_v on first paint; building it
        # before they exist draws empty plot.
        self._build_coordinate_inputs(left_col)
        self._build_xy_board(right_col).pack(side="top", anchor="nw")

        self._build_arm_selector(parent)
        self._build_computed_targets(parent)
        self._build_telemetry(parent)
        self._build_progress(parent)
        self._build_p2p_buttons(parent)

    # ── coordinates ──────────────────────────────────────────────────
    def _build_coordinate_inputs(self, parent):
        # A directly above B — easier to eyeball X/Y/Z between the two than
        # old side-by-side layout, and frees right column for the board.
        start_col = tk.Frame(parent, bg=PANEL_BG)
        start_col.pack(side="top", fill="x")
        tk.Label(start_col, text="POINT A (mm from HOME)", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(anchor="w")
        start_grid = tk.Frame(start_col, bg=PANEL_BG)
        start_grid.pack(fill="x", pady=(4, 0))
        # Defaults must be genuinely reachable. Z measured UP FROM HOME
        # (0..285), radius never below 133.2 mm — "0,0,0" still not a valid
        # point even as reference: at HOME arm retracted, centre 133.2 mm out.
        self.x0_v = tk.StringVar(value=f"{DEFAULT_POINT_A[0]:g}")
        self.y0_v = tk.StringVar(value=f"{DEFAULT_POINT_A[1]:g}")
        self.z0_v = tk.StringVar(value=f"{DEFAULT_POINT_A[2]:g}")
        make_coord_card(start_grid, 0, "X0", AXIS_X_COLOR, self.x0_v, compact=True)
        make_coord_card(start_grid, 1, "Y0", AXIS_Y_COLOR, self.y0_v, compact=True)
        make_coord_card(start_grid, 2, "Z0", AXIS_Z_COLOR, self.z0_v, compact=True)
        # make_coord_card sets weight=1 on each column (also serves old
        # full-width side-by-side layout elsewhere). Row sits in narrow left
        # column here, so pin columns to natural size instead of stretching
        # -- three compact cards, not ballooned across whole column.
        for _c in range(3):
            start_grid.grid_columnconfigure(_c, weight=0)
        # Real height of deck, live, under the typed Z. Z is from HOME now,
        # so entry alone no longer says where arm actually is off floor —
        # that's the number compared against cassette, load port, tape measure.
        self.z0_real_v = tk.StringVar(value="")
        tk.Label(start_col, textvariable=self.z0_real_v, bg=PANEL_BG,
                 fg=AXIS_Z_COLOR, font=FONT_MONO).pack(anchor="w", pady=(2, 0))

        target_col = tk.Frame(parent, bg=PANEL_BG)
        target_col.pack(side="top", fill="x", pady=(12, 0))
        tk.Label(target_col, text="POINT B (mm from HOME)", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(anchor="w")
        target_grid = tk.Frame(target_col, bg=PANEL_BG)
        target_grid.pack(fill="x", pady=(4, 0))
        self.x1_v = tk.StringVar(value=f"{DEFAULT_POINT_B[0]:g}")
        self.y1_v = tk.StringVar(value=f"{DEFAULT_POINT_B[1]:g}")
        self.z1_v = tk.StringVar(value=f"{DEFAULT_POINT_B[2]:g}")
        make_coord_card(target_grid, 0, "X1", AXIS_X_COLOR, self.x1_v, compact=True)
        make_coord_card(target_grid, 1, "Y1", AXIS_Y_COLOR, self.y1_v, compact=True)
        self.z1_entry = make_coord_card(target_grid, 2, "Z1", AXIS_Z_COLOR, self.z1_v,
                                        compact=True)
        for _c in range(3):
            target_grid.grid_columnconfigure(_c, weight=0)
        self.z1_real_v = tk.StringVar(value="")
        tk.Label(target_col, textvariable=self.z1_real_v, bg=PANEL_BG,
                 fg=AXIS_Z_COLOR, font=FONT_MONO).pack(anchor="w", pady=(2, 0))

        self.z0_v.trace_add("write", self._sync_z_for_both_mode)
        # Readouts follow typed value directly. Recomputing only on LOAD
        # would leave stale height on screen while operator still deciding
        # — exactly when they're reading it.
        self.z0_v.trace_add("write", self._refresh_real_heights)
        self.z1_v.trace_add("write", self._refresh_real_heights)
        # Board follows boxes every keystroke, so typed target visible
        # before LOAD accepts or refuses it.
        for _v in (self.x0_v, self.y0_v, self.x1_v, self.y1_v):
            _v.trace_add("write", lambda *_a: self._refresh_xy_board())
        self._refresh_real_heights()

        # Frame spelled out. HOME is reference: X/Y from turntable axis,
        # signed; Z up from HOME, never negative.
        #
        # wraplength IS the actual fix for board pushed off right edge:
        # unwrapped one-line Label reports NATURAL width as full text length
        # (~900+ px here), made `left_col` that wide regardless of how
        # compact coordinate cards above were -- cards sat left-aligned in a
        # secretly enormous column. Capped to cards' own width, wraps onto
        # a few lines instead, column shrinks to match.
        _caption_width = 3 * 150   # ~= one row of 3 compact coord cards
        tk.Label(parent,
                 text=("HOME = X 0 · Y 0 · Z 0.  X, Y measured from the turntable "
                       "axis and may be negative; Z is height above HOME and may "
                       "not."),
                 bg=PANEL_BG, fg=TEXT_MUTED, wraplength=_caption_width,
                 justify="left",
                 font=FONT_MONO).pack(anchor="w", pady=(6, 0))
        # Workspace line is LIVE, reports YOUR limits, not structural
        # envelope. Old line quoted fixed 133.2-613.2 mm reach; floor
        # assumed elbow zero really is folded home pose and fold angle is
        # motor degrees over unmeasured gear ratio. IK no longer enforces
        # it, so advertising it here would name a boundary nothing applies.
        self.workspace_hint_v = tk.StringVar(value="")
        tk.Label(parent, textvariable=self.workspace_hint_v,
                 bg=PANEL_BG, fg=TEXT_MUTED, wraplength=_caption_width,
                 justify="left",
                 font=FONT_MONO).pack(anchor="w", pady=(2, 0))
        self._refresh_workspace_hint()

    def _build_arm_selector(self, parent):
        row = tk.Frame(parent, bg=PANEL_BG)
        row.pack(fill="x", pady=(14, 0))
        tk.Label(row, text="ARM SELECT:", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(side="left", padx=(0, 10))

        for cfg in ARM_CONFIGS:
            active = cfg == self.arm_config
            b = RoundedButton(row, text=cfg,
                              bg_color=ACCENT_MINT if active else SURFACE,
                              fg_color=INK_DARK if active else TEXT_LIGHT,
                              width=90, height=32,
                              command=lambda c=cfg: self.set_arm_config(c))
            b.pack(side="left", padx=4)
            self.arm_buttons[cfg] = b

        tk.Label(row, text="  (A1M / A2M = one arm · BOTH = both at once — the two "
                           "points must share a bearing and differ only in radius)",
                 bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_HINT).pack(side="left")

    def _build_computed_targets(self, parent):
        tk.Label(parent, text="COMPUTED JOINT TARGETS (d1 mm / RM ° / A1M base ° / A2M base °  —  base 0° = retracted, 90° = straight out)",
                 bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(anchor="w", pady=(14, 0))

        self.calc_a_d1_v = tk.StringVar(value="—")
        self.calc_a_rot_v = tk.StringVar(value="—")
        self.calc_a_a1_v = tk.StringVar(value="—")
        self.calc_a_a2_v = tk.StringVar(value="—")
        self._build_target_row(parent, "A", self.calc_a_d1_v, self.calc_a_rot_v,
                               self.calc_a_a1_v, self.calc_a_a2_v)

        self.calc_b_d1_v = tk.StringVar(value="—")
        self.calc_b_rot_v = tk.StringVar(value="—")
        self.calc_b_a1_v = tk.StringVar(value="—")
        self.calc_b_a2_v = tk.StringVar(value="—")
        self._build_target_row(parent, "B", self.calc_b_d1_v, self.calc_b_rot_v,
                               self.calc_b_a1_v, self.calc_b_a2_v)

    def _build_target_row(self, parent, label, d1_v, rot_v, a1_v, a2_v):
        row = tk.Frame(parent, bg=PANEL_BG)
        row.pack(fill="x", pady=(4, 0))
        tk.Label(row, text=label, bg=PANEL_BG, fg=AXIS_X_COLOR, width=2,
                 font=FONT_READOUT).pack(side="left")
        cards = tk.Frame(row, bg=PANEL_BG)
        cards.pack(side="left", fill="x", expand=True)
        make_led_card(cards, 0, "D1", d1_v, AXIS_Z_COLOR)
        make_led_card(cards, 1, "ROT", rot_v, ROT_COLOR)
        make_led_card(cards, 2, "A1M", a1_v, ARM_COLOR)
        make_led_card(cards, 3, "A2M", a2_v, ARM2_COLOR)

    def _build_telemetry(self, parent):
        tk.Label(parent, text="LIVE TELEMETRY (from ClearCore)", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(anchor="w", pady=(16, 0))

        self.curr_d1_v = tk.StringVar(value="0.00 mm")
        self.curr_rot_v = tk.StringVar(value="0.00 deg")
        self.curr_a1_v = tk.StringVar(value="0.00 deg")
        self.curr_a2_v = tk.StringVar(value="0.00 deg")

        row = tk.Frame(parent, bg=PANEL_BG)
        row.pack(fill="x", pady=(4, 0))
        make_led_card(row, 0, "D1_CURR", self.curr_d1_v, AXIS_Z_COLOR)
        make_led_card(row, 1, "ROT_CURR", self.curr_rot_v, ROT_COLOR)
        make_led_card(row, 2, "A1M_CURR", self.curr_a1_v, ARM_COLOR)
        make_led_card(row, 3, "A2M_CURR", self.curr_a2_v, ARM2_COLOR)

        self.calc_xyz_v = tk.StringVar(
            value="Cartesian (from forward kinematics): X=0.00  Y=0.00  Z=0.00 mm")
        tk.Label(parent, textvariable=self.calc_xyz_v, bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_MONO).pack(anchor="w", pady=(4, 0))

    def _build_progress(self, parent):
        block = tk.Frame(parent, bg=PANEL_BG)
        block.pack(fill="x", pady=(10, 0))
        tk.Label(block, text="PROGRESS", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(anchor="w")

        bar_row = tk.Frame(block, bg=PANEL_BG)
        bar_row.pack(fill="x", pady=(4, 0))
        self.progress_var = tk.DoubleVar(value=0.0)
        self.progress_bar = ttk.Progressbar(bar_row, orient="horizontal", mode="determinate",
                                            style="Neu.Horizontal.TProgressbar",
                                            variable=self.progress_var, maximum=100)
        self.progress_bar.pack(side="left", fill="x", expand=True)
        self.progress_pct_var = tk.StringVar(value="0%")
        tk.Label(bar_row, textvariable=self.progress_pct_var, bg=PANEL_BG, fg=ACCENT_MINT,
                 font=FONT_MONO, width=5).pack(side="left", padx=(8, 0))

    def _build_p2p_buttons(self, parent):
        row1 = tk.Frame(parent, bg=PANEL_BG)
        row1.pack(pady=(18, 6))
        self.btn_p2p_load = RoundedButton(row1, text="LOAD PARAMETERS", icon="⇩",
                                          bg_color=SURFACE, fg_color=TEXT_LIGHT,
                                          width=210, height=44,
                                          command=self.p2p_load_parameters)
        self.btn_p2p_load.pack(side="left", padx=6)
        self.btn_p2p_run = RoundedButton(row1, text="RUN PROGRAM", icon="▶",
                                         bg_color=ACCENT_GREEN, fg_color=INK_DARK,
                                         width=190, height=44,
                                         command=self.p2p_run_program)
        self.btn_p2p_run.pack(side="left", padx=6)
        self.p2p_home_btn = HomeButton(row1, command=self.home, size=56)
        self.p2p_home_btn.pack(side="left", padx=6)

        tk.Label(parent, text="ENTER = RUN PROGRAM · SPACE = E-STOP  "
                             "(not while a coordinate box has focus)",
                 bg=PANEL_BG, fg=TEXT_MUTED, font=FONT_HINT).pack()

        # Plain STOP button gone by request. EMERGENCY STOP remains: single
        # audited stop path (emergency_stop_all) SPACE also fires — removing
        # STOP lost a duplicate, not ability to halt a running program.
        row2 = tk.Frame(parent, bg=PANEL_BG)
        row2.pack(pady=(0, 4))
        RoundedButton(row2, text="EMERGENCY STOP", icon="⬛", bg_color=ACCENT_RED,
                      fg_color=INK_DARK, width=240, height=44,
                      command=self.emergency_stop_all).pack(side="left", padx=6)

        # RESET COORDINATES lives here, not Settings: machine action used
        # while jogging to reference; a dialog meant leaving the controls
        # you were using. Each button still confirms before acting. JOG
        # panel builds same row from same builder — see ui/coord_reset.py —
        # so the two can't drift apart.
        self._build_plc_sensor_row(parent)
        self._build_coord_reset_row(parent)

        # Every arm button locked during motion — original list left out
        # "BOTH", so config could still be switched mid-program.
        self.motion_lock_widgets += [self.btn_p2p_load, self.btn_p2p_run, self.p2p_home_btn]
        self.motion_lock_widgets += list(self.arm_buttons.values())
