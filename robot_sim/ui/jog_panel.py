"""Section 3 — Joystick / jog panel.

A1M and A2M are separate motors, separate frog-leg linkages — each gets
its own pad pair. LINK toggle re-couples them into single "reach in/out"
gesture, tested on real hardware.
"""

import tkinter as tk

from ..config import JOG_STOP_COMMAND
from ..theme import (
    FONT_CAPTION,
    FONT_HINT,
    FONT_MONO,
    ACCENT_RED,
    ARM2_COLOR,
    ARM_COLOR,
    BORDER,
    INK_DARK,
    HI_ARM2,
    HI_PURPLE,
    HI_ROT,
    JZ_COLOR,
    PANEL_BG,
    ROT_COLOR,
    SURFACE,
    TEXT_LIGHT,
    TEXT_MUTED,
)
from ..widgets import (HomeButton, JogPad, RoundedButton, RoundedFrame,
                       make_led_card)


class JogPanelMixin:
    def _build_jog_panel(self, parent):
        top_row = tk.Frame(parent, bg=PANEL_BG)
        top_row.pack()

        self._build_rot_card(top_row)
        self._build_arm_cards(top_row)
        self._build_z_card(top_row)
        self._build_jog_readout(parent)
        self._build_jog_status(parent)

        # EMERGENCY STOP, back by request. Removed once on argument jog is
        # dead-man control and SPACE fires same path — both still true, but
        # neither helps someone hand-on-mouse watching machine not keyboard.
        # Stop control on one motion panel, not other, is its own hazard.
        #
        # Calls emergency_stop_all — SAME single audited path as P2P button
        # and SPACE key. No second stop implementation; that's the point.
        estop_row = tk.Frame(parent, bg=PANEL_BG)
        estop_row.pack(pady=(18, 0))
        self.jog_estop_btn = RoundedButton(
            estop_row, text="EMERGENCY STOP", icon="⬛", bg_color=ACCENT_RED,
            fg_color=INK_DARK, width=240, height=44,
            command=self.emergency_stop_all)
        self.jog_estop_btn.pack()
        # NOT in motion_lock_widgets deliberately: that list disables while
        # machine moves — precisely when this button must work.

        # RESET COORDINATES, same row P2P has. Declaring reference IS a
        # jogging job — drive to pose by hand, say "zero" — P2P-only meant
        # switching mode mid-operation, and mode switch auto-stops motion.
        # Shared builder, see ui/coord_reset.py; confirm/refuse-while-moving
        # checks live in reset_coordinates(), not duplicated here.
        self._build_plc_sensor_row(parent)
        self._build_coord_reset_row(parent, pady=(18, 0))

        self.jog_hint_v = tk.StringVar(value=self._jog_hint_text())
        tk.Label(parent, textvariable=self.jog_hint_v,
                 bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_HINT).pack(pady=(16, 0))

        self.motion_lock_widgets += [self.home_btn, self.link_btn]

    @staticmethod
    def _caps():
        from .. import keybinds
        return keybinds.to_keycaps(keybinds.active_map())

    def _pair_label(self, prefix, fwd, back):
        c = self._caps()
        return f"{prefix}  ({c.get(fwd, '?')}/{c.get(back, '?')})"

    @staticmethod
    def _jog_hint_text():
        from .. import keybinds
        return ("Hold a direction to move · release to stop · dead-man switch "
                "(no latching)   |   " + keybinds.to_hint(keybinds.active_map())
                + " · SPACE = E-STOP")

    def refresh_jog_keycaps(self):
        """Repaints every pad's keycap and hint strip from live layout.
        Called after Controls tab applies a change."""
        from .. import keybinds
        caps = keybinds.to_keycaps(keybinds.active_map())
        for command, pad in self.jog_pads.items():
            if command in caps:
                pad.set_keycap(caps[command])
        # Every key-name display must move together, or one keeps telling
        # operator to press a key that no longer does anything.
        for var, args in (("rot_title_v", ("ROTATION RM", "ROT_CCW", "ROT_CW")),
                          ("z_title_v",   ("Z AXIS ZM", "Z_UP", "Z_DOWN")),
                          ("a1_title_v",  ("A1M", "A1_FWD", "A1_BACK")),
                          ("a2_title_v",  ("A2M", "A2_FWD", "A2_BACK"))):
            if hasattr(self, var):
                getattr(self, var).set(self._pair_label(*args))
        if hasattr(self, "jog_hint_v"):
            self.jog_hint_v.set(self._jog_hint_text())

    def _build_rot_card(self, parent):
        wrap = RoundedFrame(parent, bg=PANEL_BG, border=BORDER, border_w=1)
        wrap.pack(side="left", padx=(0, 10), fill="y")
        card = wrap.body
        self.rot_title_v = tk.StringVar(
            value=self._pair_label("ROTATION RM", "ROT_CCW", "ROT_CW"))
        tk.Label(card, textvariable=self.rot_title_v, bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(anchor="w", padx=12, pady=(8, 0))

        grid = tk.Frame(card, bg=PANEL_BG)
        grid.pack(padx=14, pady=(6, 14))
        self._jog_pad(grid, 0, 0, "◀", "ROT CCW", "ROT_CCW", ROT_COLOR, HI_ROT)
        self.home_btn = HomeButton(grid, command=self.home, size=78)
        self.home_btn.grid(row=0, column=1, padx=4, pady=4)
        self._jog_pad(grid, 0, 2, "▶", "ROT CW", "ROT_CW", ROT_COLOR, HI_ROT)

    def _build_arm_cards(self, parent):
        wrap = RoundedFrame(parent, bg=PANEL_BG, border=BORDER, border_w=1)
        wrap.pack(side="left", padx=(0, 10), fill="y")
        card = wrap.body

        header = tk.Frame(card, bg=PANEL_BG)
        header.pack(fill="x", padx=12, pady=(8, 0))
        tk.Label(header, text="ARMS — INDEPENDENT", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(side="left")

        self.link_btn = RoundedButton(header, text="LINK: OFF", icon="🔗",
                                      bg_color=SURFACE, fg_color=TEXT_LIGHT,
                                      width=112, height=28, radius=12,
                                      command=self.toggle_arm_link)
        self.link_btn.pack(side="right")

        body = tk.Frame(card, bg=PANEL_BG)
        body.pack(padx=14, pady=(6, 14))

        # A2M LEFT, A1M RIGHT: default layout matches screen order to hand
        # order — stops wrong arm driven into something. Custom binding can
        # break pairing; labels stay live so they never lie about it.
        a2 = tk.Frame(body, bg=PANEL_BG)
        a2.grid(row=0, column=0, padx=(0, 10))
        self.a2_title_v = tk.StringVar(
            value=self._pair_label("A2M", "A2_FWD", "A2_BACK"))
        tk.Label(a2, textvariable=self.a2_title_v,
                 bg=PANEL_BG, fg=ARM2_COLOR,
                 font=FONT_CAPTION).pack(pady=(0, 4))
        self._jog_pad(a2, None, None, "▲", "EXTEND", "A2_FWD", ARM2_COLOR, HI_ARM2)
        self._jog_pad(a2, None, None, "▼", "RETRACT", "A2_BACK", ARM2_COLOR, HI_ARM2)

        a1 = tk.Frame(body, bg=PANEL_BG)
        a1.grid(row=0, column=1)
        self.a1_title_v = tk.StringVar(
            value=self._pair_label("A1M", "A1_FWD", "A1_BACK"))
        tk.Label(a1, textvariable=self.a1_title_v,
                 bg=PANEL_BG, fg=ARM_COLOR,
                 font=FONT_CAPTION).pack(pady=(0, 4))
        self._jog_pad(a1, None, None, "▲", "EXTEND", "A1_FWD", ARM_COLOR, HI_PURPLE)
        self._jog_pad(a1, None, None, "▼", "RETRACT", "A1_BACK", ARM_COLOR, HI_PURPLE)

    def _build_z_card(self, parent):
        wrap = RoundedFrame(parent, bg=PANEL_BG, border=BORDER, border_w=1)
        wrap.pack(side="left", fill="y")
        card = wrap.body
        self.z_title_v = tk.StringVar(
            value=self._pair_label("Z AXIS ZM", "Z_UP", "Z_DOWN"))
        tk.Label(card, textvariable=self.z_title_v, bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(anchor="w", padx=12, pady=(8, 0))

        grid = tk.Frame(card, bg=PANEL_BG)
        grid.pack(padx=14, pady=(6, 14))
        # JZ_COLOR not ACCENT_GREEN: pads were hardcoded green, so ZM looked
        # same in every scheme while other axes changed — ACCENT_GREEN is
        # "go" colour, roughly green everywhere by design. ZM shares RM's
        # tone instead (JZ_COLOR == ROT_COLOR).
        self._jog_pad(grid, 0, 0, "▲", "Z UP", "Z_UP", JZ_COLOR, HI_ROT)
        tk.Frame(grid, bg=PANEL_BG, height=8).grid(row=1, column=0)
        self._jog_pad(grid, 2, 0, "▼", "Z DOWN", "Z_DOWN", JZ_COLOR, HI_ROT)

    def _jog_pad(self, parent, row, col, glyph, label, start_cmd, base, hi):
        # Keycap looked up from JOG_KEYCAPS, not passed in — rebinding can
        # never leave pad showing old letter.
        from .. import keybinds
        keycap = keybinds.to_keycaps(keybinds.active_map()).get(start_cmd, "?")
        stop_cmd = JOG_STOP_COMMAND[start_cmd]
        pad = JogPad(parent, 78, 78, glyph, label, keycap, base, hi,
                     on_press=lambda: self.jog_start(start_cmd),
                     on_release=lambda: self.jog_stop(start_cmd, stop_cmd))
        if row is None:
            pad.pack(pady=3)
        else:
            pad.grid(row=row, column=col, padx=4, pady=4)
        self.jog_pads[start_cmd] = pad
        return pad

    def _build_jog_readout(self, parent):
        tk.Label(parent, text="LIVE JOG POSITION", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(anchor="w", pady=(16, 0))

        self.rot_pos_v = tk.StringVar(value="0.00 deg")
        # MOTOR degrees, 0 at home — not old CAD-flavoured 60.
        self.a1_pos_v = tk.StringVar(value="0.00 base deg")
        self.a2_pos_v = tk.StringVar(value="0.00 base deg")
        self.jz_pos_v = tk.StringVar(value="0.00 mm")

        row = tk.Frame(parent, bg=PANEL_BG)
        row.pack(fill="x", pady=(4, 0))
        make_led_card(row, 0, "ROT_POS", self.rot_pos_v, ROT_COLOR)
        make_led_card(row, 1, "A1M_BASE", self.a1_pos_v, ARM_COLOR)
        make_led_card(row, 2, "A2M_BASE", self.a2_pos_v, ARM2_COLOR)
        make_led_card(row, 3, "Z_POS (d1)", self.jz_pos_v, JZ_COLOR)

        # Elbow angle alone hard to judge; show radius it produces.
        self.a1_reach_v = tk.StringVar(value="R1 = 133.2 mm")
        self.a2_reach_v = tk.StringVar(value="R2 = 133.2 mm")
        reach_row = tk.Frame(parent, bg=PANEL_BG)
        reach_row.pack(fill="x", pady=(4, 0))
        tk.Label(reach_row, textvariable=self.a1_reach_v, bg=PANEL_BG, fg=ARM_COLOR,
                 font=FONT_MONO).pack(side="left", padx=(4, 18))
        tk.Label(reach_row, textvariable=self.a2_reach_v, bg=PANEL_BG, fg=ARM2_COLOR,
                 font=FONT_MONO).pack(side="left")

    def _build_jog_status(self, parent):
        row = tk.Frame(parent, bg=PANEL_BG)
        row.pack(fill="x", pady=(14, 0))

        self.jog_dot = tk.Canvas(row, width=10, height=10, bg=PANEL_BG, highlightthickness=0)
        self.jog_dot.pack(side="left", padx=(0, 6))
        self._jog_dot_id = self.jog_dot.create_oval(1, 1, 9, 9, fill=TEXT_MUTED, outline="")

        self.jog_status_var = tk.StringVar(value="Idle — hold a key or pad to jog")
        tk.Label(row, textvariable=self.jog_status_var, bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_MONO).pack(side="left")

        self.boost_btn = RoundedButton(row, text="BOOST: OFF", icon="⚡", bg_color=SURFACE,
                                       fg_color=TEXT_LIGHT, width=140, height=32,
                                       command=self.cycle_boost)
        self.boost_btn.pack(side="right")
        tk.Label(row, text="Jog too slow? BOOST gives x1.5 / x2:",
                 bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_HINT).pack(side="right", padx=(0, 8))
