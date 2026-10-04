"""Jog panel, second layout — XYZ: the tool point along Cartesian axes.

WHAT IT IS
----------
The JOINT layout gives each key one motor. This one gives each key one
AXIS OF THE TOOL: hold +X and the wafer centre travels along +X, which
takes RM and the selected elbow together. The board does the coordinating
(XJOG in the firmware); this file is the pads, the arm and speed controls,
and the picture.

THE FRAME IS THE P2P ONE
------------------------
X, Y from the turntable axis, Z above HOME — so HOME is not the origin, it
is the point (133.2, 0, 0): the tool's own position with the arm retracted
and the lift down. The view marks it, and the readout gives the offset
from it, because "how far have I moved from HOME" is the question a jog
answers and the absolute numbers are the ones P2P takes.

THE 3D VIEW — BOTH LAYOUTS' NOW
------------------------------
The scene and the projection are ui/view3d.py's, shared with the P2P
board. On top of them this draws the tool — drop line, shadow, the arm
from the lift column — and a trail of where it has been.

It is ONE canvas beside whichever pads are showing (asked for: the JOINT
layout had no picture). What it shows follows the layout, because what a
key moves does: XYZ draws the one tool the keys drive, in red; JOINT draws
BOTH arms' tools, each in its pads' colour and named, because there every
arm has its own keys. Each tool has its own trail.

It repaints from the shared pose on every telemetry line, so it must never
raise.
"""

import math
import tkinter as tk

from ..config import (
    ARM_MIN_REACH_MM,
    DEFAULT_XYZ_JOG_MM_S,
    XYZ_JOG_WARN_MM_S,
)
from ..hidpi import px
from ..theme import (
    ACCENT_CYAN,
    ACCENT_MINT,
    ACCENT_ORANGE,
    ACCENT_RED,
    ARM2_COLOR,
    ARM_COLOR,
    BORDER,
    BORDER_SOFT,
    FONT_CAPTION,
    FONT_HINT,
    FONT_MONO,
    HI_ROT,
    INK_DARK,
    JZ_COLOR,
    LED_BG,
    PANEL_BG,
    ROT_COLOR,
    SURFACE,
    TEXT_LIGHT,
    TEXT_MUTED,
)
from ..widgets import HomeButton, RoundedButton, RoundedFrame, make_inset_entry
from .view3d import VIEW_ASPECT, View, attach_view_controls, draw_scene, draw_tool

#: Canvas size and inner margin, before DPI scaling. The WIDTH is only a
#: minimum — the view stretches to the room the pads leave — and the height
#: follows it between XYZ_VIEW_H and XYZ_VIEW_MAX_H, like the P2P board's.
XYZ_VIEW_W, XYZ_VIEW_H, XYZ_VIEW_PAD = 520, 400, 26
XYZ_VIEW_MAX_H = 520

#: The trail: a point is added once the tool has moved this far, and the
#: oldest dropped past the cap. Enough for a few minutes of jogging.
XYZ_TRAIL_STEP_MM = 1.0
XYZ_TRAIL_MAX = 600


class XYZJogMixin:
    # ── the JOINT | XYZ switch ───────────────────────────────────────
    def _build_jog_frame_switch(self, parent):
        row = tk.Frame(parent, bg=PANEL_BG)
        row.pack(anchor="w", pady=(0, px(10)))
        tk.Label(row, text="JOG LAYOUT", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(side="left", padx=(0, px(10)))
        self.jog_frame_buttons = {}
        for frame, caption in (("JOINT", "JOINT — one motor per key"),
                               ("XYZ", "XYZ — move the tool point")):
            btn = RoundedButton(row, text=caption, bg_color=SURFACE,
                                fg_color=TEXT_LIGHT, width=230, height=32,
                                command=lambda f=frame: self.set_jog_frame(f))
            btn.pack(side="left", padx=(0, px(8)))
            self.jog_frame_buttons[frame] = btn
        self.motion_lock_widgets += list(self.jog_frame_buttons.values())

    # ── the XYZ layout itself ────────────────────────────────────────
    def _build_xyz_jog_frame(self, parent):
        left = tk.Frame(parent, bg=PANEL_BG)
        left.pack(anchor="w")

        pads = tk.Frame(left, bg=PANEL_BG)
        pads.pack(anchor="w")

        wrap = RoundedFrame(pads, bg=PANEL_BG, border=BORDER, border_w=1)
        wrap.pack(side="left", padx=(0, px(10)), fill="y")
        self.xy_title_v = tk.StringVar(value=self._xyz_title("XY"))
        tk.Label(wrap.body, textvariable=self.xy_title_v, bg=PANEL_BG,
                 fg=TEXT_MUTED, font=FONT_CAPTION).pack(anchor="w", padx=12, pady=(8, 0))
        grid = tk.Frame(wrap.body, bg=PANEL_BG)
        grid.pack(padx=14, pady=(6, 14))
        # A cross, laid out the way the axes run on the Oxy board: +Y up,
        # +X right.
        self._jog_pad(grid, 0, 1, "▲", "Y +", "Y_POS", ROT_COLOR, HI_ROT)
        self._jog_pad(grid, 1, 0, "◀", "X −", "X_NEG", ROT_COLOR, HI_ROT)
        # HOME in the middle of the cross, where the JOINT layout has it
        # between its two RM pads. Same command, same lock.
        self.xjog_home_btn = HomeButton(grid, command=self.home, size=78)
        self.xjog_home_btn.grid(row=1, column=1, padx=4, pady=4)
        self._jog_pad(grid, 1, 2, "▶", "X +", "X_POS", ROT_COLOR, HI_ROT)
        self._jog_pad(grid, 2, 1, "▼", "Y −", "Y_NEG", ROT_COLOR, HI_ROT)
        # RESET POS in the free corner of the cross, diagonal to HOME.
        self.xjog_reset_btn = self._build_reset_pos_button(grid, size=78)
        self.xjog_reset_btn.grid(row=2, column=2, padx=4, pady=4)

        wrap = RoundedFrame(pads, bg=PANEL_BG, border=BORDER, border_w=1)
        wrap.pack(side="left", fill="y")
        self.xz_title_v = tk.StringVar(value=self._xyz_title("Z"))
        tk.Label(wrap.body, textvariable=self.xz_title_v, bg=PANEL_BG,
                 fg=TEXT_MUTED, font=FONT_CAPTION).pack(anchor="w", padx=12, pady=(8, 0))
        grid = tk.Frame(wrap.body, bg=PANEL_BG)
        grid.pack(padx=14, pady=(6, 14))
        self._jog_pad(grid, 0, 0, "▲", "Z +", "Z_POS", JZ_COLOR, HI_ROT)
        tk.Frame(grid, bg=PANEL_BG, height=8).grid(row=1, column=0)
        self._jog_pad(grid, 2, 0, "▼", "Z −", "Z_NEG", JZ_COLOR, HI_ROT)

        # Which arm's tool, and how fast. Both are read when a key goes
        # down, so neither changes a move already under way.
        opts = tk.Frame(left, bg=PANEL_BG)
        opts.pack(anchor="w", pady=(px(12), 0))
        self.xjog_arm_btn = RoundedButton(opts, text="", bg_color=SURFACE,
                                          fg_color=TEXT_LIGHT, width=150, height=32,
                                          command=self.toggle_xjog_arm)
        self.xjog_arm_btn.pack(side="left", padx=(0, px(14)))
        tk.Label(opts, text="Speed (mm/s)", bg=PANEL_BG, fg=TEXT_LIGHT,
                 font=FONT_MONO).pack(side="left", padx=(0, px(6)))
        self.xjog_speed_v = tk.StringVar(value=f"{DEFAULT_XYZ_JOG_MM_S:g}")
        entry_wrap, entry = make_inset_entry(opts, self.xjog_speed_v, width=6)
        entry_wrap.pack(side="left")
        # Jog keys are ignored while a text field has focus (W would type a
        # W), so ENTER hands the keyboard back. Clicking a pad does too.
        entry.bind("<Return>", lambda _e: self.root.focus_set())
        # Empty unless there is something to say: no speed limit was asked
        # for, so a fast one is warned about and still used.
        self.xjog_speed_warn_v = tk.StringVar(value="")
        tk.Label(left, textvariable=self.xjog_speed_warn_v, bg=PANEL_BG,
                 fg=ACCENT_ORANGE, font=FONT_HINT, justify="left",
                 wraplength=px(360)).pack(anchor="w", pady=(px(6), 0))
        self.xjog_speed_v.trace_add("write", lambda *_a: self._refresh_xjog_speed_warn())
        self._refresh_xjog_speed_warn()
        self.motion_lock_widgets += [self.xjog_arm_btn, self.xjog_home_btn]

    def _build_jog_view(self, right):
        """The 3D view, to the right of whichever pads are showing: ONE
        canvas for both layouts. It takes the width the pads leave and sets
        its own height from it, like the P2P board."""
        self._xyz_view_h = px(XYZ_VIEW_H)
        self.xyz_canvas = tk.Canvas(right, width=px(XYZ_VIEW_W), height=self._xyz_view_h,
                                    bg=LED_BG, highlightthickness=1,
                                    highlightbackground=BORDER_SOFT)
        self.xyz_canvas.pack(fill="x")
        self.xyz_canvas.bind("<Configure>", self._on_xyz_view_resize)
        # Drag to turn, wheel to zoom — a jog of a few mm is a few pixels at
        # the scale that fits the whole workspace. RESET VIEW is lit while
        # the picture is turned, because the pads above match the standard
        # one only.
        self._xyz_view = View()
        self.xyz_view_reset_btn = attach_view_controls(
            self.xyz_canvas, self._xyz_view, self._refresh_xyz_view,
            lambda: (*self._xyz_size(), px(XYZ_VIEW_PAD)))
        # CLEAR PATH, beside it: the trail only ever grew until the layout
        # or the arm changed. Same corner, same size, one button to the left.
        self.xyz_clear_btn = RoundedButton(
            self.xyz_canvas, text="CLEAR PATH", bg_color=SURFACE, fg_color=TEXT_MUTED,
            width=96, height=24, radius=8, font=FONT_CAPTION,
            command=self.clear_xyz_trail)
        self.xyz_clear_btn.place(relx=1.0, x=-(px(8) + px(96) + px(6)), y=px(8),
                                 anchor="ne")
        self.xyz_readout_v = tk.StringVar(value="")
        tk.Label(right, textvariable=self.xyz_readout_v, bg=PANEL_BG, fg=TEXT_LIGHT,
                 font=FONT_MONO, justify="left").pack(anchor="w", pady=(px(4), 0))
        tk.Label(right, text="Drag to turn the view · wheel to zoom", bg=PANEL_BG,
                 fg=TEXT_MUTED, font=FONT_HINT).pack(anchor="w", pady=(px(2), 0))

    def _refresh_xjog_speed_warn(self):
        """The line under the speed box. Follows the box as it is typed."""
        typed = self.xjog_speed_v.get().strip()
        speed = self._xjog_speed()
        if typed and f"{speed:g}" != f"{self._xjog_typed_number():g}":
            self.xjog_speed_warn_v.set(
                f"⚠ Not a speed — {speed:g} mm/s will be used.")
        elif speed > XYZ_JOG_WARN_MM_S:
            self.xjog_speed_warn_v.set(
                f"⚠ Above {XYZ_JOG_WARN_MM_S:g} mm/s. The board still holds each motor "
                f"to its own top speed, so the tool may not reach it — and it takes "
                f"up to half a second to stop after a key comes up.")
        else:
            self.xjog_speed_warn_v.set("")

    def _xjog_typed_number(self):
        try:
            return float(self.xjog_speed_v.get().strip().replace(",", "."))
        except ValueError:
            return float("nan")

    def _xyz_title(self, which):
        c = self._caps()
        if which == "Z":
            return f"Z  ({c.get('Z_POS', '?')}/{c.get('Z_NEG', '?')})"
        return (f"X ({c.get('X_NEG', '?')}/{c.get('X_POS', '?')})   "
                f"Y ({c.get('Y_POS', '?')}/{c.get('Y_NEG', '?')})")

    def _style_jog_frame_controls(self):
        for frame, btn in getattr(self, "jog_frame_buttons", {}).items():
            active = frame == self.jog_frame_mode
            btn.set_config(btn.text_str, ACCENT_MINT if active else SURFACE,
                           fg_color=INK_DARK if active else TEXT_LIGHT)
        if getattr(self, "xjog_arm_btn", None) is not None:
            self.xjog_arm_btn.set_config(f"TOOL: {self.xjog_arm}", SURFACE,
                                         fg_color=TEXT_LIGHT)

    def _show_jog_frame(self):
        """Packs the layout `jog_frame_mode` names, and only that one."""
        xyz = self.jog_frame_mode == "XYZ"
        shown, hidden = ((self.jog_xyz_frame, self.jog_joint_frame) if xyz
                         else (self.jog_joint_frame, self.jog_xyz_frame))
        hidden.pack_forget()
        shown.pack(anchor="w")
        self._style_jog_frame_controls()
        if getattr(self, "jog_hint_v", None) is not None:
            self.jog_hint_v.set(self._jog_hint_text())
        self._xyz_trail = {}            # a fresh look starts a fresh trail
        self._refresh_xyz_view()

    # ── the 3D view ──────────────────────────────────────────────────
    def _on_xyz_view_resize(self, event):
        """The view's column changed width: take the height that suits it,
        redraw. Setting the height fires <Configure> again at the same
        width, which asks for the same height and stops there — no loop."""
        want = int(min(max(event.width / VIEW_ASPECT, px(XYZ_VIEW_H)), px(XYZ_VIEW_MAX_H)))
        if want != self._xyz_view_h:
            self._xyz_view_h = want
            self.xyz_canvas.configure(height=want)
            return
        self._refresh_xyz_view()

    def _xyz_size(self):
        """(w, h) to draw into. Before the canvas is mapped Tk reports 1x1,
        and a picture scaled to that is a dot."""
        w, h = self.xyz_canvas.winfo_width(), self.xyz_canvas.winfo_height()
        if w <= 1 or h <= 1:
            return px(XYZ_VIEW_W), getattr(self, "_xyz_view_h", px(XYZ_VIEW_H))
        return w, h

    def clear_xyz_trail(self):
        """Wipes the path lines. The repaint starts a new trail from where
        the tool is now."""
        self._xyz_trail = {}
        self._refresh_xyz_view()

    def _refresh_xyz_view(self):
        c = getattr(self, "xyz_canvas", None)
        if c is None:
            return
        # WHAT IS DRAWN FOLLOWS THE LAYOUT, because what a key moves does.
        # XYZ: the one tool the keys drive, red, with its offset from HOME.
        # JOINT: every arm has its own keys, so BOTH tools, each in its
        # pads' colour and named.
        xyz = getattr(self, "jog_frame_mode", "JOINT") == "XYZ"
        arms = (((self.xjog_arm, ACCENT_RED),) if xyz
                else (("A1M", ARM_COLOR), ("A2M", ARM2_COLOR)))
        try:
            tools = [(arm, colour, self.tool_xyz(arm)) for arm, colour in arms]
        except (AttributeError, TypeError, ValueError):
            return
        w, h = self._xyz_size()
        P = draw_scene(c, w, h, px(XYZ_VIEW_PAD), view=getattr(self, "_xyz_view", None))

        # ---- where each tool has been, then the tools over the trails ----
        for arm, colour, point in tools:
            trail = self._xyz_trail.setdefault(arm, [])
            if not trail or math.dist(trail[-1], point) >= XYZ_TRAIL_STEP_MM:
                trail.append(point)
                del trail[:-XYZ_TRAIL_MAX]
            if len(trail) >= 2:
                flat = []
                for step in trail:
                    flat.extend(P(*step))
                c.create_line(*flat, fill=ACCENT_CYAN if xyz else colour, width=1)
        for i, (arm, colour, point) in enumerate(tools):
            draw_tool(c, P, *point, colour=colour, name="" if xyz else arm, below=bool(i))

        if xyz:
            x, y, z = tools[0][2]
            self.xyz_readout_v.set(
                f"{self.xjog_arm}   X {x:7.1f}   Y {y:7.1f}   Z {z:6.1f} mm\n"
                f"from HOME   ΔX {x - ARM_MIN_REACH_MM:+7.1f}   ΔY {y:+7.1f}   ΔZ {z:+6.1f} mm")
        else:
            self.xyz_readout_v.set("\n".join(
                f"{arm}   X {x:7.1f}   Y {y:7.1f}   Z {z:6.1f} mm"
                for arm, _colour, (x, y, z) in tools))
