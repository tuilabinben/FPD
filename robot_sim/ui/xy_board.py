"""The Oxy workspace board — a top-down plot of the P2P move.

WHY A PLOT AT ALL
-----------------
Four numbers per point, in a frame whose zero moved twice, is hard to
picture. A target that is "reachable" arithmetically can still be on the
far side of the machine from where the operator meant, and the A -> B line
crossing the unreachable wedge is invisible in a table of coordinates.
The board answers "where is that, and how does it get there" at a glance.

WHAT IT DRAWS, and why each piece earns its ink
-----------------------------------------------
* The reachable ANNULUS, 133.2..613.2 mm. The inner hole is real: a3 + a6
  is fixed structure, so the wafer centre can never come closer than
  133.2 mm to the turntable axis. Operators reasonably expect 0,0 to be
  reachable — it is the reference — and it is not.
* The unreachable WEDGE between RM 340 and 360. RM sweeps 340 deg from its
  CCW stop; the remaining 20 deg is the gap it cannot pass through, from
  either side.
* The taught RM band, so a target refused by a boundary rather than by the
  structure is distinguishable at a glance.
* A and B, the straight line between them, and HOME.
* The LIVE tool position, which is the shared pose, so the dot moves during
  a run and during a jog.

THE LINE IS A CHORD, NOT THE PATH
---------------------------------
The A -> B line is drawn straight because it shows the OPERATOR'S INTENT.
The machine's actual tool path is a joint-space move — RM sweeps an arc
while the elbow changes radius — so the tool bulges away from this line.
That is stated on the board rather than hidden, because someone checking
clearance between two points needs to know the swept path is not the chord.
See the P2P section of CLAUDE.md: a true straight-line tool path needs
on-board interpolation.

Geometry note: screen Y grows downwards, so plot Y is negated. RM 0 points
along +X, which is drawn to the right.
"""

import math
import tkinter as tk

from ..config import (
    ARM_HOME_DEG,
    ARM_MAX_REACH_MM,
    ARM_MIN_REACH_MM,
    ROT_HOME_DEG,
    ROT_MAX_DEG,
    ROT_MIN_DEG,
    Z_HOME_MM,
)
from ..theme import (
    ACCENT_CYAN,
    ACCENT_GREEN,
    ACCENT_MINT,
    ACCENT_ORANGE,
    ACCENT_RED,
    BORDER_SOFT,
    FONT_CAPTION,
    FONT_HINT,
    LED_BG,
    PANEL_BG,
    TEXT_DIM,
    TEXT_MUTED,
)

#: Canvas size. Square, because a round workspace in a wide box wastes the
#: width and makes equal radii look unequal — the plot is CENTRED in its
#: own column instead of being stretched.
#:
#: 560: the left column's coordinate cards are now pinned to their
#: compact natural width (see p2p_panel.py's grid-based split and
#: make_coord_card's compact=True), which freed up enough room to grow
#: the board again without crowding it — was 480, before that 380 (the
#: 133.2 mm inner circle was only 24 px across at 380 and the A/B labels
#: collided with HOME).
BOARD_PX = 560

#: Margin inside the canvas, in pixels. Holds the axis labels and the radius
#: ring captions, which need more room than the axis names alone. Scaled
#: with BOARD_PX so the margin stays proportional.
BOARD_PAD = 38

#: Radius rings, in mm, drawn and labelled so the plot can be read as a
#: measurement rather than only as a shape. Anything outside the reachable
#: annulus is skipped.
BOARD_RINGS_MM = (200.0, 300.0, 400.0, 500.0, 600.0)


class XYBoardMixin:
    def _build_xy_board(self, parent):
        """The board plus its legend. Returns the containing frame."""
        wrap = tk.Frame(parent, bg=PANEL_BG)

        # Centred in its row rather than stretched: a circle in a wide box
        # would either distort or leave the radii unequal on screen.
        self.xy_canvas = tk.Canvas(wrap, width=BOARD_PX, height=BOARD_PX,
                                   bg=LED_BG, highlightthickness=1,
                                   highlightbackground=BORDER_SOFT)
        self.xy_canvas.pack(pady=(4, 2))

        self.xy_hint_v = tk.StringVar(value="")
        # The caption is one line at this width. It used to wrap to the
        # narrow side column and ran to six lines, which pushed the board
        # further out of view.
        tk.Label(wrap, textvariable=self.xy_hint_v, bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_HINT, justify="center").pack()

        self._refresh_xy_board()
        return wrap

    # ── scaling ──────────────────────────────────────────────────────
    def _xy_scale(self):
        """(centre_x, centre_y, px per mm). The full outer reach always fits,
        so the picture never changes scale as points move — a plot that
        rescales itself makes two runs impossible to compare by eye."""
        half = BOARD_PX / 2.0
        return half, half, (half - BOARD_PAD) / ARM_MAX_REACH_MM

    def _xy_to_px(self, x_mm, y_mm):
        cx, cy, k = self._xy_scale()
        return cx + x_mm * k, cy - y_mm * k      # screen Y grows downwards

    # ── drawing ──────────────────────────────────────────────────────
    def _refresh_xy_board(self):
        c = getattr(self, "xy_canvas", None)
        if c is None:
            return
        c.delete("all")
        cx, cy, k = self._xy_scale()
        r_out = ARM_MAX_REACH_MM * k
        r_in = ARM_MIN_REACH_MM * k

        # ---- reachable annulus ----
        c.create_oval(cx - r_out, cy - r_out, cx + r_out, cy + r_out,
                      outline=BORDER_SOFT, width=1)
        c.create_oval(cx - r_in, cy - r_in, cx + r_in, cy + r_in,
                      outline=BORDER_SOFT, width=1, dash=(2, 3))

        # ---- the wedge RM cannot sweep through ----
        # Tk arcs measure degrees counter-clockwise from +X, which is the
        # same convention as RM, so no conversion is needed.
        dead_span = 360.0 - (ROT_MAX_DEG - ROT_MIN_DEG)
        if dead_span > 0.01:
            c.create_arc(cx - r_out, cy - r_out, cx + r_out, cy + r_out,
                         start=ROT_MAX_DEG, extent=dead_span,
                         fill=PANEL_BG, outline=ACCENT_RED, width=1,
                         stipple="gray25")

        # ---- the taught RM band, if narrower than the travel ----
        lo, hi = self._xy_rot_band()
        if lo > ROT_MIN_DEG + 0.01 or hi < ROT_MAX_DEG - 0.01:
            for edge in (lo, hi):
                ex, ey = self._xy_to_px(ARM_MAX_REACH_MM * math.cos(math.radians(edge)),
                                        ARM_MAX_REACH_MM * math.sin(math.radians(edge)))
                c.create_line(cx, cy, ex, ey, fill=ACCENT_ORANGE, width=1, dash=(3, 3))

        # ---- radius rings, so the plot reads as a measurement ----
        for ring in BOARD_RINGS_MM:
            if not (ARM_MIN_REACH_MM < ring < ARM_MAX_REACH_MM):
                continue
            rr = ring * k
            c.create_oval(cx - rr, cy - rr, cx + rr, cy + rr,
                          outline=BORDER_SOFT, width=1, dash=(1, 5))
            c.create_text(cx + 2, cy - rr - 6, text=f"{ring:.0f}", anchor="w",
                          fill=TEXT_MUTED, font=FONT_HINT)

        # ---- axes, labelled in the frame the operator types in ----
        c.create_line(BOARD_PAD, cy, BOARD_PX - BOARD_PAD, cy,
                      fill=TEXT_DIM, width=1)
        c.create_line(cx, BOARD_PAD, cx, BOARD_PX - BOARD_PAD,
                      fill=TEXT_DIM, width=1)
        c.create_text(BOARD_PX - BOARD_PAD + 2, cy - 7, text="+X", anchor="e",
                      fill=TEXT_DIM, font=FONT_HINT)
        c.create_text(cx + 12, BOARD_PAD, text="+Y", anchor="w",
                      fill=TEXT_DIM, font=FONT_HINT)

        # RM 0 is +X and is where HOME points. Marked because the whole
        # frame hangs off it.
        c.create_text(BOARD_PX - BOARD_PAD + 2, cy + 8, text="RM 0°", anchor="e",
                      fill=ACCENT_MINT, font=FONT_HINT)

        # ---- HOME: the reference, at radius 133.2 along +X ----
        hx, hy = self._xy_to_px(ARM_MIN_REACH_MM, 0.0)
        c.create_oval(hx - 3, hy - 3, hx + 3, hy + 3,
                      outline=ACCENT_MINT, width=1)
        c.create_text(hx + 6, hy - 7, text="HOME", anchor="w",
                      fill=ACCENT_MINT, font=FONT_HINT)

        # ---- the real joint-space orbit, drawn BEFORE the chord so the ----
        # ---- chord stays visually on top of it ----
        orbit = self._xy_orbit_points()
        if orbit:
            flat = []
            for ox, oy in orbit:
                flat.extend(self._xy_to_px(ox, oy))
            if len(flat) >= 4:
                c.create_line(*flat, fill=TEXT_DIM, width=1, dash=(4, 2), smooth=True)

        # ---- A, B and the chord between them ----
        pts = self._xy_points()
        if pts is not None:
            (ax, ay), (bx, by) = pts
            pax, pay = self._xy_to_px(ax, ay)
            pbx, pby = self._xy_to_px(bx, by)
            # The cycle is HOME -> A -> B -> HOME, so the legs to and from
            # HOME are drawn too, faintly: they are real motion and they
            # sweep real space.
            c.create_line(hx, hy, pax, pay, fill=TEXT_DIM, width=1, dash=(2, 4))
            c.create_line(pbx, pby, hx, hy, fill=TEXT_DIM, width=1, dash=(2, 4))
            c.create_line(pax, pay, pbx, pby, fill=ACCENT_CYAN, width=2)
            for px, py, label, colour in ((pax, pay, "A", ACCENT_GREEN),
                                          (pbx, pby, "B", ACCENT_ORANGE)):
                c.create_oval(px - 4, py - 4, px + 4, py + 4,
                              fill=colour, outline=colour)
                c.create_text(px + 7, py - 8, text=label, anchor="w",
                              fill=colour, font=FONT_CAPTION)

        # ---- live tool position, from the shared pose ----
        live = self._xy_live_point()
        if live is not None:
            lx, ly = self._xy_to_px(*live)
            c.create_oval(lx - 5, ly - 5, lx + 5, ly + 5,
                          outline=ACCENT_RED, width=2)
            c.create_line(cx, cy, lx, ly, fill=ACCENT_RED, width=1)

        # Short enough for one line at this width, and it still says the one
        # thing the picture would otherwise mislead about.
        self.xy_hint_v.set(
            f"Rings are mm from the turntable axis · reach "
            f"{ARM_MIN_REACH_MM:.0f}–{ARM_MAX_REACH_MM:.0f} mm · "
            f"A→B is the commanded chord; the real joint-space path bows "
            f"away from it")

    # ── data the board needs, each guarded so a bad entry never raises ──
    def _xy_rot_band(self):
        try:
            return self._limit_pair("rot")
        except Exception:
            return ROT_MIN_DEG, ROT_MAX_DEG

    def _xy_points(self):
        """((xa, ya), (xb, yb)) from the entry boxes, or None while either
        is not yet a number. The board must survive half-typed input."""
        try:
            xa = float(self.x0_v.get().strip().replace(",", "."))
            ya = float(self.y0_v.get().strip().replace(",", "."))
            xb = float(self.x1_v.get().strip().replace(",", "."))
            yb = float(self.y1_v.get().strip().replace(",", "."))
        except (AttributeError, ValueError):
            return None
        return (xa, ya), (xb, yb)

    def _xy_orbit_points(self):
        """HOME->A->B polyline in joint space, or None on bad/unreachable
        input. This is the real swept path the caption already promises —
        the chord above is drawn from the same entry boxes but stays
        straight on purpose (operator intent, not the real tool path)."""
        pts = self._xy_points()
        if pts is None:
            return None
        (xa, ya), (xb, yb) = pts
        from ..kinematics import (fold_angle_from_motor_deg, motor_deg_from_fold_angle,
                                  sample_joint_path, solve_ik, z_offset_for)
        arm = self.arm_config if getattr(self, "arm_config", None) in ("A1M", "A2M") else "A1M"
        # Z doesn't matter for this top-down (x, y) plot -- forward_kinematics's
        # x/y are independent of d1 -- so it is synthesized as z_offset_for(arm)
        # (giving d1=0, always in-range) rather than read from the Z entry
        # boxes. That keeps this plot independent of the Z-stroke validation
        # _refresh_real_heights() already owns.
        z = z_offset_for(arm)
        try:
            idle_motor = self._idle_arm_angle()
        except AttributeError:
            idle_motor = None
        idle = None if idle_motor is None else fold_angle_from_motor_deg(idle_motor)
        try:
            d1a, rota, a1a, a2a = solve_ik(xa, ya, z, arm, idle_deg=idle)
            d1b, rotb, a1b, a2b = solve_ik(xb, yb, z, arm, idle_deg=idle)
        except (ValueError, AttributeError, TypeError):
            return None
        home = (Z_HOME_MM, ROT_HOME_DEG, ARM_HOME_DEG, ARM_HOME_DEG)
        a_j = (d1a, rota, motor_deg_from_fold_angle(a1a), motor_deg_from_fold_angle(a2a))
        b_j = (d1b, rotb, motor_deg_from_fold_angle(a1b), motor_deg_from_fold_angle(a2b))
        return (sample_joint_path(home, a_j, arm=arm)
                + sample_joint_path(a_j, b_j, arm=arm))

    def _xy_live_point(self):
        """The tool's current (x, y) from the shared pose."""
        from ..kinematics import fold_angle_from_motor_deg, forward_kinematics
        try:
            d1, rot, a1, a2 = self.current_joints
        except (AttributeError, TypeError, ValueError):
            return None
        arm = self.arm_config if self.arm_config in ("A1M", "A2M") else "A1M"
        x, y, _z = forward_kinematics(d1, rot,
                                      fold_angle_from_motor_deg(a1),
                                      fold_angle_from_motor_deg(a2), arm=arm)
        return x, y
