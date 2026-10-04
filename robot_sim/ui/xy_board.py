"""P2P workspace board — the move, in 3D.

WHY A PICTURE AT ALL
--------------------
Three numbers per point, in a frame whose zero moved twice, is hard to
picture. A target "reachable" arithmetically can still be on far side of
machine from where operator meant; an A->B line crossing the unreachable
wedge is invisible in a coordinate table. The board answers "where is
that, how does it get there" at a glance.

WHY 3D
------
It was a top-down Oxy plot, and Z — a third of every point — was not on it
at all: two targets 200 mm apart in height drew as the same dot. The view
is ui/view3d.py's, the one the XYZ jog uses, so the two panels show the
same machine the same way round. (The file and the `_xy_*` names are the
plot's old ones; every caller still reaches it through them.)

WHAT IT DRAWS, and why each piece earns its ink
-----------------------------------------------
* The floor at HOME height: the reach ring, 133.2..613.2 mm. Inner hole
  real: a3+a6 is fixed structure, wafer centre never closer than 133.2 mm
  to the turntable axis. Operators reasonably expect 0,0 reachable (it's
  the reference) — it isn't.
* The unreachable WEDGE between RM 340 and 360, and the taught RM band —
  target refused by boundary vs by structure, distinguishable at a glance.
* A and B AT THEIR HEIGHTS, each with a drop line and a shadow on the
  floor so the height reads, the straight line between them, and HOME.
* The LIVE tool, from the shared pose — it moves during a run and a jog.

THE LINE IS A CHORD, NOT THE PATH
---------------------------------
A->B is drawn straight because it shows OPERATOR'S INTENT. The machine's
actual tool path is a joint-space move — RM sweeps an arc while the elbow
changes radius and the lift climbs — and it bulges away from that line.
It is drawn too, solid white, and the caption says which is which: checking
clearance between two points needs the swept path, not the chord.

IT FILLS ITS COLUMN
-------------------
The canvas takes the width the panel gives it and sets its own height
from the view's aspect, between a floor and a ceiling. It used to be a
fixed 560 px square, which on a scaled display was a small picture in a
large empty column. The SCALE still depends on the canvas alone, never on
what is plotted — moving a point does not reframe the picture.
"""

import tkinter as tk

from ..config import (
    ARM_HOME_DEG,
    ARM_MIN_REACH_MM,
    D1_MAX_MM,
    D1_MIN_MM,
    ROT_HOME_DEG,
    ROT_MAX_DEG,
    ROT_MIN_DEG,
    Z_HOME_MM,
)
from ..hidpi import px
from ..theme import (
    ACCENT_CYAN,
    ACCENT_GREEN,
    ACCENT_ORANGE,
    BORDER_SOFT,
    FONT_HINT,
    LED_BG,
    PANEL_BG,
    TEXT_DIM,
    TEXT_LIGHT,
    TEXT_MUTED,
)
from .view3d import (VIEW_ASPECT, View, attach_view_controls, draw_marker, draw_scene,
                     draw_tool)

#: Canvas size before DPI scaling. The WIDTH is only a minimum — the board
#: stretches to its column — and the height follows the width between these
#: two, so a very wide window does not push everything under it off screen.
BOARD_MIN_W = 360
BOARD_MIN_H = 480
BOARD_MAX_H = 520

#: Margin inside the canvas: room for the axis labels at the ring's edge.
BOARD_PAD = 26


class XYBoardMixin:
    def _build_xy_board(self, parent):
        """Board plus its caption. Returns containing frame."""
        wrap = tk.Frame(parent, bg=PANEL_BG)

        self._xy_board_h = px(BOARD_MIN_H)
        self.xy_canvas = tk.Canvas(wrap, width=px(BOARD_MIN_W), height=self._xy_board_h,
                                   bg=LED_BG, highlightthickness=1,
                                   highlightbackground=BORDER_SOFT)
        self.xy_canvas.pack(fill="x", pady=(4, 2))
        self.xy_canvas.bind("<Configure>", self._on_xy_board_resize)
        # Drag to turn, wheel to zoom, RESET VIEW to go back. A new View
        # every build: the board always opens on the standard picture.
        self._xy_view = View()
        self.xy_view_reset_btn = attach_view_controls(
            self.xy_canvas, self._xy_view, self._refresh_xy_board,
            lambda: (*self._xy_size(), px(BOARD_PAD)))

        self.xy_hint_v = tk.StringVar(value="")
        # Wrapped to the board's own width, and re-wrapped when that
        # changes: unwrapped, this one line is the widest thing in the
        # column and holds the whole panel wider than a narrow window.
        self._xy_hint_label = tk.Label(wrap, textvariable=self.xy_hint_v, bg=PANEL_BG,
                                       fg=TEXT_MUTED, font=FONT_HINT, justify="center",
                                       wraplength=px(BOARD_MIN_W))
        self._xy_hint_label.pack()

        self._refresh_xy_board()
        return wrap

    # ── sizing ───────────────────────────────────────────────────────
    def _on_xy_board_resize(self, event):
        """The column changed width: take the height that suits it, redraw.

        Setting the height fires <Configure> again at the same width, which
        asks for the same height and stops there — no loop.
        """
        label = getattr(self, "_xy_hint_label", None)
        if label is not None:
            label.configure(wraplength=max(event.width - px(8), px(BOARD_MIN_W)))
        want = int(min(max(event.width / VIEW_ASPECT, px(BOARD_MIN_H)), px(BOARD_MAX_H)))
        if want != self._xy_board_h:
            self._xy_board_h = want
            self.xy_canvas.configure(height=want)
            return
        self._refresh_xy_board()

    def _xy_size(self):
        """(w, h) to draw into. Before the canvas is mapped Tk reports 1x1,
        and a picture scaled to that is a dot."""
        w, h = self.xy_canvas.winfo_width(), self.xy_canvas.winfo_height()
        if w <= 1 or h <= 1:
            return px(BOARD_MIN_W), getattr(self, "_xy_board_h", px(BOARD_MIN_H))
        return w, h

    # ── drawing ──────────────────────────────────────────────────────
    def _refresh_xy_board(self):
        c = getattr(self, "xy_canvas", None)
        if c is None:
            return
        w, h = self._xy_size()
        P = draw_scene(c, w, h, px(BOARD_PAD), rot_band=self._xy_rot_band(),
                       view=getattr(self, "_xy_view", None))
        hx, hy = P(ARM_MIN_REACH_MM, 0.0, 0.0)

        # ---- the real joint-space path, drawn BEFORE the chord so the ----
        # ---- chord stays visually on top of it ----
        orbit = self._xy_orbit_points()
        if orbit:
            flat = []
            for point in orbit:
                flat.extend(P(*point))
            if len(flat) >= 4:
                # SOLID WHITE, asked for: it was a dim dashed line, and it is
                # the one line here that is where the tool really goes. The
                # chord over it stays the accent colour, and thicker.
                c.create_line(*flat, fill=TEXT_LIGHT, width=1)

        # ---- A, B at their heights, and the chord between them ----
        pts = self._xy_points()
        if pts is not None:
            (ax, ay), (bx, by) = pts
            za, zb = self._xy_heights()
            pax, pay = P(ax, ay, za)
            pbx, pby = P(bx, by, zb)
            # The cycle is HOME -> A -> B -> HOME, so the legs to and from
            # HOME are drawn too, faintly: they are real motion and they
            # sweep real space.
            c.create_line(hx, hy, pax, pay, fill=TEXT_DIM, width=1, dash=(2, 4))
            c.create_line(pbx, pby, hx, hy, fill=TEXT_DIM, width=1, dash=(2, 4))
            c.create_line(pax, pay, pbx, pby, fill=ACCENT_CYAN, width=2)
            draw_marker(c, P, ax, ay, za, "A", ACCENT_GREEN)
            draw_marker(c, P, bx, by, zb, "B", ACCENT_ORANGE)

        # ---- live tool position, from the shared pose ----
        live = self._xy_live_point()
        if live is not None:
            draw_tool(c, P, *live)

        # Short, and it still says the one thing the picture would
        # otherwise mislead about.
        self.xy_hint_v.set(
            "Floor rings are mm from the turntable axis, at HOME height · "
            "A→B is the commanded chord; the real joint-space path (white) "
            "bows away from it · drag to turn the view, wheel to zoom")

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

    def _xy_heights(self):
        """(za, zb) above HOME from the Z boxes, each held inside the lift's
        stroke. A Z that is half-typed, or out of range, draws on the FLOOR
        rather than not at all: the point's X/Y is still worth seeing, and
        the real-height line under the box is what says the Z is wrong."""
        out = []
        for name in ("z0_v", "z1_v"):
            try:
                z = float(getattr(self, name).get().strip().replace(",", "."))
            except (AttributeError, ValueError):
                z = D1_MIN_MM
            out.append(min(max(z, D1_MIN_MM), D1_MAX_MM))
        return tuple(out)

    def _xy_orbit_points(self):
        """HOME->A->B polyline in joint space as (x, y, z), or None on
        bad/unreachable input. This is the real swept path the caption
        promises — the chord is drawn from the same entry boxes but stays
        straight on purpose (operator intent, not the real tool path).

        NOT cached, deliberately. It looked like the obvious thing to cache
        — the board repaints on every telemetry line, 20 a second while an
        axis is held, and the orbit depends only on the entry boxes — but
        measured it costs 0.14 ms, so caching it saves 3 ms of every second
        and buys a staleness hazard in exchange. The repaint's real cost is
        the canvas drawing, not this.
        """
        pts = self._xy_points()
        if pts is None:
            return None
        (xa, ya), (xb, yb) = pts
        za, zb = self._xy_heights()
        from ..kinematics import (fold_angle_from_motor_deg, motor_deg_from_fold_angle,
                                  sample_joint_path, solve_ik, z_abs_from_home)
        arm = self.arm_config if getattr(self, "arm_config", None) in ("A1M", "A2M") else "A1M"
        try:
            idle_motor = self._idle_arm_angle()
        except AttributeError:
            idle_motor = None
        idle = None if idle_motor is None else fold_angle_from_motor_deg(idle_motor)
        try:
            d1a, rota, a1a, a2a = solve_ik(xa, ya, z_abs_from_home(za, arm), arm,
                                           idle_deg=idle)
            d1b, rotb, a1b, a2b = solve_ik(xb, yb, z_abs_from_home(zb, arm), arm,
                                           idle_deg=idle)
        except (ValueError, AttributeError, TypeError):
            return None
        home = (Z_HOME_MM, ROT_HOME_DEG, ARM_HOME_DEG, ARM_HOME_DEG)
        a_j = (d1a, rota, motor_deg_from_fold_angle(a1a), motor_deg_from_fold_angle(a2a))
        b_j = (d1b, rotb, motor_deg_from_fold_angle(a1b), motor_deg_from_fold_angle(a2b))
        return (sample_joint_path(home, a_j, arm=arm, with_z=True)
                + sample_joint_path(a_j, b_j, arm=arm, with_z=True))

    def _xy_live_point(self):
        """The tool's current (x, y, z) from the shared pose — Z above
        HOME, which is the lift's own travel."""
        from ..kinematics import fold_angle_from_motor_deg, forward_kinematics
        try:
            d1, rot, a1, a2 = self.current_joints
        except (AttributeError, TypeError, ValueError):
            return None
        arm = self.arm_config if self.arm_config in ("A1M", "A2M") else "A1M"
        x, y, _z = forward_kinematics(d1, rot,
                                      fold_angle_from_motor_deg(a1),
                                      fold_angle_from_motor_deg(a2), arm=arm)
        return x, y, d1
