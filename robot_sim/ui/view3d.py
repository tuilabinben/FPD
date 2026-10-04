"""The 3D workspace view — one projection and one scene, drawn by both the
P2P board and the XYZ jog.

An axonometric projection, no perspective, and the scale depends on the
canvas and the camera alone, never on what is plotted. A picture that
reframes itself as a point moves cannot be compared between two looks.
In the STANDARD view +X runs right and toward the viewer, +Y right and
away, +Z up.

THE CAMERA MOVES, BUT ONLY WHEN THE OPERATOR MOVES IT — asked for. Drag
turns the view, the wheel zooms about the pointer (View, and
attach_view_controls() below). A fixed angle always hides something: from
straight above this is the old Oxy plot, from the side it is the heights.
Every view OPENS on the standard one and nothing is saved, so the default
picture is still the comparable one, and RESET VIEW goes back to it. That
button is lit while the view is not standard, because the XYZ jog's pads
are laid out to match the standard picture — turned half round, the +X
pad moves the tool LEFT on screen.

The frame is the P2P one: X, Y from the turntable axis, Z above HOME. So
the floor drawn here is the plane at HOME height, and HOME itself is the
point (133.2, 0, 0) — the tool with the arm retracted and the lift down.

Everything takes the canvas size as an argument rather than reading a
constant, so the same code draws a fixed-size view and one that follows
its window.
"""

import math

from ..config import (
    ARM_MAX_REACH_MM,
    ARM_MIN_REACH_MM,
    D1_MAX_MM,
    ROT_MAX_DEG,
    ROT_MIN_DEG,
)
from ..hidpi import px
from ..theme import (
    ACCENT_MINT,
    ACCENT_ORANGE,
    ACCENT_RED,
    AXIS_X_COLOR,
    AXIS_Y_COLOR,
    AXIS_Z_COLOR,
    BORDER_SOFT,
    FONT_CAPTION,
    FONT_HINT,
    INK_DARK,
    SURFACE,
    TEXT_DIM,
    TEXT_LIGHT,
    TEXT_MUTED,
)
from ..widgets import RoundedButton

#: The STANDARD camera: what every view opens on and RESET VIEW goes back
#: to. AZ turns the world about Z, EL tilts it toward the viewer.
VIEW_AZ_DEG = -25.0
VIEW_EL_DEG = 33.0

#: The mouse. Degrees turned per pixel dragged, the zoom one wheel notch
#: gives, and how far each may go. EL stops short of 0 — side-on, the floor
#: is a line with nothing readable on it — and at 90, straight down.
VIEW_DRAG_DEG_PER_PX = 0.4
VIEW_ZOOM_STEP = 1.2
VIEW_ZOOM_MIN, VIEW_ZOOM_MAX = 0.5, 40.0
VIEW_EL_MIN_DEG, VIEW_EL_MAX_DEG = 5.0, 90.0

#: Floor rings, mm from the turntable axis, so a distance can be read off.
#: Anything outside the reachable annulus is skipped.
VIEW_RINGS_MM = (200.0, 300.0, 400.0, 500.0)

#: Samples round a floor ring. A projected circle is an ellipse Tk cannot
#: draw tilted, so it is a polyline.
_RING_STEPS = 72


def _extent(el_deg=VIEW_EL_DEG):
    """(below, above) the origin on screen, in mm: the ring's near edge
    below it, its far edge plus the whole lift stroke above."""
    el = math.radians(el_deg)
    below = ARM_MAX_REACH_MM * math.sin(el)
    return below, below + D1_MAX_MM * math.cos(el)


#: Width : height of everything the STANDARD view can ever show. A canvas of
#: this shape is filled edge to edge; any other shape along one side.
VIEW_ASPECT = 2.0 * ARM_MAX_REACH_MM / sum(_extent())


def _frame(w, h, pad, el_deg):
    """(mm -> px scale, origin x, origin y) at zoom 1 for a camera tilted
    `el_deg`: the whole reach ring at every lift height fits, centred in
    whatever room is left. It follows the TILT — from straight above the
    ring is as tall as it is wide — and never the azimuth or the plot."""
    below, above = _extent(el_deg)
    k = min((w / 2.0 - pad) / ARM_MAX_REACH_MM, (h - 2.0 * pad) / (above + below))
    return k, w / 2.0, (h - k * (above + below)) / 2.0 + k * above


class View:
    """One canvas's camera: where it looks from, how far in, and where the
    zoom has moved the picture to. Starts on the standard view and is never
    saved — a fresh window is the comparable picture."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.az, self.el = VIEW_AZ_DEG, VIEW_EL_DEG
        self.zoom, self.ox, self.oy = 1.0, 0.0, 0.0

    @property
    def standard(self):
        return ((self.az, self.el, self.zoom, self.ox, self.oy)
                == (VIEW_AZ_DEG, VIEW_EL_DEG, 1.0, 0.0, 0.0))

    def orbit(self, dx, dy):
        """A drag of (dx, dy) px. The near side of the floor follows the
        pointer: right turns it right, down tips the view toward overhead."""
        self.az = (self.az + dx * VIEW_DRAG_DEG_PER_PX + 180.0) % 360.0 - 180.0
        self.el = min(max(self.el + dy * VIEW_DRAG_DEG_PER_PX, VIEW_EL_MIN_DEG),
                      VIEW_EL_MAX_DEG)

    def zoom_at(self, factor, mx, my, w, h, pad):
        """Zooms by `factor`, keeping whatever is under (mx, my) under it."""
        new = min(max(self.zoom * factor, VIEW_ZOOM_MIN), VIEW_ZOOM_MAX)
        f = new / self.zoom
        _k, cx, cy = _frame(w, h, pad, self.el)
        self.ox = mx - cx - f * (mx - cx - self.ox)
        self.oy = my - cy - f * (my - cy - self.oy)
        self.zoom = new


def xyz_project(x, y, z, w, h, pad, view=None):
    """World mm -> canvas px, for a canvas w x h. Pure. `view` is the
    camera; None is the standard one."""
    az_deg, el_deg, zoom, ox, oy = ((VIEW_AZ_DEG, VIEW_EL_DEG, 1.0, 0.0, 0.0)
                                    if view is None else
                                    (view.az, view.el, view.zoom, view.ox, view.oy))
    az, el = math.radians(az_deg), math.radians(el_deg)
    k, cx, cy = _frame(w, h, pad, el_deg)
    k *= zoom
    xr = x * math.cos(az) - y * math.sin(az)
    yr = x * math.sin(az) + y * math.cos(az)
    return cx + ox + k * xr, cy + oy - k * (yr * math.sin(el) + z * math.cos(el))


def attach_view_controls(canvas, view, redraw, size):
    """Makes `canvas`'s view movable: drag turns it, the wheel zooms about
    the pointer, and a RESET VIEW button in its corner puts the standard
    view back. `redraw()` repaints; `size()` is the (w, h, pad) it draws
    into. Returns the button.

    One helper for both views, so they cannot come to behave differently.
    """
    last = [0, 0]

    def show():
        redraw()
        # LIT while the view is not the standard one. On the XYZ jog the
        # pads are laid out to match the standard picture; this is what
        # says the picture no longer matches them.
        btn.set_config("RESET VIEW", SURFACE if view.standard else ACCENT_ORANGE,
                       fg_color=TEXT_MUTED if view.standard else INK_DARK)

    def press(event):
        last[:] = event.x, event.y

    def drag(event):
        view.orbit(event.x - last[0], event.y - last[1])
        last[:] = event.x, event.y
        show()

    def wheel(event):
        view.zoom_at(VIEW_ZOOM_STEP ** (event.delta / 120.0), event.x, event.y, *size())
        show()
        return "break"          # everywhere else the wheel scrolls the page

    def reset():
        view.reset()
        show()

    # A child of the canvas, so its corners take the canvas's colour; placed,
    # not a canvas item, because every redraw deletes those.
    btn = RoundedButton(canvas, text="RESET VIEW", bg_color=SURFACE, fg_color=TEXT_MUTED,
                        width=96, height=24, radius=8, font=FONT_CAPTION, command=reset)
    btn.place(relx=1.0, x=-px(8), y=px(8), anchor="ne")
    canvas.configure(cursor="fleur")
    canvas.bind("<ButtonPress-1>", press)
    canvas.bind("<B1-Motion>", drag)
    canvas.bind("<MouseWheel>", wheel)
    return btn


def draw_scene(c, w, h, pad, rot_band=None, view=None):
    """Clears `c` and draws what never moves: the floor at HOME height (the
    reach ring, the inner HOME-reach ring, distance rings, the wedge RM
    cannot sweep), the three axes and HOME. `rot_band` (lo, hi) adds the
    taught RM boundaries when they are narrower than the travel.

    Returns the projector P(x, y, z) -> (px, py) for whatever is drawn next.
    `view` is the camera (None: the standard one).
    """
    def P(wx, wy, wz):
        return xyz_project(wx, wy, wz, w, h, pad, view)

    def ring(radius, lo=0.0, hi=360.0):
        pts = []
        for i in range(_RING_STEPS + 1):
            a = math.radians(lo + (hi - lo) * i / _RING_STEPS)
            pts.extend(P(radius * math.cos(a), radius * math.sin(a), 0.0))
        return pts

    c.delete("all")
    r_out, r_home = ARM_MAX_REACH_MM, ARM_MIN_REACH_MM

    # ---- the floor ----
    c.create_line(*ring(r_out), fill=BORDER_SOFT, width=1)
    c.create_line(*ring(r_home), fill=BORDER_SOFT, width=1, dash=(2, 3))
    for radius in VIEW_RINGS_MM:
        if not (r_home < radius < r_out):
            continue
        c.create_line(*ring(radius), fill=BORDER_SOFT, width=1, dash=(1, 5))
        lx, ly = P(-radius, 0.0, 0.0)
        c.create_text(lx, ly - px(7), text=f"{radius:.0f}", anchor="center",
                      fill=TEXT_MUTED, font=FONT_HINT)
    # The wedge RM cannot sweep through, from its CW stop round to 0.
    c.create_line(*ring(r_out, ROT_MAX_DEG, 360.0), fill=ACCENT_RED, width=2)
    edge = math.radians(ROT_MAX_DEG)
    c.create_line(*P(0, 0, 0), *P(r_out * math.cos(edge), r_out * math.sin(edge), 0),
                  fill=ACCENT_RED, width=1, dash=(3, 3))

    # ---- the taught RM band, when it is narrower than the travel ----
    if rot_band is not None:
        lo, hi = rot_band
        if lo > ROT_MIN_DEG + 0.01 or hi < ROT_MAX_DEG - 0.01:
            for limit in (lo, hi):
                a = math.radians(limit)
                c.create_line(*P(0, 0, 0), *P(r_out * math.cos(a), r_out * math.sin(a), 0),
                              fill=ACCENT_ORANGE, width=1, dash=(3, 3))

    # ---- the axes, from the turntable axis ----
    c.create_line(*P(-r_out, 0, 0), *P(0, 0, 0), fill=TEXT_DIM, width=1, dash=(1, 4))
    c.create_line(*P(0, -r_out, 0), *P(0, 0, 0), fill=TEXT_DIM, width=1, dash=(1, 4))
    for end, colour, label in (((r_out, 0, 0), AXIS_X_COLOR, f"+X  {r_out:.0f}"),
                               ((0, r_out, 0), AXIS_Y_COLOR, "+Y"),
                               ((0, 0, D1_MAX_MM), AXIS_Z_COLOR, f"+Z  {D1_MAX_MM:.0f}")):
        ex, ey = P(*end)
        c.create_line(*P(0, 0, 0), ex, ey, fill=colour, width=1)
        c.create_text(ex + px(6), ey - px(6), text=label, anchor="w",
                      fill=colour, font=FONT_HINT)

    # ---- HOME: the tool's own position at the reference ----
    hx, hy = P(r_home, 0, 0)
    d = px(4)
    c.create_oval(hx - d, hy - d, hx + d, hy + d, outline=ACCENT_MINT, width=2)
    c.create_text(hx + px(7), hy + px(9), text="HOME", anchor="w",
                  fill=ACCENT_MINT, font=FONT_HINT)
    return P


def draw_drop(c, P, x, y, z, colour=AXIS_Z_COLOR):
    """A point's height made readable: its shadow on the floor and the line
    up to it. Returns the point's canvas position."""
    sx, sy = P(x, y, 0.0)
    tx, ty = P(x, y, z)
    c.create_line(sx, sy, tx, ty, fill=colour, width=1, dash=(2, 3))
    d = px(3)
    c.create_oval(sx - d, sy - d, sx + d, sy + d, outline=TEXT_DIM, width=1)
    return tx, ty


def draw_marker(c, P, x, y, z, label, colour):
    """A named target — A, B — standing at its height above the floor."""
    tx, ty = draw_drop(c, P, x, y, z, colour)
    d = px(4)
    c.create_oval(tx - d, ty - d, tx + d, ty + d, fill=colour, outline=colour)
    c.create_text(tx + px(8), ty - px(9), text=label, anchor="w",
                  fill=colour, font=FONT_CAPTION)
    return tx, ty


def draw_tool(c, P, x, y, z, colour=None, name="", below=False):
    """The live tool: shadow, drop line, the arm reaching out from the lift
    column at its height, and where it is in numbers.

    `colour` and `name` tell two tools apart when both arms are drawn (the
    JOINT jog); `below` hangs the numbers under the marker, so two tools on
    one spot — both arms at HOME — do not print over each other. `colour`
    defaults to None, not ACCENT_RED: a default is bound once, and would
    keep the old red through a theme switch."""
    colour = colour or ACCENT_RED
    sx, sy = P(x, y, 0.0)
    c.create_line(*P(0, 0, 0), sx, sy, fill=TEXT_DIM, width=1, dash=(2, 3))
    tx, ty = draw_drop(c, P, x, y, z)
    c.create_line(*P(0, 0, z), tx, ty, fill=colour, width=2)
    d = px(5)
    c.create_oval(tx - d, ty - d, tx + d, ty + d, fill=colour, outline=colour)
    c.create_text(tx + px(9), ty + (px(9) if below else -px(9)), anchor="w",
                  fill=TEXT_LIGHT, font=FONT_HINT,
                  text=(name + "  " if name else "") + f"{x:.0f}, {y:.0f}, {z:.0f}")
    return tx, ty
