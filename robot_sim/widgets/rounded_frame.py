"""A container with genuinely rounded, anti-aliased corners.

WHY A CANVAS BACKDROP RATHER THAN A FRAME
-----------------------------------------
Tk frames are rectangles. There is no corner-radius option, and no amount
of `highlightthickness` fakes one — which is why the section cards, the
readout wells and the tab strip all still had hard 90-degree corners while
every button around them was round.

The construction here is:

    RoundedFrame          a plain Frame, coloured like its PARENT so it
     ├─ Canvas            disappears into the background
     │   (placed to fill) draws the rounded backdrop, anti-aliased
     └─ body Frame        the real container callers pack into

The body is a rectangle, so it would poke out through the rounded corners
if it filled the widget exactly. It is inset instead. The geometry is
exact rather than eyeballed: the corner of a rectangle inset by `d` sits
inside a circle of radius `r` when

    d >= r * (1 - 1/sqrt(2))  ~=  0.293 * r

so `_corner_inset()` uses 0.3 * r, the smallest padding that is provably
enough. Any less and the square corners show; any more and the content
floats away from its own frame.
"""

import math
import tkinter as tk

from ..hidpi import px
from ..theme import PANEL_BG, RADIUS_CARD
from .draw import clear, paint_rounded

#: 1 - 1/sqrt(2), the exact fraction of the radius a square corner must be
#: inset by to fall inside the arc.
_CORNER_K = 1.0 - 1.0 / math.sqrt(2.0)


def _corner_inset(radius):
    return max(2, int(math.ceil(radius * _CORNER_K)) + 1)


def _parent_bg(parent):
    try:
        return parent["bg"]
    except Exception:
        try:
            return parent.cget("background")
        except Exception:
            return PANEL_BG


class RoundedFrame(tk.Frame):
    """Rounded container. Pack/grid children into `.body`.

    `RoundedFrame` itself takes the PARENT's colour so its own square
    edges vanish; only the drawn backdrop is visible.
    """

    def __init__(self, parent, bg=PANEL_BG, radius=RADIUS_CARD, border=None,
                 border_w=1, padx=0, pady=0, **kw):
        surface = _parent_bg(parent)
        super().__init__(parent, bg=surface, highlightthickness=0, bd=0, **kw)
        self.radius = px(radius)
        self.fill = bg
        self.border = border
        self.border_w = border_w
        self.surface = surface

        self._canvas = tk.Canvas(self, bg=surface, highlightthickness=0, bd=0)
        self._canvas.place(x=0, y=0, relwidth=1, relheight=1)

        inset = _corner_inset(self.radius)
        self.body = tk.Frame(self, bg=bg, highlightthickness=0, bd=0)
        self.body.pack(fill="both", expand=True,
                       padx=inset + px(padx), pady=inset + px(pady))

        # Redraw on resize. Bound to the frame rather than the canvas
        # because the canvas is `place`d and does not generate its own
        # size events until after the frame has settled.
        self.bind("<Configure>", self._redraw)

    def _redraw(self, _event=None):
        w, h = self.winfo_width(), self.winfo_height()
        if w <= 1 or h <= 1:
            return
        clear(self._canvas)
        paint_rounded(self._canvas, 0, 0, w, h, self.radius,
                      self.fill, self.surface,
                      border=self.border, border_w=self.border_w)

    # ── public API ───────────────────────────────────────────────────
    def set_fill(self, colour):
        self.fill = colour
        self.body.config(bg=colour)
        self._redraw()

    def set_border(self, colour, width=None):
        """Recolours the outline without touching the fill — used for the
        amber over-speed warning and for validation errors."""
        self.border = colour
        if width is not None:
            self.border_w = width
        self._redraw()

    def configure_body(self, **kw):
        self.body.config(**kw)
