"""Container w/ genuinely rounded, anti-aliased corners.

CANVAS BACKDROP, NOT FRAME: Tk frames are rectangles, no corner-radius
option, no `highlightthickness` fakes one — why section cards, readout
wells, tab strip all had hard 90-degree corners while buttons around
them were round.

Construction:

    RoundedFrame          plain Frame, coloured like PARENT so it
     ├─ Canvas            disappears into background
     │   (placed to fill) draws rounded backdrop, anti-aliased
     └─ body Frame        real container callers pack into

Body is rectangle, would poke through rounded corners if filled widget
exactly — inset instead. Geometry exact, not eyeballed: corner of
rectangle inset by `d` sits inside circle of radius `r` when

    d >= r * (1 - 1/sqrt(2))  ~=  0.293 * r

so `_corner_inset()` uses 0.3 * r, smallest padding provably enough. Any
less: square corners show. Any more: content floats away from own frame.
"""

import math
import tkinter as tk

from ..hidpi import px
from ..theme import PANEL_BG, RADIUS_CARD
from .draw import clear, paint_rounded

#: 1 - 1/sqrt(2): exact fraction of radius a square corner must be inset
#: by to fall inside the arc.
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

    `RoundedFrame` itself takes PARENT's colour so its own square edges
    vanish; only drawn backdrop is visible.
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

        # redraw on resize; bound to frame not canvas — canvas is `place`d,
        # doesn't generate own size events until frame has settled
        self.bind("<Configure>", self._redraw)

    def _redraw(self, _event=None):
        w, h = self.winfo_width(), self.winfo_height()
        if w <= 1 or h <= 1:
            return
        clear(self._canvas)
        paint_rounded(self._canvas, 0, 0, w, h, self.radius,
                      self.fill, self.surface,
                      border=self.border, border_w=self.border_w)

    def set_fill(self, colour):
        self.fill = colour
        self.body.config(bg=colour)
        self._redraw()

    def set_border(self, colour, width=None):
        """Recolours outline w/o touching fill — used for amber over-speed
        warning and validation errors."""
        self.border = colour
        if width is not None:
            self.border_w = width
        self._redraw()

    def configure_body(self, **kw):
        self.body.config(**kw)
