"""The polar view of one layer, drawn on a plain Tk canvas.

No plotting library: one canvas, a few hundred dots, redrawn on a timer.
Pulling in matplotlib for this would be most of the install weight of the
whole tool.
"""

import math
import tkinter as tk

from .config import (ACCENT, BG, GRID, INK, MUTED, OK,
                     FONT_CAPTION, FONT_MONO,
                     PLOT_MIN_RANGE_MM, PLOT_RINGS)


class PolarPlot:
    """Draws the current layer, with the previous one ghosted behind it.

    The ghost is what makes a scan readable while it runs: a step in the
    wall between two heights shows up as the two outlines separating, and
    that is the thing a stacked scan is for.
    """

    def __init__(self, parent, size=380):
        self.size = size
        self.canvas = tk.Canvas(parent, width=size, height=size,
                                bg=BG, highlightthickness=0)
        self._range_mm = PLOT_MIN_RANGE_MM

    def pack(self, **kw):
        self.canvas.pack(**kw)
        return self.canvas

    # -- geometry ------------------------------------------------------
    @property
    def _centre(self):
        return self.size / 2.0, self.size / 2.0

    @property
    def _radius_px(self):
        return self.size / 2.0 - 28.0

    def _to_xy(self, deg, mm):
        """Polar to canvas. Screen Y grows downward, so the sine is
        negated -- without that the plot is mirrored about the X axis and
        a notch on the left appears on the right."""
        cx, cy = self._centre
        scale = self._radius_px / max(self._range_mm, 1.0)
        rad = math.radians(deg)
        return cx + mm * scale * math.cos(rad), cy - mm * scale * math.sin(rad)

    def set_range(self, mm):
        self._range_mm = max(float(mm), PLOT_MIN_RANGE_MM)

    @property
    def range_mm(self):
        return self._range_mm

    # -- drawing -------------------------------------------------------
    def redraw(self, points, ghost=(), title=""):
        c = self.canvas
        c.delete("all")
        cx, cy = self._centre
        rpx = self._radius_px

        for i in range(1, PLOT_RINGS + 1):
            r = rpx * i / PLOT_RINGS
            c.create_oval(cx - r, cy - r, cx + r, cy + r, outline=GRID)
            c.create_text(cx + 4, cy - r - 7,
                          text=f"{self._range_mm * i / PLOT_RINGS:.0f}",
                          fill=MUTED, font=FONT_CAPTION, anchor="w")

        # 0 deg points along +X, which is where the arm points at RM 0.
        for deg in range(0, 360, 45):
            x, y = self._to_xy(deg, self._range_mm)
            c.create_line(cx, cy, x, y, fill=GRID)
            lx, ly = self._to_xy(deg, self._range_mm * 1.08)
            c.create_text(lx, ly, text=f"{deg}", fill=MUTED, font=FONT_CAPTION)

        # The 20 degree wedge the turntable cannot sweep through. Drawn so
        # a gap there is read as "the machine cannot look here", not as a
        # sensor that failed.
        x1, y1 = self._to_xy(340, self._range_mm)
        c.create_line(cx, cy, x1, y1, fill="#3a3020")
        c.create_text(*self._to_xy(350, self._range_mm * 0.62),
                      text="no\nsweep", fill="#6a5a30",
                      font=FONT_CAPTION, justify="center")

        for deg, mm in ghost:
            x, y = self._to_xy(deg, mm)
            c.create_oval(x - 1, y - 1, x + 1, y + 1, fill=GRID, outline="")

        for deg, mm in points:
            x, y = self._to_xy(deg, mm)
            c.create_oval(x - 2, y - 2, x + 2, y + 2, fill=ACCENT, outline="")

        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill=OK, outline="")
        if title:
            c.create_text(cx, 12, text=title, fill=INK, font=FONT_MONO)
