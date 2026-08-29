"""Polar view of one scan layer, plain Tk canvas.

No plotting library: a few hundred dots, redrawn on a timer. Same drawing
as the stand-alone Scan/ tool, theme colours read at construction.
"""

import math
import tkinter as tk

from ..config import SCAN_PLOT_MIN_RANGE_MM, SCAN_PLOT_RINGS, SCAN_PLOT_SIZE
from ..theme import (ACCENT_MINT, ACCENT_GREEN, ACCENT_ORANGE, BORDER,
                     FONT_CAPTION, FONT_MONO, LED_BG, TEXT_LIGHT, TEXT_MUTED)


class ScanPolarPlot:
    """Current layer, previous one ghosted. The ghost is what makes a step
    in the wall readable while the scan runs."""

    def __init__(self, parent, size=SCAN_PLOT_SIZE):
        self.size = size
        self.canvas = tk.Canvas(parent, width=size, height=size,
                                bg=LED_BG, highlightthickness=0)
        self._range_mm = SCAN_PLOT_MIN_RANGE_MM

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
        """Polar to canvas. Screen Y grows down, so negate the sine — else
        a notch on the left draws on the right."""
        cx, cy = self._centre
        scale = self._radius_px / max(self._range_mm, 1.0)
        rad = math.radians(deg)
        return cx + mm * scale * math.cos(rad), cy - mm * scale * math.sin(rad)

    def set_range(self, mm):
        self._range_mm = max(float(mm), SCAN_PLOT_MIN_RANGE_MM)

    @property
    def range_mm(self):
        return self._range_mm

    # -- drawing -------------------------------------------------------
    def redraw(self, points, ghost=(), title=""):
        c = self.canvas
        c.delete("all")
        cx, cy = self._centre
        rpx = self._radius_px

        for i in range(1, SCAN_PLOT_RINGS + 1):
            r = rpx * i / SCAN_PLOT_RINGS
            c.create_oval(cx - r, cy - r, cx + r, cy + r, outline=BORDER)
            c.create_text(cx + 4, cy - r - 7,
                          text=f"{self._range_mm * i / SCAN_PLOT_RINGS:.0f}",
                          fill=TEXT_MUTED, font=FONT_CAPTION, anchor="w")

        # 0 deg points along +X, which is where the arm points at RM 0.
        for deg in range(0, 360, 45):
            x, y = self._to_xy(deg, self._range_mm)
            c.create_line(cx, cy, x, y, fill=BORDER)
            lx, ly = self._to_xy(deg, self._range_mm * 1.08)
            c.create_text(lx, ly, text=f"{deg}", fill=TEXT_MUTED, font=FONT_CAPTION)

        # The 20 deg RM cannot sweep. Drawn, or the gap reads as a fault.
        x1, y1 = self._to_xy(340, self._range_mm)
        c.create_line(cx, cy, x1, y1, fill=ACCENT_ORANGE)
        c.create_text(*self._to_xy(350, self._range_mm * 0.62),
                      text="no\nsweep", fill=ACCENT_ORANGE,
                      font=FONT_CAPTION, justify="center")

        for deg, mm in ghost:
            x, y = self._to_xy(deg, mm)
            c.create_oval(x - 1, y - 1, x + 1, y + 1, fill=BORDER, outline="")

        for deg, mm in points:
            x, y = self._to_xy(deg, mm)
            c.create_oval(x - 2, y - 2, x + 2, y + 2, fill=ACCENT_MINT, outline="")

        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill=ACCENT_GREEN, outline="")
        if title:
            c.create_text(cx, 12, text=title, fill=TEXT_LIGHT, font=FONT_MONO)
