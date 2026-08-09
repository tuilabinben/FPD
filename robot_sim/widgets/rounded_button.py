"""Flat canvas button with anti-aliased rounded corners.

Depth is carried entirely by the fill: hover lightens it, press darkens
it. No shadow stack, because faking a blur on a Tk canvas costs six
stacked shapes and produces visible banding — see widgets/draw.py.
"""

import tkinter as tk
import tkinter.font as tkfont

from ..theme import (
    FONT_BUTTON,
    ACCENT_MINT,
    INK_DARK,
    PANEL_BG,
    RADIUS_MD,
    SURFACE,
    SURFACE_HI,
    TEXT_DIM,
    TEXT_LIGHT,
    mix,
    shade,
)
from ..hidpi import px
from .draw import clear, paint_rounded

#: Font objects are expensive to build and get rebuilt on every hover,
#: so they are cached by their spec tuple.
_FONT_CACHE = {}


def _parent_bg(parent):
    """Background of `parent`, tolerating ttk widgets that have no 'bg'."""
    try:
        return parent["bg"]
    except (tk.TclError, KeyError, IndexError):
        try:
            return parent.cget("background")
        except Exception:
            return PANEL_BG


class RoundedButton(tk.Canvas):
    def __init__(self, parent, text, command=None, icon="", radius=RADIUS_MD,
                 bg_color=ACCENT_MINT, fg_color=INK_DARK, hover_color=None,
                 width=160, height=38, font=FONT_BUTTON):
        surface = _parent_bg(parent)
        # Pixel dimensions are written for 96 DPI; px() maps them onto the
        # display's real grid so the button is the intended physical size.
        # Pixel dimensions scale; the FONT does not. Tk resolves point
        # sizes against the real DPI once `tk scaling` is set, so scaling
        # the size here too was double-scaling — see hidpi.font().
        width, height, radius = px(width), px(height), px(radius)
        super().__init__(parent, width=width, height=height, bg=surface,
                         highlightthickness=0, bd=0, cursor="hand2")
        self.command = command
        self.base_color = bg_color
        self.hover_color = hover_color
        self.radius = radius
        self.w, self.h = width, height
        self.text_str = text
        self.icon_str = icon
        self.fg_color = fg_color
        self.font = font
        self.enabled = True
        self._pressed = False
        self._hovering = False
        self.surface = surface

        self._draw()
        self.bind("<Enter>", self._on_enter)
        self.bind("<Leave>", self._on_leave)
        self.bind("<ButtonPress-1>", self._on_press)
        self.bind("<ButtonRelease-1>", self._on_release)

    # ── rendering ────────────────────────────────────────────────────
    def _is_neutral(self):
        """Neutral buttons are a surface tone; accent buttons carry a
        colour. The two need different hover and text treatment, so the
        distinction is made in one place."""
        return self.base_color in (SURFACE, SURFACE_HI, self.surface, PANEL_BG)

    def _fill_for_state(self):
        if not self.enabled:
            return mix(self.surface, TEXT_DIM, 0.10)
        if self._is_neutral():
            base = SURFACE
            if self._pressed:
                return shade(base, 0.86)
            return SURFACE_HI if self._hovering else base
        base = self.base_color
        if self._pressed:
            return shade(base, 0.86)
        return (self.hover_color or shade(base, 1.10)) if self._hovering else base

    def _fit_to_text(self, display):
        """Grows the button if its label would not fit.

        A fixed width plus a variable label is a clipping bug waiting to
        happen: "SETTINGS ESC", "LINK: OFF" and "BOOST: OFF" all render
        wider than the widths they were given, and set_config() can make a
        label longer at runtime. Measuring is the only way to be sure.

        It only ever grows. Shrinking would make a row of buttons jump
        around as their labels change, which is worse than a little extra
        padding.
        """
        try:
            key = tuple(self.font)
            f = _FONT_CACHE.get(key)
            if f is None:
                f = _FONT_CACHE[key] = tkfont.Font(font=self.font)
            needed = f.measure(display) + px(30)
        except Exception:
            # No Tk font engine (headless tests) — leave the width alone.
            return
        if needed > self.w:
            self.w = needed
            self.configure(width=needed)

    def _draw(self):
        clear(self)
        display = f"{self.icon_str}  {self.text_str}" if self.icon_str else self.text_str
        self._fit_to_text(display)

        fill = self._fill_for_state()
        # A single hairline, one step off the fill. Enough to define the
        # edge against the card behind it without reading as a border.
        border = mix(fill, "#ffffff", 0.07) if self.enabled else None
        paint_rounded(self, 0, 0, self.w, self.h, self.radius,
                      fill, self.surface, border=border, border_w=1)

        if not self.enabled:
            text_color = TEXT_DIM
        elif self._is_neutral():
            text_color = ACCENT_MINT if self._hovering else TEXT_LIGHT
        else:
            text_color = self.fg_color

        self.create_text(self.w // 2, self.h // 2, text=display,
                         fill=text_color, font=self.font)

    def _refresh(self):
        self._draw()

    # ── public API ───────────────────────────────────────────────────
    def set_config(self, text, bg_color, icon="", fg_color=None):
        self.text_str = text
        self.icon_str = icon
        self.base_color = bg_color
        self.hover_color = None
        if fg_color is not None:
            self.fg_color = fg_color
        self._refresh()

    def set_enabled(self, enabled):
        self.enabled = enabled
        if not enabled:
            self._pressed = False
            self._hovering = False
        self.configure(cursor="hand2" if enabled else "arrow")
        self._refresh()

    # ── events ───────────────────────────────────────────────────────
    def _on_enter(self, _event=None):
        self._hovering = True
        if self.enabled:
            self._refresh()

    def _on_leave(self, _event=None):
        self._hovering = False
        self._pressed = False
        if self.enabled:
            self._refresh()

    def _on_press(self, _event=None):
        if self.enabled:
            self._pressed = True
            self._refresh()

    def _on_release(self, _event=None):
        # Fire only if the press started here AND the pointer is still
        # inside, so dragging off the button cancels the click.
        was_pressed, self._pressed = self._pressed, False
        if self.enabled:
            self._refresh()
        if self.enabled and was_pressed and self._hovering and self.command:
            self.command()
