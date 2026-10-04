"""Circular HOME button, used by the Jog and P2P panels — and RESET POS, its twin."""

import tkinter as tk

from ..theme import (
    FONT_CAPTION,
    FONT_GLYPH,
    ACCENT_MINT,
    INK_DARK,
    PANEL_BG,
    SURFACE,
    SURFACE_HI,
    TEXT_DIM,
    TEXT_LIGHT,
    TEXT_MUTED,
    mix,
)
from ..hidpi import px
from .draw import clear, paint_circle


class HomeButton(tk.Canvas):
    """Circular button in the same flat language as the jog pads.
    Pressing it in either panel sends the same HOME command.

    RESET POS is this widget with another glyph, caption and accent (see
    _build_reset_pos_button in ui/coord_reset.py) — one round button, not two.
    `accent` defaults to None, not ACCENT_MINT: a default argument is bound
    once, and would keep the old colour through a theme switch."""

    def __init__(self, parent, command, size=78, glyph="⌂", label="HOME",
                 accent=None):
        try:
            surface = parent["bg"]
        except Exception:
            surface = PANEL_BG
        size = px(size)
        super().__init__(parent, width=size, height=size, bg=surface,
                         highlightthickness=0, cursor="hand2")
        self.size = size
        self.surface = surface
        self.command = command
        self.glyph, self.label = glyph, label
        self.accent = accent or ACCENT_MINT
        self.state = "idle"
        self.enabled = True
        self._pressed = False
        self._hovering = False
        self._draw()
        self.bind("<ButtonPress-1>", lambda e: self._on_press())
        self.bind("<ButtonRelease-1>", lambda e: self._on_release())
        self.bind("<Enter>", lambda e: self._on_enter())
        self.bind("<Leave>", lambda e: self._on_leave())

    def _draw(self):
        clear(self)
        s = self.size
        active = self.enabled and self.state == "active"

        if not self.enabled:
            fill, border = mix(self.surface, TEXT_DIM, 0.08), None
            glyph_fill = text_fill = TEXT_DIM
        elif active:
            fill = self.accent
            border = mix(self.accent, "#ffffff", 0.18)
            glyph_fill = text_fill = INK_DARK
        elif self.state == "hover":
            fill = SURFACE_HI
            border = mix(SURFACE_HI, self.accent, 0.55)
            glyph_fill, text_fill = self.accent, TEXT_LIGHT
        else:
            fill = SURFACE
            border = mix(SURFACE, "#ffffff", 0.06)
            glyph_fill, text_fill = TEXT_MUTED, TEXT_MUTED

        paint_circle(self, 0, 0, s, fill, self.surface,
                     border=border, border_w=1)

        cx = s / 2
        # A two-line caption takes the room under the glyph, so both move up.
        two = "\n" in self.label
        self.create_text(cx, s * (0.31 if two else 0.42), text=self.glyph,
                         font=FONT_GLYPH, fill=glyph_fill)
        self.create_text(cx, s * (0.67 if two else 0.68), text=self.label,
                         font=FONT_CAPTION, fill=text_fill, justify="center")

    def set_enabled(self, enabled):
        self.enabled = enabled
        self._pressed = False
        self.configure(cursor="hand2" if enabled else "arrow")
        self.state = "hover" if (enabled and self._hovering) else "idle"
        self._draw()

    def _on_press(self):
        if self.enabled:
            self._pressed = True
            self.state = "active"
            self._draw()

    def _on_release(self):
        was_pressed, self._pressed = self._pressed, False
        if not self.enabled:
            return
        self.state = "hover" if self._hovering else "idle"
        self._draw()
        # Fire only if press started here, pointer never left — HOME is real
        # motion command, accidental drag must not trigger it.
        if was_pressed and self._hovering and self.command:
            self.command()

    def _on_enter(self):
        self._hovering = True
        if self.enabled and self.state != "active":
            self.state = "hover"
            self._draw()

    def _on_leave(self):
        self._hovering = False
        if not self.enabled:
            return
        self._pressed = False
        self.state = "idle"
        self._draw()
