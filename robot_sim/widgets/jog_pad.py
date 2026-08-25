"""Dead-man-switch jog button: press to move, release to stop.

Flat, axis colour does signalling. Idle = plain surface tile w/ coloured
glyph; hover lifts fill one step; ACTIVE floods whole tile in axis colour
so moving axis is unmistakable across room. That last part is the
functional requirement — on machine controller "which axis is live"
must be readable at a glance, subtle shadow change would not be.
"""

import tkinter as tk

from ..theme import (
    FONT_CAPTION,
    FONT_GLYPH,
    FONT_KEYCAP,
    INK_DARK,
    LED_BG,
    PANEL_BG,
    RADIUS_LG,
    RADIUS_SM,
    SURFACE,
    SURFACE_HI,
    TEXT_DIM,
    TEXT_LIGHT,
    TEXT_MUTED,
    mix,
)
from ..hidpi import px
from .draw import clear, paint_rounded


class JogPad(tk.Canvas):
    def __init__(self, parent, w, h, glyph, label, keycap,
                 base_color, hi_color, on_press, on_release):
        try:
            surface = parent["bg"]
        except Exception:
            surface = PANEL_BG
        w, h = px(w), px(h)
        super().__init__(parent, width=w, height=h, bg=surface,
                         highlightthickness=0, bd=0, cursor="hand2")
        self.w, self.h = w, h
        self.glyph = glyph
        self.label = label
        self.keycap = keycap
        self.base_color = base_color
        self.hi_color = hi_color
        self.surface = surface
        self.on_press = on_press
        self.on_release = on_release
        self.state = "idle"
        self.enabled = True

        self._draw()
        self.bind("<ButtonPress-1>", self._press)
        self.bind("<ButtonRelease-1>", self._release)
        self.bind("<Enter>", self._enter)
        self.bind("<Leave>", self._leave)

    def _draw(self):
        clear(self)
        r = px(RADIUS_LG)
        active = self.enabled and self.state == "active"

        if not self.enabled:
            fill = mix(self.surface, TEXT_DIM, 0.08)
            border = None
            glyph_fill = label_fill = TEXT_DIM
        elif active:
            fill = self.base_color
            border = mix(self.base_color, "#ffffff", 0.18)
            glyph_fill = label_fill = INK_DARK
        elif self.state == "hover":
            fill = SURFACE_HI
            # axis colour arrives on border first — hover already tells
            # which axis you're about to move
            border = mix(SURFACE_HI, self.base_color, 0.55)
            glyph_fill, label_fill = self.hi_color, TEXT_LIGHT
        else:
            fill = SURFACE
            border = mix(SURFACE, "#ffffff", 0.06)
            glyph_fill, label_fill = self.base_color, TEXT_MUTED

        paint_rounded(self, 0, 0, self.w, self.h, r,
                      fill, self.surface, border=border, border_w=1)

        cx = self.w / 2
        self.create_text(cx, self.h * 0.33, text=self.glyph,
                         font=FONT_GLYPH, fill=glyph_fill)
        self.create_text(cx, self.h * 0.60, text=self.label,
                         font=FONT_CAPTION, fill=label_fill)

        # keycap chip: dark well normally; active pad sits on bright fill,
        # so it inverts to stay readable
        kc_w, kc_h = px(22), px(15)
        kx1, ky1 = cx - kc_w / 2, self.h * 0.82 - kc_h / 2
        if active:
            chip = mix(self.base_color, INK_DARK, 0.35)
            cap_fill = TEXT_LIGHT
        else:
            chip = LED_BG
            cap_fill = TEXT_MUTED if self.enabled else TEXT_DIM
        paint_rounded(self, kx1, ky1, kc_w, kc_h, px(RADIUS_SM * 0.6), chip, fill)
        self.create_text(cx, self.h * 0.82, text=self.keycap,
                         font=FONT_KEYCAP, fill=cap_fill)

    def set_state(self, state):
        if self.state != state:
            self.state = state
            self._draw()

    def set_enabled(self, enabled):
        # disabling mid-motion must not leave axis latched on: fire
        # release callback before greying out
        if not enabled and self.state == "active":
            self.state = "idle"
            self.on_release()
        self.enabled = enabled
        self.configure(cursor="hand2" if enabled else "arrow")
        if not enabled:
            self.state = "idle"
        self._draw()

    def set_keycap(self, keycap):
        """Updates chip letter after rebinding, w/o rebuilding panel —
        pad showing old key is worse than pad showing none."""
        if keycap != self.keycap:
            self.keycap = keycap
            self._draw()

    def key_activate(self):
        if self.enabled:
            self.set_state("active")

    def key_deactivate(self):
        self.set_state("idle")

    def _press(self, _=None):
        if not self.enabled:
            return
        self.set_state("active")
        self.on_press()

    def _release(self, _=None):
        if not self.enabled:
            return
        was_active = self.state == "active"
        self.set_state("hover")
        if was_active:
            self.on_release()

    def _enter(self, _=None):
        if self.enabled and self.state != "active":
            self.set_state("hover")

    def _leave(self, _=None):
        if not self.enabled:
            return
        if self.state == "active":
            self.on_release()
        self.set_state("idle")
