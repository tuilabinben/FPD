"""RESET COORDINATES row, shared by both motion panels.

Built by P2P AND JOG, not just one: declaring reference is a jogging action —
drive machine to pose by hand, say "zero" — P2P-only forced leaving the panel
you were in. Wrong half of the original layout.

One builder, called twice, not two copies. Rows are independent widgets (Tk
widgets belong one parent) but share this code, so a change to confirm path,
colours or axis list can't land on one panel and miss the other.

`self.coord_reset_buttons` accumulates across both calls — `_set_motion_locked()`
walks it, every button from every panel must be in it. Cleared in `_init_state()`
and again on theme rebuild, next to `motion_lock_widgets` — else list keeps
destroyed widgets, `set_enabled()` raises.
"""

import tkinter as tk

from ..theme import (
    ACCENT_ORANGE,
    ACCENT_RED,
    FONT_CAPTION,
    INK_DARK,
    PANEL_BG,
    SURFACE,
    TEXT_LIGHT,
    TEXT_MUTED,
)
from ..widgets import HomeButton, RoundedButton

# "Z"/"ROT"/"A1"/"A2" = wire names in RESET_COORD:<axis>; second element =
# operator-facing label.
COORD_RESET_AXES = (("Z", "ZM"), ("ROT", "RM"), ("A1", "A1M"), ("A2", "A2M"))


class CoordResetRowMixin:
    def _build_coord_reset_row(self, parent, pady=(10, 6)):
        """Builds one RESET COORDINATES row, returns its frame.

        Every button goes through `reset_coordinates()` — confirms first,
        refuses while moving or a jog axis held. Presentation only, no
        second code path that could skip those checks.
        """
        row = tk.Frame(parent, bg=PANEL_BG)
        row.pack(pady=pady)
        tk.Label(row, text="RESET COORDINATES", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(side="left", padx=(6, 10))

        buttons = [RoundedButton(row, text="ALL", icon="⌖", bg_color=ACCENT_ORANGE,
                                 fg_color=INK_DARK, width=96, height=34, radius=10,
                                 font=FONT_CAPTION, command=self.reset_coordinates)]
        buttons[0].pack(side="left", padx=(0, 8))
        for axis, label in COORD_RESET_AXES:
            btn = RoundedButton(row, text=label, bg_color=SURFACE,
                                fg_color=TEXT_LIGHT, width=76, height=34, radius=10,
                                font=FONT_CAPTION,
                                command=lambda a=axis: self.reset_coordinates(a))
            btn.pack(side="left", padx=(0, 5))
            buttons.append(btn)

        # RESET POS is NOT in this row any more. Every button here only
        # DECLARES a position; that one drives the machine, so it sits
        # beside HOME instead — _build_reset_pos_button() below.

        # Extend, never reassign: other panel's row already in here.
        if not hasattr(self, "coord_reset_buttons"):
            self.coord_reset_buttons = []
        self.coord_reset_buttons += buttons
        # Zeroing counter mid-move would record a position already left.
        self.motion_lock_widgets += buttons
        return row

    def _build_reset_pos_button(self, parent, size=56):
        """RESET POS as a round button, HOME's twin. Returned unplaced: each
        panel puts it beside its own HOME.

        Red where HOME is mint, because it is the other way to a reference:
        it drives the machine to 0,0,0,0 under the board's own control
        (reset_position() in safety.py), no switches consulted. One builder
        for all four places, for the same reason the row above has one.
        """
        btn = HomeButton(parent, command=self.reset_position, size=size,
                         glyph="⟲", label="RESET\nPOS", accent=ACCENT_RED)
        # Locked while the machine moves, like HOME.
        self.motion_lock_widgets.append(btn)
        return btn
