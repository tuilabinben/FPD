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
    BORDER_SOFT,
    FONT_CAPTION,
    INK_DARK,
    PANEL_BG,
    SURFACE,
    TEXT_LIGHT,
    TEXT_MUTED,
)
from ..widgets import RoundedButton

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

        # Break before RESET POSITION: unlike the five buttons above, it
        # actually MOVES the machine (see reset_position() in safety.py) —
        # others are declare-only, never drive a motor. Red accent + own
        # gap keep that distinction visible, not blended into a no-move row.
        tk.Frame(row, width=1, bg=BORDER_SOFT).pack(side="left", fill="y",
                                                     padx=(8, 8), pady=4)
        reset_pos_btn = RoundedButton(row, text="RESET POS", icon="⟲",
                                      bg_color=ACCENT_RED, fg_color=INK_DARK,
                                      width=118, height=34, radius=10,
                                      font=FONT_CAPTION, command=self.reset_position)
        reset_pos_btn.pack(side="left", padx=(0, 5))
        buttons.append(reset_pos_btn)

        # Extend, never reassign: other panel's row already in here.
        if not hasattr(self, "coord_reset_buttons"):
            self.coord_reset_buttons = []
        self.coord_reset_buttons += buttons
        # Zeroing counter mid-move would record a position already left.
        self.motion_lock_widgets += buttons
        return row
