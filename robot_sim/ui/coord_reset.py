"""The RESET COORDINATES row, shared by both motion panels.

It is built by P2P *and* by JOG, and deliberately not by only one of
them: declaring the reference is a jogging action — you drive the machine
to the pose by hand and then say "this is zero" — so the operator was
being made to leave the panel they were working in to reach it. Having it
in P2P only was the original layout and it was the wrong half.

One builder, called twice, rather than two copies. The two rows are
independent widgets (Tk widgets belong to one parent), but they share
this code, so a change to the confirmation path, the colours or the axis
list cannot land on one panel and miss the other.

`self.coord_reset_buttons` accumulates across both calls — it is what
`_set_motion_locked()` walks, so every button from every panel has to be
in it. It is cleared in `_init_state()` and again on a theme rebuild,
next to `motion_lock_widgets`, for the same reason: the list would
otherwise keep destroyed widgets and `set_enabled()` would raise.
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

# "Z"/"ROT"/"A1"/"A2" are the wire names in RESET_COORD:<axis>; the second
# element is what the operator sees on the machine.
COORD_RESET_AXES = (("Z", "ZM"), ("ROT", "RM"), ("A1", "A1M"), ("A2", "A2M"))


class CoordResetRowMixin:
    def _build_coord_reset_row(self, parent, pady=(10, 6)):
        """Builds one RESET COORDINATES row and returns its frame.

        Every button still goes through `reset_coordinates()`, which
        confirms first and refuses while anything is moving or a jog axis
        is held — so this is presentation only, with no second code path
        that could skip those checks.
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

        # A visible break before RESET POSITION: unlike the five buttons
        # above, it actually MOVES the machine (see reset_position() in
        # safety.py) — the other five are declare-only, never driving a
        # motor. A red accent and its own gap keep that distinction
        # visible rather than blending a moving action into a no-move row.
        tk.Frame(row, width=1, bg=BORDER_SOFT).pack(side="left", fill="y",
                                                     padx=(8, 8), pady=4)
        reset_pos_btn = RoundedButton(row, text="RESET POS", icon="⟲",
                                      bg_color=ACCENT_RED, fg_color=INK_DARK,
                                      width=118, height=34, radius=10,
                                      font=FONT_CAPTION, command=self.reset_position)
        reset_pos_btn.pack(side="left", padx=(0, 5))
        buttons.append(reset_pos_btn)

        # Extend, never reassign: the other panel's row is already in here.
        if not hasattr(self, "coord_reset_buttons"):
            self.coord_reset_buttons = []
        self.coord_reset_buttons += buttons
        # Zeroing a counter mid-move would record a position already left.
        self.motion_lock_widgets += buttons
        return row
