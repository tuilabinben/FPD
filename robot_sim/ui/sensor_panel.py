"""M30..M32 PLC travel-limit row, shared by both motion panels.

Built by P2P AND JOG, same reason as coord-reset row (see coord_reset.py):
which limits covered decides whether next command moves at all, must be
visible in whichever mode operator works in. One builder called twice, not
two copies.

`self.plc_sensor_lamps` accumulates across both calls, keyed by bit, holds
list of lamps per bit — same bit has lamp on each panel.
`_refresh_plc_sensor_lamps()` walks all of them.

ROW USED TO SHOW M5..M8, THE HOME SENSORS. They lit a lamp and decided
nothing, while M30 was the bit actually refusing a jog and had no lamp at
all — an operator watched "M5 ZM lift = CLEAR" while ZM would not move
down. Board no longer reads any M but these three.
"""

import tkinter as tk

from ..config import (
    PLC_HOME_STATE_CLEAR_BITS,
    PLC_HOME_STATE_ON_BITS,
    PLC_SENSOR_BOTH_ENDS,
    PLC_SENSOR_PANEL,
    PLC_SENSOR_STALE_MS,
    PLC_SENSOR_UNKNOWN_TEXT,
)
from ..theme import (
    ACCENT_GREEN,
    ACCENT_ORANGE,
    ACCENT_PURPLE,
    FONT_CAPTION,
    FONT_HINT,
    LED_BG,
    PANEL_BG,
    TEXT_DIM,
    TEXT_MUTED,
)
from ..widgets import make_status_led


class SensorPanelMixin:
    def _build_plc_sensor_row(self, parent, pady=(8, 4)):
        """One row of four sensor lamps plus a HOME STATE lamp."""
        if not hasattr(self, "plc_sensor_lamps"):
            self.plc_sensor_lamps = {}
            self.plc_home_state_lamps = []

        row = tk.Frame(parent, bg=PANEL_BG)
        row.pack(pady=pady)

        tk.Label(row, text="PLC LIMITS", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(side="left", padx=(6, 10))

        for bit, label, _axis, _cmd, end in PLC_SENSOR_PANEL:
            # End in caption because the three do NOT sit at the same end —
            # ZM and A2M stop at their minimum, RM at its maximum because it
            # is mounted inverted. "Covered" means the opposite thing for
            # them, so reading the lamp without the end is guesswork.
            # A2M's is wired at BOTH ends, so its caption cannot name one.
            # Which end it caught shows in the lamp text instead, because it
            # changes while the machine runs.
            suffix = ("min/max" if bit in PLC_SENSOR_BOTH_ENDS
                      else ("max" if end > 0 else "min"))
            # Starts UNKNOWN not CLEAR. Until device read lands, board has
            # no idea; CLEAR would claim good news.
            card = make_status_led(row, f"{bit}  {label} ({suffix})",
                                   PLC_SENSOR_UNKNOWN_TEXT, ACCENT_PURPLE)
            card["frame"].pack(side="left", padx=(0, 6))
            self.plc_sensor_lamps.setdefault(bit, []).append(card)

        # HOME STATE is combination, not a device: all three covered. Zeroes
        # the coordinates, earns own lamp rather than leaving operator to
        # read three others and do the logic.
        # "?" not "NO": before device read lands, "not at home" is a claim
        # this board can't make either.
        home_card = make_status_led(row, "HOME STATE", "?", ACCENT_PURPLE)
        home_card["frame"].pack(side="left", padx=(10, 0))
        self.plc_home_state_lamps.append(home_card)

        # Where enforcement lives isn't guessable from the lamps.
        tk.Label(row, text="blocks P2P · warns in JOG", bg=PANEL_BG,
                 fg=TEXT_MUTED, font=FONT_HINT).pack(side="left", padx=(10, 0))
        return row

    # ── state ────────────────────────────────────────────────────────
    def plc_sensors_known(self):
        """True once a PLC device read has landed and isn't stale.

        Everything reading sensors must ask this first. Treating unknown as
        "not covered" is what made a dead link look safe.
        """
        if not getattr(self, "plc_sensor_data_seen", False):
            return False
        stale_at = getattr(self, "_plc_sensor_seen_at", None)
        if stale_at is None:
            return True
        try:
            import time
            return (time.monotonic() - stale_at) * 1000.0 < PLC_SENSOR_STALE_MS
        except Exception:
            return True

    def plc_sensor_end_for(self, bit: str):
        """Which end that switch is refusing: -1 minimum, +1 maximum.

        Fixed for ZM and RM. A2M's switch is wired at BOTH ends, and only
        the BOARD can tell them apart -- it watches the rising edge against
        the direction the axis was travelling. This just reads what it
        reported; deriving it here would mean guessing between polls, and a
        wrong guess refuses the one direction that comes off the switch.
        """
        ends = getattr(self, "plc_sensor_end", None)
        if ends and bit in ends:
            return ends[bit]
        return next((e for b, _l, _a, _c, e in PLC_SENSOR_PANEL if b == bit), -1)

    def plc_sensor_at_home_end(self, bit: str):
        """True when this switch is tripped at its HOME-side end.

        Only that end is the reference. A both-ends switch caught at the far
        end says the arm is fully EXTENDED, and reading that as "at home"
        would zero the counters at the wrong end of the travel.
        """
        home_end = next((e for b, _l, _a, _c, e in PLC_SENSOR_PANEL if b == bit), -1)
        return self.plc_sensor_end_for(bit) == home_end

    def plc_home_state(self):
        """True when machine sits on its reference: home-end sensors
        covered, far-end ones clear.

        False while readings unknown — unknown machine is not a homed
        machine; gates automatic coordinate reset.
        """
        if not self.plc_sensors_known():
            return False
        st = getattr(self, "plc_sensor_state", {})
        # at_home_end matters for A2M only, and it is the whole point of
        # tracking the end: its switch also trips fully EXTENDED, and
        # reading that as the reference would zero the counters 90 deg away
        # from where the machine actually is.
        return (all(st.get(b, False) and self.plc_sensor_at_home_end(b)
                    for b in PLC_HOME_STATE_ON_BITS)
                and not any(st.get(b, False) for b in PLC_HOME_STATE_CLEAR_BITS))

    def _refresh_plc_sensor_lamps(self):
        from ..widgets import set_led
        st = getattr(self, "plc_sensor_state", {})
        known = self.plc_sensors_known()
        for bit, _label, _axis, _cmd, _end in PLC_SENSOR_PANEL:
            for card in self.plc_sensor_lamps.get(bit, ()):
                if not known:
                    # PURPLE, not the orange COVERED uses. They were the same
                    # colour, so a dead link painted three orange lamps that
                    # read at a glance as three tripped limits — operator saw
                    # "ZM covered" while its IO-3 lamp was blinking CLEAR.
                    # Not TEXT_DIM either: unknown must not look like CLEAR,
                    # which is the older half of this same bug.
                    set_led(card, PLC_SENSOR_UNKNOWN_TEXT, ACCENT_PURPLE)
                elif st.get(bit, False):
                    if bit in PLC_SENSOR_BOTH_ENDS:
                        side = "MAX" if self.plc_sensor_end_for(bit) > 0 else "MIN"
                        set_led(card, f"COVERED {side}", ACCENT_ORANGE)
                    else:
                        set_led(card, "COVERED", ACCENT_ORANGE)
                else:
                    set_led(card, "CLEAR", TEXT_DIM)
        for card in getattr(self, "plc_home_state_lamps", ()):
            if not known:
                set_led(card, "?", ACCENT_PURPLE)
            elif self.plc_home_state():
                set_led(card, "AT HOME", ACCENT_GREEN)
            else:
                set_led(card, "NO", TEXT_DIM)
