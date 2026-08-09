"""The M5..M8 PLC sensor row, shared by both motion panels.

Built by P2P *and* by JOG for the same reason the coordinate-reset row is
(see coord_reset.py): which sensors are covered decides whether the next
command will move at all, so it has to be visible in whichever mode the
operator is working in. One builder called twice, not two copies.

`self.plc_sensor_lamps` accumulates across both calls, keyed by bit, and
holds a list of lamps per bit because the same bit has a lamp on each
panel. `_refresh_plc_sensor_lamps()` walks all of them.

M7 and M8 are broken on this machine, so they render as a fixed
placeholder rather than a state. Showing them as CLEAR would be claiming a
reading the machine cannot make, and showing them as COVERED would suggest
the elbows are blocked when nothing is blocking them.
"""

import tkinter as tk

from ..config import (
    PLC_HOME_STATE_CLEAR_BITS,
    PLC_HOME_STATE_ON_BITS,
    PLC_SENSOR_PANEL,
    PLC_SENSOR_STALE_MS,
    PLC_SENSOR_UNKNOWN_TEXT,
)
from ..theme import (
    ACCENT_GREEN,
    ACCENT_ORANGE,
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

        tk.Label(row, text="PLC SENSORS", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(side="left", padx=(6, 10))

        for bit, label, _axis, _cmd, end in PLC_SENSOR_PANEL:
            # The end is in the caption because M5/M6 and M7/M8 are at
            # OPPOSITE ends, and "covered" means the opposite thing for each
            # pair — reading the lamp without that is guesswork.
            suffix = "far" if end > 0 else "home"
            # Starts UNKNOWN, not CLEAR. Until a device read lands this board
            # has no idea, and saying CLEAR would be claiming good news.
            card = make_status_led(row, f"{bit}  {label} ({suffix})",
                                   PLC_SENSOR_UNKNOWN_TEXT, ACCENT_ORANGE)
            card["frame"].pack(side="left", padx=(0, 6))
            self.plc_sensor_lamps.setdefault(bit, []).append(card)

        # HOME STATE is the combination, not a device: M5 and M6 covered
        # while M7 and M8 are clear. It is what zeroes the coordinates, so
        # it earns its own lamp rather than leaving the operator to read
        # four others and do the logic.
        # "?" not "NO": before a device read lands, "not at home" is a claim
        # this board cannot make either.
        home_card = make_status_led(row, "HOME STATE", "?", ACCENT_ORANGE)
        home_card["frame"].pack(side="left", padx=(10, 0))
        self.plc_home_state_lamps.append(home_card)

        # Where enforcement lives is not guessable from the lamps.
        tk.Label(row, text="blocks P2P · warns in JOG", bg=PANEL_BG,
                 fg=TEXT_MUTED, font=FONT_HINT).pack(side="left", padx=(10, 0))
        return row

    # ── state ────────────────────────────────────────────────────────
    def plc_sensors_known(self):
        """True once a PLC device read has landed and is not stale.

        Everything that reads the sensors has to ask this first. Treating
        unknown as "not covered" is what made a dead link look safe.
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

    def plc_home_state(self):
        """True when the machine is sitting on its reference: the home-end
        sensors covered and the far-end ones clear.

        False while the readings are unknown — an unknown machine is not a
        homed machine, and this gates an automatic coordinate reset.
        """
        if not self.plc_sensors_known():
            return False
        st = getattr(self, "plc_sensor_state", {})
        return (all(st.get(b, False) for b in PLC_HOME_STATE_ON_BITS)
                and not any(st.get(b, False) for b in PLC_HOME_STATE_CLEAR_BITS))

    def _refresh_plc_sensor_lamps(self):
        from ..widgets import set_led
        st = getattr(self, "plc_sensor_state", {})
        known = self.plc_sensors_known()
        for bit, _label, _axis, _cmd, _end in PLC_SENSOR_PANEL:
            for card in self.plc_sensor_lamps.get(bit, ()):
                if not known:
                    set_led(card, PLC_SENSOR_UNKNOWN_TEXT, ACCENT_ORANGE)
                elif st.get(bit, False):
                    set_led(card, "COVERED", ACCENT_ORANGE)
                else:
                    set_led(card, "CLEAR", TEXT_DIM)
        for card in getattr(self, "plc_home_state_lamps", ()):
            if not known:
                set_led(card, "?", ACCENT_ORANGE)
            elif self.plc_home_state():
                set_led(card, "AT HOME", ACCENT_GREEN)
            else:
                set_led(card, "NO", TEXT_DIM)
