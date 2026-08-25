"""PING/PONG keep-alive and connection-health LEDs.

Heartbeat no longer drives own lamp — that slot now shows PLC Ethernet
link. PING/PONG state is serial-link health, already shown by COM PORT
and CLEARCORE lamps; survives here because it notices dead board and
triggers all-stop.
"""

from ..config import (
    HEARTBEAT_INTERVAL_MS,
    MISSED_BEAT_LIMIT,
    PING_TIMEOUT_MS,
)
from ..theme import ACCENT_GREEN, ACCENT_RED
from ..widgets import set_led


class HeartbeatMixin:
    def manual_ping(self):
        if not self.is_connected:
            from tkinter import messagebox
            messagebox.showwarning("Warning", "No COM port connected.")
            return
        self._missed_beats = 0
        self.send("PING")
        self.log("Manual PING sent — waiting for PONG from ClearCore...")
        self._schedule("_ping_timeout_job", PING_TIMEOUT_MS, self._on_ping_timeout)

    def _schedule_next_heartbeat(self):
        if self.is_connected:
            self._schedule("_heartbeat_job", HEARTBEAT_INTERVAL_MS, self._send_heartbeat_ping)

    def _send_heartbeat_ping(self):
        self._heartbeat_job = None
        if not self.is_connected:
            return
        # silent TX — RX "<< PONG" line is visible confirmation
        self.send("PING", log_tx=False)
        # same tick asks board for PLC link state, drives PLC lamp.
        # piggybacked so there's one cadence, not two.
        self.send("PLC_STATUS", log_tx=False)
        self._schedule("_ping_timeout_job", PING_TIMEOUT_MS, self._on_ping_timeout)
        self._schedule_next_heartbeat()

    def _on_heartbeat_pong(self):
        self._cancel_job("_ping_timeout_job")
        recovered = self._missed_beats > 0
        self._missed_beats = 0

        if not self.hw_confirmed:
            self.hw_confirmed = True
            set_led(self.hw_led_card, "IO0 ON ✓", ACCENT_GREEN)
            self.status_var.set("CONNECTED — ClearCore IO0 ACTIVE | Heartbeat OK")
            self.log("✓ PONG received. ClearCore IO0 is lit. Handshake successful.")
            # board holds speed/travel limits in RAM, power cycle silently
            # drops it to factory envelope. GUI is system of record: push
            # saved setup once handshake proves something's listening.
            # skip this and shown limits can disagree with enforced limits
            # after any board reset.
            self._push_settings_to_board(reason="handshake")
        elif recovered:
            self.status_var.set("CONNECTED — Heartbeat recovered.")
            self.log("✓ Heartbeat recovered.")

    def _blink_heartbeat_led(self):
        """Single place a live beat is acknowledged. No longer lights a
        lamp — protocol.py still calls it on every [ALIVE]; removing call
        sites would lose missed-beat reset."""
        self._missed_beats = 0

    def _on_ping_timeout(self):
        self._ping_timeout_job = None
        self._missed_beats += 1
        remaining = MISSED_BEAT_LIMIT - self._missed_beats

        if self._missed_beats < MISSED_BEAT_LIMIT:
            self.status_var.set(
                f"WARNING — PONG miss #{self._missed_beats} ({remaining} more until LOST)")
            self.log(f"⚠ PONG timeout #{self._missed_beats}/{MISSED_BEAT_LIMIT}.", tag="warn")
            return

        set_led(self.hw_led_card, "LOST", ACCENT_RED)
        # board gone, so whatever it last said about PLC is stale
        self._plc_link_lost()
        self.hw_confirmed = False
        self.status_var.set("⛔ CONNECTION LOST — ClearCore is not responding.")
        self.log(f"✗ CONNECTION LOST after {MISSED_BEAT_LIMIT} consecutive missed PONGs.", tag="error")
        self.log("→ Check board power, the USB cable, and the ClearCore firmware.", tag="error")

        # lost link is also all-stop: board may still be moving, can't see
        # it anymore. Previous version left motion running and killed
        # heartbeat outright, so link could never recover.
        self._release_all_jog_axes()
        if self.is_running or self.is_homing:
            self.send("STOP")
            self.is_running = False
            self.is_homing = False
            self._cancel_job("anim_job")
            self._set_motion_locked(False)
            self.log("Sent STOP because the link dropped mid-motion.", tag="warn")

        if self.is_connected:
            self._schedule_next_heartbeat()   # keep trying, allow recovery
