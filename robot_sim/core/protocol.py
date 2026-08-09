"""Parsing of the lines the ClearCore board sends back."""

import re

import re as _re

from ..config import (
    ARM_HOME_DEG,
    PLC_LED_STATES,
    PLC_SENSOR_PANEL,
    ROT_HOME_DEG,
    Z_HOME_MM,
)
from ..theme import ACCENT_RED, TEXT_MUTED

POS_RE = re.compile(
    r"D1:\s*(-?[\d.]+)\s*mm\s*\|\s*ROT:\s*(-?[\d.]+)\s*deg\s*\|\s*"
    r"A1M:\s*(-?[\d.]+)\s*deg\s*\|\s*A2M:\s*(-?[\d.]+)\s*deg\s*\((\d+)%\)",
    re.IGNORECASE,
)

# v9 reports both elbows separately. The older single-ARM form is still
# accepted so a board still running v8 firmware keeps working.
JOG_RE_V9 = re.compile(
    r"ROT:\s*(-?[\d.]+)\s*deg\s*\|\s*A1M:\s*(-?[\d.]+)\s*deg\s*\|\s*"
    r"A2M:\s*(-?[\d.]+)\s*deg\s*\|\s*Z:\s*(-?[\d.]+)\s*mm",
    re.IGNORECASE,
)
#: v9.2+ appends the derived frog-leg angle and reach. Its ABSENCE is how
#: an older board is detected: a pre-v9.2 board reports the elbow in the old
#: frame (it printed 60 at home), so adopting its numbers would make the
#: readout jump to 60 and stay there, which looks exactly like a fault in
#: this GUI. Those values are refused instead, loudly.
FOLD_FIELDS_RE = re.compile(r"FOLD1:\s*-?[\d.]+", re.IGNORECASE)

JOG_RE_LEGACY = re.compile(
    r"ROT:\s*(-?[\d.]+)\s*deg\s*\|\s*ARM:\s*(-?[\d.]+)\s*deg\s*\|\s*Z:\s*(-?[\d.]+)\s*mm",
    re.IGNORECASE,
)


class ProtocolMixin:
    def _parse_hardware_response(self, response: str):
        text = response.strip()
        upper = text.upper()

        if upper in ("PONG", "[PONG]"):
            self._on_heartbeat_pong()
            return

        if upper.startswith("[ALIVE]"):
            self._missed_beats = 0
            self._blink_heartbeat_led()
            return

        if "CLEARCORE POS" in upper:
            # Joint-space telemetry: the board reports D1/ROT/A1M/A2M
            # directly and no longer does any IK/FK itself.
            m = POS_RE.search(text)
            if m:
                d1, rot, a1, a2, pct = m.groups()
                if not self._board_reports_motor_degrees(text):
                    return
                self._update_p2p_telemetry(float(d1), float(rot), float(a1),
                                           float(a2), int(pct))
            else:
                self.log("Could not parse the CLEARCORE POS line (unexpected format).", tag="warn")
            return

        if "JOG POS" in upper:
            # Keep the simulation in step with the real machine so a later
            # offline jog continues from the true position.
            m = JOG_RE_V9.search(text)
            if m:
                rot, a1, a2, z = (float(v) for v in m.groups())
                if not self._board_reports_motor_degrees(text):
                    # Still take the two axes whose frame did NOT change, so
                    # the operator keeps a live RM and ZM readout.
                    self.sim_rot, self.sim_z = rot, z
                    self._update_jog_readout()
                    return
                self.sim_rot, self.sim_a1, self.sim_a2, self.sim_z = rot, a1, a2, z
                self._update_jog_readout()
                return
            m = JOG_RE_LEGACY.search(text)
            if m:
                rot, arm, z = (float(v) for v in m.groups())
                self.sim_rot, self.sim_z = rot, z
                self.sim_a1 = self.sim_a2 = arm      # v8 firmware: one shared angle
                self._update_jog_readout()
                self.log("Board is running old firmware (JOG POS reports a single ARM "
                         "angle) — flash v9 for independent arm control.", tag="warn")
                return
            self.log("Could not parse the JOG POS line (unexpected format).", tag="warn")
            return

        if upper.startswith("[LOADED]"):
            self.status_var.set("LOADED — ClearCore acknowledged Point A/B.")
            return

        # Completion ("[RUN] TARGET REACHED") must fall through to the
        # dedicated check below, not be swallowed here as generic progress —
        # it used to be, so real-hardware completion never unlocked the GUI.
        if upper.startswith("[RUN]") and "TARGET REACHED" not in upper:
            self.status_var.set(f"RUNNING — {text.split(']', 1)[-1].strip()}")
            return

        if upper.startswith("[PARAMS_OK]"):
            self.log("ClearCore acknowledged the new PID gains.")
            return

        if upper.startswith("[MOTION_OK]"):
            self.log("ClearCore acknowledged the speed percentages.")
            return

        if upper.startswith("[COORD_RESET]"):
            # The board zeroed its counters, so the GUI's own reference is
            # now valid too — whether the reset came from this app or from
            # someone typing RESET_COORD into a terminal.
            self.is_homed = True
            self._refresh_jog_status()
            self.log("ClearCore reset its coordinates — soft limits are now active.")
            return

        if upper.startswith("[LIMIT_SET]"):
            self.log("Board confirmed limit: " + text.split("]", 1)[-1].strip())
            return

        # "[LIMITS] ..." and "[LIMITS_INFO] ..." are the board's own view of
        # the envelope. Logged, not parsed back into the settings: the GUI
        # is the system of record and re-sends on connect, so quietly
        # adopting whatever the board says would let a stale board
        # overwrite the operator's saved setup.
        if upper.startswith("[LIMITS"):
            self.log(text, tag="rx")
            return

        if upper.startswith("[SPEED]") or upper.startswith("[PROFILE]"):
            self.log(text, tag="rx")
            return

        if upper.startswith("[LIMIT]"):
            # e.g. "[LIMIT] ROT_CW" / "[LIMIT] Z_UP" / "[LIMIT] A1_FWD — ..."
            # v9.1 appends an explanation after an em dash; take only the
            # axis token or the lookup in _on_limit_triggered fails.
            payload = text.split("]", 1)[-1].strip()
            self._on_limit_triggered(payload.split()[0].upper() if payload else "")
            return

        if upper.startswith("[PLC_HOME]"):
            self._on_plc_home_line(text)
            return

        if upper.startswith("[PLC_STATE]"):
            # Polled every heartbeat tick, so it is NOT logged — 20 lines a
            # minute of unchanged status would bury everything else. Only a
            # CHANGE of state is worth a line.
            self._on_plc_state(text)
            return

        if upper.startswith("[PLC]"):
            self.log(text, tag="rx")
            if "TCP SOCKET OPEN" in upper or "CONNECTED TO" in upper:
                # A socket is not a conversation. Reporting this as CONNECTED
                # is what made the lamp flap: the socket genuinely opens and
                # closes every few seconds when the PLC answers nothing, so
                # the lamp followed it. At best this is "no reply yet"; only
                # a landed device read promotes it to CONNECTED.
                if self._plc_led_state != "connected":
                    self._set_plc_led("no_reply")
            elif "NO REPLY WITHIN" in upper:
                self._set_plc_led("no_reply")
            elif "SOCKET DROPPED" in upper:
                self._set_plc_led("unknown")
            return

        if upper.startswith("[ERROR]") and "PLC UNREACHABLE" in upper:
            self._set_plc_led("unreachable")
            self.log(text, tag="error")
            return

        if upper.startswith("[WARN]") and "NO ETHERNET LINK" in upper:
            self._set_plc_led("unreachable")
            self.log(text, tag="warn")
            return

        if upper.startswith("[HOME]"):
            if "COMPLETE" in upper:
                self._on_home_complete(simulated="SIMULATED" in upper)
                return
            if "FAILED" in upper or "TIMEOUT" in upper:
                # The PLC never answered. Unlock the GUI — otherwise every
                # control except STOP stays disabled forever.
                self.is_homing = False
                self._cancel_job("_home_sim_job")
                self._set_motion_locked(False)
                self.jog_dot.itemconfig(self._jog_dot_id, fill=ACCENT_RED)
                self.jog_status_var.set("HOME failed")
                self.status_var.set("HOME FAILED — the PLC never returned DONE.")
                self.log("HOME failed: no DONE from the PLC. Check the Ethernet link to "
                         "the PLC and the PLC program.", tag="error")
                return
            return

        # Own branch, placed BEFORE the generic "TARGET REACHED" catch-all
        # below — that check would otherwise treat this as a P2P RUN
        # completion (wrong status text, wrong side effects), the same
        # shadowing mistake "[RUN] TARGET REACHED" used to hit.
        if upper.startswith("[RESET_POSITION]"):
            if "TARGET REACHED" in upper:
                self._on_reset_position_complete(simulated=False)
            else:
                self.status_var.set(f"RESETTING POSITION — {text.split(']', 1)[-1].strip()}")
            return

        if upper.startswith("[WATCHDOG]"):
            # The board stopped a jog itself because our keep-alive stopped
            # arriving. Clear the local axes so the UI matches the machine.
            self._release_all_jog_axes(send_stop=False)
            self.status_var.set("JOG STOPPED — board watchdog tripped.")
            self.log("The board stopped the jog itself — no JOG_HB keep-alive arrived. "
                     "Check CPU load and the serial link.", tag="warn")
            return

        # v9.2 emits English. The Vietnamese strings a v8 board sends are
        # still accepted so an un-flashed board keeps working — same
        # back-compatibility rule as JOG_RE_LEGACY above.
        if "TARGET REACHED" in upper or "DA DEN DIEM DICH THANH CONG" in upper:
            self.is_running = False
            self._set_motion_locked(False)
            self._set_progress(100)
            self.status_var.set("READY — Target point (B) reached.")
            return

        if "EMERGENCY STOP" in upper or "DUNG KHAN CAP" in upper:
            self.is_running = False
            self.is_homing = False
            self._cancel_job("anim_job")
            self._release_all_jog_axes(send_stop=False)
            self._set_motion_locked(False)
            self.jog_dot.itemconfig(self._jog_dot_id, fill=TEXT_MUTED)
            self.status_var.set("STOPPED — Emergency Stop Triggered!")
            return

    # ── PLC Ethernet link lamp ───────────────────────────────────────
    _PLC_STATE_RE = _re.compile(r"link=(\w+)\s+socket=(\w+)", _re.IGNORECASE)
    #: data=NONE|STALE|OK — whether DEVICE READS are landing. This, not the
    #: socket, is what the lamp reports: a PLC that accepts connections but
    #: answers nothing made the socket go up and down every few seconds, and
    #: a socket-driven lamp flapped green/red with it.
    _PLC_DATA_RE = _re.compile(r"data=(NONE|STALE|OK)", _re.IGNORECASE)
    #: conn=<succeeded>/<attempted>. A socket that has opened even once
    #: proves the cable and the address are fine, so a currently-closed
    #: socket is NOT "unreachable" — it is a conversation that is not
    #: happening. Without this the lamp alternated NO REPLY / UNREACHABLE as
    #: the socket cycled, which is the same flap one level down.
    _PLC_CONN_RE = _re.compile(r"conn=(\d+)/(\d+)", _re.IGNORECASE)

    def _on_plc_state(self, text: str):
        """Drives the PLC lamp from the board's [PLC_STATE] reply.

        Three states are distinguished because they need different actions:
        UNREACHABLE is a cable or address problem, NO REPLY means the socket
        opened but the Ethernet module is not answering device reads (almost
        always MC protocol not enabled on the port), and CONNECTED means
        HOME can actually work.
        """
        bits = self._PLC_HOME_BITS_RE.search(text)
        if bits:
            field = bits.group(1)
            if "?" in field or "NO DEVICE DATA" in text.upper():
                # The board is telling us it does not know. Do NOT write
                # False into the sensor state — that is exactly how a dead
                # link came to look like four clear sensors.
                self._mark_plc_sensors_unknown()
            else:
                # Authoritative, and polled: an edge message can be missed,
                # this cannot. Order is Z/R/A1/A2 == M5/M6/M7/M8.
                for bit, ch in zip(("M5", "M6", "M7", "M8"), field):
                    self._set_plc_sensor(bit, ch == "1")
                self._mark_plc_sensors_seen()
                self._latch_home_state_if_new()

        m = self._PLC_STATE_RE.search(text)
        if not m:
            self.log(text, tag="rx")
            return
        link, socket = m.group(1).upper(), m.group(2).upper()
        data = self._PLC_DATA_RE.search(text)
        if data:
            # Preferred: the board says whether reads are landing.
            #   OK    -> the conversation works, which is what HOME needs
            #   STALE -> it worked and stopped
            #   NONE  -> the socket may open and close all day; no data ever
            got = data.group(1).upper()
            conn = self._PLC_CONN_RE.search(text)
            ever_opened = bool(conn) and int(conn.group(1)) > 0
            if got == "OK":
                state = "connected"
            elif got == "STALE":
                state = "no_reply"
            elif ever_opened:
                # The endpoint answers TCP but not device reads. Steady, even
                # while the socket itself opens and closes on each timeout.
                state = "no_reply"
            elif link != "UP" or socket != "OPEN":
                state = "unreachable"
            else:
                state = "no_reply"
        elif link != "UP" or socket != "OPEN":
            state = "unreachable"
        elif "word=----" in text:
            state = "no_reply"
        else:
            state = "connected"
        self._set_plc_led(state, detail=text)

    def _latch_home_state_if_new(self):
        """Fires the coordinate reset on the RISING edge of the home state.

        Edge-triggered because the condition stays true while the machine
        sits at home, and re-zeroing on every 3 s poll would quietly eat any
        real motion away from the reference.
        """
        now = self.plc_home_state()
        was = getattr(self, "_plc_home_state_prev", False)
        self._plc_home_state_prev = now
        if not now or was:
            return
        if self.motion_locked or self.is_running or self.jog_active:
            self._plc_home_state_prev = False    # try again once stopped
            return
        self._adopt_home_state_reset()

    def _plc_link_lost(self):
        """Serial or PLC link gone: the sensors are unknown, not clear."""
        self._set_plc_led("unknown")
        self._mark_plc_sensors_unknown()

    def _set_plc_led(self, state: str, detail: str = ""):
        """Sets the lamp, and logs ONLY on a change of state."""
        label, colour_name = PLC_LED_STATES.get(state, PLC_LED_STATES["unknown"])
        if getattr(self, "_plc_led_state", None) == state:
            return
        self._plc_led_state = state
        card = getattr(self, "plc_led_card", None)
        if card is None:
            return                      # called before the UI exists
        from .. import theme
        from ..widgets import set_led
        set_led(card, label, getattr(theme, colour_name))
        if state == "connected":
            self.log("PLC Ethernet link UP — the Mitsubishi is answering device reads.")
        elif state == "no_reply":
            self.log("PLC socket is open but the PLC is not answering device reads. "
                     "Check that MC protocol is enabled on that port.", tag="warn")
        elif state == "unreachable":
            self.log("PLC Ethernet link DOWN — HOME will time out. Check the cable, "
                     "the subnet, and the PLC's Ethernet module.", tag="error")

    # ── M5..M8 sensor state ──────────────────────────────────────────
    # Accepts "????" as well as four bits: the board sends that when it has
    # no device data, and it is the difference between "not covered" and
    # "nobody knows".
    _PLC_HOME_BITS_RE = _re.compile(
        r"home\s+Z/R/A1/A2\s*=\s*([01?]{4})", _re.IGNORECASE)
    _PLC_HOME_LINE_RE = _re.compile(
        r"\[PLC_HOME\]\s+(\w+)\s+home sensor\s+(M\d)\b.*?\b(REACHED|left)\b",
        _re.IGNORECASE)

    def _on_plc_home_line(self, text: str):
        """One sensor changed state, or the HOME state latched."""
        self.log(text, tag="rx")
        m = self._PLC_HOME_LINE_RE.search(text)
        if m:
            bit, event = m.group(2).upper(), m.group(3).upper()
            self._set_plc_sensor(bit, event == "REACHED")
            return
        if "HOME STATE" in text.upper():
            # The board latched the home state and zeroed its counters, so
            # the GUI's copy of the pose has to follow or the two disagree
            # from this instant on.
            self._adopt_home_state_reset()

    def _mark_plc_sensors_seen(self):
        import time
        self.plc_sensor_data_seen = True
        self._plc_sensor_seen_at = time.monotonic()

    def _mark_plc_sensors_unknown(self):
        """Readings are unknown again — the board has no device data.

        The stored bits are left alone rather than zeroed: plc_sensors_known()
        gates every consumer, so there is nothing to gain from overwriting
        them and something to lose if a later reader forgets the gate.
        """
        was = getattr(self, "plc_sensor_data_seen", False)
        self.plc_sensor_data_seen = False
        self._plc_sensor_seen_at = None
        if getattr(self, "plc_sensor_lamps", None):
            self._refresh_plc_sensor_lamps()
        if was:
            self.log("PLC sensor readings are UNKNOWN — the board is not getting "
                     "device data. The lamps show NO DATA, not CLEAR. Send PLC_TEST "
                     "to the board to find out why.", tag="error")

    def _set_plc_sensor(self, bit: str, covered: bool):
        state = getattr(self, "plc_sensor_state", None)
        if state is None or bit not in state:
            return
        # All four sensors are wired and working now. Nothing is filtered
        # out here, and nothing is latched: enforcement happens in P2P
        # (_sensor_violation) and jog only warns.
        state[bit] = bool(covered)
        self._mark_plc_sensors_seen()
        if getattr(self, "plc_sensor_lamps", None):
            self._refresh_plc_sensor_lamps()

    def plc_sensor_covered_for_jog(self, command: str):
        """The bit a jog command would drive INTO, or None.

        Jog is NOT blocked by it — see warn_if_jogging_into_sensor(). The
        board does not block it either; both only warn.
        """
        state = getattr(self, "plc_sensor_state", {})
        for bit, _label, _axis, cmd, _end in PLC_SENSOR_PANEL:
            if cmd == command and state.get(bit, False):
                return bit
        return None

    def warn_if_jogging_into_sensor(self, command: str):
        """Warns, and lets the jog proceed.

        Jog is a dead-man control: it moves only while held, the operator is
        watching, and jogging is how you come OFF a tripped sensor. Blocking
        it would also risk pinning the machine on its own switch. P2P is
        where the sensors are enforced, because a program runs unattended.
        """
        bit = self.plc_sensor_covered_for_jog(command)
        if bit is None:
            return
        label = next((l for b, l, _a, _c, _e in PLC_SENSOR_PANEL if b == bit), bit)
        self.log(f"⚠ {command} is driving INTO {bit} ({label.strip()}), which is "
                 f"covered. Jog is not blocked — watch the machine.", tag="warn")

    def _adopt_home_state_reset(self):
        """The board zeroed its counters because M5+M6 latched. Mirror it."""
        self.current_joints = [Z_HOME_MM, ROT_HOME_DEG, ARM_HOME_DEG, ARM_HOME_DEG]
        self.is_homed = True
        self._update_jog_readout()
        self._update_p2p_telemetry(*self.current_joints, pct=0)
        self._refresh_jog_status()
        self._invalidate_loaded_program(
            reason="Coordinates reset at HOME state — LOAD again before RUN.")
        self.log("HOME STATE (M5 + M6 covered, M7/M8 clear) — coordinates reset to "
                 "d1=0.00 mm, RM=0.00°, A1M=A2M=0.00 motor°.")
        self.status_var.set("AT HOME — coordinates reset from the PLC sensors.")

    def _board_reports_motor_degrees(self, text: str) -> bool:
        """True when the board is new enough to report the elbow in MOTOR
        degrees, i.e. its telemetry carries the derived FOLD1/FOLD2 fields.

        A pre-v9.2 board reports the elbow in the old frame, where the
        retracted home pose read 60. Adopting that makes the A1M/A2M
        readout jump to 60 and stay there however the machine is jogged —
        which looks like a bug in this GUI and is actually stale firmware.
        Warned once per connection rather than on every 50 ms telemetry
        line.
        """
        if FOLD_FIELDS_RE.search(text):
            return True
        if not getattr(self, "_warned_old_elbow_frame", False):
            self._warned_old_elbow_frame = True
            self.log(
                "The board is running firmware older than v9.2: its telemetry has "
                "no FOLD1/FOLD2 fields, so its A1M/A2M numbers are in the OLD elbow "
                "frame (60 at home). Those two values are being IGNORED — re-flash "
                "RobotMotionController_v9_ClearCore to get motor degrees. RM and ZM "
                "are unaffected.", tag="error")
            self.status_var.set("OLD FIRMWARE — re-flash the board; elbow angles ignored.")
        return False

    def _on_home_complete(self, simulated: bool):
        """A finished HOME resets the coordinate system to the standard
        home pose, on BOTH sides.

        The board zeroes its four step counters (see finishHoming). This
        used to reset the GUI's own copies only in the SIMULATED case, so
        after a real home the readouts kept whatever they happened to hold
        — usually the pose the machine was in when HOME was pressed. Every
        later jog and every offline preview then counted from a reference
        the machine did not share, and the first absolute move went to the
        wrong place. The reset is unconditional now.

        Standard home is, by definition, zero on all four axes:
            d1  = 0 mm        lift at the bottom of its stroke
            RM  = 0 deg       turntable centred
            A1M = A2M = 0     MOTOR degrees from home -> fold 0 deg,
                              R = 133.2 mm, fully retracted
        """
        self.sim_rot = ROT_HOME_DEG
        self.sim_a1 = ARM_HOME_DEG
        self.sim_a2 = ARM_HOME_DEG
        self.sim_z = Z_HOME_MM
        self.rot_limit = {k: False for k in self.rot_limit}
        self.z_limit = {k: False for k in self.z_limit}
        self._update_jog_readout()
        # The P2P side keeps its own copy of the pose, and it has to agree
        # or the progress bar and the Cartesian readout would be computed
        # from a stale start point.
        self.current_joints = [Z_HOME_MM, ROT_HOME_DEG, ARM_HOME_DEG, ARM_HOME_DEG]
        self._update_p2p_telemetry(Z_HOME_MM, ROT_HOME_DEG, ARM_HOME_DEG,
                                   ARM_HOME_DEG, pct=0)
        # A loaded program was solved against the OLD reference, so it must
        # not be runnable against the new one.
        self._invalidate_loaded_program(
            reason="Coordinates reset by HOME — LOAD again before RUN.")
        self.log("Coordinates reset to standard home: d1=0.00 mm, RM=0.00°, "
                 "A1M=0.00 motor°, A2M=0.00 motor° (fold 0.00°, R = 133.2 mm).")
        # A completed home is what makes the joint positions meaningful,
        # so this is the moment soft limits become enforceable.
        self.is_homed = True
        self.is_homing = False
        self._set_motion_locked(False)
        self.jog_dot.itemconfig(self._jog_dot_id, fill=TEXT_MUTED)
        self.jog_status_var.set("Idle — hold a key or button to jog")
        suffix = " (simulated)" if simulated else ""
        self.status_var.set(f"READY — Home position reached{suffix}.")
        self.log(f"Homing complete{suffix}.")

    def _on_reset_position_complete(self, simulated: bool):
        """RESET_POSITION drove to (0,0,0,0) under the board's own motor
        control — not a PLC handshake, so unlike _on_home_complete() this
        does NOT set is_homed (no reference was re-anchored, the board's
        existing zero didn't change) and does NOT invalidate the loaded
        P2P program (the origin it was solved against is unchanged).
        """
        self.sim_rot = ROT_HOME_DEG
        self.sim_a1 = ARM_HOME_DEG
        self.sim_a2 = ARM_HOME_DEG
        self.sim_z = Z_HOME_MM
        self._update_jog_readout()
        self.current_joints = [Z_HOME_MM, ROT_HOME_DEG, ARM_HOME_DEG, ARM_HOME_DEG]
        self._update_p2p_telemetry(Z_HOME_MM, ROT_HOME_DEG, ARM_HOME_DEG,
                                   ARM_HOME_DEG, pct=0)
        self._set_motion_locked(False)
        self.jog_dot.itemconfig(self._jog_dot_id, fill=TEXT_MUTED)
        self.jog_status_var.set("Idle — hold a key or button to jog")
        suffix = " (simulated)" if simulated else ""
        self.status_var.set(f"READY — Position reset to (0,0,0,0){suffix}.")
        self.log(f"RESET_POSITION complete{suffix} — the board drove to (0,0,0,0) "
                 "under its own motor control. No PLC handshake was used.")
