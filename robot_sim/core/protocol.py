"""Parsing of the lines the ClearCore board sends back."""

import re

import re as _re

from ..config import (
    ARM_HOME_DEG,
    PLC_LED_STATES,
    PLC_SENSOR_BOTH_ENDS,
    PLC_SENSOR_JOG_CMD,
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

# v9 reports both elbows separately. Older single-ARM form still accepted
# so a board running v8 firmware keeps working.
JOG_RE_V9 = re.compile(
    r"ROT:\s*(-?[\d.]+)\s*deg\s*\|\s*A1M:\s*(-?[\d.]+)\s*deg\s*\|\s*"
    r"A2M:\s*(-?[\d.]+)\s*deg\s*\|\s*Z:\s*(-?[\d.]+)\s*mm",
    re.IGNORECASE,
)
#: v9.2+ appends derived frog-leg angle and reach. ABSENCE is how older
#: board detected: pre-v9.2 board reports elbow in old frame (printed 60 at
#: home) — adopting its numbers would jump readout to 60 and stick, looks
#: exactly like a GUI fault. Values refused instead, loudly.
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
            # Joint-space telemetry: board reports D1/ROT/A1M/A2M directly,
            # no longer does any IK/FK itself.
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
            # Keeps simulation in step w/ real machine so later offline jog
            # continues from true position.
            m = JOG_RE_V9.search(text)
            if m:
                rot, a1, a2, z = (float(v) for v in m.groups())
                if not self._board_reports_motor_degrees(text):
                    # Still take the two axes whose frame did NOT change —
                    # keeps a live RM/ZM readout.
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

        # Completion ("[RUN] TARGET REACHED") must fall through to dedicated
        # check below, not be swallowed here as generic progress — it used
        # to be, so real-hardware completion never unlocked the GUI.
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
            # Board zeroed its counters, so GUI's own reference now valid
            # too — whether reset came from this app or someone typing
            # RESET_COORD into a terminal.
            self.is_homed = True
            self._refresh_jog_status()
            self.log("ClearCore reset its coordinates — soft limits are now active.")
            return

        if upper.startswith("[LIMIT_SET]"):
            self.log("Board confirmed limit: " + text.split("]", 1)[-1].strip())
            return

        # "[LIMITS]"/"[LIMITS_INFO]" are board's own view of envelope.
        # Logged, not parsed back into settings: GUI is system of record,
        # re-sends on connect — quietly adopting whatever board says would
        # let a stale board overwrite operator's saved setup.
        if upper.startswith("[LIMITS"):
            self.log(text, tag="rx")
            return

        if upper.startswith("[SPEED]") or upper.startswith("[PROFILE]"):
            self.log(text, tag="rx")
            return

        if upper.startswith("[LIMIT]"):
            # e.g. "[LIMIT] ROT_CW" / "[LIMIT] Z_UP" / "[LIMIT] A1_FWD — ..."
            # v9.1 appends an explanation after an em dash; take only axis
            # token or lookup in _on_limit_triggered fails.
            payload = text.split("]", 1)[-1].strip()
            self._on_limit_triggered(payload.split()[0].upper() if payload else "")
            return

        # Every [SCAN_*] reply. RX pump already logged the line.
        if upper.startswith("[SCAN"):
            self._on_scan_line(text)
            return

        if upper.startswith("[PLC_HOME]"):
            self._on_plc_home_line(text)
            return

        if upper.startswith("[PLC_STATE]"):
            # Polled every heartbeat tick, NOT logged — 20 lines/min of
            # unchanged status would bury everything else. Only a CHANGE
            # of state worth a line.
            self._on_plc_state(text)
            return

        if upper.startswith("[PLC]"):
            self.log(text, tag="rx")
            if "TCP SOCKET OPEN" in upper or "CONNECTED TO" in upper:
                # A socket is not a conversation. Reporting this as CONNECTED
                # made the lamp flap: socket genuinely opens/closes every few
                # seconds when PLC answers nothing, lamp followed it. At
                # best "no reply yet"; only a landed device read promotes
                # to CONNECTED.
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

        if upper.startswith("[ERROR]"):
            # If it was the scan, drop the GUI's run too, or START stays
            # greyed out with nothing running. RX pump logged it already.
            self.scan_refused_by_board()
            return

        if upper.startswith("[HOME]"):
            if "COMPLETE" in upper:
                self._on_home_complete(simulated="SIMULATED" in upper)
                return
            if "FAILED" in upper or "TIMEOUT" in upper:
                # PLC never answered. Unlock GUI — else every control
                # except STOP stays disabled forever.
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

        # Own branch, placed BEFORE generic "TARGET REACHED" catch-all
        # below — else treated as P2P RUN completion (wrong status text,
        # wrong side effects), same shadowing mistake "[RUN] TARGET
        # REACHED" used to hit.
        if upper.startswith("[RESET_POSITION]"):
            if "TARGET REACHED" in upper:
                self._on_reset_position_complete(simulated=False)
            else:
                self.status_var.set(f"RESETTING POSITION — {text.split(']', 1)[-1].strip()}")
            return

        if upper.startswith("[WATCHDOG]"):
            # Board stopped jog itself — keep-alive stopped arriving. Clear
            # local axes so UI matches machine.
            self._release_all_jog_axes(send_stop=False)
            self.status_var.set("JOG STOPPED — board watchdog tripped.")
            self.log("The board stopped the jog itself — no JOG_HB keep-alive arrived. "
                     "Check CPU load and the serial link.", tag="warn")
            return

        # v9.2 emits English. Vietnamese strings a v8 board sends still
        # accepted so an un-flashed board keeps working — same back-compat
        # rule as JOG_RE_LEGACY above.
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

    _PLC_STATE_RE = _re.compile(r"link=(\w+)\s+socket=(\w+)", _re.IGNORECASE)
    #: data=NONE|STALE|OK — whether DEVICE READS are landing. Lamp reports
    #: this, not the socket: a PLC that accepts connections but answers
    #: nothing made the socket go up and down every few seconds, and a
    #: socket-driven lamp flapped green/red with it.
    _PLC_DATA_RE = _re.compile(r"data=(NONE|STALE|OK)", _re.IGNORECASE)
    #: conn=<succeeded>/<attempted>. A socket that opened even once proves
    #: cable and address are fine, so a currently-closed socket is NOT
    #: "unreachable" — it's a conversation not happening yet. W/o this the
    #: lamp alternated NO REPLY / UNREACHABLE as the socket cycled, same
    #: flap one level down.
    _PLC_CONN_RE = _re.compile(r"conn=(\d+)/(\d+)", _re.IGNORECASE)

    def _on_plc_state(self, text: str):
        """Drives the PLC lamp from the board's [PLC_STATE] reply.

        Four states, different actions: DISABLED = SET_PLC_LINK:0, on
        purpose — checked first so it can never be mistaken for a fault.
        UNREACHABLE = cable/address problem. NO REPLY = socket opened but
        Ethernet module not answering device reads (almost always MC
        protocol not enabled on port). CONNECTED = HOME can actually work.
        """
        bits = self._PLC_HOME_BITS_RE.search(text)
        if bits:
            field = bits.group(1)
            if "?" in field or "NO DEVICE DATA" in text.upper():
                # Board telling us it doesn't know. Do NOT write False into
                # sensor state — exactly how a dead link came to look like
                # four clear sensors.
                self._mark_plc_sensors_unknown()
            else:
                # Authoritative, polled: an edge message can be missed, this
                # can't. The field is ordered by AXIS (Z, then R, then A2),
                # and the devices are NOT in tidy numeric order: M32 is ZM's
                # and M30 is A2M's, measured on the machine. Decoding this
                # positionally as M30/M31/M32 put ZM's lamp on A2M's switch.
                for bit, ch in zip(("M32", "M31", "M30"), field):
                    self._set_plc_sensor(bit, ch == "1")
                self._read_plc_limit_ends(text)
                self._mark_plc_sensors_seen()
                self._latch_home_state_if_new()

        m = self._PLC_STATE_RE.search(text)
        if not m:
            self.log(text, tag="rx")
            return
        link, socket = m.group(1).upper(), m.group(2).upper()
        if link == "DISABLED":
            self._set_plc_led("disabled", detail=text)
            return
        data = self._PLC_DATA_RE.search(text)
        if data:
            # Preferred: board says whether reads are landing.
            #   OK    -> conversation works, what HOME needs
            #   STALE -> worked, then stopped
            #   NONE  -> socket may open/close all day; no data ever
            got = data.group(1).upper()
            conn = self._PLC_CONN_RE.search(text)
            ever_opened = bool(conn) and int(conn.group(1)) > 0
            if got == "OK":
                state = "connected"
            elif got == "STALE":
                state = "no_reply"
            elif ever_opened:
                # Endpoint answers TCP but not device reads. Steady even
                # while the socket itself opens/closes on each timeout.
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

        Edge-triggered: condition stays true while machine sits at home,
        re-zeroing every 3 s poll would quietly eat any real motion away
        from the reference.
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
        """Serial or PLC link gone: sensors unknown, not clear."""
        self._set_plc_led("unknown")
        self._mark_plc_sensors_unknown()

    def _set_plc_led(self, state: str, detail: str = ""):
        """Sets the lamp, logs ONLY on a change of state."""
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

    # Accepts "????" as well as four bits: board sends that when it has no
    # device data — the difference between "not covered" and "nobody
    # knows".
    _PLC_HOME_BITS_RE = _re.compile(
        r"limit\s+Z/R/A2\s*=\s*([01?]{3})", _re.IGNORECASE)
    #: end Z/R/A2=-+- — which end each switch is refusing right now. Fixed
    #: for ZM and RM; A2M's switch is wired at both ends and the board is
    #: the only side that can tell them apart, from the direction of travel
    #: when the bit went on. "?" while it has no device data.
    _PLC_LIMIT_END_RE = _re.compile(
        r"end\s+Z/R/A2\s*=\s*([-+?]{3})", _re.IGNORECASE)
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
            # Board latched home state and zeroed its counters — GUI's copy
            # of pose has to follow or the two disagree from this instant.
            self._adopt_home_state_reset()

    def _read_plc_limit_ends(self, text: str):
        """Adopts the board's "end Z/R/A2=" field.

        Only the board can answer this for a switch wired at both ends: it
        watches the rising edge against the direction the axis was going.
        A missing field means an older board, and the panel's HOME-side end
        stays -- which is what that firmware enforced anyway.
        """
        m = self._PLC_LIMIT_END_RE.search(text)
        if not m:
            return
        ends = getattr(self, "plc_sensor_end", None)
        if ends is None:
            return
        for bit, ch in zip(("M32", "M31", "M30"), m.group(1)):
            if bit not in ends or ch == "?":
                continue
            ends[bit] = 1 if ch == "+" else -1

    def _mark_plc_sensors_seen(self):
        import time
        self.plc_sensor_data_seen = True
        self._plc_sensor_seen_at = time.monotonic()

    def _mark_plc_sensors_unknown(self):
        """Readings are unknown again — the board has no device data.

        Stored bits left alone rather than zeroed: plc_sensors_known()
        gates every consumer, so nothing to gain overwriting them and
        something to lose if a later reader forgets the gate.
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
        # All four sensors wired and working now. Nothing filtered out
        # here, nothing latched: enforcement happens in P2P
        # (_sensor_violation), jog only warns.
        state[bit] = bool(covered)
        self._mark_plc_sensors_seen()
        if getattr(self, "plc_sensor_lamps", None):
            self._refresh_plc_sensor_lamps()

    def plc_sensor_covered_for_jog(self, command: str):
        """The bit a jog command would drive INTO, or None.

        Jog is NOT blocked by it — see warn_if_jogging_into_sensor().
        Board doesn't block it either; both only warn.
        """
        state = getattr(self, "plc_sensor_state", {})
        for bit, _label, axis, cmd, _end in PLC_SENSOR_PANEL:
            if not state.get(bit, False):
                continue
            # A both-ends switch refuses whichever end it caught, so the
            # command that drives INTO it is not fixed. Looking only at the
            # panel's cmd warned about A2_BACK while the arm was on the
            # FORWARD switch, and said nothing when it mattered.
            if bit in PLC_SENSOR_BOTH_ENDS:
                cmd = PLC_SENSOR_JOG_CMD.get((axis, self.plc_sensor_end_for(bit)), cmd)
            if cmd == command:
                return bit
        return None

    def warn_if_jogging_into_sensor(self, command: str):
        """Warns, and lets the jog proceed.

        Jog is a dead-man control: moves only while held, operator is
        watching, jogging is how you come OFF a tripped sensor. Blocking
        it would also risk pinning the machine on its own switch. P2P is
        where sensors are enforced, because a program runs unattended.
        """
        bit = self.plc_sensor_covered_for_jog(command)
        if bit is None:
            return
        label = next((l for b, l, _a, _c, _e in PLC_SENSOR_PANEL if b == bit), bit)
        end = "MAX" if self.plc_sensor_end_for(bit) > 0 else "MIN"
        self.log(f"⚠ {command} is driving INTO {bit} ({label.strip()}), which is "
                 f"covered at its {end} end. Jog is not blocked — watch the "
                 f"machine.", tag="warn")

    def _adopt_home_state_reset(self):
        """Board zeroed its counters because M30..M32 latched. Mirror it."""
        self.current_joints = [Z_HOME_MM, ROT_HOME_DEG, ARM_HOME_DEG, ARM_HOME_DEG]
        self.is_homed = True
        self._update_jog_readout()
        self._update_p2p_telemetry(*self.current_joints, pct=0)
        self._refresh_jog_status()
        self._invalidate_loaded_program(
            reason="Coordinates reset at HOME state — LOAD again before RUN.")
        self.log("HOME STATE (M30, M31 and M32 all covered) — coordinates reset "
                 "to d1=0.00 mm, RM=0.00°, A1M=A2M=0.00 motor°.")
        self.status_var.set("AT HOME — coordinates reset from the PLC limits.")

    def _board_reports_motor_degrees(self, text: str) -> bool:
        """True when the board is new enough to report the elbow in MOTOR
        degrees, i.e. its telemetry carries the derived FOLD1/FOLD2 fields.

        Pre-v9.2 board reports the elbow in the old frame, where the
        retracted home pose read 60. Adopting that makes the A1M/A2M
        readout jump to 60 and stay there however the machine is jogged —
        looks like a GUI bug, is actually stale firmware. Warned once per
        connection, not on every 50 ms telemetry line.
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

        Board zeroes its four step counters (see finishHoming). Used to
        reset the GUI's own copies only in the SIMULATED case, so after a
        real home the readouts kept whatever they happened to hold —
        usually the pose the machine was in when HOME was pressed. Every
        later jog and every offline preview then counted from a reference
        the machine did not share, and the first absolute move went to the
        wrong place. Reset is unconditional now.

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
        # P2P side keeps its own copy of the pose, has to agree or the
        # progress bar and Cartesian readout would compute from a stale
        # start point.
        self.current_joints = [Z_HOME_MM, ROT_HOME_DEG, ARM_HOME_DEG, ARM_HOME_DEG]
        self._update_p2p_telemetry(Z_HOME_MM, ROT_HOME_DEG, ARM_HOME_DEG,
                                   ARM_HOME_DEG, pct=0)
        # Loaded program was solved against the OLD reference, must not be
        # runnable against the new one.
        self._invalidate_loaded_program(
            reason="Coordinates reset by HOME — LOAD again before RUN.")
        self.log("Coordinates reset to standard home: d1=0.00 mm, RM=0.00°, "
                 "A1M=0.00 motor°, A2M=0.00 motor° (fold 0.00°, R = 133.2 mm).")
        # A completed home is what makes joint positions meaningful — this
        # is the moment soft limits become enforceable.
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
        does NOT set is_homed (no reference re-anchored, board's existing
        zero didn't change) and does NOT invalidate the loaded P2P program
        (origin it was solved against is unchanged).
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
