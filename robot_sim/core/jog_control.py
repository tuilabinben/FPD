"""Joystick/jog motion: dead-man axes, limit-sensor locks, boost, and the
software-only motion simulation used when no hardware is confirmed."""

from ..config import (
    ARM_MOTOR_RPM_MAX,
    ARM_SIM_MAX_DEG,
    ARM_SIM_MIN_DEG,
    BOOST_LEVELS,
    ARM_GEAR_RATIO,
    JOG_ARM_AXES,
    JOG_HEARTBEAT_MS,
    JOG_KEYCAPS,
    JOG_LINK_PROMOTION,
    JOG_SIM_TICK_MS,
    JOG_STOP_COMMAND,
    LIMITS_ENABLED_KEY,
    LIMIT_ENFORCE_BY_AXIS,
    LIMIT_OPPOSITE,
    ROT_VEL_MAX_DEG_S,
    Z_VEL_MAX_MM_S,
    arm_motor_rpm,
    rot_speed_deg_s,
    z_speed_mm_s,
)
from ..kinematics import fold_angle_from_motor_deg, motor_deg_to_reach
from ..theme import ACCENT_MINT, ACCENT_ORANGE, SURFACE, INK_DARK, TEXT_LIGHT, TEXT_MUTED

IDLE_STATUS = "Idle — hold a key or button to jog"


class JogControlMixin:
    # ── start / stop one axis ────────────────────────────────────────
    def _resolve_jog_command(self, command):
        """Applies the LINK toggle. With LINK on, a press on either arm pad
        is promoted to the both-arms command so the elbows stay in step."""
        if self.arms_linked:
            return JOG_LINK_PROMOTION.get(command, command)
        return command

    def jog_start(self, command):
        command = self._resolve_jog_command(command)

        # The pads are greyed out during RUN/HOME, but the keyboard used to
        # bypass that entirely and could start a jog mid-program. Guard the
        # logic itself, not just the widgets.
        if self.motion_locked:
            self.log(f"{command} ignored — a program is running and the jog axes are "
                     f"locked.", tag="warn")
            return
        if command in self.jog_active:
            return
        if self._is_limited(command):
            self.log(f"{command} blocked — the optical limit sensor is triggered. "
                     f"Jog the opposite way to come off it.", tag="warn")
            return

        # Two commands must never drive the same elbow at once — e.g.
        # A1_FWD while ARM_BACK is held would send the board contradictory
        # directions for AM1. Drop the conflicting axis first.
        self._release_conflicting_arm_axes(command)

        # A covered PLC sensor WARNS here and lets the jog through. It is
        # enforced in P2P instead — see warn_if_jogging_into_sensor().
        self.warn_if_jogging_into_sensor(command)

        self._clear_limit_if_opposite(command)
        self.jog_active.add(command)
        self.send(command)
        self._warn_unreferenced_once()
        self._refresh_jog_status()

        # Keep the board's dead-man watchdog fed for as long as an axis is
        # held. Without this the firmware stops the axis after
        # JOG_WATCHDOG_MS — which is exactly what we want if the GUI dies.
        self._start_jog_heartbeat()

        if not self._hardware_live() and self._jog_sim_job is None:
            self._schedule("_jog_sim_job", JOG_SIM_TICK_MS, self._jog_sim_tick)

    def _release_conflicting_arm_axes(self, command):
        spec = JOG_ARM_AXES.get(command)
        if spec is None:
            return
        arms_wanted = set(spec[0])
        for active in list(self.jog_active):
            other = JOG_ARM_AXES.get(active)
            if other and arms_wanted & set(other[0]):
                self.jog_active.discard(active)
                self.send(JOG_STOP_COMMAND.get(active, "ARM_STOP"))
                if active in self.jog_pads:
                    self.jog_pads[active].key_deactivate()

    def jog_stop(self, start_cmd, stop_cmd=None):
        # A press may have been promoted by LINK, so release whichever
        # command actually went out — otherwise the linked axis latches on.
        for candidate in (start_cmd, self._resolve_jog_command(start_cmd)):
            if candidate in self.jog_active:
                self.jog_active.discard(candidate)
                self.send(JOG_STOP_COMMAND.get(candidate, stop_cmd or "STOP"))
        self._refresh_jog_status()

    def _release_all_jog_axes(self, send_stop=True):
        """Single place that clears every active jog axis — previously this
        was copy-pasted (slightly differently) in four methods."""
        for cmd in list(self.jog_active):
            if send_stop:
                self.send(JOG_STOP_COMMAND.get(cmd, "STOP"))
        self.jog_active.clear()
        for pad in self.jog_pads.values():
            pad.key_deactivate()
        self._cancel_jobs("_jog_sim_job", "_jog_hb_job")
        self._refresh_jog_status()

    def _hardware_live(self):
        return bool(self.is_connected and self.hw_confirmed)

    # ── dead-man keep-alive ──────────────────────────────────────────
    def _start_jog_heartbeat(self):
        if self._jog_hb_job is None:
            self._jog_heartbeat()

    def _jog_heartbeat(self):
        self._jog_hb_job = None
        if not self.jog_active:
            return                      # nothing held: let the board time out
        self.send("JOG_HB", log_tx=False)
        self._schedule("_jog_hb_job", JOG_HEARTBEAT_MS, self._jog_heartbeat)

    # ── status strip ─────────────────────────────────────────────────
    def _refresh_jog_status(self):
        # Boundaries apply with or without a reference. What a missing
        # reference costs is the meaning of the NUMBERS, not the protection.
        suffix = "" if self.is_homed else "   [NO REFERENCE — positions are relative]"
        if self.jog_active:
            self.jog_dot.itemconfig(self._jog_dot_id, fill=ACCENT_MINT)
            self.jog_status_var.set("  ".join(sorted(self.jog_active)) + suffix)
        else:
            self.jog_dot.itemconfig(self._jog_dot_id, fill=TEXT_MUTED)
            self.jog_status_var.set(IDLE_STATUS + suffix)

    def _warn_unreferenced_once(self):
        if self.is_homed or getattr(self, "_unref_warned", False):
            return
        self._unref_warned = True
        self.log("No reference yet. Your taught boundaries ARE applied — they were "
                 "captured against these same counters — so jog is protected. What "
                 "is missing is any absolute meaning for the numbers: run HOME, or "
                 "RESET COORDINATES, before commanding a P2P move.", tag="warn")

    def _update_jog_readout(self):
        """sim_a1 / sim_a2 are MOTOR degrees — the raw rotation the board
        counts. The frog-leg angle and the reach are derived, and shown
        alongside rather than instead of it: the motor figure is the exact
        one, and the frog-leg figure is the one the operator thinks in.
        Displaying only the motor number was the old bug (it was labelled
        as though it were the arm angle); displaying only the frog-leg
        number would hide that it rests on a ratio taken from the model."""
        
        # ── skip redundant UI updates ──
        current_pose = (self.sim_rot, self.sim_a1, self.sim_a2, self.sim_z)
        if getattr(self, "_last_jog_readout_pose", None) == current_pose:
            return
        self._last_jog_readout_pose = current_pose
        
        self.rot_pos_v.set(f"{self.sim_rot:.2f} deg")
        self.a1_pos_v.set(f"{self.sim_a1:.2f} motor deg")
        self.a2_pos_v.set(f"{self.sim_a2:.2f} motor deg")
        self.jz_pos_v.set(f"{self.sim_z:.2f} mm")
        self.a1_reach_v.set(
            f"fold {fold_angle_from_motor_deg(self.sim_a1):.2f}° · "
            f"R1 = {motor_deg_to_reach(self.sim_a1):.1f} mm")
        self.a2_reach_v.set(
            f"fold {fold_angle_from_motor_deg(self.sim_a2):.2f}° · "
            f"R2 = {motor_deg_to_reach(self.sim_a2):.1f} mm")
        # The P2P panel reads the SAME pose, so it has to repaint too —
        # otherwise switching mode after a jog showed the stale numbers P2P
        # last wrote, and the operator had two different answers on screen
        # for where the machine was.
        self._refresh_p2p_pose_readout()

    # ── optical limit sensors ────────────────────────────────────────
    def _is_limited(self, direction):
        return bool(self.rot_limit.get(direction) or self.z_limit.get(direction))

    def _on_limit_triggered(self, direction):
        """Board reported [LIMIT] <direction>, or the software simulation
        reached its own simulated bound."""
        if direction in self.rot_limit:
            self.rot_limit[direction] = True
        elif direction in self.z_limit:
            self.z_limit[direction] = True
        elif direction in JOG_ARM_AXES:
            # An arm limit. Unlike the optical sensors on RM and ZM there
            # is nothing to latch: the elbow limit is a soft one, and the
            # opposite direction is always immediately available. Releasing
            # the axis below is the whole response.
            pass
        else:
            self.log(f"[LIMIT] {direction} — unrecognised axis.", tag="warn")
            return

        if direction in self.jog_active:
            self.jog_active.discard(direction)
            # Actually tell the board to stop this axis. The old code only
            # removed it from the local set, so a hardware-reported limit
            # left the axis command latched on the board.
            self.send(JOG_STOP_COMMAND.get(direction, "STOP"))
        if direction in self.jog_pads:
            self.jog_pads[direction].key_deactivate()
        self._refresh_jog_status()
        self.log(f"[LIMIT] {direction} — optical sensor reached; this direction is "
                 f"now blocked.", tag="warn")

    def _clear_limit_if_opposite(self, direction):
        opp = LIMIT_OPPOSITE.get(direction)
        if opp is None:
            return
        if opp in self.rot_limit:
            self.rot_limit[opp] = False
        elif opp in self.z_limit:
            self.z_limit[opp] = False

    # ── boost (x1 / x1.5 / x2) ───────────────────────────────────────
    def cycle_boost(self):
        self.boost_index = (self.boost_index + 1) % len(BOOST_LEVELS)
        mult = BOOST_LEVELS[self.boost_index]
        self.send(f"SET_BOOST:{mult}")
        label = "OFF" if mult == 1.0 else f"x{mult:g}"
        active = mult != 1.0
        self.boost_btn.set_config(f"BOOST: {label}",
                                  ACCENT_ORANGE if active else SURFACE,
                                  icon="⚡",
                                  fg_color=INK_DARK if active else TEXT_LIGHT)
        self.log(f"Boost set to x{mult:g}.")

    _cycle_boost = cycle_boost

    def _speed_scale(self):
        """Active boost multiplier. Axis speeds themselves now come from the
        Settings profile in real units, so there is nothing else to scale."""
        return BOOST_LEVELS[self.boost_index]

    def _axis_speeds(self):
        """(rot deg/s, arm deg/s, z mm/s) for the simulation.

        Derived from the universal RPM and the per-motor percentages
        through the same arithmetic — and the same ceilings — the firmware
        uses, so offline motion is a faithful preview of what the machine
        will actually do rather than an optimistic one.
        """
        s = self.settings
        master = s["master_rpm"]
        # The arm is bounded in MOTOR RPM, so the cap is applied there and
        # the result converted — not to a °/s figure derived from a gear
        # ratio nobody has measured yet.
        arm_rpm = min(arm_motor_rpm(master, s["arm_pct"]), ARM_MOTOR_RPM_MAX)
        return (
            min(rot_speed_deg_s(master, s["rot_pct"]), ROT_VEL_MAX_DEG_S),
            # MOTOR degrees per second: the elbows are simulated in motor
            # degrees, matching what the board reports and what the taught
            # limits are stored in. No gear ratio is involved, so this
            # figure is exact.
            arm_rpm * 360.0 / 60.0,
            min(z_speed_mm_s(master, s["z_pct"]), Z_VEL_MAX_MM_S),
        )

    # ── software-only jog simulation ─────────────────────────────────
    def _jog_sim_tick(self):
        self._jog_sim_job = None
        if not self.jog_active or self._hardware_live():
            return

        dt = JOG_SIM_TICK_MS / 1000.0
        scale = self._speed_scale()
        rot_v, arm_v, z_v = self._axis_speeds()

        # Where each axis was BEFORE this tick. The clamp needs it to tell
        # "crossed the boundary just now" from "started outside it and is
        # jogging back in", which are the two halves of the escape rule.
        prev_rot, prev_z = self.sim_rot, self.sim_z
        self._prev_arm = {"A1M": self.sim_a1, "A2M": self.sim_a2}

        if "ROT_CW" in self.jog_active:
            self.sim_rot += rot_v * dt * scale
        if "ROT_CCW" in self.jog_active:
            self.sim_rot -= rot_v * dt * scale

        # The elbow angle is rotation from home: 0° retracted, 120°
        # straight, and reach GROWS with it. Extending therefore
        # INCREASES it.
        # A1M and A2M are separate motors, so each integrates on its own.
        step = arm_v * dt * scale
        for command, (arms, sign) in JOG_ARM_AXES.items():
            if command not in self.jog_active:
                continue
            if "A1M" in arms:
                self.sim_a1 += sign * step
            if "A2M" in arms:
                self.sim_a2 += sign * step

        if "Z_UP" in self.jog_active:
            self.sim_z += z_v * dt * scale
        if "Z_DOWN" in self.jog_active:
            self.sim_z -= z_v * dt * scale

        rot_lo, rot_hi = self._axis_bounds(*self._limit_pair("rot"), axis="ROT")
        self.sim_rot, hit = self._apply_axis_limit(self.sim_rot, prev_rot,
                                                   rot_lo, rot_hi)
        if hit:
            self._on_limit_triggered("ROT_CW" if hit == "high" else "ROT_CCW")

        z_lo, z_hi = self._axis_bounds(*self._limit_pair("z"), axis="Z")
        self.sim_z, hit = self._apply_axis_limit(self.sim_z, prev_z, z_lo, z_hi)
        if hit:
            self._on_limit_triggered("Z_UP" if hit == "high" else "Z_DOWN")

        # Each elbow is clamped independently. Hitting A1M's stop must not
        # halt A2M — they are separate linkages on separate motors.
        self._clamp_arm("A1M")
        self._clamp_arm("A2M")

        self._update_jog_readout()

        if self.jog_active:
            self._schedule("_jog_sim_job", JOG_SIM_TICK_MS, self._jog_sim_tick)

    def _limit_pair(self, axis):
        """(lower, upper) working limit for an axis.

        Reads the live settings rather than the module constants, so a
        limit edited in Settings takes effect on the offline simulation
        the same instant it takes effect on the board — two copies of a
        limit that can disagree is the failure this avoids.

        The elbow pair is TAUGHT rather than typed (see LIMIT_CAPTURE_ONLY
        in config) and is stored UNORDERED — the operator jogs to one stop
        and presses SET HERE, then the other, and which they reached first
        is not something they should have to keep straight. Sorting
        happens here, at the point of use, so both taught numbers survive
        in the settings exactly as captured. This mirrors armBand() in the
        firmware; the two must agree or the simulation and the board would
        clamp at different places.
        """
        s = self.settings
        try:
            a, b = s[f"lim_{axis}_min"], s[f"lim_{axis}_max"]
            return (a, b) if a <= b else (b, a)
        except KeyError:
            # A settings file written before this axis existed. Fall back
            # to the structural envelope rather than raising in the middle
            # of a jog tick.
            return ARM_SIM_MIN_DEG, ARM_SIM_MAX_DEG

    def _axis_bounds(self, lo, hi, axis=None):
        """The band actually applied to an axis.

        A TAUGHT BOUNDARY APPLIES IMMEDIATELY. It does not wait for HOME or
        RESET COORDINATES.

        This used to widen the band by a full travel either side while
        `is_homed` was false, on the argument that the counters are
        meaningless without a reference. That argument does not survive
        contact with how boundaries are actually set: you jog to the stop
        and press SET HERE, so the boundary is captured against the SAME
        counters that are being compared to it. It is meaningful in exactly
        the frame it was taught in, reference or not. The widening meant a
        limit somebody had just taught at -300 let the axis run to -427,
        which is the bug this replaced.

        `axis` is "Z" / "ROT" / "A1" / "A2", checked against the per-axis
        enforcement switch and the master one, mirroring axisLimited() on
        the board. Two systems of record disagreeing about whether the
        machine is protected is the one thing worse than either answer.

        An axis that starts OUTSIDE its band is not trapped — see
        `_apply_axis_limit()` for the escape rule.
        """
        if axis is not None and not self._axis_enforced(axis):
            span = hi - lo
            return lo - span, hi + span
        return lo, hi

    @staticmethod
    def _apply_axis_limit(value, previous, lo, hi):
        """Clamps one axis, allowing it to escape a band it starts outside.

        Returns (value, "high"|"low"|None).

        Coming from inside, the axis stops ON the boundary — the ordinary
        case. Starting outside it — a boundary taught in a previous session,
        applied against a counter that powered up somewhere else — motion
        FURTHER out is refused and motion back toward the band is allowed.

        Without that second half, applying limits without a reference could
        pin an axis with no way to jog off, which is the exact failure the
        old widening was there to avoid. The escape rule solves it without
        giving up the protection.
        """
        if value > hi:
            if previous <= hi:
                return hi, "high"           # crossed out: stop on the line
            if value > previous:
                return previous, "high"     # already out, going further: freeze
            return value, None              # already out, coming back: allow
        if value < lo:
            if previous >= lo:
                return lo, "low"
            if value < previous:
                return previous, "low"
            return value, None
        return value, None

    def _axis_enforced(self, axis):
        """True when this axis's boundary is actually policing anything.

        An AND of the per-axis switch and the master one, in that order —
        the master must never re-arm an axis switched off on its own."""
        if not self.settings.get(LIMITS_ENABLED_KEY, True):
            return False
        return bool(self.settings.get(LIMIT_ENFORCE_BY_AXIS[axis], True))

    def _clamp_arm(self, arm):
        """Clamps one elbow to its travel and releases only the axes that
        are actually driving THAT elbow in the offending direction.

        Each arm is clamped against its OWN limits. Using one shared arm
        limit is how A2M used to be driven past its stop while A1M's angle
        was the one being checked."""
        angle = self.sim_a1 if arm == "A1M" else self.sim_a2
        is_a1 = arm == "A1M"
        lo, hi = self._axis_bounds(*self._limit_pair("a1" if is_a1 else "a2"),
                                   axis="A1" if is_a1 else "A2")

        # Same escape rule as the other axes: an elbow that powers up
        # outside a boundary taught in an earlier session can still be
        # jogged back into range.
        previous = getattr(self, "_prev_arm", {}).get(arm, angle)
        angle, hit = self._apply_axis_limit(angle, previous, lo, hi)
        if hit == "high":
            sign = +1
            desc = (f"fully extended ({angle:.1f} motor°, fold "
                    f"{fold_angle_from_motor_deg(angle):.1f}°, "
                    f"R = {motor_deg_to_reach(angle):.1f} mm)")
        elif hit == "low":
            sign = -1
            desc = (f"fully retracted ({angle:.1f} motor°, fold "
                    f"{fold_angle_from_motor_deg(angle):.1f}°, "
                    f"R = {motor_deg_to_reach(angle):.1f} mm)")
        else:
            if arm == "A1M":
                self.sim_a1 = angle
            else:
                self.sim_a2 = angle
            return

        if arm == "A1M":
            self.sim_a1 = angle
        else:
            self.sim_a2 = angle

        for command in list(self.jog_active):
            spec = JOG_ARM_AXES.get(command)
            if spec and arm in spec[0] and spec[1] == sign:
                self.jog_active.discard(command)
                self.send(JOG_STOP_COMMAND.get(command, "ARM_STOP"))
                if command in self.jog_pads:
                    self.jog_pads[command].key_deactivate()
                self._refresh_jog_status()
                self.log(f"{command} stopped — {arm} is {desc}.", tag="warn")

    # ── LINK toggle ──────────────────────────────────────────────────
    def toggle_arm_link(self):
        """LINK on = both elbows follow one control (the v8 gesture that was
        tested on hardware). LINK off = A1M and A2M are fully independent."""
        # Never flip modes with an axis latched — the in-flight command
        # would be released under a different resolution than it started.
        self._release_all_jog_axes()
        self.arms_linked = not self.arms_linked
        active = self.arms_linked
        self.link_btn.set_config(
            f"LINK: {'ON' if active else 'OFF'}",
            ACCENT_ORANGE if active else SURFACE,
            icon="🔗", fg_color=INK_DARK if active else TEXT_LIGHT)
        a1_keys = f"{JOG_KEYCAPS['A1_FWD']}/{JOG_KEYCAPS['A1_BACK']}"
        a2_keys = f"{JOG_KEYCAPS['A2_FWD']}/{JOG_KEYCAPS['A2_BACK']}"
        self.log(f"LINK on — {a1_keys} and {a2_keys} both drive BOTH arms together."
                 if active else
                 f"LINK off — A1M ({a1_keys}) and A2M ({a2_keys}) move independently.")
