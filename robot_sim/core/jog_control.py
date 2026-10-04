"""Joystick/jog motion: dead-man axes, limit-sensor locks, boost, and
software-only motion simulation used when no hardware confirmed."""

import math

from ..config import (
    ARM_MOTOR_RPM_MAX,
    ARM_SIM_MAX_DEG,
    ARM_SIM_MIN_DEG,
    BOOST_LEVELS,
    D1_MAX_MM,
    D1_MIN_MM,
    DEFAULT_XYZ_JOG_MM_S,
    JOG_FRAMES,
    XYZ_JOG_ARMS,
    XYZ_JOG_AXES,
    XYZ_JOG_JOINT_HEADROOM,
    XYZ_JOG_JUMP_RATIO,
    XYZ_JOG_STOP,
    JOG_ARM_AXES,
    JOG_HEARTBEAT_MS,
    JOG_LINK_PROMOTION,
    JOG_SIM_TICK_MS,
    JOG_STOP_COMMAND,
    MASTER_RPM,
    LIMITS_ENABLED_KEY,
    LIMIT_ENFORCE_BY_AXIS,
    LIMIT_FIELDS,
    LIMIT_OPPOSITE,
    ROT_VEL_MAX_DEG_S,
    Z_VEL_MAX_MM_S,
    arm_motor_rpm,
    rot_speed_deg_s,
    z_speed_mm_s,
)
from ..kinematics import (
    base_angle_from_motor_deg,
    fold_angle_from_motor_deg,
    motor_deg_from_fold_angle,
    motor_deg_to_reach,
    solve_ik,
    z_abs_from_home,
)
from ..theme import ACCENT_MINT, ACCENT_ORANGE, SURFACE, INK_DARK, TEXT_LIGHT, TEXT_MUTED

IDLE_STATUS = "Idle — hold a key or button to jog"


class JogControlMixin:
    def _resolve_jog_command(self, command):
        """Applies LINK toggle. LINK on: press on either arm pad promoted
        to both-arms command so elbows stay in step."""
        if self.arms_linked:
            return JOG_LINK_PROMOTION.get(command, command)
        return command

    def jog_start(self, command):
        command = self._resolve_jog_command(command)

        # pads greyed out during RUN/HOME, but keyboard used to bypass
        # that and could start jog mid-program. Guard logic itself, not
        # just widgets.
        if self.motion_locked:
            self.log(f"{command} ignored — a program is running and the jog axes are "
                     f"locked.", tag="warn")
            return
        if command in self.jog_active:
            return
        # An XYZ key the board (or the simulation) refused stays refused
        # until it comes up: key auto-repeat would otherwise re-send it,
        # and be refused again, thirty times a second.
        if command in self._xjog_blocked:
            return
        if self._is_limited(command):
            self.log(f"{command} blocked — that axis is on its soft limit. "
                     f"Jog the opposite way to come off it.", tag="warn")
            return

        # two commands must never drive same elbow at once — e.g. A1_FWD
        # while ARM_BACK held would send board contradictory directions
        # for AM1. drop conflicting axis first.
        self._release_conflicting_arm_axes(command)

        # covered PLC sensor WARNS here, lets jog through. enforced in
        # P2P instead — see warn_if_jogging_into_sensor().
        self.warn_if_jogging_into_sensor(command)

        self._clear_limit_if_opposite(command)
        self.jog_active.add(command)
        if command in XYZ_JOG_AXES:
            self._send_xjog()           # one command carries all three signs
        else:
            self.send(command)
        self._warn_unreferenced_once()
        self._refresh_jog_status()

        # keep board's dead-man watchdog fed while axis held. w/o this
        # firmware stops axis after JOG_WATCHDOG_MS — wanted if GUI dies.
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
        # press may've been promoted by LINK, release whichever command
        # actually went out — else linked axis latches on
        self._xjog_blocked.discard(start_cmd)       # the key is up: it may try again
        for candidate in (start_cmd, self._resolve_jog_command(start_cmd)):
            if candidate in self.jog_active:
                self.jog_active.discard(candidate)
                if candidate in XYZ_JOG_AXES:
                    self._send_xjog()   # the vector with this key out of it
                else:
                    self.send(JOG_STOP_COMMAND.get(candidate, stop_cmd or "STOP"))
        self._refresh_jog_status()

    # ── XYZ JOG: the tool point along Cartesian axes ─────────────────────
    #
    # The same dead-man keys, the same jog_active set and the same
    # heartbeat as a joint jog -- what differs is the command. The board
    # takes all three signs at once and walks a Cartesian setpoint itself
    # (see XJOG in the firmware), so every press and release re-sends the
    # whole vector.
    def _xjog_vector(self):
        """[sx, sy, sz] from the keys held. Opposite keys cancel."""
        v = [0, 0, 0]
        for cmd in self.jog_active:
            spec = XYZ_JOG_AXES.get(cmd)
            if spec:
                v[spec[0]] += spec[1]
        return v

    def _xjog_speed(self):
        """mm/s from the panel's box, with NO upper limit (asked for; the
        panel warns past XYZ_JOG_WARN_MM_S). Never raises: a half-typed
        value is the default, not an exception on a key press."""
        try:
            speed = float(self.xjog_speed_v.get().strip().replace(",", "."))
        except (AttributeError, ValueError):
            return DEFAULT_XYZ_JOG_MM_S
        if not (speed > 0 and math.isfinite(speed)):
            return DEFAULT_XYZ_JOG_MM_S
        return speed

    def _send_xjog(self):
        sx, sy, sz = self._xjog_vector()
        if not (sx or sy or sz):
            self.send(XYZ_JOG_STOP)
            return
        arm = 2 if self.xjog_arm == "A2M" else 1
        self.send(f"XJOG:{arm},{sx},{sy},{sz},{self._xjog_speed():g}")

    def _xjog_release(self, axes, why=None):
        """Drops the held keys of the given axes (0 X, 1 Y, 2 Z) after a
        refusal. Sends nothing: the refuser has already stopped them."""
        dropped = False
        for cmd, (axis, _sign) in XYZ_JOG_AXES.items():
            if axis in axes and cmd in self.jog_active:
                self.jog_active.discard(cmd)
                self._xjog_blocked.add(cmd)
                dropped = True
                if cmd in self.jog_pads:
                    self.jog_pads[cmd].key_deactivate()
        if dropped:
            self._refresh_jog_status()
            if why:
                self.log(why, tag="warn")

    def _on_xjog_line(self, text):
        """[XJOG] from the board. The RX pump already logged the line."""
        upper = text.upper()
        if "XY STOPPED" in upper:
            self._xjog_release((0, 1))
        elif "Z STOPPED" in upper:
            self._xjog_release((2,))

    def tool_xyz(self, arm=None):
        """The selected arm's tool point, (x, y, z) mm: X,Y from the
        turntable axis, Z above HOME -- the P2P frame."""
        d1, rot, a1, a2 = self.current_joints
        r = motor_deg_to_reach(a2 if (arm or self.xjog_arm) == "A2M" else a1)
        rad = math.radians(rot)
        return r * math.cos(rad), r * math.sin(rad), d1

    def toggle_xjog_arm(self):
        """Which arm's tool the XYZ jog moves. The other one holds still."""
        self._release_all_jog_axes()
        i = XYZ_JOG_ARMS.index(self.xjog_arm)
        self.xjog_arm = XYZ_JOG_ARMS[(i + 1) % len(XYZ_JOG_ARMS)]
        self._style_jog_frame_controls()
        self._xyz_trail = {}
        self._refresh_xyz_view()
        self.log(f"XYZ jog now moves {self.xjog_arm}'s tool point.")

    def set_jog_frame(self, frame):
        """JOINT (one motor per key) or XYZ (the tool along an axis).

        Never with an axis latched, like LINK: a held key would be
        released under a different layout than it was pressed in.
        """
        if frame not in JOG_FRAMES or frame == self.jog_frame_mode:
            return
        if self.motion_locked:
            self.log("Cannot change the jog layout while a program is running.",
                     tag="warn")
            return
        self._release_all_jog_axes()
        self.jog_frame_mode = frame
        self._show_jog_frame()
        # The two layouts share keys (W/S/A/D by default), so the binder
        # has to follow the switch.
        self._bind_keys()
        self.root.focus_set()
        self.log("Jog layout: XYZ — keys move the tool point along X / Y / Z."
                 if frame == "XYZ" else
                 "Jog layout: JOINT — each key drives one motor.")

    def _release_all_jog_axes(self, send_stop=True):
        """Single place clearing every active jog axis — previously
        copy-pasted (slightly differently) in four methods."""
        for cmd in list(self.jog_active):
            if send_stop:
                self.send(JOG_STOP_COMMAND.get(cmd, "STOP"))
        self.jog_active.clear()
        self._xjog_blocked.clear()
        for pad in self.jog_pads.values():
            pad.key_deactivate()
        self._cancel_jobs("_jog_sim_job", "_jog_hb_job")
        self._refresh_jog_status()

    def _hardware_live(self):
        return bool(self.is_connected and self.hw_confirmed)

    def _start_jog_heartbeat(self):
        if self._jog_hb_job is None:
            self._jog_heartbeat()

    def _jog_heartbeat(self):
        self._jog_hb_job = None
        if not self.jog_active:
            return                      # nothing held: let board time out
        self.send("JOG_HB", log_tx=False)
        self._schedule("_jog_hb_job", JOG_HEARTBEAT_MS, self._jog_heartbeat)

    def _refresh_jog_status(self):
        # boundaries apply with or without reference. missing reference
        # costs meaning of NUMBERS, not protection.
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
        """sim_a1/sim_a2 are MOTOR degrees — raw rotation board counts.
        Primary card shows the derived BASE angle (-30 at HOME, the
        operator's frame) instead — motor deg alone was the old bug (shown as
        if it were the arm angle). Fold + reach stay in the line below,
        alongside, for anyone who wants the raw figure or the reach it
        implies."""
        self.rot_pos_v.set(f"{self.sim_rot:.2f} deg")
        self.a1_pos_v.set(f"{base_angle_from_motor_deg(self.sim_a1):.2f} base deg")
        self.a2_pos_v.set(f"{base_angle_from_motor_deg(self.sim_a2):.2f} base deg")
        self.jz_pos_v.set(f"{self.sim_z:.2f} mm")
        self.a1_reach_v.set(
            f"fold {fold_angle_from_motor_deg(self.sim_a1):.2f}° · "
            f"R1 = {motor_deg_to_reach(self.sim_a1):.1f} mm")
        self.a2_reach_v.set(
            f"fold {fold_angle_from_motor_deg(self.sim_a2):.2f}° · "
            f"R2 = {motor_deg_to_reach(self.sim_a2):.1f} mm")
        # P2P panel reads SAME pose, must repaint too — else switching
        # mode after jog showed stale numbers P2P last wrote, operator had
        # two different answers on screen for where machine was.
        self._refresh_p2p_pose_readout()
        # ...and the XYZ layout's 3D view, the same pose again.
        refresh = getattr(self, "_refresh_xyz_view", None)
        if refresh is not None:
            refresh()

    def _is_limited(self, direction):
        return bool(self.rot_limit.get(direction) or self.z_limit.get(direction))

    def _on_limit_triggered(self, direction):
        """Board reported [LIMIT] <direction>, or software simulation
        reached its own simulated bound."""
        if direction in self.rot_limit:
            self.rot_limit[direction] = True
        elif direction in self.z_limit:
            self.z_limit[direction] = True
        elif direction in JOG_ARM_AXES:
            # arm limit: nothing to latch, the opposite direction is always
            # immediately available. releasing the axis below is the whole
            # response.
            pass
        else:
            self.log(f"[LIMIT] {direction} — unrecognised axis.", tag="warn")
            return

        if direction in self.jog_active:
            self.jog_active.discard(direction)
            # actually tell board to stop this axis. old code only removed
            # from local set, so hardware-reported limit left axis command
            # latched on board.
            self.send(JOG_STOP_COMMAND.get(direction, "STOP"))
        if direction in self.jog_pads:
            self.jog_pads[direction].key_deactivate()
        self._refresh_jog_status()
        self.log(f"[LIMIT] {direction} — soft limit reached; this direction is "
                 f"now blocked.", tag="warn")

    def _clear_limit_if_opposite(self, direction):
        opp = LIMIT_OPPOSITE.get(direction)
        if opp is None:
            return
        if opp in self.rot_limit:
            self.rot_limit[opp] = False
        elif opp in self.z_limit:
            self.z_limit[opp] = False

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
        """Active boost multiplier. Axis speeds come from Settings profile
        in real units now, nothing else to scale."""
        return BOOST_LEVELS[self.boost_index]

    def _axis_speeds(self):
        """(rot deg/s, arm deg/s, z mm/s) for simulation.

        Derived from universal RPM and per-motor percentages through same
        arithmetic — and same ceilings — firmware uses, so offline motion
        is faithful preview of what machine actually does, not optimistic.
        """
        s = self.settings
        master = MASTER_RPM         # a constant now, not a setting
        # arm bounded in MOTOR RPM, cap applied there and result converted
        # — not to °/s figure derived from gear ratio nobody's measured yet
        arm_rpm = min(arm_motor_rpm(master, s["arm_pct"]), ARM_MOTOR_RPM_MAX)
        return (
            min(rot_speed_deg_s(master, s["rot_pct"]), ROT_VEL_MAX_DEG_S),
            # MOTOR degrees/sec: elbows simulated in motor degrees,
            # matching what board reports and taught limits are stored in.
            # no gear ratio involved, figure exact.
            arm_rpm * 360.0 / 60.0,
            min(z_speed_mm_s(master, s["z_pct"]), Z_VEL_MAX_MM_S),
        )

    def _jog_sim_tick(self):
        self._jog_sim_job = None
        if not self.jog_active or self._hardware_live():
            return

        dt = JOG_SIM_TICK_MS / 1000.0
        scale = self._speed_scale()
        rot_v, arm_v, z_v = self._axis_speeds()

        # where each axis was BEFORE this tick. clamp needs it to tell
        # "crossed boundary just now" from "started outside, jogging back
        # in" — the two halves of escape rule.
        prev_rot, prev_z = self.sim_rot, self.sim_z
        self._prev_arm = {"A1M": self.sim_a1, "A2M": self.sim_a2}

        if "ROT_CW" in self.jog_active:
            self.sim_rot += rot_v * dt * scale
        if "ROT_CCW" in self.jog_active:
            self.sim_rot -= rot_v * dt * scale

        # elbow angle is rotation from home: 0° retracted, 120° straight,
        # reach GROWS with it. extending therefore INCREASES it.
        # A1M/A2M separate motors, each integrates on its own.
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

        xyz = self._xjog_vector()
        if any(xyz):
            self._xjog_sim_step(xyz, dt, rot_v, arm_v, z_v)
            prev_rot, prev_z = self.sim_rot, self.sim_z

        rot_lo, rot_hi = self._axis_bounds(*self._limit_pair("rot"), axis="ROT")
        self.sim_rot, hit = self._apply_axis_limit(self.sim_rot, prev_rot,
                                                   rot_lo, rot_hi)
        if hit:
            self._on_limit_triggered("ROT_CW" if hit == "high" else "ROT_CCW")

        z_lo, z_hi = self._axis_bounds(*self._limit_pair("z"), axis="Z")
        self.sim_z, hit = self._apply_axis_limit(self.sim_z, prev_z, z_lo, z_hi)
        if hit:
            self._on_limit_triggered("Z_UP" if hit == "high" else "Z_DOWN")

        # each elbow clamped independently. hitting A1M's stop must not
        # halt A2M — separate linkages, separate motors.
        self._clamp_arm("A1M")
        self._clamp_arm("A2M")

        self._update_jog_readout()

        if self.jog_active:
            self._schedule("_jog_sim_job", JOG_SIM_TICK_MS, self._jog_sim_tick)

    def _xjog_sim_step(self, vec, dt, rot_v, arm_v, z_v):
        """One offline tick of the XYZ jog, by the board's own rules: solve
        the stepped point through IK, refuse it BEFORE taking it (travel,
        then the taught bands with the escape rule), and shorten the step
        until no joint is asked past its speed. Z is the lift alone, so
        the XY half and the Z half stand or fall separately.

        The caller re-reads its "previous" values afterwards: the step has
        been checked here, and a second opinion from the joint clamps
        would latch a joint-jog limit nobody is holding.

        Not mirrored: the board RAMPS the setpoint's speed and this snaps
        to it, like the rest of the offline jog.
        """
        arm = self.xjog_arm
        speed = self._xjog_speed()
        d1, rot, a1, a2 = self.current_joints
        motor = a2 if arm == "A2M" else a1
        x, y, _z = self.tool_xyz(arm)

        if vec[0] or vec[1]:
            def solve(nx, ny):
                # Z is the lift's business: an out-of-stroke counter must
                # not refuse X/Y.
                z = z_abs_from_home(min(max(d1, D1_MIN_MM), D1_MAX_MM), arm)
                _d1, n_rot, f1, f2 = solve_ik(nx, ny, z, arm, reference_deg=rot)
                n_motor = motor_deg_from_fold_angle(f2 if arm == "A2M" else f1)
                for value, prev, pair, axis, name in (
                        (n_rot, rot, "rot", "ROT", "RM"),
                        (n_motor, motor, arm[:2].lower(), arm[:2], arm)):
                    lo, hi = self._axis_bounds(*self._limit_pair(pair), axis=axis)
                    if self._apply_axis_limit(value, prev, lo, hi)[1]:
                        raise ValueError(f"{name} would leave its taught band")
                return n_rot, n_motor

            nx, ny = x + vec[0] * speed * dt, y + vec[1] * speed * dt
            try:
                n_rot, n_motor = solve(nx, ny)
                def joint_over():
                    return max(abs(n_rot - rot) / (rot_v * XYZ_JOG_JOINT_HEADROOM * dt),
                               abs(n_motor - motor) / (arm_v * XYZ_JOG_JOINT_HEADROOM * dt))

                over = joint_over()
                if over > 1.0:
                    n_rot, n_motor = solve(x + (nx - x) / over, y + (ny - y) / over)
                    # Still far past its speed after shortening: the IK
                    # answered on another branch (RM's counter outside its
                    # travel, the tool at the axis). A jump, not a step.
                    if joint_over() > XYZ_JOG_JUMP_RATIO:
                        raise ValueError("a joint would have to jump to follow — HOME "
                                         "or reset the coordinates first")
            except ValueError as e:
                self._xjog_release(
                    (0, 1), f"XYZ jog: X/Y stopped — {str(e).split('. ')[0]}.")
            else:
                self.sim_rot = n_rot
                if arm == "A2M":
                    self.sim_a2 = n_motor
                else:
                    self.sim_a1 = n_motor
                self._prev_arm["A2M" if arm == "A2M" else "A1M"] = n_motor

        if vec[2]:
            nz = d1 + vec[2] * min(speed, z_v * XYZ_JOG_JOINT_HEADROOM) * dt
            lo, hi = self._axis_bounds(*self._limit_pair("z"), axis="Z")
            if (nz < D1_MIN_MM - 1e-6 and vec[2] < 0) or (nz > D1_MAX_MM + 1e-6 and vec[2] > 0):
                self._xjog_release((2,), f"XYZ jog: Z stopped — the lift's stroke is "
                                         f"{D1_MIN_MM:g}..{D1_MAX_MM:g} mm above HOME.")
            elif self._apply_axis_limit(nz, d1, lo, hi)[1]:
                self._xjog_release((2,), "XYZ jog: Z stopped — ZM would leave its "
                                         "taught band.")
            else:
                self.sim_z = nz

    def _limit_pair(self, axis):
        """(lower, upper) working limit for an axis.

        Reads live settings not module constants, so limit edited in
        Settings takes effect on offline simulation same instant it takes
        effect on board — two disagreeing copies of a limit is failure
        this avoids.

        Elbow pair TAUGHT not typed (see LIMIT_CAPTURE_ONLY in config),
        stored UNORDERED — operator jogs to one stop, presses SET HERE,
        then other; which reached first not something to keep straight.
        Sorting happens here, at point of use, so both taught numbers
        survive in settings exactly as captured. Mirrors armBand() in
        firmware; both must agree or simulation and board clamp at
        different places.
        """
        s = self.settings
        try:
            a, b = s[f"lim_{axis}_min"], s[f"lim_{axis}_max"]
            return (a, b) if a <= b else (b, a)
        except KeyError:
            # settings file written before this axis existed. fall back to
            # structural envelope rather than raising mid jog-tick.
            return ARM_SIM_MIN_DEG, ARM_SIM_MAX_DEG

    def _axis_bounds(self, lo, hi, axis=None):
        """Band actually applied to an axis.

        TAUGHT BOUNDARY APPLIES IMMEDIATELY. Does not wait for HOME or
        RESET COORDINATES.

        Used to widen band by full travel either side while `is_homed`
        false, arguing counters meaningless without reference. Argument
        doesn't survive contact with how boundaries are set: jog to stop,
        press SET HERE, boundary captured against SAME counters compared
        to it. Meaningful in exactly frame taught in, reference or not.
        Widening meant limit just taught at -300 let axis run to -427 —
        bug this replaced.

        `axis` is "Z"/"ROT"/"A1"/"A2", checked against per-axis
        enforcement switch and master one, mirroring axisLimited() on
        board. Two systems of record disagreeing whether machine is
        protected is worse than either answer.

        Axis starting OUTSIDE its band not trapped — see
        `_apply_axis_limit()` for escape rule.

        ONE EXCEPTION: a FACTORY-DEFAULT floor, while UNREFERENCED.
        HOME is minimum of every axis, so default floors sit at 0 — and
        w/o reference counter reads 0 wherever it powered up, not at
        bottom of travel. Axis then sits exactly ON its floor and every
        downward tick clamps back: readout frozen, axis pinned, nothing
        to escape from because escape rule counts sitting on boundary as
        inside. Ceiling still applies — far end cannot coincide with
        counter origin.

        Narrow on purpose. A TAUGHT floor still applies unreferenced —
        that is the -341.89 boundary jog once ran past to -427.16, and
        widening it back is the bug this file exists to prevent. Only an
        untouched default is relaxed, because a default was never
        captured against any counter. Teach the floor, or reference the
        machine, and it applies again.
        """
        if axis is not None and not self._axis_enforced(axis):
            span = hi - lo
            return lo - span, hi + span
        if (axis is not None and not getattr(self, "is_homed", False)
                and self._floor_is_factory_default(axis)):
            return lo - (hi - lo), hi
        return lo, hi

    def _floor_is_factory_default(self, axis):
        """True when this axis's LOW boundary is still the shipped value.

        Compares against LIMIT_FIELDS' own default rather than a literal,
        so re-calibrating the arm frame cannot leave this reading a
        number that no longer means what it did.
        """
        key = f"lim_{axis.lower()}_min"
        field = LIMIT_FIELDS.get(key)
        if field is None:
            return False
        return self.settings.get(key) == field[6]

    @staticmethod
    def _apply_axis_limit(value, previous, lo, hi):
        """Clamps one axis, allows escape from band it starts outside.

        Returns (value, "high"|"low"|None).

        Coming from inside, axis stops ON boundary — ordinary case.
        Starting outside it — boundary taught in previous session, applied
        against counter that powered up elsewhere — motion FURTHER out
        refused, motion back toward band allowed.

        W/o that second half, applying limits w/o reference could pin axis
        with no way to jog off — exact failure old widening was there to
        avoid. Escape rule solves it without giving up protection.
        """
        if value > hi:
            if previous <= hi:
                return hi, "high"           # crossed out: stop on line
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
        """True when this axis's boundary actually policing anything.

        AND of per-axis switch and master one, in that order — master
        must never re-arm axis switched off on its own."""
        if not self.settings.get(LIMITS_ENABLED_KEY, True):
            return False
        return bool(self.settings.get(LIMIT_ENFORCE_BY_AXIS[axis], True))

    def _clamp_arm(self, arm):
        """Clamps one elbow to its travel, releases only axes actually
        driving THAT elbow in offending direction.

        Each arm clamped against its OWN limits. One shared arm limit is
        how A2M used to be driven past its stop while A1M's angle was
        checked."""
        angle = self.sim_a1 if arm == "A1M" else self.sim_a2
        is_a1 = arm == "A1M"
        lo, hi = self._axis_bounds(*self._limit_pair("a1" if is_a1 else "a2"),
                                   axis="A1" if is_a1 else "A2")

        # same escape rule as other axes: elbow powering up outside a
        # boundary taught in earlier session can still jog back into range
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

    def toggle_arm_link(self):
        """LINK on = both elbows follow one control (v8 gesture tested on
        hardware). LINK off = A1M and A2M fully independent."""
        # never flip modes with axis latched — in-flight command would be
        # released under different resolution than it started
        self._release_all_jog_axes()
        self.arms_linked = not self.arms_linked
        active = self.arms_linked
        self.link_btn.set_config(
            f"LINK: {'ON' if active else 'OFF'}",
            ACCENT_ORANGE if active else SURFACE,
            icon="🔗", fg_color=INK_DARK if active else TEXT_LIGHT)
        # the LIVE layout: the import-time copy named the old keys after a rebind
        caps = self._caps()
        a1_keys = f"{caps['A1_FWD']}/{caps['A1_BACK']}"
        a2_keys = f"{caps['A2_FWD']}/{caps['A2_BACK']}"
        self.log(f"LINK on — {a1_keys} and {a2_keys} both drive BOTH arms together."
                 if active else
                 f"LINK off — A1M ({a1_keys}) and A2M ({a2_keys}) move independently.")
