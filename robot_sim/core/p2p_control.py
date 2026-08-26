"""Point-to-point motion: arm selection, IK LOAD, RUN, STOP and
software-only playback used when no hardware confirmed."""

from tkinter import messagebox

from ..config import (ARM_CONFIGS, ARM_HOME_DEG, PLC_SENSOR_JOINT_INDEX,
                      PLC_SENSOR_PANEL, ROT_HOME_DEG, Z_HOME_ABS_MM,
                      Z_HOME_MM, Z_INPUT_MAX_MM, Z_INPUT_MIN_MM)
from ..kinematics import (
    base_angle_from_motor_deg,
    fold_angle_from_motor_deg,
    forward_kinematics,
    is_near_singularity,
    motor_deg_from_fold_angle,
    motor_deg_to_reach,
    reach_band_from_motor_deg,
    solve_ik,
    solve_ik_both,
    z_abs_from_home,
    z_home_from_abs,
)
from ..theme import ACCENT_MINT, SURFACE, INK_DARK, TEXT_LIGHT

ANIM_STEPS = 25
ANIM_TICK_MS = 150


class P2PControlMixin:
    def set_arm_config(self, cfg):
        if cfg not in ARM_CONFIGS:
            return
        self.arm_config = cfg
        # changing arm invalidates joint targets already sent to board;
        # requiring fresh LOAD prevents RUN executing program computed for
        # other arm
        self._invalidate_loaded_program(
            reason=f"Arm changed to {cfg} — LOAD again before RUN.")
        self._sync_z_for_both_mode()
        # real height is per DECK, switching arms changes it even though
        # typed Z hasn't moved
        self._refresh_real_heights()
        for c, btn in self.arm_buttons.items():
            if c == cfg:
                btn.set_config(c, ACCENT_MINT, fg_color=INK_DARK)
            else:
                btn.set_config(c, SURFACE, fg_color=TEXT_LIGHT)
        self.log(f"Arm config set to {cfg}.")

    _set_elbow_config = set_arm_config

    def _invalidate_loaded_program(self, reason=None):
        if self.loaded_program is None:
            return
        self.loaded_program = None
        for var in (self.calc_a_d1_v, self.calc_a_rot_v, self.calc_a_a1_v, self.calc_a_a2_v,
                    self.calc_b_d1_v, self.calc_b_rot_v, self.calc_b_a1_v, self.calc_b_a2_v):
            var.set("—")
        if reason:
            self.status_var.set(reason)
            self.log(reason, tag="warn")

    def _sync_z_for_both_mode(self, *_args):
        """BOTH mode rides one ZM carriage, so in HOME frame there's
        exactly ONE Z and Z1 simply mirrors Z0.

        Used to subtract ARM2_Z_DROP_MM: Z was ABSOLUTE height then, and
        arm 2's deck really sits 9 mm lower, so same carriage position was
        two different absolute numbers. Now Z measured from HOME it's
        carriage's own travel, identical for both decks, and 9 mm lives
        where it belongs — inside z_abs_from_home(), applied per arm when
        value crosses into kinematics.

        Subtracting it here too would apply it twice, command arm 2
        9 mm low."""
        if getattr(self, "z1_entry", None) is None:
            return
        if self.arm_config == "BOTH":
            try:
                z0 = float(self.z0_v.get().strip().replace(",", "."))
            except ValueError:
                self.z1_entry.config(state="disabled")
                return
            self.z1_v.set(f"{z0:g}")
            self.z1_entry.config(state="disabled")
        else:
            self.z1_entry.config(state="normal")

    def _refresh_workspace_hint(self, *_args):
        """Reachable band, computed from YOUR taught boundaries.

        No structural reach envelope any more — solve_ik refuses only
        radii geometry can't solve at all — so this line is the only
        place working envelope is stated, must come from same numbers
        that enforce it. Hard-coded 133.2–613.2 would name a boundary
        nothing applies.

        Axis with enforcement switched off says so, rather than showing a
        band it's not policing.
        """
        if getattr(self, "workspace_hint_v", None) is None:
            return
        parts = []
        for axis, label in (("a1", "A1M"), ("a2", "A2M")):
            enforce_axis = axis.upper()
            if not self._axis_enforced(enforce_axis):
                parts.append(f"{label} not enforced")
                continue
            lo, hi = self._limit_pair(axis)
            r_lo, r_hi = reach_band_from_motor_deg(lo, hi)
            parts.append(f"{label} R {r_lo:.1f}..{r_hi:.1f}")
        if self._axis_enforced("Z"):
            z_lo, z_hi = self._limit_pair("z")
            z_txt = f"Z {z_lo:.0f}..{z_hi:.0f} above HOME"
        else:
            z_txt = "Z not enforced"
        if self._axis_enforced("ROT"):
            rot_lo, rot_hi = self._limit_pair("rot")
            rot_txt = f"RM {rot_lo:.0f}..{rot_hi:.0f}°"
        else:
            rot_txt = "RM not enforced"
        self.workspace_hint_v.set(
            f"Your limits (Settings → Boundaries):  {' | '.join(parts)} mm  ·  "
            f"{z_txt}  ·  {rot_txt}   [Z 0 = {Z_HOME_ABS_MM:.1f} mm real on A1M, "
            f"9 mm less on A2M]")

    def _refresh_real_heights(self, *_args):
        """Shows deck's REAL height under each Z entry, live.

        Z measured from HOME, typed number no longer says anything about
        where arm is off floor. Absolute figure is what gets compared
        against cassette slot or tape measure, shown rather than left as
        arithmetic for operator.

        Per arm, decks are 9 mm apart: in BOTH mode Z0 is arm 1's deck,
        Z1 is arm 2's, from SAME carriage position.
        """
        if getattr(self, "z0_real_v", None) is None:
            return
        pairs = ((self.z0_v, self.z0_real_v, "A1M" if self.arm_config in ("A1M", "BOTH")
                  else "A2M"),
                 (self.z1_v, self.z1_real_v, "A2M" if self.arm_config == "BOTH"
                  else (self.arm_config if self.arm_config in ("A1M", "A2M") else "A1M")))
        for var, out, arm in pairs:
            raw = var.get().strip().replace(",", ".")
            try:
                z = float(raw)
            except ValueError:
                # mid-typing, or empty. say nothing rather than show height
                # computed from a number that isn't one yet
                out.set("")
                continue
            out.set(f"real height {arm}: {z_abs_from_home(z, arm):.1f} mm"
                    + ("" if Z_INPUT_MIN_MM <= z <= Z_INPUT_MAX_MM
                       else f"   ⚠ outside the {Z_INPUT_MIN_MM:.0f}–"
                            f"{Z_INPUT_MAX_MM:.0f} mm stroke"))

    def _read_points(self):
        """Returns (x0,y0,z0,x1,y1,z1) or None after showing an error."""
        fields = (("X0", self.x0_v), ("Y0", self.y0_v), ("Z0", self.z0_v),
                  ("X1", self.x1_v), ("Y1", self.y1_v), ("Z1", self.z1_v))
        values = []
        for name, var in fields:
            raw = var.get().strip().replace(",", ".")
            try:
                values.append(float(raw))
            except ValueError:
                # name offending field — old message just said "coordinates
                # must be numbers" with no clue which one
                messagebox.showerror("Error", f"“{name}” is not a valid number: {var.get()!r}")
                return None
        return tuple(values)

    def p2p_load_parameters(self):
        if self.motion_locked:
            self.log("LOAD ignored — a program is running.", tag="warn")
            return

        if self.arm_config == "BOTH":
            self._sync_z_for_both_mode()

        points = self._read_points()
        if points is None:
            return
        x0, y0, z0, x1, y1, z1 = points

        # THE ONE PLACE Z FRAME CHANGES.
        #
        # operator types Z as height above HOME (0..285). solve_ik and
        # MATLAB parity sweep work in absolute frame where arm 1's deck is
        # at 514.3 mm with lift down, must keep doing so — sweep proves
        # this module still reproduces mophong_init.m exactly. translation
        # happens here, once, per arm because decks are 9 mm apart.
        arm_for_z = "A1M" if self.arm_config in ("A1M", "BOTH") else "A2M"
        z0_abs = z_abs_from_home(z0, arm_for_z)
        z1_abs = z_abs_from_home(z1, "A2M" if self.arm_config == "BOTH" else arm_for_z)

        # solve_ik works in FROG-LEG degrees; board, taught limits, every
        # readout in MOTOR degrees. conversion happens here, single
        # boundary between the two, nothing downstream must know which
        # unit it holds.
        try:
            if self.arm_config == "BOTH":
                d1, rot, a1, a2 = solve_ik_both(x0, y0, z0_abs, x1, y1, z1_abs)
                a1 = motor_deg_from_fold_angle(a1)
                a2 = motor_deg_from_fold_angle(a2)
            else:
                # arm NOT selected holds its current angle. parking it at
                # home would move an arm the operator never commanded —
                # real collision risk if something's on it. idle arm's
                # live angle is motor degrees; solve_ik wants frog-leg.
                idle_motor = self._idle_arm_angle()
                idle = (None if idle_motor is None
                        else fold_angle_from_motor_deg(idle_motor))
                d1a, rota, a1a, a2a = solve_ik(x0, y0, z0_abs, self.arm_config,
                                               idle_deg=idle)
                d1b, rotb, a1b, a2b = solve_ik(x1, y1, z1_abs, self.arm_config,
                                               idle_deg=idle)
                a1a, a2a = motor_deg_from_fold_angle(a1a), motor_deg_from_fold_angle(a2a)
                a1b, a2b = motor_deg_from_fold_angle(a1b), motor_deg_from_fold_angle(a2b)
        except ValueError as e:
            self._invalidate_loaded_program()
            messagebox.showerror("Inverse kinematics failed", str(e))
            self.log(f"IK error: {e}", tag="error")
            return

        # solve_ik only knows machine's PHYSICAL envelope. operator's own
        # working limits are narrower, point outside them must be refused
        # here rather than sent, half-executed, stopped part-way through
        # by board's limit check.
        targets = ([(d1, rot, a1, a2)] if self.arm_config == "BOTH"
                   else [(d1a, rota, a1a, a2a), (d1b, rotb, a1b, a2b)])
        for label, (td1, trot, ta1, ta2) in zip(("A", "B"), targets):
            violation = self._limit_violation(td1, trot, ta1, ta2)
            if violation:
                self._invalidate_loaded_program()
                messagebox.showerror(
                    "Outside the working envelope",
                    f"Point {label} is outside the limits you set:\n\n{violation}\n\n"
                    f"Adjust the coordinates, or widen the limits in "
                    f"Settings → Boundaries.")
                self.log(f"Point {label} rejected: {violation}", tag="error")
                return

            # PLC sensors enforced for P2P (jog only warns). checked after
            # taught limits because sensor is physical fact, its message
            # about machine, not a setting.
            blocked = self._sensor_violation(td1, trot, ta1, ta2)
            if blocked:
                self._invalidate_loaded_program()
                messagebox.showerror(
                    "Blocked by a PLC sensor",
                    f"Point {label} cannot be run:\n\n{blocked}")
                self.log(f"Point {label} blocked by a PLC sensor: {blocked}",
                         tag="error")
                return

        self._set_progress(0)
        # is_near_singularity() is about frog-leg geometry, must be handed
        # fold degrees, not motor degrees stored above.
        if self.arm_config == "BOTH":
            self._warn_if_near_singularity(
                ("A1M", fold_angle_from_motor_deg(a1)),
                ("A2M", fold_angle_from_motor_deg(a2)))
        else:
            active_a = a1a if self.arm_config == "A1M" else a2a
            active_b = a1b if self.arm_config == "A1M" else a2b
            self._warn_if_near_singularity(
                ("A", fold_angle_from_motor_deg(active_a)),
                ("B", fold_angle_from_motor_deg(active_b)))

        if self.arm_config == "BOTH":
            self.loaded_program = {"mode": "both", "arm": "BOTH", "target": (d1, rot, a1, a2)}
            # both rows show same shared d1/rot; A row highlights A1M
            # target, B row A2M target
            self.calc_a_d1_v.set(f"{d1:.2f} mm")
            self.calc_a_rot_v.set(f"{rot:.2f} deg")
            self.calc_a_a1_v.set(f"{base_angle_from_motor_deg(a1):.2f} base deg")
            self.calc_a_a2_v.set(f"{base_angle_from_motor_deg(a2):.2f} base deg")
            self.calc_b_d1_v.set(f"{d1:.2f} mm")
            self.calc_b_rot_v.set(f"{rot:.2f} deg")
            self.calc_b_a1_v.set(f"{base_angle_from_motor_deg(a1):.2f} base deg")
            self.calc_b_a2_v.set(f"{base_angle_from_motor_deg(a2):.2f} base deg")

            self.send(f"LOAD_BOTH:{d1:.3f},{rot:.3f},{a1:.3f},{a2:.3f}")
            self.log(f"IK solved (both arms simultaneously) and loaded: "
                     f"d1={d1:.1f} rot={rot:.1f} a1m={a1:.1f} a2m={a2:.1f}")
            self.status_var.set("LOADED — Simultaneous dual-arm move ready. Press RUN PROGRAM.")
            return

        self.loaded_program = {
            "mode": "single",
            "arm": self.arm_config,
            "a": (d1a, rota, a1a, a2a),
            "b": (d1b, rotb, a1b, a2b),
        }
        self.calc_a_d1_v.set(f"{d1a:.2f} mm")
        self.calc_a_rot_v.set(f"{rota:.2f} deg")
        self.calc_a_a1_v.set(f"{base_angle_from_motor_deg(a1a):.2f} base deg")
        self.calc_a_a2_v.set(f"{base_angle_from_motor_deg(a2a):.2f} base deg")
        self.calc_b_d1_v.set(f"{d1b:.2f} mm")
        self.calc_b_rot_v.set(f"{rotb:.2f} deg")
        self.calc_b_a1_v.set(f"{base_angle_from_motor_deg(a1b):.2f} base deg")
        self.calc_b_a2_v.set(f"{base_angle_from_motor_deg(a2b):.2f} base deg")

        self.send(f"LOAD:{d1a:.3f},{rota:.3f},{a1a:.3f},{a2a:.3f},"
                  f"{d1b:.3f},{rotb:.3f},{a1b:.3f},{a2b:.3f}")
        self.log(f"IK solved and loaded. "
                 f"A=(d1={d1a:.1f},rot={rota:.1f},a1m={a1a:.1f},a2m={a2a:.1f}) "
                 f"B=(d1={d1b:.1f},rot={rotb:.1f},a1m={a1b:.1f},a2m={a2b:.1f})")
        self.status_var.set("LOADED — Program A → B ready. Press RUN PROGRAM.")

    def _sensor_violation(self, d1, rot, a1, a2):
        """Describes first PLC sensor a target would drive further into,
        or None.

        Where sensors are ENFORCED. Program runs unattended, operator not
        watching axis, so leg pushing further into covered sensor must
        not start. Jog only warns, opposite reason — see
        warn_if_jogging_into_sensor().

        Comparison against LIVE pose, answers "would this move make it
        worse" not "is a sensor covered". Target moving AWAY from covered
        sensor is exactly what operator needs to command.
        """
        state = getattr(self, "plc_sensor_state", {})
        target = (d1, rot, a1, a2)
        for bit, label, axis, _cmd, end in PLC_SENSOR_PANEL:
            if not state.get(bit, False):
                continue
            # A2M's switch sits at BOTH ends, so the end it is refusing is
            # whichever one the board latched, not the fixed home-side end
            # in the table. Using the table blocked legs retracting the arm
            # while it was actually stuck on the FORWARD switch.
            end = self.plc_sensor_end_for(bit)
            idx = PLC_SENSOR_JOINT_INDEX[axis]
            now, want = self.current_joints[idx], target[idx]
            if abs(want - now) < 1e-3:
                continue                     # axis not moving
            if (1 if want > now else -1) != end:
                continue                     # moving away, allowed
            side = "MAX" if end > 0 else "MIN"
            return (f"{label.strip()} is sitting on {bit} at its {side} end, and "
                    f"this point would drive it further in ({now:.2f} → "
                    f"{want:.2f}). Jog it off the sensor first.")
        return None

    def _limit_violation(self, d1, rot, a1, a2):
        """Returns human description of first working-limit breach, or
        None when pose inside every limit operator set.

        THIS is the reach envelope now. solve_ik no longer carries a
        structural one — refuses only radii geometry can't solve at all —
        so working limit is operator's taught elbow band and nothing
        else. One system of record.

        Two things it has to get right, both learned elsewhere in
        codebase:

        * Pair read through `_limit_pair()`, which SORTS. Elbow boundaries
          stored exactly as taught, may sit in either order; comparing
          against raw min/max would reject every pose whenever somebody
          taught far stop first.
        * Switched-off axis skipped, via `_axis_enforced()`. Refusing a
          target against boundary board's been told to stop applying
          would mean GUI and machine disagreed whether axis protected.
        """
        checks = (
            ("ZM (d1)", d1, "z",   "Z",   "mm"),
            ("RM",      rot, "rot", "ROT", "°"),
            ("A1M",     a1,  "a1",  "A1",  "motor °"),
            ("A2M",     a2,  "a2",  "A2",  "motor °"),
        )
        for label, value, axis, enforce_axis, unit in checks:
            if not self._axis_enforced(enforce_axis):
                continue
            lo, hi = self._limit_pair(axis)
            if value < lo - 1e-6 or value > hi + 1e-6:
                extra = ""
                if axis in ("a1", "a2"):
                    # elbow number means little alone; radius it produces
                    # is what operator was aiming at
                    r_lo, r_hi = reach_band_from_motor_deg(lo, hi)
                    extra = (f" — that is R = {motor_deg_to_reach(value):.1f} mm, "
                             f"and your taught band allows "
                             f"{r_lo:.1f}..{r_hi:.1f} mm")
                return (f"{label} = {value:.2f} {unit}, outside the range you set "
                        f"in Settings → Boundaries [{lo:.2f}, {hi:.2f}] {unit}"
                        + extra)
        return None

    def _idle_arm_angle(self):
        """Live elbow angle, MOTOR degrees, of arm not selected, so
        single-arm program leaves it untouched."""
        if self.arm_config == "A1M":
            return self.current_joints[3]     # A2M holds
        if self.arm_config == "A2M":
            return self.current_joints[2]     # A1M holds
        return None

    def _warn_if_near_singularity(self, *labelled_angles):
        """Advisory only. At 120° from home frog-leg straight, radial
        stiffness collapses, small elbow error becomes large radial
        error. Move still allowed (full mathematical envelope chosen
        deliberately), operator told."""
        for label, angle in labelled_angles:
            if is_near_singularity(angle):
                self.log(
                    f"⚠ {label}: elbow at {angle:.1f}° — close to the straight-arm "
                    f"singularity. Radial stiffness collapses there, so a small "
                    f"elbow error becomes a large error at the end effector. "
                    f"Move slowly.", tag="warn")

    def p2p_run_program(self):
        if self.loaded_program is None:
            messagebox.showwarning("Warning",
                                   "Press LOAD PARAMETERS before running.")
            return
        if self.motion_locked:
            self.log("RUN ignored — a motion is already running.", tag="warn")
            return

        self._cancel_job("anim_job")
        self.is_running = True
        self._set_motion_locked(True)
        self._set_progress(0)

        if self.loaded_program["mode"] == "both":
            self.status_var.set("RUNNING — Simultaneous dual-arm move executing...")
            self.log("RUN PROGRAM — moving both arms simultaneously (A1M + A2M).")
        else:
            self.status_var.set("RUNNING — HOME → A → B → HOME...")
            self.log("RUN PROGRAM — HOME → A → B → HOME (4 legs).")

        if self._hardware_live():
            self.send("RUN")
            return

        if self.is_connected and not self.hw_confirmed:
            self.log("WARNING: command sent, but ClearCore has not confirmed the "
                     "handshake.", tag="warn")
        self._simulate_p2p_run()

    def p2p_stop(self):
        self.send("STOP")
        self.is_running = False
        self.is_homing = False           # STOP during HOME must clear it too
        self._cancel_jobs("anim_job", "_home_sim_job", "_reset_position_sim_job")
        self._set_motion_locked(False)
        self.status_var.set("STOPPED — Program halted.")
        self.log("STOP — P2P program halted.", tag="warn")

    def _simulate_p2p_run(self):
        prog = self.loaded_program
        start = tuple(self.current_joints)

        if prog["mode"] == "both":
            legs = [(start, prog["target"])]
            done = "Simultaneous dual-arm move complete."
        else:
            # HOME -> A -> B -> HOME, four legs, matching board and
            # mophong_init.m's P_home -> A -> B -> P_home trajectory.
            #
            # starting/ending at reference makes cycle repeatable: every
            # run begins from same pose whatever was done by hand
            # beforehand, parks where next one starts.
            home = (Z_HOME_MM, ROT_HOME_DEG, ARM_HOME_DEG, ARM_HOME_DEG)
            legs = [(start, home), (home, prog["a"]),
                    (prog["a"], prog["b"]), (prog["b"], home)]
            done = "Cycle complete — HOME → A → B → HOME."

        self._run_legs(legs, 0, done)

    def _run_legs(self, legs, index, done_message):
        if not self.is_running:
            return
        if index >= len(legs):
            self._finish_run(done_message)
            return
        start_j, target_j = legs[index]
        self._animate_leg(0, ANIM_STEPS, start_j, target_j, legs, index, done_message)

    def _animate_leg(self, step, total_steps, start_j, target_j, legs, index, done_message):
        if not self.is_running:
            return

        t = step / total_steps
        joints = tuple(s + (e - s) * t for s, e in zip(start_j, target_j))
        # progress spans all legs so bar fills once across whole run
        overall = (index + t) / len(legs)
        self._update_p2p_telemetry(*joints, pct=int(round(overall * 100)))

        if step < total_steps:
            self._schedule("anim_job", ANIM_TICK_MS, self._animate_leg,
                           step + 1, total_steps, start_j, target_j, legs, index, done_message)
        else:
            self._schedule("anim_job", ANIM_TICK_MS, self._run_legs,
                           legs, index + 1, done_message)

    def _finish_run(self, done_message):
        self.anim_job = None
        self.is_running = False
        self._set_motion_locked(False)
        self._set_progress(100)
        self.status_var.set(f"READY — {done_message}")
        self.log(f"Simulation complete. {done_message}")

    def _set_progress(self, pct):
        pct = max(0, min(100, int(pct)))
        self.progress_var.set(pct)
        self.progress_pct_var.set(f"{pct}%")

    def _update_p2p_telemetry(self, d1, rot, a1, a2, pct):
        """Writes the shared live pose, then repaints BOTH panels.

        current_joints is the single store — sim_z/sim_rot/sim_a1/sim_a2 are
        properties onto it — so assigning here also moves the jog readout,
        which is why the jog labels are refreshed at the end.
        """
        # Joints are kept as floats. The old code re-parsed them out of the
        # formatted label strings, which silently fell back to 0.0 whenever
        # the format changed.
        self.current_joints = [float(d1), float(rot), float(a1), float(a2)]
        self._set_progress(pct)
        self._refresh_p2p_pose_readout()
        # Jog reads the same pose; repaint it unless its widgets do not
        # exist yet (this runs once during construction).
        if getattr(self, "rot_pos_v", None) is not None:
            self.rot_pos_v.set(f"{rot:.2f} deg")
            self.a1_pos_v.set(f"{a1:.2f} motor deg")
            self.a2_pos_v.set(f"{a2:.2f} motor deg")
            self.jz_pos_v.set(f"{d1:.2f} mm")
            self.a1_reach_v.set(
                f"fold {fold_angle_from_motor_deg(a1):.2f}° · "
                f"R1 = {motor_deg_to_reach(a1):.1f} mm")
            self.a2_reach_v.set(
                f"fold {fold_angle_from_motor_deg(a2):.2f}° · "
                f"R2 = {motor_deg_to_reach(a2):.1f} mm")

    def _refresh_p2p_pose_readout(self):
        """Repaints the P2P live-pose labels from the shared pose."""
        # The board's live dot is the same pose, so it moves with the run
        # and with a jog.
        if getattr(self, "xy_canvas", None) is not None:
            self._refresh_xy_board()
        if getattr(self, "curr_d1_v", None) is None:
            return
        d1, rot, a1, a2 = self.current_joints
        self.curr_d1_v.set(f"{d1:.2f} mm")
        self.curr_rot_v.set(f"{rot:.2f} deg")
        # a1/a2 are MOTOR degrees on the wire; the operator reads the BASE
        # angle, so convert at the point of display. The frog-leg angle and
        # reach go on the Cartesian line below, where there is room to
        # label them.
        self.curr_a1_v.set(f"{base_angle_from_motor_deg(a1):.2f} base deg")
        self.curr_a2_v.set(f"{base_angle_from_motor_deg(a2):.2f} base deg")

        # forward_kinematics() is frog-leg geometry, so the motor degrees
        # held in a1/a2 have to be converted before it sees them. Feeding
        # motor degrees straight in was the bug: at a ratio of 2 it put the
        # reported end effector at twice the elbow angle it really had.
        f1 = fold_angle_from_motor_deg(a1)
        f2 = fold_angle_from_motor_deg(a2)
        arm = self.arm_config if self.arm_config in ("A1M", "A2M") else None
        x, y, z = forward_kinematics(d1, rot, f1, f2, arm=arm)
        # forward_kinematics returns the ABSOLUTE height; the operator reads
        # and types Z from HOME, so it is converted back the same way it was
        # converted in. Both are shown: the from-HOME figure is the one that
        # matches the entry boxes, the absolute one is the real height of
        # the deck off the floor, which is what a tape measure would say.
        if self.arm_config == "BOTH":
            x2, y2, z2 = forward_kinematics(d1, rot, f1, f2, arm="A2M")
            x1, y1, z1 = forward_kinematics(d1, rot, f1, f2, arm="A1M")
            self.calc_xyz_v.set(
                f"Cartesian (forward kinematics): A1M X={x1:.2f} Y={y1:.2f} | "
                f"A2M X={x2:.2f} Y={y2:.2f} | "
                f"Z={z_home_from_abs(z1, 'A1M'):.2f} mm above HOME "
                f"(real {z1:.1f} / {z2:.1f} mm)  ·  "
                f"fold A1M={f1:.2f}° R1={motor_deg_to_reach(a1):.1f} mm | "
                f"fold A2M={f2:.2f}° R2={motor_deg_to_reach(a2):.1f} mm")
        else:
            active_fold = f1 if self.arm_config == "A1M" else f2
            active_motor = a1 if self.arm_config == "A1M" else a2
            arm_for_z = self.arm_config
            self.calc_xyz_v.set(
                f"Cartesian (from forward kinematics): X={x:.2f}  Y={y:.2f}  "
                f"Z={z_home_from_abs(z, arm_for_z):.2f} mm above HOME "
                f"(real {z:.1f} mm)  ·  {self.arm_config} fold={active_fold:.2f}° "
                f"R={motor_deg_to_reach(active_motor):.1f} mm "
                f"(motor {active_motor:.2f}°)")
