"""Emergency stop, HOME, coordinate reset, motion lock, mode switching
and shutdown."""

import os
import sys
from tkinter import messagebox

from ..config import ARM_HOME_DEG, ROT_HOME_DEG, Z_HOME_MM
from ..theme import ACCENT_RED, TEXT_MUTED


class SafetyMixin:
    def reset_coordinates(self, axis=None):
        """Declares "the machine is at its reference right now" and zeroes
        every axis counter there.

        Bench substitute for PLC homing. Confirmed rather than instant,
        refused while anything is moving, for one reason: it measures
        nothing. Trusts operator jogged machine to the pose they mean. Get
        it wrong and every later absolute move — and every soft limit —
        is offset by the same amount, silently.

        `axis` selects ONE axis ("Z", "ROT", "A1", "A2"); None means all
        four. One at a time is useful because the four axes are referenced
        by different means and rarely at the same moment — ZM and RM have
        optical stops, elbows set by eye — so being forced to re-declare
        all four to correct one meant either lying about three axes or
        leaving the wrong one wrong.

        Single-axis reset deliberately does NOT set is_homed. Reference
        only complete when every axis has one; claiming it early would
        switch soft limits on against three counters still meaningless.
        """
        axis = None if axis in (None, "", "ALL") else str(axis).upper()
        if axis is not None and axis not in ("Z", "ROT", "A1", "A2"):
            self.log(f"RESET COORDINATES ignored — unknown axis {axis!r}.", tag="warn")
            return
        if self.motion_locked or self.is_running or self.is_homing:
            self.log("RESET COORDINATES ignored — something is moving. Stop first.",
                     tag="warn")
            return
        if self.jog_active:
            self.log("RESET COORDINATES ignored — a jog axis is held. Release it first.",
                     tag="warn")
            return

        if axis is None:
            question = (
                "Set the CURRENT position as the coordinate origin?\n\n"
                "Every step counter (ZM / RM / A1M / A2M) is zeroed where it "
                "stands, and soft limits start being enforced from this point.\n\n"
                "This command MEASURES NOTHING — it trusts that you have jogged "
                "the machine to the reference pose (arms fully retracted, lift at "
                "the bottom, turntable at 0°). Get it wrong and every later "
                "absolute move is offset by exactly that error."
                "\n\nContinue?")
        else:
            question = (
                f"Zero the {axis} counter ONLY, at its current position?\n\n"
                f"The other three axes are left exactly as they are.\n\n"
                f"This does not complete the machine reference, so it does not "
                f"switch the soft limits on by itself."
                f"\n\nContinue?")
        if not messagebox.askyesno("Reset coordinates", question):
            return

        self.send("RESET_COORD" if axis is None else f"RESET_COORD:{axis}")

        if axis is not None:
            # Mirror the single axis, nothing else.
            if axis == "Z":
                self.sim_z = Z_HOME_MM
                self.z_limit = {k: False for k in self.z_limit}
                self.current_joints[0] = Z_HOME_MM
            elif axis == "ROT":
                self.sim_rot = ROT_HOME_DEG
                self.rot_limit = {k: False for k in self.rot_limit}
                self.current_joints[1] = ROT_HOME_DEG
            elif axis == "A1":
                self.sim_a1 = ARM_HOME_DEG
                self.current_joints[2] = ARM_HOME_DEG
            else:
                self.sim_a2 = ARM_HOME_DEG
                self.current_joints[3] = ARM_HOME_DEG
            self._update_jog_readout()
            self._update_p2p_telemetry(*self.current_joints, pct=0)
            self._refresh_jog_status()
            self.status_var.set(f"READY — {axis} coordinate reset.")
            self.log(f"RESET COORDINATES — {axis} only, zeroed at its current "
                     f"position. The other three axes are unchanged."
                     + ("" if self.is_homed else
                        " The machine is still UNREFERENCED overall, so soft limits "
                        "stay suspended."))
            return

        # Mirror it locally so readouts, simulation and soft limits all
        # agree with the board from this instant. Leaving GUI on old
        # numbers is how the two drift apart.
        self.sim_rot = ROT_HOME_DEG
        self.sim_a1 = ARM_HOME_DEG
        self.sim_a2 = ARM_HOME_DEG
        self.sim_z = Z_HOME_MM
        self.rot_limit = {k: False for k in self.rot_limit}
        self.z_limit = {k: False for k in self.z_limit}
        self.current_joints = [Z_HOME_MM, ROT_HOME_DEG, ARM_HOME_DEG, ARM_HOME_DEG]
        self.is_homed = True
        self._update_jog_readout()
        self._update_p2p_telemetry(*self.current_joints, pct=0)
        self._refresh_jog_status()

        self.status_var.set("READY — coordinates reset at the current position.")
        self.log(f"RESET COORDINATES — ZM=0 mm, RM=0°, "
                 f"A1M=A2M={ARM_HOME_DEG:.0f} motor° (fold 0°, R = 133.2 mm). "
                 f"Soft limits are now ACTIVE.")

    def _set_motion_locked(self, locked):
        """Disables everything except STOP/ESTOP during RUN/HOME."""
        self.motion_locked = bool(locked)
        for w in self.motion_lock_widgets:
            w.set_enabled(not locked)
        for pad in self.jog_pads.values():
            pad.set_enabled(not locked)

    def home(self):
        if self.motion_locked:
            self.log("HOME ignored — a motion is already running.", tag="warn")
            return

        self._release_all_jog_axes()
        self._cancel_job("anim_job")
        self.is_running = False

        self.is_homing = True
        self._set_motion_locked(True)
        self.send("HOME")

        self.jog_status_var.set("Homing...")
        self.status_var.set("HOMING — waiting for the PLC to return DONE...")
        # HOME is now a request to the PLC, which owns the reference
        # position — this controller doesn't drive the axes itself.
        self.log("HOME sent. ClearCore holds its IO-0 output ON into the PLC's X0 "
                 "input — a wire, not a network message — and waits for DONE (M1). "
                 "Every other motion control is locked except ESTOP.")

        if not self._hardware_live():
            self._schedule("_home_sim_job", 800, self._simulate_home_complete)

    _home_clicked = home

    def _simulate_home_complete(self):
        self._home_sim_job = None
        self._on_home_complete(simulated=True)

    def reset_position(self):
        """Drives to (0,0,0,0) under the board's own motor control.

        NOT the PLC HOME cycle: never touches is_homing or any
        PLC-specific state, like reset_coordinates() — but unlike
        reset_coordinates() it actually moves the machine, like home().
        """
        if self.motion_locked:
            self.log("RESET POSITION ignored — a motion is already running.",
                     tag="warn")
            return
        if not messagebox.askyesno(
                "Reset position",
                "Drive the machine to (0,0,0,0) under its own motor control?\n\n"
                "This is NOT the PLC HOME cycle — it does not wait for the PLC, "
                "and it skips the M30..M32 limit block a normal P2P leg respects. "
                "Taught soft limits still apply.\n\nContinue?"):
            return

        self._release_all_jog_axes()
        self._cancel_job("anim_job")
        self.is_running = False
        self._set_motion_locked(True)
        self.send("RESET_POSITION")

        self.status_var.set("RESETTING POSITION — driving to (0,0,0,0)...")
        self.log("RESET_POSITION sent — no PLC handshake, M30..M32 limit block "
                 "skipped.")

        if not self._hardware_live():
            self._schedule("_reset_position_sim_job", 800,
                           self._simulate_reset_position_complete)

    def _simulate_reset_position_complete(self):
        self._reset_position_sim_job = None
        self._on_reset_position_complete(simulated=True)

    def restart_app(self):
        """Relaunches the application in place.

        Refused while anything is moving, for the same reason the theme
        switch is: process is about to be replaced, and an axis under
        power with no controller attached is a runaway nothing in this
        app can stop. Board's own jog watchdog would eventually catch it,
        but "eventually" is not a stop button.

        On the way out sends BYE and closes the port, so board sees a
        clean disconnect rather than a host that simply vanished.
        """
        if self.motion_locked or self.is_running or self.is_homing or self.jog_active:
            self.log("Restart refused — the machine is moving. Stop first.",
                     tag="warn")
            return

        if not messagebox.askyesno(
                "Restart",
                "Restart the application?\n\n"
                "The serial port will be closed and reopened, so the board "
                "loses its speed and boundary settings until the handshake "
                "re-sends them. Anything not applied will be lost."):
            return

        self.log("Restarting…")
        try:
            self.send("BYE", log_tx=False)
            self._rx_generation += 1
            self._cancel_jobs("_ping_timeout_job", "_heartbeat_job", "_hb_blink_job",
                              "anim_job", "_jog_sim_job", "_jog_hb_job",
                              "_home_sim_job", "_reset_position_sim_job",
                              "_disconnect_job")
            self._close_port()
        except Exception:
            pass

        try:
            self.root.destroy()
        except Exception:
            pass

        # execv REPLACES this process, nothing after it runs. Same
        # interpreter and argv is what makes it a restart rather than a
        # second instance fighting for the same COM port.
        os.execv(sys.executable, [sys.executable] + sys.argv)

    def emergency_stop_all(self):
        """The only ESTOP code path in the app."""
        for _ in range(3):
            self.send("ESTOP")

        self._release_all_jog_axes(send_stop=False)   # ESTOP already covers it
        self.is_running = False
        self.is_homing = False
        self._cancel_jobs("anim_job", "_home_sim_job", "_reset_position_sim_job")
        self._set_motion_locked(False)

        self.jog_dot.itemconfig(self._jog_dot_id, fill=ACCENT_RED)
        self.jog_status_var.set("EMERGENCY STOP SENT")
        self.status_var.set("STOPPED — Emergency Stop Triggered!")
        self.log("EMERGENCY STOP (ESTOP x3) — all motion halted.", tag="warn")

    def set_mode(self, mode):
        if mode == self.mode:
            return
        if self.motion_locked:
            self.log("Cannot change mode while a program is running. Stop first.", tag="warn")
            return

        self._stop_all_motion_for_mode_switch()
        self.mode = mode
        if mode == "P2P":
            self.jog_frame.pack_forget()
            self.p2p_frame.pack(fill="both", expand=True)
            self.motion_title_label.config(text="3. MOTION CONTROL — POINT TO POINT")
            self._style_mode_buttons(p2p_active=True)
        else:
            self.p2p_frame.pack_forget()
            self.jog_frame.pack(fill="both", expand=True)
            self.motion_title_label.config(text="3. MOTION CONTROL — JOYSTICK")
            self._style_mode_buttons(p2p_active=False)
        self.root.focus_set()
        self.log(f"Mode switched to {mode}.")

    def _stop_all_motion_for_mode_switch(self):
        if self.is_running:
            self.is_running = False
            self._cancel_job("anim_job")
            self.send("STOP")
        self._release_all_jog_axes()
        self.jog_dot.itemconfig(self._jog_dot_id, fill=TEXT_MUTED)

    def on_close(self):
        self.send("BYE", log_tx=False)
        self._rx_generation += 1
        self._cancel_jobs("_ping_timeout_job", "_heartbeat_job", "_hb_blink_job",
                          "anim_job", "_jog_sim_job", "_home_sim_job",
                          "_reset_position_sim_job", "_disconnect_job")
        self.root.after(120, self._finish_close)

    _on_close = on_close

    def _finish_close(self):
        self._close_port()
        try:
            self.root.destroy()
        except Exception:            # pragma: no cover
            pass
