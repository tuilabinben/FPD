"""RobotControlApp — assembles every mixin into the actual window."""

import tkinter as tk

from . import hidpi
from .config import (
    ARM_CONFIGS,
    ARM_HOME_DEG,
    ROT_HOME_DEG,
    Z_HOME_MM,
    DEFAULT_KD,
    DEFAULT_KI,
    DEFAULT_KP,
    DEFAULT_N_FILTER,
    LIMIT_FIELDS,
    LIMITS_ENABLED_KEY,
    LIMIT_ENFORCE_KEYS,
    DEFAULT_LIMITS_ENABLED,
    PLC_LINK_ENABLED_KEY,
    DEFAULT_PLC_LINK_ENABLED,
    DEFAULT_LIMIT_ENFORCED,
    PLC_SENSOR_PANEL,
    PID_LOCK_KEYS,
    SPEED_FIELDS,
    ACCEL_FIELDS,
    WINDOW_GEOMETRY,
    WINDOW_MIN_SIZE,
    WINDOW_TITLE,
)
from .core import (
    EventLogMixin,
    GamepadMixin,
    HeartbeatMixin,
    JogControlMixin,
    KeyboardMixin,
    P2PControlMixin,
    ProtocolMixin,
    SafetyMixin,
    SerialLinkMixin,
    TimerMixin,
)
from .theme import BG
from .ui import (CoordResetRowMixin, SensorPanelMixin, XYBoardMixin,
                 JogPanelMixin, LayoutMixin, P2PPanelMixin,
                 SettingsDialogMixin)


class RobotControlApp(
    TimerMixin,
    EventLogMixin,
    SerialLinkMixin,
    HeartbeatMixin,
    ProtocolMixin,
    JogControlMixin,
    GamepadMixin,
    P2PControlMixin,
    SafetyMixin,
    KeyboardMixin,
    LayoutMixin,
    CoordResetRowMixin,
    SensorPanelMixin,
    XYBoardMixin,
    P2PPanelMixin,
    JogPanelMixin,
    SettingsDialogMixin,
):
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title(WINDOW_TITLE)
        self.root.configure(bg=BG)
        # Geometry is written for 96 DPI, so it has to be scaled as well —
        # otherwise a DPI-aware window opens at half its intended size.
        w, h = (int(v) for v in WINDOW_GEOMETRY.lower().split("x"))
        self.root.geometry(f"{hidpi.px(w)}x{hidpi.px(h)}")
        self.root.minsize(hidpi.px(WINDOW_MIN_SIZE[0]), hidpi.px(WINDOW_MIN_SIZE[1]))

        self._init_state()
        self._configure_styles()
        self._build_ui()
        self._bind_keys()
        self._init_gamepad()
        # Paint the readouts from the real initial pose instead of leaving the
        # hard-coded "0.00" placeholders, which disagreed with the state.
        self._update_jog_readout()
        self._update_p2p_telemetry(*self.current_joints, pct=0)
        # Anything the settings loader wanted to say had to wait until the
        # log widget existed.
        self._flush_settings_notes()
        self.refresh_com_ports()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ══════════════════════════════════════════════════════════════════
    # LIVE THEME SWITCH
    #
    # A Tk widget's colours are fixed when it is constructed, so changing
    # scheme means throwing the widget tree away and building it again.
    # Everything that is not a widget — the serial link, the settings, the
    # machine's position, the log — survives untouched; the job here is to
    # carry the widget-held state across the gap so the rebuild is
    # invisible apart from the colours.
    # ══════════════════════════════════════════════════════════════════
    def apply_theme(self, palette_name):
        """Switches colour scheme in place. Returns True if it happened.

        REFUSED while anything is moving. Rebuilding the window destroys
        the jog pads and the ESTOP button, and doing that with an axis
        under power — even for the few milliseconds it takes — removes the
        operator's ability to stop the machine. Waiting is free; that is
        not.
        """
        from .theme import PALETTE_NAME, apply_palette

        if palette_name == PALETTE_NAME:
            return False
        if self.motion_locked or self.is_running or self.is_homing or self.jog_active:
            self.log("Cannot change the colour scheme while the machine is "
                     "moving — the rebuild would briefly remove the stop "
                     "controls. Stop first.", tag="warn")
            return False

        ok, _updated = apply_palette(palette_name)
        if not ok:
            self.log(f"Unknown colour scheme: {palette_name!r}", tag="error")
            return False
        self._rebuild_ui()
        return True

    def _snapshot_ui_state(self):
        """Everything living in a widget that must survive the rebuild.

        StringVars are collected generically rather than by name: there
        are ~30 of them across the panels, and a hand-written list would
        silently lose whichever one got added last.
        """
        snap = {"vars": {}, "log": "", "mode": self.mode}
        for attr in dir(self):
            if not (attr.endswith("_v") or attr.endswith("_var")):
                continue
            try:
                value = getattr(self, attr)
            except Exception:
                continue
            if isinstance(value, (tk.StringVar, tk.DoubleVar, tk.IntVar)):
                snap["vars"][attr] = value.get()
        try:
            snap["log"] = self.log_text.get("1.0", "end-1c")
        except Exception:
            pass
        return snap

    def _restore_ui_state(self, snap):
        for attr, value in snap["vars"].items():
            target = getattr(self, attr, None)
            if isinstance(target, (tk.StringVar, tk.DoubleVar, tk.IntVar)):
                try:
                    target.set(value)
                except Exception:
                    pass
        if snap["log"]:
            self.log_text.config(state="normal")
            self.log_text.delete("1.0", "end")
            self.log_text.insert("1.0", snap["log"])
            self.log_text.see("end")
            self.log_text.config(state="disabled")

    def _rebuild_ui(self):
        from .theme import BG as NEW_BG

        snap = self._snapshot_ui_state()

        # Timers hold references to widgets that are about to stop
        # existing. The heartbeat is restarted below; the rest are either
        # idle or belong to motion, which we have already refused to
        # rebuild through.
        self._cancel_jobs("_ping_timeout_job", "_heartbeat_job", "_hb_blink_job",
                          "_jog_sim_job", "_jog_hb_job", "_home_sim_job",
                          "anim_job")

        for child in self.root.winfo_children():
            child.destroy()

        # Collections that hold destroyed widgets have to be emptied, or
        # the rebuilt UI appends to a list of dead references and every
        # later set_enabled() raises.
        self.jog_pads = {}
        self.arm_buttons = {}
        self.motion_lock_widgets = []
        # Both motion panels append to these, so they have to be emptied
        # here too or the rebuilt UI stacks live widgets on dead ones and
        # the next refresh walks destroyed Tk objects.
        self.coord_reset_buttons = []
        self.plc_sensor_lamps = {}
        self.plc_home_state_lamps = []

        self.root.configure(bg=NEW_BG)
        self._configure_styles()
        self._build_ui()
        # bind_all without add="+" REPLACES the previous binding, so this
        # cannot accumulate duplicate handlers across repeated switches.
        self._bind_keys()

        self._restore_ui_state(snap)
        self._update_jog_readout()
        self._update_p2p_telemetry(*self.current_joints, pct=0)
        self.refresh_com_ports()
        self._restore_connection_indicators()

        if snap["mode"] == "JOG":
            # set_mode() early-returns when the mode is unchanged, and the
            # freshly built UI is showing P2P, so nudge the field first.
            self.mode = "P2P"
            self.set_mode("JOG")

        from .theme import PALETTE_LABEL
        self.log(f"Colour scheme changed to “{PALETTE_LABEL}”.")

    def _restore_connection_indicators(self):
        """Repaints the three status LEDs and the connect button.

        The link itself was never interrupted — only the widgets showing
        it were — so this is presentation catching up with state that
        never changed.
        """
        from .theme import ACCENT_GREEN, ACCENT_MINT, ACCENT_RED, TEXT_MUTED
        from .widgets import set_led

        if self.is_connected:
            port = self.com_var.get()
            set_led(self.com_led_card, "OPEN", ACCENT_GREEN)
            self.btn_connect.set_config("DISCONNECT", ACCENT_RED, icon="🔌")
            if self.hw_confirmed:
                set_led(self.hw_led_card, "IO0 ON ✓", ACCENT_GREEN)
            else:
                set_led(self.hw_led_card, "NO SIGNAL", ACCENT_RED)
            if port:
                self.com_var.set(port)
            self._schedule_next_heartbeat()
        else:
            set_led(self.com_led_card, "CLOSED", ACCENT_RED)
            set_led(self.hw_led_card, "NO SIGNAL", ACCENT_RED)
            self._plc_link_lost()

    # ── state ────────────────────────────────────────────────────────
    def _init_state(self):
        # Serial / connection
        self.ser = None
        self.is_connected = False
        self.hw_confirmed = False
        # Reset per connection so a re-flash is noticed.
        self._warned_old_elbow_frame = False
        self._plc_led_state = None
        self._missed_beats = 0
        self._rx_generation = 0

        # Pending `after` jobs, all cancelled through TimerMixin
        self._ping_timeout_job = None
        self._heartbeat_job = None
        self._hb_blink_job = None
        self._jog_sim_job = None
        self._jog_hb_job = None
        self._home_sim_job = None
        self._disconnect_job = None
        self.anim_job = None

        # PID gains, mirrored to the board via SET_PID. One preset only,
        # with each gain individually lockable.
        self.settings = {
            "kp": DEFAULT_KP, "ki": DEFAULT_KI, "kd": DEFAULT_KD,
            "n": DEFAULT_N_FILTER,
        }
        for lock_key in PID_LOCK_KEYS:
            self.settings[lock_key] = False
        # Per-axis enforcement and the master enforcement switch. Both live
        # in the same settings dict so they persist and are re-pushed to the
        # board on every handshake, like the limits are. Both default ON —
        # a fresh install must not be less safe than a configured one.
        for enforce_key in LIMIT_ENFORCE_KEYS:
            self.settings[enforce_key] = DEFAULT_LIMIT_ENFORCED
        self.settings[LIMITS_ENABLED_KEY] = DEFAULT_LIMITS_ENABLED
        self.settings[PLC_LINK_ENABLED_KEY] = DEFAULT_PLC_LINK_ENABLED
        # Per-motor percentages of the fixed reference speed — SPEED_FIELDS.
        for key, (_label, _unit, default, _lo, _hi) in SPEED_FIELDS.items():
            self.settings[key] = default
        # Per-motor acceleration percentages — ACCEL_FIELDS, independent of
        # the speed ones above.
        for key, (_label, _unit, default, _lo, _hi) in ACCEL_FIELDS.items():
            self.settings[key] = default
        # Operator-defined travel limits — see LIMIT_FIELDS.
        for key, spec in LIMIT_FIELDS.items():
            self.settings[key] = spec[6]
        self._settings_dlg = None
        # Settings saved from a previous run win over the defaults above.
        self._load_settings_file()

        # P2P
        self.is_running = False
        self.is_homing = False
        self.motion_locked = False
        # Until a real HOME reference exists the joint positions are
        # unknown, so soft limits must not be enforced — see _axis_bounds().
        self.is_homed = False
        self.loaded_program = None
        self.arm_config = ARM_CONFIGS[0]
        self.arm_buttons = {}
        self.motion_lock_widgets = []
        # Filled by both motion panels — see ui/coord_reset.py.
        self.coord_reset_buttons = []
        # ── THE ONE LIVE POSE, SHARED BY JOG AND P2P ─────────────────
        #
        # [d1 mm, rot deg, a1m motor deg, a2m motor deg], starting at the
        # folded home pose.
        #
        # sim_z / sim_rot / sim_a1 / sim_a2 are PROPERTIES onto this list,
        # not separate variables. They used to be a second copy: jog
        # integrated sim_*, P2P integrated current_joints, and nothing kept
        # them together — so jogging the machine and then switching to P2P
        # ran the program from wherever P2P last left off, not from where
        # the arm actually was. One store makes that impossible.
        self.current_joints = [Z_HOME_MM, ROT_HOME_DEG, ARM_HOME_DEG, ARM_HOME_DEG]

        # Jog
        self.jog_active = set()
        self.jog_pads = {}
        self.arms_linked = False         # LINK toggle, see toggle_arm_link()
        self.boost_index = 0
        self.rot_limit = {"ROT_CW": False, "ROT_CCW": False}
        self.z_limit = {"Z_UP": False, "Z_DOWN": False}
        # Live M5..M8, from the board's [PLC_STATE] / [PLC_HOME] lines.
        # M7/M8 are broken and are never set true from a reading — see
        # PLC_SENSOR_PANEL.
        # Bits start False but plc_sensor_data_seen starts FALSE too, and it
        # is that flag — not the bits — which the lamps and every consumer
        # read. Until a device read lands the panel says NO DATA.
        self.plc_sensor_state = {bit: False for bit, *_r in PLC_SENSOR_PANEL}
        self.plc_sensor_data_seen = False
        self._plc_sensor_seen_at = None
        self._plc_home_state_prev = False
        self.plc_sensor_lamps = {}
        self.plc_home_state_lamps = []

        self.mode = "P2P"

    # ── backwards-compatible aliases for the pre-refactor names ──────
    @property
    def elbow_config(self):
        return self.arm_config

    @elbow_config.setter
    def elbow_config(self, value):
        self.arm_config = value

    @property
    def elbow_buttons(self):
        return self.arm_buttons

    # ── the shared live pose ─────────────────────────────────────────
    # One list, four names. Every readout on both panels reads through
    # these, so the two modes cannot disagree about where the machine is.
    @property
    def sim_z(self):
        return self.current_joints[0]

    @sim_z.setter
    def sim_z(self, value):
        self.current_joints[0] = float(value)

    @property
    def sim_rot(self):
        return self.current_joints[1]

    @sim_rot.setter
    def sim_rot(self, value):
        self.current_joints[1] = float(value)

    @property
    def sim_a1(self):
        return self.current_joints[2]

    @sim_a1.setter
    def sim_a1(self, value):
        self.current_joints[2] = float(value)

    @property
    def sim_a2(self):
        return self.current_joints[3]

    @sim_a2.setter
    def sim_a2(self, value):
        self.current_joints[3] = float(value)

    @property
    def sim_arm(self):
        """Legacy single-arm alias — reports the more-extended elbow."""
        return max(self.sim_a1, self.sim_a2)

    @sim_arm.setter
    def sim_arm(self, value):
        self.sim_a1 = self.sim_a2 = value


def main():
    # DPI awareness must be declared BEFORE the first widget exists.
    # Without it Windows renders the whole window at 96 DPI and then
    # bitmap-stretches it to the display's real size — which is what made
    # everything look low-resolution no matter how carefully it was drawn.
    hidpi.enable()
    root = tk.Tk()
    hidpi.enable(root)          # second pass: now that a window exists,
                                # read the real DPI and set Tk's scaling
    RobotControlApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
