"""Xbox controller support for JOYSTICK (jog) mode.

OPTIONAL, needs pygame (pygame-ce) for SDL game-controller API. Missing/
unplugged -> app runs same as before. ADDITIONAL input path onto same
jog_start()/jog_stop() calls keyboard and on-screen pads use, not a
second motion path with own safety logic. LINK, taught limits, PLC sensor
warning, motion_locked, dead-man heartbeat all apply unchanged — board
never sees where command came from.

MAPPING fixed, not user-editable (unlike keyboard layout in keybinds.py).
Keyboard has dozens of equally-plausible keys, operator habit should
decide; controller has 8 relevant inputs w/ physical factory-fixed names
— nothing here for a rebind UI to offer.

    LT (left trigger)   -> A2_FWD   (AM2 extend)
    LB (left bumper)    -> A2_BACK  (AM2 retract)
    RT (right trigger)  -> A1_FWD   (AM1 extend)   -- mirrors LT/LB
    RB (right bumper)   -> A1_BACK  (AM1 retract)
    X                   -> ROT_CCW  (X sits left of button diamond)
    B                   -> ROT_CW   (B sits right of button diamond)
    Y                   -> Z_UP     (Y sits above diamond)
    A                   -> Z_DOWN   (A sits below diamond)

pygame reports Xbox controllers via SDL GAME CONTROLLER api
(`pygame._sdl2.controller`) -> fixed names ("x", "b", "leftshoulder", ...)
not raw driver-dependent indices — same button same name regardless of
backend (XInput/DirectInput). Triggers come back as axis 0..32768 (SDL
range, not -1..1 pygame joystick axes use), need threshold not bool.
"""

GAMEPAD_POLL_MS = 50            # matches JOG_SIM_TICK_MS cadence
TRIGGER_THRESHOLD = 8000        # of 0..32768, ~quarter pull

BUTTON_COMMAND = {
    "rightshoulder": "A1_BACK",
    "leftshoulder": "A2_BACK",
    "x": "ROT_CCW",
    "b": "ROT_CW",
    "y": "Z_UP",
    "a": "Z_DOWN",
}

TRIGGER_COMMAND = {
    "lefttrigger": "A2_FWD",
    "righttrigger": "A1_FWD",
}

# SDL axis name -> CONTROLLER_AXIS_* suffix pygame exposes it under
_TRIGGER_AXIS_CONST = {
    "lefttrigger": "TRIGGERLEFT",
    "righttrigger": "TRIGGERRIGHT",
}


def gamepad_commands_from_state(buttons, triggers, threshold=TRIGGER_THRESHOLD):
    """{button name: bool} + {trigger name: 0..32768} -> set of jog commands.

    Plain function, separate from pygame polling below, so mapping can be
    tested with synthetic input — no controller or pygame needed to run test.
    """
    active = set()
    for name, cmd in BUTTON_COMMAND.items():
        if buttons.get(name):
            active.add(cmd)
    for name, cmd in TRIGGER_COMMAND.items():
        if triggers.get(name, 0) >= threshold:
            active.add(cmd)
    return active


class GamepadMixin:
    def _init_gamepad(self):
        """Starts polling if pygame importable. Safe to call when not —
        app just has no controller input, same as before feature existed."""
        self._gamepad_job = None
        self._gamepad_active = set()     # commands currently held by pad
        self._gamepad_controller = None
        if self._gamepad_try_import():
            self._schedule("_gamepad_job", GAMEPAD_POLL_MS, self._gamepad_poll)

    @staticmethod
    def _gamepad_try_import():
        try:
            import pygame
            import pygame._sdl2.controller as sdl_controller
        except Exception:
            return False
        try:
            if not pygame.get_init():
                pygame.init()
            if not sdl_controller.get_init():
                sdl_controller.init()
        except Exception:
            return False
        return True

    def _gamepad_quit(self):
        """Releases SDL controller subsystem on shutdown. Best-effort —
        process exiting either way."""
        self._cancel_job("_gamepad_job")
        try:
            import pygame._sdl2.controller as sdl_controller
            sdl_controller.quit()
        except Exception:
            pass

    def _gamepad_jog_enabled(self):
        """Mirrors _jog_keys_enabled() minus text-focus check: controller
        button press can't land in an Entry widget like a keypress can,
        nothing there to guard against."""
        return self.mode == "JOG" and not self.motion_locked

    def _gamepad_poll(self):
        self._gamepad_job = None
        import pygame
        import pygame._sdl2.controller as sdl_controller
        pygame.event.pump()

        controller = self._gamepad_acquire(sdl_controller)
        if controller is None:
            self._gamepad_apply(set())
        else:
            self._gamepad_read(pygame, controller)

        self._schedule("_gamepad_job", GAMEPAD_POLL_MS, self._gamepad_poll)

    def _gamepad_acquire(self, sdl_controller):
        """Returns live Controller, reconnecting/detecting as needed."""
        c = self._gamepad_controller
        if c is not None:
            try:
                if c.attached():
                    return c
            except Exception:
                pass
            self._gamepad_controller = None
            self.log("Xbox controller disconnected.", tag="warn")

        if sdl_controller.get_count() == 0:
            return None
        try:
            new = sdl_controller.Controller(0)
        except Exception:
            return None
        self._gamepad_controller = new
        try:
            name = new.name
        except Exception:
            name = "controller"
        self.log(f"Xbox controller connected: {name}.")
        return new

    def _gamepad_read(self, pygame, controller):
        try:
            buttons = {
                name: bool(controller.get_button(
                    getattr(pygame, f"CONTROLLER_BUTTON_{name.upper()}")))
                for name in BUTTON_COMMAND
            }
            triggers = {
                name: controller.get_axis(
                    getattr(pygame, f"CONTROLLER_AXIS_{_TRIGGER_AXIS_CONST[name]}"))
                for name in TRIGGER_COMMAND
            }
        except Exception:
            # read failing mid-poll (e.g. pad yanked out between
            # attached() and get_button()) must not crash poll loop —
            # treat this tick as nothing held
            self._gamepad_controller = None
            self._gamepad_apply(set())
            return

        wanted = (gamepad_commands_from_state(buttons, triggers)
                  if self._gamepad_jog_enabled() else set())
        self._gamepad_apply(wanted)

    def _gamepad_apply(self, wanted):
        """Reconciles `wanted` against what pad currently holds, going
        through jog_start()/jog_stop() like a key press — so LINK, limit
        sensors, heartbeat all apply unchanged."""
        from ..config import JOG_STOP_COMMAND

        for cmd in self._gamepad_active - wanted:
            if cmd in self.jog_pads:
                self.jog_pads[cmd].key_deactivate()
            self.jog_stop(cmd, JOG_STOP_COMMAND.get(cmd))
        for cmd in wanted - self._gamepad_active:
            if cmd in self.jog_pads:
                self.jog_pads[cmd].key_activate()
            self.jog_start(cmd)
        self._gamepad_active = wanted
