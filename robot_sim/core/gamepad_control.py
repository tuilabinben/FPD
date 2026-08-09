"""Xbox controller support for JOYSTICK (jog) mode.

OPTIONAL. This needs pygame (pygame-ce) for SDL's game-controller API. If
it is not installed, or nothing is plugged in, the app runs exactly as it
did before — this is an ADDITIONAL input path onto the same jog_start() /
jog_stop() calls the keyboard and the on-screen pads already use, not a
second motion path with its own safety logic. LINK, the taught limits, the
PLC sensor warning, motion_locked and the dead-man heartbeat all apply
unchanged, because the board never sees where a command came from.

MAPPING (fixed, not user-editable — unlike the keyboard layout in
keybinds.py). A keyboard has dozens of equally-plausible keys and the
operator's own habits are the only thing that should decide it; a
controller has eight relevant inputs with physical, factory-fixed names,
so there is nothing here for a rebinding UI to usefully offer.

    LT (left trigger)   -> A2_FWD   (AM2 extend)
    LB (left bumper)    -> A2_BACK  (AM2 retract)
    RT (right trigger)  -> A1_FWD   (AM1 extend)   -- mirrors LT/LB
    RB (right bumper)   -> A1_BACK  (AM1 retract)
    X                   -> ROT_CCW  (X sits left of the button diamond)
    B                   -> ROT_CW   (B sits right of the button diamond)
    Y                   -> Z_UP     (Y sits above the diamond)
    A                   -> Z_DOWN   (A sits below the diamond)

pygame reports Xbox controllers through SDL's GAME CONTROLLER api
(`pygame._sdl2.controller`), which gives fixed names ("x", "b",
"leftshoulder", ...) instead of raw, driver-dependent button/axis
indices — the same button always has the same name regardless of which
backend (XInput/DirectInput) enumerated the pad. Triggers come back as an
axis 0..32768 (SDL's own range, not the -1..1 pygame uses for joystick
axes), so they need a threshold rather than a bool.
"""

GAMEPAD_POLL_MS = 50            # matches JOG_SIM_TICK_MS's cadence
TRIGGER_THRESHOLD = 8000        # of 0..32768 — roughly a quarter pull

#: {SDL button name: jog command}
BUTTON_COMMAND = {
    "rightshoulder": "A1_BACK",
    "leftshoulder": "A2_BACK",
    "x": "ROT_CCW",
    "b": "ROT_CW",
    "y": "Z_UP",
    "a": "Z_DOWN",
}

#: {SDL trigger axis name: jog command}
TRIGGER_COMMAND = {
    "lefttrigger": "A2_FWD",
    "righttrigger": "A1_FWD",
}

#: SDL axis name -> the CONTROLLER_AXIS_* suffix pygame exposes it under.
_TRIGGER_AXIS_CONST = {
    "lefttrigger": "TRIGGERLEFT",
    "righttrigger": "TRIGGERRIGHT",
}


def gamepad_commands_from_state(buttons, triggers, threshold=TRIGGER_THRESHOLD):
    """{button name: bool} + {trigger name: 0..32768} -> set of jog commands.

    Kept as a plain function, separate from the pygame polling below, so
    the mapping can be tested with synthetic input and no controller (or
    even pygame) has to be present to run the test.
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
    # ── setup ─────────────────────────────────────────────────────────
    def _init_gamepad(self):
        """Starts polling if pygame is importable. Safe to call even when
        it is not — the app simply has no controller input, same as
        before this feature existed."""
        self._gamepad_job = None
        self._gamepad_active = set()     # commands currently held BY THE PAD
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
        """Releases the SDL controller subsystem on shutdown. Best-effort —
        the process is exiting either way."""
        self._cancel_job("_gamepad_job")
        try:
            import pygame._sdl2.controller as sdl_controller
            sdl_controller.quit()
        except Exception:
            pass

    # ── gating ────────────────────────────────────────────────────────
    def _gamepad_jog_enabled(self):
        """Mirrors _jog_keys_enabled() minus the text-focus check: a
        controller button press cannot land in an Entry widget the way a
        keypress can, so there is nothing there to guard against."""
        return self.mode == "JOG" and not self.motion_locked

    # ── polling ───────────────────────────────────────────────────────
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
        """Returns the live Controller, reconnecting/detecting as needed."""
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
            # A read failing mid-poll (e.g. the pad was yanked out between
            # attached() and get_button()) must not crash the poll loop —
            # just treat this tick as nothing held.
            self._gamepad_controller = None
            self._gamepad_apply(set())
            return

        wanted = (gamepad_commands_from_state(buttons, triggers)
                  if self._gamepad_jog_enabled() else set())
        self._gamepad_apply(wanted)

    # ── apply to the shared jog path ─────────────────────────────────
    def _gamepad_apply(self, wanted):
        """Reconciles `wanted` against what the pad is currently holding,
        going through jog_start()/jog_stop() exactly like a key press —
        so LINK, the limit sensors and the heartbeat all apply unchanged."""
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
