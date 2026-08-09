"""Global keyboard bindings for jogging, HOME and the panic key."""

import tkinter as tk
from tkinter import ttk

from .. import keybinds
from ..config import JOG_STOP_COMMAND


class KeyboardMixin:
    def _bind_keys(self):
        """(Re)binds every jog key from the CURRENT layout.

        Safe to call again after a rebinding: the previous bindings are
        cleared first, because bind_all only replaces a binding for the
        SAME sequence — a key that is no longer used would otherwise keep
        working and quietly move an axis nobody expects.
        """
        for key in getattr(self, "_bound_jog_keys", ()):
            for k in ([key, key.upper()] if len(key) == 1 else [key]):
                self.root.unbind_all(f"<KeyPress-{k}>")
                self.root.unbind_all(f"<KeyRelease-{k}>")

        keymap = keybinds.to_tk_keymap(keybinds.active_map())
        self._bound_jog_keys = tuple(keymap)
        for key, start_cmd in keymap.items():
            keys = [key, key.upper()] if len(key) == 1 else [key]
            for k in keys:
                self.root.bind_all(f"<KeyPress-{k}>",
                                   lambda e, s=start_cmd: self._key_press(s))
                self.root.bind_all(f"<KeyRelease-{k}>",
                                   lambda e, s=start_cmd: self._key_release(s))

        self.root.bind_all("<KeyPress-space>", lambda e: self.emergency_stop_all())
        # HOME is bound from keybinds.HOME_KEY rather than a literal, so the
        # key that is RESERVED and the key that actually homes are the same
        # one by construction. They had drifted: RESERVED_KEYS said
        # backspace while this bound "h"/"H", so homing fired on a letter
        # that was no longer protected and could also be taken by a jog
        # axis — one keypress would then jog and home at the same time.
        self.root.bind_all(f"<KeyPress-{keybinds.HOME_KEY}>",
                           lambda e: self._home_key_pressed())

        # ENTER runs the loaded P2P program — the keyboard equivalent of
        # pressing RUN PROGRAM. Gated on mode and focus the same way HOME
        # is: only in P2P, and not while a text field has focus, so typing
        # a coordinate and finishing the value with Enter cannot start an
        # unattended run. p2p_run_program() itself still checks
        # loaded_program and motion_locked, same as the button.
        self.root.bind_all("<KeyPress-Return>",
                           lambda e: self._run_key_pressed())

        # ESC toggles Settings. Deliberately NOT gated on
        # _jog_keys_enabled(): it is not a motion command, and being unable
        # to reach the settings because a text field has focus would be its
        # own small annoyance.
        #
        # This is the ONLY Escape binding in the app. bind_all is
        # application-wide, so it fires for the Settings window too — adding
        # a second binding on that window made one keypress close and then
        # immediately reopen it. "break" stops any further propagation.
        self.root.bind_all("<KeyPress-Escape>", self._escape_pressed)

    def _jog_keys_enabled(self):
        if self.mode != "JOG":
            return False
        if self.motion_locked:
            return False
        focused = self.root.focus_get()
        if isinstance(focused, (tk.Entry, ttk.Entry, ttk.Combobox, tk.Text, tk.Spinbox)):
            return False
        return True

    def _key_press(self, start_cmd):
        if not self._jog_keys_enabled():
            return
        if start_cmd in self.jog_pads:
            self.jog_pads[start_cmd].key_activate()
        self.jog_start(start_cmd)

    def _key_release(self, start_cmd):
        # Deliberately NOT gated on _jog_keys_enabled(): if the mode or focus
        # changed while a key was held, the release must still stop the axis.
        if start_cmd in self.jog_pads:
            self.jog_pads[start_cmd].key_deactivate()

        # RUNAWAY BUG: this used to bail out when `start_cmd` was not in
        # jog_active. With LINK on, pressing W queues the PROMOTED command
        # ("ARM_FWD"), not "A1_FWD" — so the check failed, the function
        # returned, and no stop was ever sent. The axis kept moving until
        # ESTOP. jog_stop() already resolves both spellings, so it must be
        # called unconditionally and allowed to decide.
        self.jog_stop(start_cmd, JOG_STOP_COMMAND.get(start_cmd))

    def _home_key_pressed(self):
        if not self._jog_keys_enabled():
            return
        self.home()

    def _run_key_enabled(self):
        if self.mode != "P2P":
            return False
        focused = self.root.focus_get()
        if isinstance(focused, (tk.Entry, ttk.Entry, ttk.Combobox, tk.Text, tk.Spinbox)):
            return False
        return True

    def _run_key_pressed(self):
        if not self._run_key_enabled():
            return
        self.p2p_run_program()

    def _escape_pressed(self, _event=None):
        """ESC toggles the Settings window.

        Refused mid-motion: Settings can change speeds and travel limits,
        and re-teaching an envelope while an axis is moving is not something
        a stray keypress should be able to start.

        Returns "break" so the event stops here. Combined with this being
        the app's only Escape binding, that guarantees one keypress
        produces exactly one toggle — the close-then-reopen bug was two
        handlers each doing their half of the job on the same event.
        """
        if self.motion_locked:
            self.log("Settings unavailable while a program is running. "
                     "Stop first.", tag="warn")
            return "break"
        dlg = getattr(self, "_settings_dlg", None)
        if dlg is not None and dlg.winfo_exists():
            self._close_settings(dlg)
        else:
            self.open_settings_dialog()
        return "break"
