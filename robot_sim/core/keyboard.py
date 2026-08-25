"""Global keyboard bindings for jogging, HOME and the panic key."""

import tkinter as tk
from tkinter import ttk

from .. import keybinds
from ..config import JOG_STOP_COMMAND


class KeyboardMixin:
    def _bind_keys(self):
        """(Re)binds every jog key from CURRENT layout.

        Safe to call again after rebinding: previous bindings cleared
        first, because bind_all only replaces binding for SAME sequence —
        key no longer used would otherwise keep working and quietly move
        axis nobody expects.
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
        # HOME bound from keybinds.HOME_KEY not a literal, so RESERVED key
        # and key that actually homes are same one by construction. had
        # drifted: RESERVED_KEYS said backspace while this bound "h"/"H",
        # so homing fired on unprotected letter that a jog axis could also
        # take — one keypress would jog and home at same time.
        self.root.bind_all(f"<KeyPress-{keybinds.HOME_KEY}>",
                           lambda e: self._home_key_pressed())

        # ENTER runs loaded P2P program, keyboard equiv of RUN PROGRAM.
        # gated on mode+focus like HOME: only in P2P, not while text field
        # focused, so finishing a coordinate with Enter can't start
        # unattended run. p2p_run_program() still checks loaded_program
        # and motion_locked, same as button.
        self.root.bind_all("<KeyPress-Return>",
                           lambda e: self._run_key_pressed())

        # ESC toggles Settings. deliberately NOT gated on
        # _jog_keys_enabled(): not a motion command, being unable to reach
        # settings because a text field has focus would be its own
        # annoyance.
        #
        # ONLY Escape binding in app. bind_all is application-wide, fires
        # for Settings window too — second binding there made one keypress
        # close then immediately reopen it. "break" stops propagation.
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
        # deliberately NOT gated on _jog_keys_enabled(): if mode/focus
        # changed while key held, release must still stop axis.
        if start_cmd in self.jog_pads:
            self.jog_pads[start_cmd].key_deactivate()

        # RUNAWAY BUG: used to bail when `start_cmd` not in jog_active.
        # With LINK on, pressing W queues PROMOTED command ("ARM_FWD") not
        # "A1_FWD" — check failed, returned, no stop ever sent. axis kept
        # moving until ESTOP. jog_stop() already resolves both spellings,
        # must be called unconditionally and allowed to decide.
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
        """ESC toggles Settings window.

        Refused mid-motion: Settings can change speeds/travel limits,
        re-teaching envelope while axis moving not something a stray
        keypress should start.

        Returns "break" so event stops here. Combined with being app's
        only Escape binding, guarantees one keypress produces exactly one
        toggle — close-then-reopen bug was two handlers each doing their
        half on same event.
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
