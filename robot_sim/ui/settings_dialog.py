"""Settings window — Speed, Boundaries, Controls, PID, Appearance.

STRUCTURE
---------
Five tabs, not one long scroll. The concerns are unrelated: you come here
to change a speed, or set up boundaries, or rebind a key, or look at
gains. A tab you are not in is not a tab you scroll past.

Each tab body scrolls on its own when its content is taller than the
window, and the scrollbar only appears when it is actually needed. The
action bar is packed against the bottom BEFORE the body, so APPLY is
never the thing that scrolls out of reach.

Each tab owns its own APPLY and DEFAULT, and they act ONLY on that tab.
A single global "reset to defaults" that also wiped the boundaries because
you wanted to undo a speed change is the kind of thing that costs someone
an afternoon of re-teaching positions.

WHAT IT DELIBERATELY DOESN'T DO
-------------------------------
  * No editable master RPM. It is a constant (config.MASTER_RPM) and only
    the percentages move. Two knobs that produce the same motion is one
    knob too many.
  * No P/PI/PD presets or controller-form selector — one gain set is what
    this machine uses, and the form field configured a structure that does
    not exist on an open-loop board.
  * No limit pair may invert or collapse. A MIN above its MAX makes every
    position illegal and the axis cannot be jogged back out of it.
  * Nothing is applied on keystroke. Edits are staged until APPLY, so a
    half-typed number is never a command.
"""

import json
import os
import sys
import tkinter as tk
from tkinter import messagebox, ttk

from ..config import (
    LIMIT_FIELDS,
    LIMITS_ENABLED_KEY,
    LIMIT_GROUPS,
    LIMIT_ENFORCE_BY_AXIS,
    LIMIT_ENFORCE_KEYS,
    LIMIT_KEYS,
    LIMIT_CAPTURE_ONLY,
    LIMIT_LIVE_SOURCE,
    LIMIT_PRESET_NAME_MAX,
    LIMIT_PRESETS_FILE,
    MASTER_ACC_RPM_S,
    MASTER_RPM,
    N_FILTER_MAX,
    N_FILTER_MIN,
    PID_FIELDS,
    PID_LOCK_KEYS,
    PID_PRESET,
    PID_PRESET_NAME,
    ARM_FRAME_V2_RESET_KEYS,
    ROT_FRAME_V4_RESET_KEYS,
    SETTINGS_FILE,
    SETTINGS_SCHEMA,
    SETTINGS_SCHEMA_KEY,
    SETTINGS_GEOMETRY,
    SETTINGS_MIN_SIZE,
    SPEED_FIELDS,
    SPEED_KEYS,
    SPEED_PREVIEW_BY_KEY,
    SPEED_WARN_PCT,
    ACCEL_FIELDS,
    ACCEL_KEYS,
    ACCEL_PREVIEW_BY_KEY,
    ACCEL_WARN_PCT,
)
from .. import keybinds
from ..hidpi import px
from ..palettes import DEFAULT_PALETTE, PALETTES, save_active_name
from ..theme import (
    FONT_READOUT,
    FONT_BUTTON,
    FONT_CAPTION,
    FONT_ENTRY,
    FONT_HINT,
    FONT_LABEL,
    FONT_MONO,
    FONT_SMALL,
    FONT_TITLE,
    ACCENT_GREEN,
    ACCENT_MINT,
    ACCENT_ORANGE,
    ACCENT_RED,
    INK_DARK,
    LED_BG,
    PANEL_BG,
    SURFACE,
    TEXT_DIM,
    TEXT_LIGHT,
    RADIUS_MD,
    RADIUS_SM,
    TEXT_MUTED,
    WARN_BORDER,
    BORDER,
    PALETTE_NAME,
    available_palettes,
)
from ..widgets import (
    NEUTRAL_BORDER,
    RoundedButton,
    RoundedFrame,
    make_inset_entry,
    make_well,
    set_entry_border,
)

SPEED_HELP = (
    "One fixed reference speed. Each motor runs at its own percentage of it.\n"
    f"100% = {MASTER_RPM:g} motor RPM. Per-axis gearing is already folded in, so the\n"
    "percentages are directly comparable across all three axes.\n"
    "Above the recommended value a field turns amber — still allowed, but past\n"
    "what has been tested stable on this machine."
)

LIMIT_HELP = (
    "Your working envelope, narrower than the machine's mechanical one.\n"
    "Jog to the position, then press SET HERE — no measuring, no arithmetic.\n"
    "Boundaries only take effect once the machine has a reference\n"
    "(HOME or RESET COORDINATES); before that, joint positions mean nothing.\n"
    "\n"
    "The two ELBOW rows are SET HERE only, and they take WHATEVER A1M_POS /\n"
    "A2M_POS reads — no range is enforced, so a four-figure angle is fine.\n"
    "The board's elbow angle rides on a gear ratio that has never been\n"
    "measured, so any envelope written here would be a guess, and a guess\n"
    "that rejects a pose you are physically standing at is worse than none.\n"
    "The two rows are also interchangeable: teach either end first, and\n"
    "whichever is lower is used as the lower limit.\n"
    "\n"
    "The button beside each heading says whether THAT boundary is switched\n"
    "on. ENFORCED = the axis stops there. NOT ENFORCED = the numbers are\n"
    "kept and still shown, but nothing stops the axis — and only that axis\n"
    "is affected. ENFORCEMENT at the bottom is the same switch for all\n"
    "four at once; while it is off, an axis that is individually on reads\n"
    "ON (MASTER OFF), because it is not protecting anything either.\n"
    "Switching a boundary off never changes or erases its values, and a\n"
    "switched-off boundary can still be edited and taught."
)

CONTROLS_HELP = (
    "Click a key box, then press the key you want. Escape cancels.\n"
    "If the key is already used, the two swap rather than leaving an axis\n"
    "unbound. SPACE (e-stop), ESC (settings) and H (home) are reserved.\n"
    "\n"
    "APPLY saves the layout to keybinds.json, so it is still here next time\n"
    "you start the app. DEFAULTS goes back to A/D · I/K · O/L · W/S."
)

APPEARANCE_HELP = (
    "APPLY switches the scheme immediately — the window is rebuilt in place,\n"
    "keeping your connection, settings, field values and the event log.\n"
    "It is refused while the machine is moving: the rebuild briefly destroys\n"
    "the stop controls, and waiting a moment is cheaper than that.\n"
    "Every scheme keeps the same meanings — RED is stop, AMBER is a warning —\n"
    "and the four motor colours are tones of the accent, separated by\n"
    "lightness so they stay readable with red-green colour blindness."
)

PID_HELP = (
    "Source: Stepper MATLAB report, Table 2. Plant identified as\n"
    "G(s) = 12.5 / (s(s+12.5)) rad per pulse, pole placement at ζ = 0.7071.\n"
    "THE BOARD RUNS OPEN LOOP — the encoder only monitors — so these gains are\n"
    "stored and echoed, not used to close a loop. They exist so the controller\n"
    "configuration matches the documented Simulink model.\n"
    "Locking a term freezes just that one; the others keep being sent."
)


class SettingsDialogMixin:
    # ══════════════════════════════════════════════════════════════
    # Persistence — the GUI is the system of record for machine setup
    # ══════════════════════════════════════════════════════════════
    def _load_settings_file(self):
        """Merges the saved setup over the defaults.

        Called from _init_state(), i.e. before the log widget exists, so it
        must never call self.log(). Anything worth saying is queued in
        _pending_settings_notes and flushed once the UI is up.
        """
        self._pending_settings_notes = []
        try:
            with open(SETTINGS_FILE, "r", encoding="utf-8") as fh:
                saved = json.load(fh)
        except FileNotFoundError:
            return
        except (OSError, ValueError) as e:
            # A corrupt file must not stop the app starting — fall back to
            # the safe defaults and say so, loudly, later.
            self._pending_settings_notes.append(
                (f"Could not read {os.path.basename(SETTINGS_FILE)} ({e}). "
                 f"Using default settings.", "error"))
            return

        if not isinstance(saved, dict):
            self._pending_settings_notes.append(
                ("Settings file has the wrong format — using defaults.", "error"))
            return

        # A file written before the elbow angle moved to the from-home
        # frame holds taught limits that no longer mean what they say.
        # Drop exactly those and keep the rest — speeds and the ZM/RM
        # boundaries are in units that did not change.
        saved_schema = saved.get(SETTINGS_SCHEMA_KEY, 1)
        if saved_schema < 3:
            dropped = [k for k in ARM_FRAME_V2_RESET_KEYS if k in saved]
            for key in dropped:
                saved.pop(key)
            if dropped:
                self._pending_settings_notes.append(
                    ("The elbow angle now reads 0° at home and counts up as "
                     "the arm turns; it used to start at 60°. Your saved "
                     "elbow boundaries were in the old frame, so they have "
                     "been reset — jog to each stop and press SET HERE to "
                     "teach them again. Speeds and the ZM/RM boundaries are "
                     "unaffected.", "warn"))
        if saved_schema < 4:
            # RM's zero moved from mid-travel to the CCW stop. A stored
            # ±150 read in the new frame refuses every bearing past 150°,
            # which includes any negative X (bearing 180°) — the exact
            # "there is a limit" complaint on a reachable target.
            dropped_rot = [k for k in ROT_FRAME_V4_RESET_KEYS if k in saved]
            for key in dropped_rot:
                saved.pop(key)
            if dropped_rot:
                self._pending_settings_notes.append(
                    ("RM now reads 0° at its counter-clockwise stop (which is "
                     "HOME) and counts up to 340°; it used to be centred, "
                     "-170..+170. Your saved RM boundaries were in the old "
                     "frame — a stored ±150 would have refused every target "
                     "past 150°, including any negative X — so they have been "
                     "reset to the full travel. Jog to each stop and press "
                     "SET HERE to teach them again.", "warn"))

        # Only keys the app knows about, and only if they still parse. A
        # stale file from an older version must not be able to inject an
        # arbitrary value into the settings dict.
        applied = 0
        for key, value in saved.items():
            if key not in self.settings:
                continue
            # Booleans, not numbers: float(True) would store 1.0 and then
            # every later `is True` check would quietly be false.
            if key in PID_LOCK_KEYS or key in LIMIT_ENFORCE_KEYS \
                    or key == LIMITS_ENABLED_KEY:
                self.settings[key] = bool(value)
                applied += 1
                continue
            try:
                self.settings[key] = float(value)
                applied += 1
            except (TypeError, ValueError):
                self._pending_settings_notes.append(
                    (f"Ignoring invalid value in settings file: "
                     f"{key}={value!r}", "warn"))
        if applied:
            self._pending_settings_notes.append(
                (f"Loaded {applied} settings from "
                 f"{os.path.basename(SETTINGS_FILE)}.", "default"))

    def _flush_settings_notes(self):
        for message, tag in getattr(self, "_pending_settings_notes", []):
            self.log(message, tag=tag)
        self._pending_settings_notes = []

    def _save_settings_file(self):
        try:
            payload = dict(self.settings)
            payload[SETTINGS_SCHEMA_KEY] = SETTINGS_SCHEMA
            with open(SETTINGS_FILE, "w", encoding="utf-8") as fh:
                json.dump(payload, fh, indent=2, ensure_ascii=False)
        except OSError as e:
            self.log(f"Could not save settings: {e}", tag="error")
            return False
        return True

    # ══════════════════════════════════════════════════════════════
    # Pushing setup to the board
    # ══════════════════════════════════════════════════════════════
    def _send_speed(self):
        s = self.settings
        self.send(f"SET_SPEED:{MASTER_RPM:g},{MASTER_ACC_RPM_S:g},"
                  f"{s['rot_pct']:g},{s['arm_pct']:g},{s['z_pct']:g},"
                  f"{s['rot_acc_pct']:g},{s['arm_acc_pct']:g},{s['z_acc_pct']:g}")

    def _send_pid(self):
        s = self.settings
        self.send(f"SET_PID:{s['kp']:g},{s['ki']:g},{s['kd']:g},{s['n']:g}")

    def _send_limits(self):
        s = self.settings
        # VALUES FIRST, then the switches. The board holds limits in RAM
        # only, so this runs on every handshake; sending the switches first
        # would leave a window where an axis is armed against whatever the
        # board happened to be holding.
        #
        # No unlock/write/re-lock dance any more — that was needed because a
        # locked axis refused SET_LIMIT. Enforcement does not gate writes, so
        # a switched-off axis still takes its values.
        for key in LIMIT_KEYS:
            _label, axis, end, _unit, _lo, _hi, _default, _dec = LIMIT_FIELDS[key]
            self.send(f"SET_LIMIT:{axis},{end},{s[key]:g}", log_tx=False)
        for axis, enforce_key in LIMIT_ENFORCE_BY_AXIS.items():
            on = 1 if s.get(enforce_key, True) else 0
            self.send(f"SET_LIMIT_ENFORCE:{axis},{on}", log_tx=False)
        self.send(f"SET_LIMITS_ENABLED:{1 if s.get(LIMITS_ENABLED_KEY, True) else 0}",
                  log_tx=False)

    def _push_settings_to_board(self, reason=""):
        """Sends speed, PID and every travel limit.

        The board keeps all of this in RAM, so it must be re-sent after any
        board reset — otherwise the limits on screen and the limits actually
        enforced drift apart, which is the worst possible failure mode for
        a limit.
        """
        self._send_speed()
        self._send_pid()
        self._send_limits()
        if reason:
            self.log(f"Synced speed, PID and travel limits to the board ({reason}).")

    # ══════════════════════════════════════════════════════════════
    # Named limit presets — save / load / delete
    # ══════════════════════════════════════════════════════════════
    def _read_limit_presets(self):
        try:
            with open(LIMIT_PRESETS_FILE, "r", encoding="utf-8") as fh:
                data = json.load(fh)
        except FileNotFoundError:
            return {}
        except (OSError, ValueError) as e:
            self.log(f"Could not read saved limit sets ({e}).", tag="error")
            return {}
        if not isinstance(data, dict):
            return {}
        # Only presets carrying every key AND parsing as numbers. A partial
        # preset would load some axes and silently leave others on whatever
        # was in the boxes — the worst kind of half-applied limit.
        clean = {}
        for name, values in data.items():
            if not isinstance(values, dict):
                continue
            try:
                entry = {k: float(values[k]) for k in LIMIT_KEYS}
            except (KeyError, TypeError, ValueError):
                self.log(f"Skipping corrupt or incomplete limit set: {name!r}",
                         tag="warn")
                continue
            clean[str(name)] = entry
        return clean

    def _write_limit_presets(self, presets):
        try:
            with open(LIMIT_PRESETS_FILE, "w", encoding="utf-8") as fh:
                json.dump(presets, fh, indent=2, ensure_ascii=False)
        except OSError as e:
            self.log(f"Could not save limit set: {e}", tag="error")
            return False
        return True

    def _refresh_preset_list(self, select=None):
        self._limit_presets = self._read_limit_presets()
        names = sorted(self._limit_presets)
        combo = getattr(self, "_preset_combo", None)
        if combo is not None:
            combo.configure(values=names)
        if select is not None:
            self._preset_name_v.set(select)
        elif self._preset_name_v.get() not in names:
            self._preset_name_v.set(names[0] if names else "")
        self._update_preset_count()

    def _update_preset_count(self):
        var = getattr(self, "_preset_count_v", None)
        if var is None:
            return
        n = len(getattr(self, "_limit_presets", {}))
        var.set("No limit sets saved yet." if not n
                else f"{n} limit set{'s' if n != 1 else ''} saved.")

    def _save_limit_preset(self):
        """Saves whatever is CURRENTLY IN THE BOXES, after validating it.

        Validating first matters: a preset is something the operator recalls
        later and trusts without re-reading, so saving an invalid or
        inverted set would plant a fault to be found at the worst moment.
        """
        name = self._preset_name_v.get().strip()
        if not name:
            messagebox.showerror("Error", "Give the limit set a name before saving.")
            return
        if len(name) > LIMIT_PRESET_NAME_MAX:
            messagebox.showerror(
                "Error", f"Name too long (max {LIMIT_PRESET_NAME_MAX} characters).")
            return

        limits, error = self._collect_limits()
        if error:
            messagebox.showerror("Cannot save — limits are not valid", error)
            return

        presets = self._read_limit_presets()
        if name in presets and not messagebox.askyesno(
                "Overwrite?", f"A limit set named “{name}” already exists.\n\n"
                              f"Overwrite it with the current values?"):
            return

        presets[name] = limits
        if not self._write_limit_presets(presets):
            return
        self._refresh_preset_list(select=name)
        self.log(f"Saved limit set “{name}”.")

    def _load_limit_preset(self):
        name = self._preset_name_v.get().strip()
        presets = self._read_limit_presets()
        if name not in presets:
            messagebox.showerror("Error", f"No limit set named “{name}”.")
            return
        for key, value in presets[name].items():
            self._limit_vars[key].set(f"{value:g}")
        self.log(f"Loaded limit set “{name}” into the form — press APPLY to "
                 f"write it to the board.")

    def _delete_limit_preset(self):
        name = self._preset_name_v.get().strip()
        presets = self._read_limit_presets()
        if name not in presets:
            messagebox.showerror("Error", f"No limit set named “{name}”.")
            return
        if not messagebox.askyesno(
                "Delete limit set",
                f"Permanently delete the limit set “{name}”?\n\n"
                f"The limits currently running on the machine are NOT affected "
                f"— only this saved copy is removed."):
            return
        del presets[name]
        if not self._write_limit_presets(presets):
            return
        self._refresh_preset_list()
        self.log(f"Deleted limit set “{name}”.")

    # ══════════════════════════════════════════════════════════════
    # Window
    # ══════════════════════════════════════════════════════════════
    def open_settings_dialog(self):
        existing = getattr(self, "_settings_dlg", None)
        if existing is not None and existing.winfo_exists():
            existing.lift()
            existing.focus_set()
            return

        dlg = tk.Toplevel(self.root)
        self._settings_dlg = dlg
        dlg.title("Settings")
        dlg.configure(bg=PANEL_BG)
        sw, sh = (int(v) for v in SETTINGS_GEOMETRY.lower().split("x"))
        dlg.geometry(f"{px(sw)}x{px(sh)}")
        dlg.minsize(px(SETTINGS_MIN_SIZE[0]), px(SETTINGS_MIN_SIZE[1]))
        dlg.resizable(True, True)
        dlg.transient(self.root)
        dlg.grab_set()
        dlg.protocol("WM_DELETE_WINDOW", lambda: self._close_settings(dlg))
        # NO <Escape> binding on this window, deliberately.
        #
        # KeyboardMixin already binds Escape with bind_all, which is
        # application-wide and fires for this Toplevel too. Binding it here
        # as well meant ONE keypress ran BOTH handlers: this one closed the
        # window, then the global toggle saw no window open and reopened it
        # — so Escape appeared to do nothing at all. The global toggle is
        # the single owner of the key.

        self._build_settings_vars()

        header = tk.Frame(dlg, bg=PANEL_BG)
        header.pack(fill="x", padx=18, pady=(14, 0))
        tk.Label(header, text="SETTINGS", bg=PANEL_BG, fg=TEXT_LIGHT,
                 font=FONT_TITLE).pack(side="left")
        tk.Label(header, text="   ESC to close", bg=PANEL_BG, fg=TEXT_DIM,
                 font=FONT_SMALL).pack(side="left", pady=(6, 0))

        # A ttk notebook draws square tabs, and its tab shape cannot be
        # rounded without supplying a custom image element per state. The
        # strip is built from RoundedButtons instead: fully rounded, and
        # styled by the same code as every other control in the app.
        strip = tk.Frame(dlg, bg=PANEL_BG)
        strip.pack(fill="x", padx=px(16), pady=(px(12), 0))

        holder = tk.Frame(dlg, bg=PANEL_BG)
        holder.pack(fill="both", expand=True, padx=px(14), pady=(px(10), px(14)))

        self._tab_buttons_bar = {}
        self._tab_pages = {}
        for key, label, builder in (("speed", "Speed", self._build_speed_tab),
                                    ("limits", "Boundaries", self._build_limits_tab),
                                    ("controls", "Controls", self._build_controls_tab),
                                    ("pid", "PID", self._build_pid_tab),
                                    ("appearance", "Appearance", self._build_appearance_tab)):
            btn = RoundedButton(strip, text=label, bg_color=SURFACE,
                                fg_color=TEXT_MUTED, width=128, height=34,
                                radius=RADIUS_MD, font=FONT_BUTTON,
                                command=lambda k=key: self._show_tab(k))
            btn.pack(side="left", padx=(0, px(6)))
            self._tab_buttons_bar[key] = btn

            page = tk.Frame(holder, bg=PANEL_BG)
            self._tab_pages[key] = page
            builder(page, dlg)
            # The tab's widgets exist now, so hook the wheel to them too.
            for child in page.winfo_children():
                handler = getattr(child, "_wheel_handler", None)
                if handler:
                    self._bind_wheel_deep(child, handler)

        self._active_tab = None
        self._show_tab("speed")
        self._refresh_speed_preview()

    def _show_tab(self, key):
        """Raises one page and restyles the strip. Only one page is packed
        at a time, so an unseen tab costs nothing to lay out."""
        if getattr(self, "_active_tab", None) == key:
            return
        for k, page in self._tab_pages.items():
            page.pack_forget()
        self._tab_pages[key].pack(fill="both", expand=True)
        for k, btn in self._tab_buttons_bar.items():
            active = k == key
            btn.set_config(btn.text_str,
                           ACCENT_MINT if active else SURFACE,
                           fg_color=INK_DARK if active else TEXT_MUTED)
        self._active_tab = key

    def _build_settings_vars(self):
        s = self.settings
        self._pid_vars = {k: tk.StringVar(value=f"{s[k]:g}")
                          for k, _lab, _lock in PID_FIELDS}
        self._pid_locks = {lock: bool(s.get(lock, False))
                           for _k, _lab, lock in PID_FIELDS}
        self._speed_vars = {k: tk.StringVar(value=f"{s[k]:g}") for k in SPEED_FIELDS}
        self._accel_vars = {k: tk.StringVar(value=f"{s[k]:g}") for k in ACCEL_FIELDS}
        self._limit_vars = {k: tk.StringVar(value=f"{s[k]:g}") for k in LIMIT_FIELDS}
        self._speed_preview_vars = {}
        self._speed_entry_wraps = {}
        self._accel_preview_vars = {}
        self._accel_entry_wraps = {}
        self._pid_entries = {}
        self._pid_lock_buttons = {}

        # Live readout of what each percentage actually means. Traced rather
        # than recomputed on APPLY — the whole point of one fixed reference
        # speed is being able to see the effect while you type.
        for var in self._speed_vars.values():
            var.trace_add("write", lambda *_a: self._refresh_speed_preview())
        for var in self._accel_vars.values():
            var.trace_add("write", lambda *_a: self._refresh_speed_preview())

    # ── shared chrome ────────────────────────────────────────────────
    def _tab_body(self, page):
        """Scrollable content area for one tab.

        The tabs are not all the same height — Boundaries with a saved-set
        row is far taller than PID — and on a short window the bottom of
        the tallest one used to be simply unreachable. Scrolling is per
        tab rather than for the whole window so the action bar stays
        pinned: APPLY must never be the thing that scrolls away.

        The scrollbar is created but only PACKED when the content actually
        overflows, so a short tab shows no furniture it does not need.
        """
        holder = tk.Frame(page, bg=PANEL_BG)
        holder.pack(fill="both", expand=True, padx=px(18), pady=(px(16), px(4)))

        canvas = tk.Canvas(holder, bg=PANEL_BG, highlightthickness=0, bd=0)
        bar = ttk.Scrollbar(holder, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=bar.set)
        canvas.pack(side="left", fill="both", expand=True)

        body = tk.Frame(canvas, bg=PANEL_BG)
        window = canvas.create_window((0, 0), window=body, anchor="nw")

        def _sync(_event=None):
            canvas.configure(scrollregion=canvas.bbox("all"))
            needed = body.winfo_reqheight() > canvas.winfo_height() + 2
            if needed and not bar.winfo_ismapped():
                bar.pack(side="right", fill="y")
            elif not needed and bar.winfo_ismapped():
                bar.pack_forget()
                canvas.yview_moveto(0)

        def _fit(event):
            canvas.itemconfig(window, width=event.width)
            _sync()

        body.bind("<Configure>", _sync)
        canvas.bind("<Configure>", _fit)

        # Wheel bound to this subtree only. bind_all would hijack the wheel
        # for the whole application, including the main window behind this
        # one and the combobox in the Boundaries tab.
        def _wheel(event):
            if not bar.winfo_ismapped():
                return None                     # nothing to scroll
            if getattr(event, "num", 0) == 5:
                delta = -1
            elif getattr(event, "num", 0) == 4:
                delta = 1
            elif sys.platform == "darwin":
                delta = int(-event.delta)
            else:
                delta = int(-event.delta / 120)
            canvas.yview_scroll(delta, "units")
            return "break"

        for widget in (canvas, body):
            widget.bind("<MouseWheel>", _wheel)
            widget.bind("<Button-4>", _wheel)
            widget.bind("<Button-5>", _wheel)
        body._wheel_handler = _wheel
        self._bind_wheel_deep(body, _wheel)
        return body

    def _bind_wheel_deep(self, root_widget, handler):
        """Forwards the wheel from children created later.

        Frames stacked on the canvas swallow wheel events, so the scroll
        would die wherever the pointer happened to be. Re-run after a tab
        is populated.
        """
        def apply(widget):
            try:
                widget.bind("<MouseWheel>", handler)
                widget.bind("<Button-4>", handler)
                widget.bind("<Button-5>", handler)
            except Exception:
                pass
            for child in widget.winfo_children():
                apply(child)
        root_widget.after_idle(lambda: apply(root_widget))

    def _tab_buttons(self, page, on_apply, on_default, what):
        """Per-tab action bar.

        APPLY and DEFAULT are scoped to THIS tab only. A global reset that
        also wiped the travel limits because someone wanted to undo a speed
        change is exactly the failure this avoids.
        """
        bar = tk.Frame(page, bg=PANEL_BG)
        bar.pack(side="bottom", fill="x", padx=18, pady=(0, 16))
        tk.Label(bar, text=f"Applies to {what} only", bg=PANEL_BG, fg=TEXT_DIM,
                 font=FONT_HINT).pack(side="left")
        RoundedButton(bar, text="APPLY", icon="✔", bg_color=ACCENT_GREEN,
                      fg_color=INK_DARK, width=118, height=36,
                      command=on_apply).pack(side="right", padx=(6, 0))
        RoundedButton(bar, text="DEFAULTS", icon="↺", bg_color=SURFACE,
                      fg_color=TEXT_LIGHT, width=124, height=36,
                      command=on_default).pack(side="right")

    def _help_block(self, parent, text):
        well = make_well(parent)
        well.pack(fill="x", pady=(px(14), 0))
        tk.Label(well.body, text=text, bg=LED_BG, fg=TEXT_DIM,
                 font=FONT_SMALL, justify="left",
                 anchor="w").pack(fill="x", padx=px(10), pady=px(8))
        return well

    def _field_row(self, parent, row, label, var, unit, width=9,
                   font=FONT_ENTRY):
        parent.grid_columnconfigure(0, weight=1)
        tk.Label(parent, text=label, bg=PANEL_BG, fg=TEXT_LIGHT,
                 font=FONT_LABEL, anchor="w").grid(
            row=row, column=0, sticky="w", pady=5, padx=(0, 14))
        wrap, entry = make_inset_entry(parent, var, width=width, font=font)
        wrap.grid(row=row, column=1, sticky="e")
        tk.Label(parent, text=unit, bg=PANEL_BG, fg=TEXT_MUTED, width=4,
                 anchor="w", font=FONT_LABEL).grid(
            row=row, column=2, sticky="w", padx=(8, 0))
        return wrap, entry

    # ══════════════════════════════════════════════════════════════
    # TAB 1 — SPEED
    # ══════════════════════════════════════════════════════════════
    def _build_speed_tab(self, page, dlg):
        self._tab_buttons(page, self._apply_speed, self._default_speed, "speed")
        body = self._tab_body(page)

        ref_w = make_well(body)
        ref_w.pack(fill="x")
        ref = ref_w.body
        tk.Label(ref, text="REFERENCE SPEED (fixed)", bg=LED_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(anchor="w", padx=12, pady=(9, 0))
        tk.Label(ref, text=f"{MASTER_RPM:g} RPM   ·   {MASTER_ACC_RPM_S:g} RPM/s ramp",
                 bg=LED_BG, fg=ACCENT_MINT,
                 font=FONT_READOUT).pack(anchor="w", padx=12, pady=(0, 9))

        grid = tk.Frame(body, bg=PANEL_BG)
        grid.pack(fill="x", pady=(16, 0))
        grid.grid_columnconfigure(0, weight=1)

        for row, key in enumerate(SPEED_KEYS):
            label, unit, _default, _lo, _hi = SPEED_FIELDS[key]
            wrap, _entry = self._field_row(grid, row, label,
                                           self._speed_vars[key], unit)
            self._speed_entry_wraps[key] = wrap
            # Real motor RPM sits right beside the percentage, because the
            # percentage on its own is not a quantity anyone can sanity-check.
            var = tk.StringVar(value="—")
            self._speed_preview_vars[key] = var
            tk.Label(grid, textvariable=var, bg=PANEL_BG, fg=TEXT_MUTED,
                     font=FONT_MONO, anchor="w").grid(
                row=row, column=3, sticky="w", padx=(14, 0))

        # ACCELERATION — independent per-axis %, own grid so it reads as a
        # separate family rather than a continuation of the speed rows.
        tk.Label(body, text="ACCELERATION (%)", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(anchor="w", pady=(16, 0))
        accel_grid = tk.Frame(body, bg=PANEL_BG)
        accel_grid.pack(fill="x", pady=(6, 0))
        accel_grid.grid_columnconfigure(0, weight=1)

        for row, key in enumerate(ACCEL_KEYS):
            label, unit, _default, _lo, _hi = ACCEL_FIELDS[key]
            wrap, _entry = self._field_row(accel_grid, row, label,
                                           self._accel_vars[key], unit)
            self._accel_entry_wraps[key] = wrap
            var = tk.StringVar(value="—")
            self._accel_preview_vars[key] = var
            tk.Label(accel_grid, textvariable=var, bg=PANEL_BG, fg=TEXT_MUTED,
                     font=FONT_MONO, anchor="w").grid(
                row=row, column=3, sticky="w", padx=(14, 0))

        self._speed_warn_v = tk.StringVar(value="")
        tk.Label(body, textvariable=self._speed_warn_v, bg=PANEL_BG,
                 fg=WARN_BORDER, font=FONT_CAPTION, justify="left",
                 anchor="w", wraplength=600).pack(fill="x", pady=(12, 0))

        self._help_block(body, SPEED_HELP)

    def _refresh_speed_preview(self):
        """Recomputes the readouts from whatever is currently typed, and
        recolours any field that is above its recommended value.

        Mirrors the firmware's arithmetic including the clamp, so the
        operator sees the consequence before pressing APPLY rather than
        reading about it in the log afterwards.
        """
        if not getattr(self, "_speed_preview_vars", None):
            return
        over = []
        # (vars dict, entry-wraps dict, preview-vars dict, PREVIEW_BY_KEY,
        #  WARN_PCT, keys, reference RPM) — one loop body drives both the
        # speed rows and the accel rows, which are identical in shape and
        # differ only in which reference RPM they scale from.
        families = (
            (self._speed_vars, self._speed_entry_wraps, self._speed_preview_vars,
             SPEED_PREVIEW_BY_KEY, SPEED_WARN_PCT, SPEED_KEYS, MASTER_RPM),
            (self._accel_vars, self._accel_entry_wraps, self._accel_preview_vars,
             ACCEL_PREVIEW_BY_KEY, ACCEL_WARN_PCT, ACCEL_KEYS, MASTER_ACC_RPM_S),
        )
        for vars_, wraps, preview_vars, preview_by_key, warn_pct, keys, ref_rpm in families:
            for key in keys:
                name, _pk, scale, fn, unit, ceiling, rpm_ceiling = preview_by_key[key]
                wrap = wraps.get(key)
                try:
                    pct = float(vars_[key].get().strip().replace(",", "."))
                except ValueError:
                    preview_vars[key].set("— invalid —")
                    if wrap is not None:
                        set_entry_border(wrap, ACCENT_RED)
                    continue

                motor = ref_rpm * (pct / 100.0) * scale
                speed = fn(ref_rpm, pct)

                if ceiling is None:
                    # The arm's rate depends on a gear ratio nobody has
                    # measured, so quoting it as fact would be lying with a
                    # decimal point.
                    text = f"= {motor:.1f} RPM"
                    if motor > rpm_ceiling:
                        text += f"  ▸ capped {rpm_ceiling:g}"
                else:
                    text = f"= {motor:.1f} RPM  ·  {speed:.2f} {unit}"
                    if speed > ceiling:
                        text += f"  ▸ capped {ceiling:g}"
                preview_vars[key].set(text)

                warn_at = warn_pct.get(key)
                if wrap is not None:
                    if warn_at is not None and pct > warn_at:
                        set_entry_border(wrap, WARN_BORDER)
                        over.append(f"{name} {pct:g}% (recommended {warn_at:g}%)")
                    else:
                        set_entry_border(wrap, NEUTRAL_BORDER)

        if getattr(self, "_speed_warn_v", None) is not None:
            self._speed_warn_v.set(
                "⚠  Above the recommended value: " + ", ".join(over)
                + ".  Higher speeds are allowed but can cause instability — "
                  "lost steps, overshoot and vibration on an open-loop drive."
                if over else "")

    def _apply_speed(self):
        speed, error = self._collect_speed()
        if error:
            messagebox.showerror("Error", error)
            return
        over = [f"  · {SPEED_PREVIEW_BY_KEY[k][0]}: {speed[k]:g}% "
                f"(recommended {SPEED_WARN_PCT[k]:g}%)"
                for k in SPEED_KEYS
                if k in SPEED_WARN_PCT and speed[k] > SPEED_WARN_PCT[k]]
        over += [f"  · {ACCEL_PREVIEW_BY_KEY[k][0]} accel: {speed[k]:g}% "
                 f"(recommended {ACCEL_WARN_PCT[k]:g}%)"
                 for k in ACCEL_KEYS
                 if k in ACCEL_WARN_PCT and speed[k] > ACCEL_WARN_PCT[k]]
        if over and not messagebox.askyesno(
                "Above recommended speed",
                "These axes are set above their recommended value:\n\n"
                + "\n".join(over)
                + "\n\nThis is allowed, but it is past what has been tested "
                  "stable on this machine and can cause instability — lost "
                  "steps, overshoot and vibration on an open-loop drive.\n\n"
                  "Apply anyway?"):
            return

        self.settings.update(**speed)
        self._send_speed()
        self._save_settings_file()
        self.log(f"Speed applied — RM {speed['rot_pct']:g}% · "
                 f"A1M/A2M {speed['arm_pct']:g}% · ZM {speed['z_pct']:g}%  "
                 f"(reference {MASTER_RPM:g} RPM)")
        self.log(f"Accel applied — RM {speed['rot_acc_pct']:g}% · "
                 f"A1M/A2M {speed['arm_acc_pct']:g}% · ZM {speed['z_acc_pct']:g}%  "
                 f"(reference {MASTER_ACC_RPM_S:g} RPM/s)")
        if over:
            self.log("One or more axes are above their recommended speed/accel.",
                     tag="warn")

    def _default_speed(self):
        for key in SPEED_KEYS:
            self._speed_vars[key].set(f"{SPEED_FIELDS[key][2]:g}")
        for key in ACCEL_KEYS:
            self._accel_vars[key].set(f"{ACCEL_FIELDS[key][2]:g}")
        self._refresh_speed_preview()
        self.log("Speed/accel fields reset to defaults — press APPLY to send them.")

    def _collect_speed(self):
        out = {}
        for key in SPEED_KEYS:
            label, unit, _default, lo, hi = SPEED_FIELDS[key]
            value, error = self._read_number(self._speed_vars[key], label)
            if error:
                return None, error
            if value < lo:
                return None, (f"“{label}” must be at least {lo:g}{unit}.\n\n"
                              f"You entered {value:g}{unit}. 0% would freeze the "
                              f"axis and a negative value would reverse it — "
                              f"neither is a speed setting.")
            if hi is not None and value > hi:
                return None, (f"“{label}” must be between {lo:g} and {hi:g}{unit}.\n\n"
                              f"You entered {value:g}{unit}.")
            out[key] = value
        for key in ACCEL_KEYS:
            label, unit, _default, lo, hi = ACCEL_FIELDS[key]
            value, error = self._read_number(self._accel_vars[key], label)
            if error:
                return None, error
            if value < lo:
                return None, (f"“{label}” must be at least {lo:g}{unit}.\n\n"
                              f"You entered {value:g}{unit}. 0% would freeze the "
                              f"axis's ramp and a negative value would reverse it — "
                              f"neither is an acceleration setting.")
            if hi is not None and value > hi:
                return None, (f"“{label}” must be between {lo:g} and {hi:g}{unit}.\n\n"
                              f"You entered {value:g}{unit}.")
            out[key] = value
        return out, None

    # ══════════════════════════════════════════════════════════════
    # TAB 2 — TRAVEL LIMITS
    # ══════════════════════════════════════════════════════════════
    def _build_limits_tab(self, page, dlg):
        self._tab_buttons(page, self._apply_limits, self._default_limits,
                          "boundaries")
        body = self._tab_body(page)

        grid = tk.Frame(body, bg=PANEL_BG)
        grid.pack(fill="x")
        grid.grid_columnconfigure(0, weight=1)

        self._limit_entries = {}
        self._limit_enforce_buttons = {}

        row = 0
        for heading, min_key, max_key, _span in LIMIT_GROUPS:
            axis = LIMIT_FIELDS[min_key][1]
            enforce_key = LIMIT_ENFORCE_BY_AXIS[axis]
            head = tk.Frame(grid, bg=PANEL_BG)
            head.grid(row=row, column=0, columnspan=4, sticky="ew", pady=(10, 2))
            tk.Label(head, text=heading, bg=PANEL_BG, fg=ACCENT_MINT,
                     font=FONT_BUTTON).pack(side="left")
            # One switch per PAIR, not per box. A boundary with only one end
            # enforced is not a boundary anyone asked for, and the two ends
            # are taught together anyway.
            btn = RoundedButton(head, text="", bg_color=SURFACE,
                                fg_color=TEXT_LIGHT, width=150, height=26,
                                radius=10, font=FONT_CAPTION,
                                command=lambda ek=enforce_key:
                                    self._toggle_limit_enforce(ek))
            btn.pack(side="right")
            self._limit_enforce_buttons[enforce_key] = btn
            row += 1
            for key in (min_key, max_key):
                label, _axis, _end, unit, *_ = LIMIT_FIELDS[key]
                _wrap, entry = self._field_row(grid, row, "    " + label,
                                               self._limit_vars[key], unit)
                if key in LIMIT_CAPTURE_ONLY:
                    # Read-only, not disabled: the value must stay legible.
                    # Typing here would mean typing against the elbow scale,
                    # which is exactly the thing that is not trustworthy.
                    entry.config(state="readonly")
                self._limit_entries[key] = entry
                # Nothing else makes this entry read-only. A switched-off
                # boundary stays fully editable: teaching a boundary while
                # it is not yet policing anything is the normal order of
                # work, not an edge case.
                RoundedButton(grid, text="SET HERE", bg_color=SURFACE,
                              fg_color=TEXT_LIGHT, width=106, height=28, radius=10,
                              font=FONT_CAPTION,
                              command=lambda k=key: self._capture_limit_here(k)).grid(
                    row=row, column=3, sticky="w", padx=(12, 0))
                row += 1
        self._refresh_limit_enforce()

        self._build_preset_row(body)

        # ---- master enforcement switch ----
        # Deliberately NOT staged behind APPLY. "Turn the limits off" is
        # something an operator needs while a move is being set up, and a
        # switch that only takes effect after a second button is a switch
        # people believe they have already thrown.
        enable_row = tk.Frame(body, bg=PANEL_BG)
        enable_row.pack(fill="x", pady=(16, 0))
        tk.Label(enable_row, text="ENFORCEMENT", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(side="left")
        self._limits_enabled_btn = RoundedButton(
            enable_row, text="", bg_color=SURFACE, fg_color=TEXT_LIGHT,
            width=210, height=30, radius=10, font=FONT_CAPTION,
            command=self._toggle_limits_enabled)
        self._limits_enabled_btn.pack(side="left", padx=(12, 0))
        self._refresh_limits_enabled()

        # RESET COORDINATES moved to section 3, MOTION CONTROL. It is a
        # machine action used while jogging to the reference pose, so making
        # it reachable only through this dialog meant leaving the controls
        # you were already using.

        self._help_block(body, LIMIT_HELP)

    def _toggle_limit_enforce(self, enforce_key):
        """Switches ONE axis's boundary on or off.

        Not staged behind APPLY, for the same reason the master switch is
        not: this is something an operator reaches for while setting up a
        move, and a safety switch that needs a second button to take effect
        is a switch people believe they have already thrown.

        Turning it off is confirmed and logged as a warning. Turning it
        back on is neither — making the machine safer is not an event that
        needs to interrupt anybody.
        """
        axis = next(a for a, k in LIMIT_ENFORCE_BY_AXIS.items() if k == enforce_key)
        want = not self.settings.get(enforce_key, True)
        if not want and not messagebox.askyesno(
                f"Disable the {axis} boundary",
                f"Turn OFF soft-limit enforcement on {axis}?\n\n"
                f"Its taught boundary is kept and still shown, but nothing "
                f"will stop {axis} at it. The other three axes are unaffected, "
                f"and the PLC's physical switches are the only protection "
                f"{axis} has left.\n\nContinue?"):
            return
        self.settings[enforce_key] = want
        self._refresh_limit_enforce()
        # A switched-off axis stops advertising a band on the P2P panel.
        self._refresh_workspace_hint()
        # The board holds the same flag, so a bare terminal and the GUI
        # cannot end up disagreeing about whether an axis is protected.
        self.send(f"SET_LIMIT_ENFORCE:{axis},{1 if want else 0}")
        self._save_settings_file()
        if want:
            self.log(f"{axis} boundary ENFORCED — the axis stops at it again.")
        else:
            self.log(f"⚠ {axis} BOUNDARY NOT ENFORCED. Its values are kept but "
                     f"nothing will stop {axis} at them.", tag="warn")

    def _refresh_limit_enforce(self):
        # The master switch is shown here too: with it off, an axis that is
        # individually ON is still not protected, and a button reading
        # ENFORCED in that state would be false. Hence the third caption.
        master = self.settings.get(LIMITS_ENABLED_KEY, True)
        for _axis, enforce_key in LIMIT_ENFORCE_BY_AXIS.items():
            on = self.settings.get(enforce_key, True)
            btn = self._limit_enforce_buttons.get(enforce_key)
            if btn is None:
                continue
            if not on:
                btn.set_config("NOT ENFORCED", ACCENT_RED, icon="⚠",
                               fg_color=INK_DARK)
            elif not master:
                btn.set_config("ON (MASTER OFF)", ACCENT_ORANGE, icon="⚠",
                               fg_color=INK_DARK)
            else:
                btn.set_config("ENFORCED", SURFACE, icon="🛡",
                               fg_color=TEXT_LIGHT)

    def _toggle_limits_enabled(self):
        """Master switch for soft-limit ENFORCEMENT, all axes at once.

        It ANDs with the per-axis switches rather than overriding them:
        turning this back on must not silently re-arm an axis the operator
        switched off by itself.
        """
        want = not self.settings.get(LIMITS_ENABLED_KEY, True)
        if not want and not messagebox.askyesno(
                "Disable soft limits",
                "Turn OFF soft-limit enforcement on every axis?\n\n"
                "Your taught boundaries are kept and still shown, but nothing "
                "will stop an axis at them. The PLC's physical switches "
                "(M5..M8) become the only protection the machine has.\n\n"
                "Continue?"):
            return
        self.settings[LIMITS_ENABLED_KEY] = want
        self.send(f"SET_LIMITS_ENABLED:{1 if want else 0}")
        self._save_settings_file()
        self._refresh_limits_enabled()
        self._refresh_workspace_hint()
        # The per-axis captions depend on this switch — an axis that is
        # individually ON reads "ON (MASTER OFF)" while it is off — so they
        # have to be repainted with it, or four buttons keep claiming
        # ENFORCED on a machine that is enforcing nothing.
        self._refresh_limit_enforce()
        if want:
            off = [a for a, k in LIMIT_ENFORCE_BY_AXIS.items()
                   if not self.settings.get(k, True)]
            self.log("Soft limits ENABLED — boundaries are enforced again."
                     + (f" {', '.join(off)} stay switched off individually."
                        if off else ""))
        else:
            self.log("⚠ SOFT LIMITS DISABLED. Taught boundaries are kept but not "
                     "enforced; only the PLC's physical switches will stop an axis.",
                     tag="warn")

    def _refresh_limits_enabled(self):
        on = self.settings.get(LIMITS_ENABLED_KEY, True)
        btn = getattr(self, "_limits_enabled_btn", None)
        if btn is None:
            return
        btn.set_config("BOUNDARIES ENFORCED" if on else "BOUNDARIES DISABLED",
                       SURFACE if on else ACCENT_RED,
                       icon="🛡" if on else "⚠",
                       fg_color=TEXT_LIGHT if on else INK_DARK)

    def _build_preset_row(self, parent):
        """Save / load / delete named limit sets.

        The name box is an editable combobox on purpose: typing a new name
        saves a new set, picking an existing one targets that set. One
        control, both jobs, no mode to get wrong.
        """
        box_w = make_well(parent)
        box_w.pack(fill="x", pady=(px(16), 0))
        box = box_w.body

        head = tk.Frame(box, bg=LED_BG)
        head.pack(fill="x", padx=12, pady=(9, 4))
        tk.Label(head, text="SAVED LIMIT SETS", bg=LED_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(side="left")
        self._preset_count_v = tk.StringVar(value="")
        tk.Label(head, textvariable=self._preset_count_v, bg=LED_BG, fg=TEXT_DIM,
                 font=FONT_HINT).pack(side="right")

        row = tk.Frame(box, bg=LED_BG)
        row.pack(fill="x", padx=12, pady=(0, 10))
        self._preset_name_v = tk.StringVar(value="")
        self._preset_combo = ttk.Combobox(row, textvariable=self._preset_name_v,
                                          width=22, values=[])
        self._preset_combo.pack(side="left", padx=(0, 10))
        RoundedButton(row, text="SAVE", bg_color=SURFACE, fg_color=TEXT_LIGHT,
                      width=86, height=28, radius=10, font=FONT_CAPTION,
                      command=self._save_limit_preset).pack(side="left", padx=(0, 6))
        RoundedButton(row, text="LOAD", bg_color=SURFACE, fg_color=TEXT_LIGHT,
                      width=86, height=28, radius=10, font=FONT_CAPTION,
                      command=self._load_limit_preset).pack(side="left", padx=(0, 6))
        RoundedButton(row, text="DELETE", bg_color=ACCENT_RED, fg_color=INK_DARK,
                      width=90, height=28, radius=10, font=FONT_CAPTION,
                      command=self._delete_limit_preset).pack(side="left")

        self._refresh_preset_list()

    def _capture_limit_here(self, key):
        """"Set the current position as this limit" — the point of the
        button is that nobody has to read a number off a screen and retype
        it into the right box."""
        _label, axis, _end, unit, _lo, _hi, _default, decimals = LIMIT_FIELDS[key]
        source = LIMIT_LIVE_SOURCE.get(axis)
        if source is None:
            return
        value = getattr(self, source)
        self._limit_vars[key].set(f"{value:.{decimals}f}")
        self.log(f"Captured current position as {axis} {LIMIT_FIELDS[key][0]}: "
                 f"{value:.{decimals}f} {unit}. Press APPLY to write it to the board.")
        if key in LIMIT_CAPTURE_ONLY:
            self.log("   The two elbow limits are interchangeable — teach "
                     "either end first; the lower of the two is used as the "
                     "lower limit.")
        if not self.is_homed:
            self.log("⚠ The machine has NO reference yet (no HOME / RESET "
                     "COORDINATES), so that figure is a raw step count, not a "
                     "real position.", tag="warn")

    def _apply_limits(self):
        # APPLY writes every pair, including axes whose enforcement is off.
        # There is no per-axis refusal here any more: the old LOCK made a
        # pair read-only, and skipping those was what stopped a preset LOAD
        # overwriting a boundary somebody had frozen. Enforcement is not a
        # write guard, so treating it as one would mean a switched-off axis
        # could never be taught — which is exactly when you teach it.
        limits, error = self._collect_limits()
        if error:
            messagebox.showerror("Error", error)
            return
        self.settings.update(**limits)
        self._send_limits()
        # The P2P panel advertises the reachable band from these very
        # numbers, so it has to be repainted here. It is the ONLY statement
        # of the working envelope now that the structural one is gone —
        # leaving it stale would have the panel naming limits that no
        # longer exist.
        self._refresh_workspace_hint()
        self._save_settings_file()
        self.log(f"Boundaries applied — "
                 f"ZM {limits['lim_z_min']:g}..{limits['lim_z_max']:g} mm · "
                 f"RM {limits['lim_rot_min']:g}..{limits['lim_rot_max']:g}° · "
                 f"A1M {limits['lim_a1_min']:g}..{limits['lim_a1_max']:g}° · "
                 f"A2M {limits['lim_a2_min']:g}..{limits['lim_a2_max']:g}°")
        if not self._hardware_live():
            self.log("No board confirmed yet — limits will be re-sent as soon as "
                     "the handshake succeeds.", tag="warn")

    def _default_limits(self):
        for key, spec in LIMIT_FIELDS.items():
            self._limit_vars[key].set(f"{spec[6]:g}")
        self.log("Boundaries reset to the mechanical envelope — press APPLY "
                 "to send them.")

    def _collect_limits(self):
        out = {}
        for key in LIMIT_KEYS:
            label, axis, _end, unit, floor_v, ceil_v, _default, _dec = LIMIT_FIELDS[key]
            value, error = self._read_number(self._limit_vars[key], f"{axis} {label}")
            if error:
                return None, error
            # floor/ceil None = no envelope. The elbows are the case: their
            # reported angle rides on an unmeasured gear ratio and really
            # does reach four figures, so any number the machine shows is
            # a number the machine can be standing at.
            if floor_v is not None and not (floor_v - 1e-6 <= value <= ceil_v + 1e-6):
                return None, (
                    f"“{axis} {label}” = {value:g}{unit} is outside the MECHANICAL "
                    f"envelope [{floor_v:g}, {ceil_v:g}]{unit}.\n\n"
                    f"That is the structure's own limit — no setting can allow "
                    f"the machine past it.")
            out[key] = value

        for heading, min_key, max_key, span in LIMIT_GROUPS:
            lo, hi = out[min_key], out[max_key]
            unit = LIMIT_FIELDS[min_key][3]

            if span is None:
                # Unordered pair: teach two positions, in any order. The
                # only thing that cannot work is teaching the SAME
                # position twice — a zero-width band pins the axis where
                # it stands and no jog gets it out.
                if lo == hi:
                    return None, (
                        f"{heading}: both limits are the same position "
                        f"({lo:g}{unit}).\n\n"
                        f"That leaves the axis no room to move at all. Jog to "
                        f"the other end of the travel you want and press SET "
                        f"HERE there.")
                # Stored RAW, in the box it was captured into. Sorting
                # here would fold the first taught position against
                # whatever stale value sat in the other box, so the
                # operator's second SET HERE would quietly overwrite
                # their first. The pair is sorted where it is READ
                # (_limit_pair) instead, so both numbers survive.
                continue

            # An ordered pair that inverts or collapses would make every
            # position illegal and the axis could not be jogged back out.
            if hi - lo < span:
                return None, (
                    f"{heading}: the upper limit ({hi:g}{unit}) must be at least "
                    f"{span:g}{unit} above the lower limit ({lo:g}{unit}).\n\n"
                    f"Otherwise every position counts as a violation and the axis "
                    f"cannot be jogged back out.")
        return out, None

    # ══════════════════════════════════════════════════════════════
    # TAB 3 — PID
    # ══════════════════════════════════════════════════════════════
    def _build_pid_tab(self, page, dlg):
        self._tab_buttons(page, self._apply_pid, self._default_pid, "PID")
        body = self._tab_body(page)

        note_w = make_well(body)
        note_w.pack(fill="x")
        note = note_w.body
        tk.Label(note, text=f"{PID_PRESET_NAME}  ·  ts ≈ {PID_PRESET['ts']:.2f}s",
                 bg=LED_BG, fg=ACCENT_MINT,
                 font=FONT_BUTTON).pack(anchor="w", padx=12, pady=(9, 1))
        tk.Label(note, text=PID_PRESET["note"], bg=LED_BG, fg=TEXT_MUTED,
                 font=FONT_SMALL, justify="left", wraplength=580).pack(
            anchor="w", padx=12, pady=(0, 9))

        grid = tk.Frame(body, bg=PANEL_BG)
        grid.pack(fill="x", pady=(16, 0))
        grid.grid_columnconfigure(0, weight=1)

        for row, (key, label, lock_key) in enumerate(PID_FIELDS):
            _wrap, entry = self._field_row(grid, row, label,
                                           self._pid_vars[key], "")
            self._pid_entries[key] = entry
            btn = RoundedButton(grid, text="", bg_color=SURFACE,
                                fg_color=TEXT_LIGHT, width=104, height=28,
                                radius=10, font=FONT_CAPTION,
                                command=lambda lk=lock_key: self._toggle_pid_lock(lk))
            btn.grid(row=row, column=3, sticky="w", padx=(12, 0))
            self._pid_lock_buttons[lock_key] = btn

        self._refresh_pid_locks()
        self._help_block(body, PID_HELP)

    def _toggle_pid_lock(self, lock_key):
        """Locks ONE gain. The others stay editable and keep being sent —
        which is the point: freezing Kd while tuning Kp is a normal thing
        to want, and the old single switch turned the whole thing off."""
        self._pid_locks[lock_key] = not self._pid_locks[lock_key]
        self._refresh_pid_locks()

    def _refresh_pid_locks(self):
        for key, _label, lock_key in PID_FIELDS:
            locked = self._pid_locks[lock_key]
            entry = self._pid_entries.get(key)
            if entry is not None:
                entry.config(state="readonly" if locked else "normal")
            btn = self._pid_lock_buttons.get(lock_key)
            if btn is not None:
                btn.set_config("LOCKED" if locked else "UNLOCKED",
                               ACCENT_ORANGE if locked else SURFACE,
                               icon="🔒" if locked else "🔓",
                               fg_color=INK_DARK if locked else TEXT_MUTED)

    def _apply_pid(self):
        pid, error = self._collect_pid()
        if error:
            messagebox.showerror("Error", error)
            return
        self.settings.update(**pid)
        self._send_pid()
        self._save_settings_file()
        locked = [label for _k, label, lock in PID_FIELDS if self._pid_locks[lock]]
        self.log(f"PID applied — Kp={pid['kp']:g} Ki={pid['ki']:g} "
                 f"Kd={pid['kd']:g} N={pid['n']:g}")
        if locked:
            self.log("Locked (kept at their current value): "
                     + ", ".join(l.split(" —")[0] for l in locked))

    def _default_pid(self):
        """Restores the preset — but NOT into locked fields. A lock that
        a 'restore defaults' silently overrode would not be a lock."""
        skipped = []
        for key, label, lock_key in PID_FIELDS:
            if self._pid_locks[lock_key]:
                skipped.append(label.split(" —")[0])
                continue
            self._pid_vars[key].set(f"{PID_PRESET[key]:g}")
        self.log("PID fields reset to the report preset — press APPLY to send them."
                 + (f" Skipped locked: {', '.join(skipped)}." if skipped else ""))

    def _collect_pid(self):
        values = {}
        for key, label, _lock in PID_FIELDS:
            value, error = self._read_number(self._pid_vars[key], label)
            if error:
                return None, error
            values[key] = value

        if min(values["kp"], values["ki"], values["kd"]) < 0:
            return None, "Kp, Ki and Kd must not be negative."
        if not (N_FILTER_MIN <= values["n"] <= N_FILTER_MAX):
            return None, (
                f"Derivative filter N must be between {N_FILTER_MIN:.0f} and "
                f"{N_FILTER_MAX:.0f}.\nThe report recommends 50–100 to suppress "
                f"encoder quantisation noise.")
        for _k, _label, lock_key in PID_FIELDS:
            values[lock_key] = self._pid_locks[lock_key]
        return values, None

    # ══════════════════════════════════════════════════════════════
    # TAB — CONTROLS (jog key bindings)
    # ══════════════════════════════════════════════════════════════
    def _build_controls_tab(self, page, dlg):
        self._tab_buttons(page, self._apply_keybinds, self._default_keybinds,
                          "key bindings")
        body = self._tab_body(page)

        self._kb_draft = dict(keybinds.active_map())
        self._kb_capturing = None
        self._kb_rows = {}

        # One row per action. No preset strip: the layout is entirely the
        # operator's, and a menu of opinions above their own bindings only
        # gets in the way.
        grid = tk.Frame(body, bg=PANEL_BG)
        grid.pack(fill="x")
        grid.grid_columnconfigure(0, weight=1)
        grid.grid_columnconfigure(3, weight=1)

        for i, action in enumerate(keybinds.ACTION_ORDER):
            col = 0 if i < 4 else 3
            row = i if i < 4 else i - 4
            tk.Label(grid, text=keybinds.ACTION_LABEL[action], bg=PANEL_BG,
                     fg=TEXT_LIGHT, font=FONT_LABEL, anchor="w").grid(
                row=row, column=col, sticky="w", pady=px(3), padx=(0, px(8)))
            btn = RoundedButton(
                grid, text=keybinds.display_key(self._kb_draft.get(action, "")),
                bg_color=SURFACE, fg_color=TEXT_LIGHT, width=92, height=30,
                radius=RADIUS_MD, font=FONT_BUTTON,
                command=lambda a=action: self._begin_capture(a))
            btn.grid(row=row, column=col + 1, sticky="w", padx=(0, px(18)))
            self._kb_rows[action] = btn

        self._kb_status_v = tk.StringVar(value="")
        tk.Label(body, textvariable=self._kb_status_v, bg=PANEL_BG,
                 fg=ACCENT_RED, font=FONT_SMALL, justify="left", anchor="w",
                 wraplength=px(640)).pack(fill="x", pady=(px(12), 0))

        self._help_block(body, CONTROLS_HELP)
        self._refresh_keybind_ui()

        # Capture happens on the dialog, not on the button: the key you
        # press must be caught wherever focus happens to be.
        dlg.bind("<KeyPress>", self._on_capture_key, add="+")

    def _begin_capture(self, action):
        self._kb_capturing = action
        self._refresh_keybind_ui()

    def _on_capture_key(self, event):
        """Grabs the next keypress for the row being rebound.

        Escape cancels rather than binding: it is reserved (it opens and
        closes this window), and a user pressing it here means "stop",
        not "make Escape jog an axis".
        """
        if self._kb_capturing is None:
            return None
        keysym = event.keysym
        if keysym == "Escape":
            self._kb_capturing = None
            self._kb_status_v.set("Rebinding cancelled.")
            self._refresh_keybind_ui()
            return "break"
        if keysym in keybinds.RESERVED_KEYS:
            self._kb_status_v.set(
                f"{keybinds.display_key(keysym)} is reserved for "
                f"{keybinds.RESERVED_KEYS[keysym]} — pick another key.")
            return "break"

        keysym = keysym.lower() if len(keysym) == 1 else keysym
        action = self._kb_capturing

        # If the key is already used elsewhere, SWAP the two rather than
        # silently leaving an axis unbound. A swap is almost always what
        # someone means when they reassign a key that is already taken.
        for other, key in list(self._kb_draft.items()):
            if key == keysym and other != action:
                self._kb_draft[other] = self._kb_draft.get(action, "")
                self._kb_status_v.set(
                    f"{keybinds.display_key(keysym)} was on "
                    f"{keybinds.ACTION_LABEL[other]} — swapped.")
                break
        else:
            self._kb_status_v.set("")
        self._kb_draft[action] = keysym
        self._kb_capturing = None
        self._refresh_keybind_ui()
        return "break"

    def _refresh_keybind_ui(self):
        for action, btn in self._kb_rows.items():
            capturing = self._kb_capturing == action
            key = self._kb_draft.get(action, "")
            btn.set_config("press a key…" if capturing
                           else (keybinds.display_key(key) if key else "—"),
                           ACCENT_MINT if capturing else SURFACE,
                           fg_color=INK_DARK if capturing else TEXT_LIGHT)

        if self._kb_capturing is not None:
            return
        errors = keybinds.validate(self._kb_draft)
        # Errors only. There is deliberately no commentary on which keys
        # sit near which — that is the operator's call, and a controller
        # second-guessing it is noise.
        self._kb_status_v.set("✖  " + "  ".join(errors) if errors else "")

    def _default_keybinds(self):
        self._kb_draft = dict(keybinds.DEFAULT_KEYMAP)
        self._kb_capturing = None
        self._kb_status_v.set("")
        self._refresh_keybind_ui()
        self.log("Key bindings reset to the default layout — press APPLY "
                 "to use them.")

    def _apply_keybinds(self):
        errors = keybinds.validate(self._kb_draft)
        if errors:
            messagebox.showerror("Cannot apply these bindings",
                                 "\n\n".join(errors))
            return
        if not keybinds.save(self._kb_draft):
            messagebox.showerror("Error", "Could not save the key bindings.")
            return

        # Rebind and repaint immediately. No restart: the binder clears its
        # old bindings first, and every on-screen key name is a StringVar.
        self._bind_keys()
        if hasattr(self, "refresh_jog_keycaps"):
            self.refresh_jog_keycaps()
        self.log("Key bindings applied and saved — " +
                 keybinds.to_hint(keybinds.active_map()))

    # ══════════════════════════════════════════════════════════════
    # TAB 5 — APPEARANCE
    # ══════════════════════════════════════════════════════════════
    def _build_appearance_tab(self, page, dlg):
        # The button bar MUST be packed before the body. The body packs
        # with expand=True and would otherwise claim the whole page,
        # leaving the bottom bar with zero height — which is exactly how
        # the APPLY button went missing here while the other three tabs,
        # which build their bar first, were fine.
        self._tab_buttons(page, self._apply_palette,
                          lambda: self._pick_palette(DEFAULT_PALETTE),
                          "appearance")
        body = self._tab_body(page)

        tk.Label(body, text="COLOUR SCHEME", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(anchor="w")

        self._palette_var = tk.StringVar(value=PALETTE_NAME)
        listing = tk.Frame(body, bg=PANEL_BG)
        listing.pack(fill="x", pady=(8, 0))

        # A swatch strip per scheme, so the choice is made by eye rather
        # than by reading a name. The strip shows the four colours that
        # actually differ between schemes: card, control, accent, warning.
        self._palette_rows = {}
        for name, label, blurb in available_palettes():
            p = PALETTES[name]
            row = tk.Frame(listing, bg=PANEL_BG, cursor="hand2")
            row.pack(fill="x", pady=2)

            sw = tk.Frame(row, bg=PANEL_BG)
            sw.pack(side="left", padx=(0, 10))
            for key in ("PANEL_BG", "SURFACE", "ACCENT", "ACCENT_ORANGE"):
                chip = RoundedFrame(sw, bg=p[key], radius=RADIUS_SM * 0.5,
                                    border=BORDER, border_w=1)
                chip.pack(side="left", padx=1)
                tk.Frame(chip.body, bg=p[key], width=px(13),
                         height=px(13)).pack()

            txt = tk.Frame(row, bg=PANEL_BG)
            txt.pack(side="left", fill="x", expand=True)
            name_lab = tk.Label(txt, text=label, bg=PANEL_BG, fg=TEXT_LIGHT,
                                font=FONT_BUTTON, anchor="w")
            name_lab.pack(anchor="w")
            tk.Label(txt, text=blurb, bg=PANEL_BG, fg=TEXT_DIM,
                     font=FONT_SMALL, anchor="w", justify="left",
                     wraplength=470).pack(anchor="w")

            mark = tk.Label(row, text="", bg=PANEL_BG, fg=ACCENT_MINT,
                            font=FONT_TITLE, width=2)
            mark.pack(side="right")
            self._palette_rows[name] = mark

            for w in (row, sw, txt, name_lab, mark):
                w.bind("<Button-1>", lambda e, n=name: self._pick_palette(n))
        self._refresh_palette_marks()

        self._palette_note_v = tk.StringVar(value="")
        tk.Label(body, textvariable=self._palette_note_v, bg=PANEL_BG,
                 fg=ACCENT_ORANGE, font=FONT_CAPTION,
                 anchor="w", justify="left").pack(fill="x", pady=(12, 0))

        self._help_block(body, APPEARANCE_HELP)

    def _pick_palette(self, name):
        self._palette_var.set(name)
        self._refresh_palette_marks()

    def _refresh_palette_marks(self):
        chosen = self._palette_var.get()
        for name, mark in self._palette_rows.items():
            mark.config(text="●" if name == chosen else "")

    def _apply_palette(self):
        """Applies the scheme immediately — no restart.

        The Settings window is closed FIRST and the switch deferred by one
        event-loop tick. Two reasons, both learned the hard way:

          * this method is running from a button INSIDE the window that
            the rebuild is about to destroy, and returning into a dead
            widget is how Tk produces its less helpful tracebacks;
          * the dialog holds a grab, so rebuilding underneath it would
            leave the new window unable to take input.
        """
        from ..theme import PALETTE_NAME as CURRENT

        name = self._palette_var.get()
        if name == CURRENT:
            self._palette_note_v.set("")
            self.log(f"Colour scheme is already “{PALETTES[name]['label']}”.")
            return

        dlg = getattr(self, "_settings_dlg", None)
        if dlg is not None and dlg.winfo_exists():
            self._close_settings(dlg)
        self.root.after_idle(lambda: self._finish_palette_switch(name))

    def _finish_palette_switch(self, name):
        if not self.apply_theme(name):
            # apply_theme refuses mid-motion and says why, so the choice is
            # saved for next time rather than silently dropped.
            save_active_name(name)

    # ══════════════════════════════════════════════════════════════
    # shared helpers
    # ══════════════════════════════════════════════════════════════
    @staticmethod
    def _read_number(var, label):
        raw = var.get().strip().replace(",", ".")
        try:
            return float(raw), None
        except ValueError:
            return None, f"“{label}” is not a valid number: {raw!r}"

    def _close_settings(self, dlg):
        self._settings_dlg = None
        try:
            dlg.grab_release()
        except tk.TclError:
            pass
        dlg.destroy()
