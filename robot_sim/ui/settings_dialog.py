"""Settings window — Speed, Boundaries, Controls, PID, Appearance.

STRUCTURE
---------
Five tabs, not one long scroll. Concerns unrelated: come here to change
speed, set up boundaries, rebind a key, or look at gains. Tab not in isn't
a tab scrolled past.

Each tab body scrolls own when content taller than window, scrollbar only
appears when needed. Action bar packed against bottom BEFORE body, so
APPLY never scrolls out of reach.

Each tab owns its own APPLY and DEFAULT, act ONLY on that tab. A single
global "reset to defaults" that also wiped boundaries because you wanted
to undo a speed change costs someone an afternoon of re-teaching positions.

WHAT IT DELIBERATELY DOESN'T DO
-------------------------------
  * No editable master RPM. Constant (config.MASTER_RPM), only percentages
    move. Two knobs producing same motion is one knob too many.
  * No P/PI/PD presets or controller-form selector — one gain set is what
    this machine uses; form field configured a structure that doesn't
    exist on an open-loop board.
  * No limit pair may invert or collapse. MIN above MAX makes every
    position illegal, axis can't be jogged back out.
  * Nothing applies on keystroke. Edits staged until APPLY — half-typed
    number is never a command.
"""

import json
import os
import tkinter as tk
from tkinter import font as tkfont, messagebox, ttk

# Tab strip metrics. The widths are DERIVED from the labels at build time --
# see open_settings_dialog() -- so these are only the padding around the
# text, the gap between tabs, and the strip's own inset. TAB_MIN_W keeps a
# short label like "PID" from becoming a stub.
TAB_TEXT_PAD = 22
TAB_MIN_W = 76
TAB_GAP = 6
TAB_STRIP_PAD = 16

from ..config import (
    LIMIT_FIELDS,
    LIMITS_ENABLED_KEY,
    PLC_LINK_ENABLED_KEY,
    PLC_SENSOR_ENFORCE_BY_AXIS,
    PLC_SENSOR_PANEL,
    ROT_MAX_DEG,
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
    SCAN_MAX_Z_KEY,
    SCAN_SETTING_FIELDS,
    SETTINGS_FILE,
    SETTINGS_SCHEMA,
    SETTINGS_SCHEMA_KEY,
    SETTINGS_GEOMETRY,
    SETTINGS_MIN_SIZE,
    SPEED_FIELDS,
    SPEED_KEYS,
    SPEED_PREVIEW_BY_KEY,
    ACCEL_FIELDS,
    ACCEL_KEYS,
    ACCEL_PREVIEW_BY_KEY,
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
from .scrolling import touchpad_scroll, wheel_scroll

SPEED_HELP = (
    "One fixed reference speed. Each motor runs at its own percentage of it.\n"
    f"100% = {MASTER_RPM:g} motor RPM. Per-axis gearing is already folded in, so the\n"
    "percentages are directly comparable across all three axes.\n"
    "There is no percentage ceiling: what is typed is what is sent."
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

SCAN_HELP = (
    "The scan panel asks for a sample rate, points per slice, slices and\n"
    "slice spacing, and derives the angular step and the sweep SPEED from\n"
    "them: 50 points at 50 Hz is one second a slice, so a 330° sweep runs\n"
    "at 330°/s. The board clamps a speed past what RM can do, and says so.\n"
    "\n"
    "The ceiling below is on the TOTAL lift one scan uses — spacing ×\n"
    "(slices − 1), because the lift only moves BETWEEN slices. Past it the\n"
    "panel warns and asks; it does not refuse. The machine's own 285 mm\n"
    "stroke is the hard limit and lives on the board — this is your own\n"
    "working ceiling, normally well inside it."
)

PID_HELP = (
    "Source: Stepper MATLAB report, Table 2. Plant identified as\n"
    "G(s) = 12.5 / (s(s+12.5)) rad per pulse, pole placement at ζ = 0.7071.\n"
    "THE BOARD RUNS OPEN LOOP — the encoder only monitors — so these gains are\n"
    "stored and echoed, not used to close a loop. They exist so the controller\n"
    "configuration matches the documented Simulink model.\n"
    "Locking a term freezes just that one; the others keep being sent."
)


# RM boundary fields show 0..-340 instead of 0..340 — operator's requested
# convention for the two RM entries only. Cosmetic at this boundary alone:
# self.settings, the wire protocol, presets, and the live jog frame all
# stay native 0..340 (RM's zero is its CCW stop, counting up — unchanged).
# A stored 0 (home) displays as -340; a stored 340 displays as 0.
_ROT_LIMIT_KEYS = ("lim_rot_min", "lim_rot_max")


def _rot_limit_to_display(key, value):
    return value - ROT_MAX_DEG if key in _ROT_LIMIT_KEYS else value


def _rot_limit_from_display(key, value):
    return value + ROT_MAX_DEG if key in _ROT_LIMIT_KEYS else value


class SettingsDialogMixin:
    # GUI is system of record for machine setup
    def _load_settings_file(self):
        """Merges saved setup over defaults.

        Called from _init_state(), before log widget exists — must never
        call self.log(). Anything worth saying queued in
        _pending_settings_notes, flushed once UI is up.
        """
        self._pending_settings_notes = []
        try:
            with open(SETTINGS_FILE, "r", encoding="utf-8") as fh:
                saved = json.load(fh)
        except FileNotFoundError:
            return
        except (OSError, ValueError) as e:
            # Corrupt file must not stop app starting — fall back to safe
            # defaults, say so loudly later.
            self._pending_settings_notes.append(
                (f"Could not read {os.path.basename(SETTINGS_FILE)} ({e}). "
                 f"Using default settings.", "error"))
            return

        if not isinstance(saved, dict):
            self._pending_settings_notes.append(
                ("Settings file has the wrong format — using defaults.", "error"))
            return

        # File written before elbow angle moved to from-home frame holds
        # taught limits that no longer mean what they say. Drop exactly
        # those, keep rest — speeds and ZM/RM boundaries are in units that
        # didn't change.
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
            # RM's zero moved from mid-travel to CCW stop. Stored ±150 read
            # in new frame refuses every bearing past 150°, incl any
            # negative X (bearing 180°) — exact "there is a limit" complaint
            # on a reachable target.
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

        # Only keys app knows about, only if they still parse. Stale file
        # from older version must not inject arbitrary value into settings.
        applied = 0
        for key, value in saved.items():
            if key not in self.settings:
                continue
            # Booleans, not numbers: float(True) would store 1.0, then
            # every later `is True` check quietly false.
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

    # Pushing setup to the board
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
        # VALUES FIRST, then switches. Board holds limits in RAM only, runs
        # every handshake; switches first would leave window where axis is
        # armed against whatever board happened to be holding.
        #
        # No unlock/write/re-lock dance any more — needed only because
        # locked axis refused SET_LIMIT. Enforcement doesn't gate writes,
        # switched-off axis still takes its values.
        for key in LIMIT_KEYS:
            _label, axis, end, _unit, _lo, _hi, _default, _dec = LIMIT_FIELDS[key]
            self.send(f"SET_LIMIT:{axis},{end},{s[key]:g}", log_tx=False)
        for axis, enforce_key in LIMIT_ENFORCE_BY_AXIS.items():
            on = 1 if s.get(enforce_key, True) else 0
            self.send(f"SET_LIMIT_ENFORCE:{axis},{on}", log_tx=False)
        self.send(f"SET_LIMITS_ENABLED:{1 if s.get(LIMITS_ENABLED_KEY, True) else 0}",
                  log_tx=False)
        self.send(f"SET_PLC_LINK:{1 if s.get(PLC_LINK_ENABLED_KEY, True) else 0}",
                  log_tx=False)
        for axis, enforce_key in PLC_SENSOR_ENFORCE_BY_AXIS.items():
            on = 1 if s.get(enforce_key, True) else 0
            self.send(f"SET_PLC_SENSOR_ENFORCE:{axis},{on}", log_tx=False)

    def _push_settings_to_board(self, reason=""):
        """Sends speed, PID, every travel limit.

        Board keeps all this in RAM, must be re-sent after any board reset
        — else on-screen limits and actually-enforced limits drift apart,
        worst possible failure mode for a limit.
        """
        self._send_speed()
        self._send_pid()
        self._send_limits()
        if reason:
            self.log(f"Synced speed, PID and travel limits to the board ({reason}).")

    # Named limit presets — save / load / delete
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
        # Only presets carrying every key AND parsing as numbers. Partial
        # preset would load some axes, silently leave others on whatever
        # was in boxes — worst kind of half-applied limit.
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
        """Saves whatever is CURRENTLY IN THE BOXES, after validating.

        Validating first matters: preset is recalled later, trusted without
        re-reading — saving invalid/inverted set plants a fault found at
        the worst moment.
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
            self._limit_vars[key].set(f"{_rot_limit_to_display(key, value):g}")
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
        # TABS ARE SIZED TO THEIR LABELS, AND THE WINDOW TO THE TABS.
        #
        # Six tabs at a flat 128 px came to 836 px of strip inside a 780 px
        # dialog, so the last and widest of them -- Appearance -- was clipped
        # off the right edge. A fixed width is wrong in both directions here:
        # it wasted 100 px on "PID" while starving "Appearance", and nothing
        # connected it to the window it had to fit inside, so a seventh tab
        # or a larger UI font would quietly do it again.
        #
        # Measured, then the geometry is widened if the strip needs it. The
        # dialog is never narrower than its own tab row by construction.
        tabs = (("speed", "Speed", self._build_speed_tab),
                ("limits", "Boundaries", self._build_limits_tab),
                ("scan", "Scan", self._build_scan_tab),
                ("controls", "Controls", self._build_controls_tab),
                ("pid", "PID", self._build_pid_tab),
                ("appearance", "Appearance", self._build_appearance_tab))
        tab_font = tkfont.Font(root=dlg, font=FONT_BUTTON)
        tab_w = {key: max(TAB_MIN_W, tab_font.measure(label) + 2 * TAB_TEXT_PAD)
                 for key, label, _b in tabs}
        strip_w = sum(tab_w.values()) + len(tabs) * TAB_GAP + 2 * TAB_STRIP_PAD

        sw, sh = (int(v) for v in SETTINGS_GEOMETRY.lower().split("x"))
        sw = max(sw, strip_w)
        dlg.geometry(f"{px(sw)}x{px(sh)}")
        dlg.minsize(px(max(SETTINGS_MIN_SIZE[0], strip_w)),
                    px(SETTINGS_MIN_SIZE[1]))
        dlg.resizable(True, True)
        dlg.transient(self.root)
        dlg.grab_set()
        dlg.protocol("WM_DELETE_WINDOW", lambda: self._close_settings(dlg))
        # NO <Escape> binding on this window, deliberately.
        #
        # KeyboardMixin already binds Escape with bind_all, application-wide,
        # fires for this Toplevel too. Binding here too meant ONE keypress
        # ran BOTH handlers: this closed window, global toggle saw no window
        # open, reopened it — Escape appeared to do nothing. Global toggle
        # is single owner of the key.

        self._build_settings_vars()

        header = tk.Frame(dlg, bg=PANEL_BG)
        header.pack(fill="x", padx=18, pady=(14, 0))
        tk.Label(header, text="SETTINGS", bg=PANEL_BG, fg=TEXT_LIGHT,
                 font=FONT_TITLE).pack(side="left")
        tk.Label(header, text="   ESC to close", bg=PANEL_BG, fg=TEXT_DIM,
                 font=FONT_SMALL).pack(side="left", pady=(6, 0))

        # ttk notebook draws square tabs, can't round shape without custom
        # image element per state. Strip built from RoundedButtons instead:
        # fully rounded, styled by same code as every other control.
        strip = tk.Frame(dlg, bg=PANEL_BG)
        strip.pack(fill="x", padx=px(TAB_STRIP_PAD), pady=(px(12), 0))

        holder = tk.Frame(dlg, bg=PANEL_BG)
        holder.pack(fill="both", expand=True, padx=px(14), pady=(px(10), px(14)))

        self._tab_buttons_bar = {}
        self._tab_pages = {}
        for key, label, builder in tabs:
            btn = RoundedButton(strip, text=label, bg_color=SURFACE,
                                fg_color=TEXT_MUTED, width=tab_w[key], height=34,
                                radius=RADIUS_MD, font=FONT_BUTTON,
                                command=lambda k=key: self._show_tab(k))
            btn.pack(side="left", padx=(0, px(TAB_GAP)))
            self._tab_buttons_bar[key] = btn

            page = tk.Frame(holder, bg=PANEL_BG)
            self._tab_pages[key] = page
            builder(page, dlg)
            # Tab's widgets exist now, hook wheel to them too.
            for child in page.winfo_children():
                handler = getattr(child, "_wheel_handler", None)
                if handler:
                    self._bind_wheel_deep(child, handler)

        self._active_tab = None
        self._show_tab("speed")
        self._refresh_speed_preview()

    def _show_tab(self, key):
        """Raises one page, restyles strip. Only one page packed at a time,
        unseen tab costs nothing to lay out."""
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
        self._limit_vars = {k: tk.StringVar(value=f"{_rot_limit_to_display(k, s[k]):g}")
                            for k in LIMIT_FIELDS}
        self._scan_vars = {k: tk.StringVar(value=f"{float(s.get(k, spec[2])):g}")
                           for k, spec in SCAN_SETTING_FIELDS.items()}
        self._speed_preview_vars = {}
        self._speed_entry_wraps = {}
        self._accel_preview_vars = {}
        self._accel_entry_wraps = {}
        self._pid_entries = {}
        self._pid_lock_buttons = {}

        # Live readout of what each percentage actually means. Traced, not
        # recomputed on APPLY — whole point of one fixed reference speed is
        # seeing effect while typing.
        for var in self._speed_vars.values():
            var.trace_add("write", lambda *_a: self._refresh_speed_preview())
        for var in self._accel_vars.values():
            var.trace_add("write", lambda *_a: self._refresh_speed_preview())

    def _tab_body(self, page):
        """Scrollable content area for one tab.

        Tabs not all same height — Boundaries with saved-set row far taller
        than PID — on short window bottom of tallest was simply unreachable.
        Scrolling per tab not whole window, so action bar stays pinned:
        APPLY must never scroll away.

        Scrollbar created but only PACKED when content actually overflows,
        so short tab shows no furniture it doesn't need.
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

        # Wheel bound to this subtree only. bind_all would hijack wheel for
        # whole app, incl main window behind this one and combobox in
        # Boundaries tab.
        def _wheel(event):
            if not bar.winfo_ismapped():
                return None                     # nothing to scroll
            wheel_scroll(canvas, event)
            return "break"

        def _touchpad(event):
            """Two-finger swipe — separate event type under Tk 9, binding
            wheel alone leaves trackpad dead."""
            if not bar.winfo_ismapped():
                return None
            touchpad_scroll(canvas, event)
            return "break"

        self._bind_scroll_events(canvas, _wheel, _touchpad)
        self._bind_scroll_events(body, _wheel, _touchpad)
        body._wheel_handler = _wheel
        body._touchpad_handler = _touchpad
        self._bind_wheel_deep(body, _wheel, _touchpad)
        return body

    @staticmethod
    def _bind_scroll_events(widget, wheel_handler, touchpad_handler):
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            widget.bind(seq, wheel_handler)
        try:
            widget.bind("<TouchpadScroll>", touchpad_handler)
        except tk.TclError:
            pass                                # Tk 8.6 has no such event

    def _bind_wheel_deep(self, root_widget, handler, touchpad_handler=None):
        """Forwards wheel from children created later.

        Frames stacked on canvas swallow wheel events, scroll would die
        wherever pointer happened to be. Re-run after tab is populated.
        """
        if touchpad_handler is None:
            touchpad_handler = getattr(root_widget, "_touchpad_handler", None)

        def apply(widget):
            try:
                for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
                    widget.bind(seq, handler)
                if touchpad_handler is not None:
                    widget.bind("<TouchpadScroll>", touchpad_handler)
            except Exception:
                pass
            for child in widget.winfo_children():
                apply(child)
        root_widget.after_idle(lambda: apply(root_widget))

    def _tab_buttons(self, page, on_apply, on_default, what):
        """Per-tab action bar.

        APPLY and DEFAULT scoped to THIS tab only. Global reset that also
        wiped travel limits because someone wanted to undo a speed change
        is exactly the failure this avoids.
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

    # TAB 1 — SPEED
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
            # Real motor RPM beside percentage — percentage alone isn't a
            # quantity anyone can sanity-check.
            var = tk.StringVar(value="—")
            self._speed_preview_vars[key] = var
            tk.Label(grid, textvariable=var, bg=PANEL_BG, fg=TEXT_MUTED,
                     font=FONT_MONO, anchor="w").grid(
                row=row, column=3, sticky="w", padx=(14, 0))

        # ACCELERATION — independent per-axis %, own grid so it reads as
        # separate family, not continuation of speed rows.
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

        self._help_block(body, SPEED_HELP)

    def _refresh_speed_preview(self):
        """Recomputes readouts from whatever is currently typed.

        Mirrors firmware's arithmetic incl the clamp, so operator sees
        consequence before pressing APPLY, not reading it in log after.
        """
        if not getattr(self, "_speed_preview_vars", None):
            return
        # (vars dict, entry-wraps dict, preview-vars dict, PREVIEW_BY_KEY,
        #  keys, reference RPM) — one loop body drives both speed
        # and accel rows, identical shape, differ only in reference RPM
        # scaled from.
        families = (
            (self._speed_vars, self._speed_entry_wraps, self._speed_preview_vars,
             SPEED_PREVIEW_BY_KEY, SPEED_KEYS, MASTER_RPM),
            (self._accel_vars, self._accel_entry_wraps, self._accel_preview_vars,
             ACCEL_PREVIEW_BY_KEY, ACCEL_KEYS, MASTER_ACC_RPM_S),
        )
        for vars_, wraps, preview_vars, preview_by_key, keys, ref_rpm in families:
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
                    # Arm's rate depends on gear ratio nobody has measured —
                    # quoting it as fact would be lying with a decimal point.
                    text = f"= {motor:.1f} RPM"
                    if motor > rpm_ceiling:
                        text += f"  ▸ capped {rpm_ceiling:g}"
                else:
                    text = f"= {motor:.1f} RPM  ·  {speed:.2f} {unit}"
                    if speed > ceiling:
                        text += f"  ▸ capped {ceiling:g}"
                preview_vars[key].set(text)

                if wrap is not None:
                    set_entry_border(wrap, NEUTRAL_BORDER)

    def _apply_speed(self):
        speed, error = self._collect_speed()
        if error:
            messagebox.showerror("Error", error)
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

    # TAB 2 — TRAVEL LIMITS
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
            # One switch per PAIR, not per box. Boundary with only one end
            # enforced is not a boundary anyone asked for; two ends taught
            # together anyway.
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
                    # Read-only, not disabled: value must stay legible.
                    # Typing here means typing against elbow scale — exactly
                    # the thing that isn't trustworthy.
                    entry.config(state="readonly")
                self._limit_entries[key] = entry
                # Nothing else makes this entry read-only. Switched-off
                # boundary stays fully editable: teaching while not yet
                # policing anything is normal order of work, not edge case.
                RoundedButton(grid, text="SET HERE", bg_color=SURFACE,
                              fg_color=TEXT_LIGHT, width=106, height=28, radius=10,
                              font=FONT_CAPTION,
                              command=lambda k=key: self._capture_limit_here(k)).grid(
                    row=row, column=3, sticky="w", padx=(12, 0))
                row += 1
        self._refresh_limit_enforce()

        self._build_preset_row(body)

        # master enforcement switch — deliberately NOT staged behind APPLY.
        # "Turn limits off" is needed while a move is being set up; a
        # switch taking effect only after a second button is a switch
        # people believe already thrown.
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

        self._plc_link_btn = RoundedButton(
            enable_row, text="", bg_color=SURFACE, fg_color=TEXT_LIGHT,
            width=210, height=30, radius=10, font=FONT_CAPTION,
            command=self._toggle_plc_link)
        self._plc_link_btn.pack(side="left", padx=(12, 0))
        self._refresh_plc_link()

        # One switch per PLC travel-limit SENSOR (M30/M31/M32), separate
        # from PLC LINK above: that one stops the whole socket, HOME
        # included. This is for ONE broken switch — a stuck or noisy
        # sensor should not have to take HOME and the other two axes'
        # protection down with it. Disabling a sensor also excludes it
        # from the M30+M31+M32 HOME condition, or a broken switch would
        # block HOME forever.
        sensor_row = tk.Frame(body, bg=PANEL_BG)
        sensor_row.pack(fill="x", pady=(8, 0))
        tk.Label(sensor_row, text="PLC SENSORS", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(side="left")
        self._plc_sensor_enforce_buttons = {}
        for bit, label, axis, _cmd, _end in PLC_SENSOR_PANEL:
            enforce_key = PLC_SENSOR_ENFORCE_BY_AXIS[axis]
            btn = RoundedButton(
                sensor_row, text="", bg_color=SURFACE, fg_color=TEXT_LIGHT,
                width=160, height=30, radius=10, font=FONT_CAPTION,
                command=lambda ek=enforce_key, b=bit, a=axis:
                    self._toggle_plc_sensor_enforce(ek, b, a))
            btn.pack(side="left", padx=(12, 0))
            self._plc_sensor_enforce_buttons[enforce_key] = btn
        self._refresh_plc_sensor_enforce()

        # RESET COORDINATES moved to section 3, MOTION CONTROL: machine
        # action used while jogging to reference pose, reachable only
        # through this dialog meant leaving controls already in use.

        self._help_block(body, LIMIT_HELP)

    def _toggle_limit_enforce(self, enforce_key):
        """Switches ONE axis's boundary on or off.

        Not staged behind APPLY, same reason as master switch: reached for
        while setting up a move, safety switch needing second button to
        take effect is a switch people believe already thrown.

        Turning off is confirmed and logged as warning. Turning back on is
        neither — making machine safer isn't an event that interrupts
        anybody.
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
        # Switched-off axis stops advertising a band on P2P panel.
        self._refresh_workspace_hint()
        # Board holds same flag, so bare terminal and GUI can't disagree
        # about whether an axis is protected.
        self.send(f"SET_LIMIT_ENFORCE:{axis},{1 if want else 0}")
        self._save_settings_file()
        if want:
            self.log(f"{axis} boundary ENFORCED — the axis stops at it again.")
        else:
            self.log(f"⚠ {axis} BOUNDARY NOT ENFORCED. Its values are kept but "
                     f"nothing will stop {axis} at them.", tag="warn")

    def _refresh_limit_enforce(self):
        # Master switch shown here too: with it off, an axis individually
        # ON is still not protected, button reading ENFORCED in that state
        # would be false. Hence third caption.
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

        ANDs with per-axis switches, doesn't override: turning back on
        must not silently re-arm an axis operator switched off by itself.
        """
        want = not self.settings.get(LIMITS_ENABLED_KEY, True)
        if not want and not messagebox.askyesno(
                "Disable soft limits",
                "Turn OFF soft-limit enforcement on every axis?\n\n"
                "Your taught boundaries are kept and still shown, but nothing "
                "will stop an axis at them. The PLC's physical switches "
                "(M30..M32) become the only protection the machine has.\n\n"
                "Continue?"):
            return
        self.settings[LIMITS_ENABLED_KEY] = want
        self.send(f"SET_LIMITS_ENABLED:{1 if want else 0}")
        self._save_settings_file()
        self._refresh_limits_enabled()
        self._refresh_workspace_hint()
        # Per-axis captions depend on this switch — axis individually ON
        # reads "ON (MASTER OFF)" while it's off — must be repainted with
        # it, or four buttons keep claiming ENFORCED on a machine enforcing
        # nothing.
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

    def _toggle_plc_link(self):
        want = not self.settings.get(PLC_LINK_ENABLED_KEY, True)
        self.settings[PLC_LINK_ENABLED_KEY] = want
        self.send(f"SET_PLC_LINK:{1 if want else 0}")
        self._save_settings_file()
        self._refresh_plc_link()
        if want:
            self.log("PLC Link ENABLED.")
        else:
            self.log("⚠ PLC LINK DISABLED. ClearCore will not connect to the PLC.", tag="warn")

    def _refresh_plc_link(self):
        on = self.settings.get(PLC_LINK_ENABLED_KEY, True)
        btn = getattr(self, "_plc_link_btn", None)
        if btn is None:
            return
        btn.set_config("PLC CONNECTED" if on else "PLC DISABLED",
                       SURFACE if on else ACCENT_RED,
                       icon="🔌" if on else "⚠",
                       fg_color=TEXT_LIGHT if on else INK_DARK)

    def _toggle_plc_sensor_enforce(self, enforce_key, bit, axis):
        """Switches ONE PLC travel-limit sensor's boundary on or off.

        For a sensor that is physically broken — stuck, noisy, or never
        fitted right. Off: the switch never stops a jog/P2P leg on that
        axis, AND it stops counting toward the M30+M31+M32 HOME condition
        (see plcHomeStateActive() on the board) — otherwise a broken
        switch blocks HOME forever, with no way to tell the board "trust
        the other two".
        """
        want = not self.settings.get(enforce_key, True)
        if not want and not messagebox.askyesno(
                f"Disable {bit} ({axis})",
                f"Turn OFF the {bit} travel-limit switch on {axis}?\n\n"
                f"{axis} will no longer be stopped by {bit}, and HOME will "
                f"complete without waiting for it. Use this only if {bit} "
                f"is physically broken — it removes real protection on "
                f"{axis}.\n\nContinue?"):
            return
        self.settings[enforce_key] = want
        self.send(f"SET_PLC_SENSOR_ENFORCE:{axis},{1 if want else 0}")
        self._save_settings_file()
        self._refresh_plc_sensor_enforce()
        if want:
            self.log(f"{bit} ({axis}) ENFORCED — the switch protects {axis} again "
                     f"and counts toward HOME.")
        else:
            self.log(f"⚠ {bit} ({axis}) NOT ENFORCED. {axis} will not stop on it, "
                     f"and HOME no longer waits for it.", tag="warn")

    def _refresh_plc_sensor_enforce(self):
        buttons = getattr(self, "_plc_sensor_enforce_buttons", None)
        if not buttons:
            return
        for bit, _label, axis, _cmd, _end in PLC_SENSOR_PANEL:
            enforce_key = PLC_SENSOR_ENFORCE_BY_AXIS[axis]
            btn = buttons.get(enforce_key)
            if btn is None:
                continue
            on = self.settings.get(enforce_key, True)
            btn.set_config(f"{bit} ENFORCED" if on else f"{bit} NOT ENFORCED",
                           SURFACE if on else ACCENT_RED,
                           icon="🛡" if on else "⚠",
                           fg_color=TEXT_LIGHT if on else INK_DARK)

    def _build_preset_row(self, parent):
        """Save / load / delete named limit sets.

        Name box is editable combobox on purpose: typing new name saves
        new set, picking existing one targets that set. One control, both
        jobs, no mode to get wrong.
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
        """"Set current position as this limit" — point is nobody reads a
        number off screen and retypes it into the right box."""
        _label, axis, _end, unit, _lo, _hi, _default, decimals = LIMIT_FIELDS[key]
        source = LIMIT_LIVE_SOURCE.get(axis)
        if source is None:
            return
        value = getattr(self, source)
        self._limit_vars[key].set(f"{_rot_limit_to_display(key, value):.{decimals}f}")
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
        # APPLY writes every pair, incl axes whose enforcement is off. No
        # per-axis refusal any more: old LOCK made a pair read-only, and
        # skipping those stopped a preset LOAD overwriting a frozen
        # boundary. Enforcement isn't a write guard — treating it as one
        # would mean switched-off axis could never be taught, exactly when
        # you teach it.
        limits, error = self._collect_limits()
        if error:
            messagebox.showerror("Error", error)
            return
        self.settings.update(**limits)
        self._send_limits()
        # P2P panel advertises reachable band from these very numbers, must
        # repaint here. ONLY statement of working envelope now structural
        # one is gone — stale would have panel naming limits that no
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
            self._limit_vars[key].set(f"{_rot_limit_to_display(key, spec[6]):g}")
        self.log("Boundaries reset to the mechanical envelope — press APPLY "
                 "to send them.")

    def _collect_limits(self):
        out = {}
        for key in LIMIT_KEYS:
            label, axis, _end, unit, floor_v, ceil_v, _default, _dec = LIMIT_FIELDS[key]
            value, error = self._read_number(self._limit_vars[key], f"{axis} {label}")
            if error:
                return None, error
            # floor/ceil None = no envelope. Elbows are the case: reported
            # angle rides on unmeasured gear ratio, really does reach four
            # figures — any number machine shows is a number it can be
            # standing at.
            #
            # RM's two keys are validated and reported in DISPLAY units
            # (0..-340) — the units the operator actually typed — then
            # converted back to native 0..340 for storage below, so
            # everything past this point never sees the display convention.
            disp_floor = None if floor_v is None else _rot_limit_to_display(key, floor_v)
            disp_ceil = None if ceil_v is None else _rot_limit_to_display(key, ceil_v)
            if disp_floor is not None and disp_floor > disp_ceil:
                disp_floor, disp_ceil = disp_ceil, disp_floor
            if disp_floor is not None and not (disp_floor - 1e-6 <= value <= disp_ceil + 1e-6):
                return None, (
                    f"“{axis} {label}” = {value:g}{unit} is outside the MECHANICAL "
                    f"envelope [{disp_floor:g}, {disp_ceil:g}]{unit}.\n\n"
                    f"That is the structure's own limit — no setting can allow "
                    f"the machine past it.")
            out[key] = _rot_limit_from_display(key, value)

        for heading, min_key, max_key, span in LIMIT_GROUPS:
            lo, hi = out[min_key], out[max_key]
            unit = LIMIT_FIELDS[min_key][3]

            if span is None:
                # Unordered pair: teach two positions, any order. Only
                # thing that can't work is teaching SAME position twice —
                # zero-width band pins axis where it stands, no jog gets
                # it out.
                if lo == hi:
                    return None, (
                        f"{heading}: both limits are the same position "
                        f"({lo:g}{unit}).\n\n"
                        f"That leaves the axis no room to move at all. Jog to "
                        f"the other end of the travel you want and press SET "
                        f"HERE there.")
                # Stored RAW, in box it was captured into. Sorting here
                # would fold first taught position against whatever stale
                # value sat in other box — second SET HERE would quietly
                # overwrite first. Pair sorted where it's READ (_limit_pair)
                # instead, both numbers survive.
                continue

            # Ordered pair that inverts or collapses would make every
            # position illegal, axis couldn't be jogged back out.
            if hi - lo < span:
                return None, (
                    f"{heading}: the upper limit ({hi:g}{unit}) must be at least "
                    f"{span:g}{unit} above the lower limit ({lo:g}{unit}).\n\n"
                    f"Otherwise every position counts as a violation and the axis "
                    f"cannot be jogged back out.")
        return out, None

    # TAB 3 — PID
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
        """Locks ONE gain. Others stay editable, keep being sent — the
        point: freezing Kd while tuning Kp is normal to want; old single
        switch turned whole thing off."""
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
        """Restores preset — but NOT into locked fields. A lock 'restore
        defaults' silently overrode wouldn't be a lock."""
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

    # TAB — CONTROLS (jog key bindings)
    # TAB — SCAN
    def _build_scan_tab(self, page, dlg):
        self._tab_buttons(page, self._apply_scan, self._default_scan,
                          "scan settings")
        body = self._tab_body(page)

        grid = tk.Frame(body, bg=PANEL_BG)
        grid.pack(fill="x")
        grid.grid_columnconfigure(0, weight=1)
        for row, (key, spec) in enumerate(SCAN_SETTING_FIELDS.items()):
            label, unit, _default, _lo, _hi = spec
            self._field_row(grid, row, label, self._scan_vars[key], unit)

        self._help_block(body, SCAN_HELP)

    def _apply_scan(self):
        out = {}
        for key, (label, unit, _default, lo, hi) in SCAN_SETTING_FIELDS.items():
            value, error = self._read_number(self._scan_vars[key], label)
            if error:
                messagebox.showerror("Error", error)
                return
            if not (lo <= value <= hi):
                messagebox.showerror(
                    "Error",
                    f"“{label}” must be between {lo:g} and {hi:g} {unit}.\n\n"
                    f"You entered {value:g} {unit}. The upper end is the "
                    f"machine's own ZM stroke — a ceiling past it would never "
                    f"warn about anything.")
                return
            out[key] = value
        self.settings.update(**out)
        self._save_settings_file()
        # Nothing is sent to the board: this ceiling is the operator's own
        # working limit and is enforced at the panel. The board keeps its
        # own hard refusal at the end of the stroke, which is a different
        # number and not this one's business.
        self.log(f"Scan settings applied — maximum ZM travel per scan "
                 f"{out[SCAN_MAX_Z_KEY]:g} mm.")
        if hasattr(self, "_refresh_scan_hint"):
            self._refresh_scan_hint()

    def _default_scan(self):
        for key, spec in SCAN_SETTING_FIELDS.items():
            self._scan_vars[key].set(f"{spec[2]:g}")
        self.log("Scan settings reset to defaults — press APPLY to keep them.")

    def _build_controls_tab(self, page, dlg):
        self._tab_buttons(page, self._apply_keybinds, self._default_keybinds,
                          "key bindings")
        body = self._tab_body(page)

        self._kb_draft = dict(keybinds.active_map())
        self._kb_capturing = None
        self._kb_rows = {}

        # One row per action. No preset strip: layout is entirely the
        # operator's, menu of opinions above own bindings only gets in the
        # way.
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

        # Capture happens on dialog, not button: key pressed must be caught
        # wherever focus happens to be.
        dlg.bind("<KeyPress>", self._on_capture_key, add="+")

    def _begin_capture(self, action):
        self._kb_capturing = action
        self._refresh_keybind_ui()

    def _on_capture_key(self, event):
        """Grabs next keypress for the row being rebound.

        Escape cancels rather than binding: it's reserved (opens/closes
        this window), pressing it here means "stop", not "make Escape jog
        an axis".
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

        # If key already used elsewhere, SWAP the two rather than silently
        # leaving axis unbound. Swap is almost always what's meant when
        # reassigning an already-taken key.
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
        # Errors only. Deliberately no commentary on which keys sit near
        # which — operator's call, controller second-guessing it is noise.
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

        # Rebind and repaint immediately. No restart: binder clears old
        # bindings first, every on-screen key name is a StringVar.
        self._bind_keys()
        if hasattr(self, "refresh_jog_keycaps"):
            self.refresh_jog_keycaps()
        self.log("Key bindings applied and saved — " +
                 keybinds.to_hint(keybinds.active_map()))

    # TAB 5 — APPEARANCE
    def _build_appearance_tab(self, page, dlg):
        # Button bar MUST be packed before body. Body packs expand=True,
        # would otherwise claim whole page, leaving bottom bar zero height
        # — exactly how APPLY button went missing here while other three
        # tabs, building bar first, were fine.
        self._tab_buttons(page, self._apply_palette,
                          lambda: self._pick_palette(DEFAULT_PALETTE),
                          "appearance")
        body = self._tab_body(page)

        tk.Label(body, text="COLOUR SCHEME", bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_CAPTION).pack(anchor="w")

        self._palette_var = tk.StringVar(value=PALETTE_NAME)
        listing = tk.Frame(body, bg=PANEL_BG)
        listing.pack(fill="x", pady=(8, 0))

        # Swatch strip per scheme, choice made by eye not by reading name.
        # Strip shows four colours that actually differ between schemes:
        # card, control, accent, warning.
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
        """Applies scheme immediately — no restart.

        Settings window closed FIRST, switch deferred by one event-loop
        tick. Two reasons, both learned the hard way:

          * this method runs from a button INSIDE the window the rebuild
            is about to destroy — returning into dead widget is how Tk
            produces its less helpful tracebacks;
          * dialog holds a grab, rebuilding underneath it would leave new
            window unable to take input.
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
            # apply_theme refuses mid-motion, says why — choice saved for
            # next time rather than silently dropped.
            save_active_name(name)

    # shared helpers
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
