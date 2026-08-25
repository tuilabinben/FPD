"""Jog key bindings — editable, validated, persisted.

Layout entirely operator's choice. No preset layouts, no advice on which keys sit near which —
comfort depends on the hands using it.

Still enforced, only things that'd leave app broken rather than merely unusual:
  * every jog action must be bound, or axis unreachable from keyboard w/ no indication why
  * no key drives two actions, same reason
  * SPACE, ESC, BackSpace, Return reserved — losing e-stop/settings/HOME/RUN PROGRAM to a
    rebind not a trade worth offering

Everything else allowed.

PERSISTENCE: layout written to keybinds.json on APPLY, read back at startup, survives restarts.
Corrupt/incomplete file discarded WHOLE not merged — half-loaded keymap = some axes on your
keys, others on defaults, harder to spot than clean revert.
"""

import json
import os

from . import paths

_HERE = paths.user_data_dir()
KEYBINDS_FILE = os.path.join(_HERE, "keybinds.json")


#: (command, label, group). `group` is the axis the action belongs to.
JOG_ACTIONS = (
    ("ROT_CCW", "RM — rotate CCW", "RM"),
    ("ROT_CW",  "RM — rotate CW",  "RM"),
    ("A1_FWD",  "A1M — extend",    "A1M"),
    ("A1_BACK", "A1M — retract",   "A1M"),
    ("A2_FWD",  "A2M — extend",    "A2M"),
    ("A2_BACK", "A2M — retract",   "A2M"),
    ("Z_UP",    "ZM — lift up",    "ZM"),
    ("Z_DOWN",  "ZM — lift down",  "ZM"),
)
ACTION_ORDER = tuple(a[0] for a in JOG_ACTIONS)
ACTION_LABEL = {a[0]: a[1] for a in JOG_ACTIONS}
ACTION_GROUP = {a[0]: a[2] for a in JOG_ACTIONS}

#: Reserved — do something else, may not be taken by jog axis.
#: KEYS MUST BE TK KEYSYMS, spelled exactly as Tk reports them — that's what captured keypress
#: compared against. Tk backspace keysym is "BackSpace" capital S; "backspace" matches nothing,
#: key looks reserved in settings list while jog axis can still bind to it.
#: HOME is BackSpace, not H — H is a letter operator may want for jog axis; homing shouldn't
#: trigger by leaning on a letter key.
RESERVED_KEYS = {
    "space": "Emergency stop",
    "Escape": "Open / close Settings",
    "BackSpace": "HOME",
    "Return": "RUN PROGRAM (P2P)",
}

#: Single source of truth for key that starts homing cycle. Reservation above and binding in
#: core/keyboard.py both read it so they can't drift apart — they had: RESERVED_KEYS said
#: backspace while binder still listened for "h".
HOME_KEY = "BackSpace"

#: How a key is shown on screen.
KEY_DISPLAY = {
    "Left": "←", "Right": "→", "Up": "↑", "Down": "↓",
    "space": "SPACE", "Escape": "ESC", "Prior": "PgUp", "Next": "PgDn",
    "BackSpace": "BKSP", "Return": "ENTER", "Tab": "TAB", "Delete": "DEL",
    "KP_0": "Num0", "KP_1": "Num1", "KP_2": "Num2", "KP_3": "Num3",
    "KP_4": "Num4", "KP_5": "Num5", "KP_6": "Num6", "KP_7": "Num7",
    "KP_8": "Num8", "KP_9": "Num9",
}


def display_key(keysym):
    return KEY_DISPLAY.get(keysym, keysym.upper() if len(keysym) == 1 else keysym)


#: A/D rotation · I/K arm 1 · O/L arm 2 · W/S lift.
#: Only where untouched install starts. Once APPLY pressed, saved layout wins; DEFAULTS is way
#: back here.
DEFAULT_KEYMAP = {
    "ROT_CCW": "a", "ROT_CW": "d",
    "A1_FWD":  "i", "A1_BACK": "k",
    "A2_FWD":  "o", "A2_BACK": "l",
    "Z_UP":    "w", "Z_DOWN":  "s",
}


def validate(keymap):
    """Returns list of errors. Empty means layout usable.

    Only conditions leaving axis unreachable or stealing reserved key count. Nothing stylistic
    — merely-unusual layout is just applied.
    """
    errors = []

    missing = [ACTION_LABEL[a] for a in ACTION_ORDER if not keymap.get(a)]
    if missing:
        errors.append("Not bound: " + ", ".join(missing))

    seen = {}
    for action in ACTION_ORDER:
        key = keymap.get(action)
        if not key:
            continue
        if key in RESERVED_KEYS:
            errors.append(f"{display_key(key)} is reserved for "
                          f"{RESERVED_KEYS[key]} — pick another key.")
        if key in seen:
            errors.append(f"{display_key(key)} is bound to both "
                          f"{ACTION_LABEL[seen[key]]} and {ACTION_LABEL[action]}.")
        else:
            seen[key] = action
    return errors


def to_tk_keymap(keymap):
    """{tk keysym -> jog command}, what the binder needs."""
    return {key: action for action, key in keymap.items() if key}


def to_keycaps(keymap):
    """{jog command -> text drawn on pad}."""
    return {action: display_key(key) for action, key in keymap.items() if key}


def to_hint(keymap):
    """One-line summary under jog pads."""
    def pair(a, b):
        return f"{display_key(keymap.get(a, '?'))}/{display_key(keymap.get(b, '?'))}"
    return (f"{pair('ROT_CCW', 'ROT_CW')} = RM · "
            f"{pair('A1_FWD', 'A1_BACK')} = A1M · "
            f"{pair('A2_FWD', 'A2_BACK')} = A2M · "
            f"{pair('Z_UP', 'Z_DOWN')} = ZM · {display_key(HOME_KEY)} = HOME")


_active = None


def load():
    """Saved layout, or default.

    Never raises. Corrupt/incomplete file discarded whole, default used instead — merging
    partial file puts some axes on your keys, others on defaults, harder to spot than clean
    revert.
    """
    global _active
    if _active is not None:
        return dict(_active)
    try:
        with open(KEYBINDS_FILE, "r", encoding="utf-8") as fh:
            data = json.load(fh)
        candidate = {a: str(data[a]) for a in ACTION_ORDER if data.get(a)}
        if len(candidate) == len(ACTION_ORDER) and not validate(candidate):
            _active = candidate
        else:
            _active = dict(DEFAULT_KEYMAP)
    except Exception:
        _active = dict(DEFAULT_KEYMAP)
    return dict(_active)


def save(keymap):
    """Persists a layout. Returns False (and writes nothing) if invalid."""
    global _active
    if validate(keymap):
        return False
    _active = dict(keymap)
    try:
        with open(KEYBINDS_FILE, "w", encoding="utf-8") as fh:
            json.dump(_active, fh, indent=2)
    except OSError:
        return False
    return True


def active_map():
    return load()
