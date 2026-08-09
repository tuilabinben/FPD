"""Jog key bindings — editable, validated, persisted.

The layout is entirely the operator's choice. There are no preset
layouts and no advice about which keys sit near which: what is
comfortable depends on the hands using it, and a controller that argues
with that is just noise between you and the machine.

What IS still enforced is the small set of things that would leave the
app broken rather than merely unusual:

  * every jog action must be bound to something, or that axis becomes
    unreachable from the keyboard with no indication why;
  * no key may drive two actions, for the same reason;
  * SPACE, ESC, BackSpace and Return are reserved — losing the emergency
    stop, the settings window, HOME or RUN PROGRAM to a rebinding is not
    a trade worth offering.

Everything else is allowed.

PERSISTENCE
-----------
The layout is written to keybinds.json on APPLY and read back at startup,
so a custom layout survives restarts. A file that is corrupt, or that is
missing an action, is discarded WHOLE rather than merged: a half-loaded
keymap would leave some axes on your keys and others on the defaults,
which is harder to notice than simply reverting.
"""

import json
import os

from . import paths

_HERE = paths.user_data_dir()
KEYBINDS_FILE = os.path.join(_HERE, "keybinds.json")


# ══════════════════════════════════════════════════════════════════════
# The bindable actions
# ══════════════════════════════════════════════════════════════════════
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

#: Reserved — these do something else and may not be taken by a jog axis.
#:
#: THE KEYS HERE MUST BE TK KEYSYMS, spelled exactly as Tk reports them,
#: because that is what a captured keypress is compared against. Tk's
#: backspace keysym is "BackSpace" with a capital S — an entry spelled
#: "backspace" never matches anything, so the key looks reserved in the
#: settings list while a jog axis can still be bound to it.
#:
#: HOME is BackSpace, not H. It was H, and H is a letter an operator may
#: reasonably want for a jog axis; a homing cycle is not something to
#: trigger by leaning on a letter key.
RESERVED_KEYS = {
    "space": "Emergency stop",
    "Escape": "Open / close Settings",
    "BackSpace": "HOME",
    "Return": "RUN PROGRAM (P2P)",
}

#: The single source of truth for which key starts a homing cycle. Both the
#: reservation above and the binding in core/keyboard.py read it, so they
#: cannot drift apart — which they had: RESERVED_KEYS said backspace while
#: the binder was still listening for "h".
HOME_KEY = "BackSpace"

#: How a key is written on screen.
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


# ══════════════════════════════════════════════════════════════════════
# Default layout
# ══════════════════════════════════════════════════════════════════════
#: A/D rotation · I/K arm 1 · O/L arm 2 · W/S lift.
#:
#: This is only where an untouched install starts. Once APPLY has been
#: pressed the saved layout wins, and DEFAULTS is the way back here.
DEFAULT_KEYMAP = {
    "ROT_CCW": "a", "ROT_CW": "d",
    "A1_FWD":  "i", "A1_BACK": "k",
    "A2_FWD":  "o", "A2_BACK": "l",
    "Z_UP":    "w", "Z_DOWN":  "s",
}


# ══════════════════════════════════════════════════════════════════════
# Validation
# ══════════════════════════════════════════════════════════════════════
def validate(keymap):
    """Returns a list of errors. Empty means the layout is usable.

    Only conditions that would leave an axis unreachable, or steal a
    reserved key, count. Nothing here is stylistic — a layout that merely
    looks unusual is simply applied.
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


# ══════════════════════════════════════════════════════════════════════
# Derived lookups the rest of the app uses
# ══════════════════════════════════════════════════════════════════════
def to_tk_keymap(keymap):
    """{tk keysym -> jog command}, which is what the binder needs."""
    return {key: action for action, key in keymap.items() if key}


def to_keycaps(keymap):
    """{jog command -> the text drawn on the pad}."""
    return {action: display_key(key) for action, key in keymap.items() if key}


def to_hint(keymap):
    """The one-line summary under the jog pads."""
    def pair(a, b):
        return f"{display_key(keymap.get(a, '?'))}/{display_key(keymap.get(b, '?'))}"
    return (f"{pair('ROT_CCW', 'ROT_CW')} = RM · "
            f"{pair('A1_FWD', 'A1_BACK')} = A1M · "
            f"{pair('A2_FWD', 'A2_BACK')} = A2M · "
            f"{pair('Z_UP', 'Z_DOWN')} = ZM · {display_key(HOME_KEY)} = HOME")


# ══════════════════════════════════════════════════════════════════════
# Persistence
# ══════════════════════════════════════════════════════════════════════
_active = None


def load():
    """The saved layout, or the default.

    Never raises. A corrupt or incomplete file is discarded whole and the
    default used instead — merging a partial file would put some axes on
    your keys and others on the defaults, which is a harder problem to
    spot than a clean revert.
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
