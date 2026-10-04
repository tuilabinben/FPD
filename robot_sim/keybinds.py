"""Jog key bindings — editable, validated, persisted.

Layout entirely operator's choice. No preset layouts, no advice on which keys sit near which —
comfort depends on the hands using it.

Still enforced, only things that'd leave app broken rather than merely unusual:
  * every jog action must be bound, or axis unreachable from keyboard w/ no indication why
  * no key drives two actions, same reason
  * SPACE, ESC, Return reserved — losing e-stop/settings/RUN PROGRAM to a rebind not a
    trade worth offering
  * RESET POS and HOME have a key each, rebindable, that no jog layout may also hold

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

#: The XYZ jog's OWN layout: the tool point along Cartesian axes instead of one motor each.
#: A separate map, not six more rows in the one above — only one of the two jog layouts is
#: live at a time (the JOINT / XYZ switch on the jog panel), so they may share keys, and by
#: default they do: W/S/A/D mean one thing on a motor and another on the tool.
XYZ_ACTIONS = (
    ("X_NEG", "XYZ — X −", "X"),
    ("X_POS", "XYZ — X +", "X"),
    ("Y_POS", "XYZ — Y +", "Y"),
    ("Y_NEG", "XYZ — Y −", "Y"),
    ("Z_POS", "XYZ — Z + (up)", "Z"),
    ("Z_NEG", "XYZ — Z − (down)", "Z"),
)
XYZ_ORDER = tuple(a[0] for a in XYZ_ACTIONS)

#: Two keys that are not jog axes: one press, one action. Rebindable — asked for. Unlike the
#: two jog layouts they are live whichever layout is, so they may share a key with NEITHER.
SHORTCUT_ACTIONS = (
    ("RESET_POS", "RESET POS — drive to 0,0,0,0", "KEYS"),
    ("HOME", "HOME", "KEYS"),
)
SHORTCUT_ORDER = tuple(a[0] for a in SHORTCUT_ACTIONS)

ACTION_LABEL = {a[0]: a[1] for a in JOG_ACTIONS + XYZ_ACTIONS + SHORTCUT_ACTIONS}

#: Reserved — do something else, may not be taken by jog axis.
#: KEYS MUST BE TK KEYSYMS, spelled exactly as Tk reports them — that's what captured keypress
#: compared against. Tk backspace keysym is "BackSpace" capital S; "backspace" matches nothing,
#: key looks reserved in settings list while jog axis can still bind to it.
#: HOME is NOT in here any more. It was reserved on BackSpace; BackSpace was asked for as
#: RESET POS, rebindable, so both became shortcuts (DEFAULT_SHORTCUTS) and HOME moved.
RESERVED_KEYS = {
    "space": "Emergency stop",
    "Escape": "Open / close Settings",
    "Return": "RUN PROGRAM (P2P)",
}

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

#: W/S = Y · A/D = X · R/F = Z, as asked for.
DEFAULT_XYZ_KEYMAP = {
    "Y_POS": "w", "Y_NEG": "s",
    "X_NEG": "a", "X_POS": "d",
    "Z_POS": "r", "Z_NEG": "f",
}

#: BackSpace = RESET POS, as asked for. HOME had BackSpace and went to the Home key: not H,
#: a letter an axis may want and one too easy to lean on.
DEFAULT_SHORTCUTS = {"RESET_POS": "BackSpace", "HOME": "Home"}


def validate(keymap, order=ACTION_ORDER):
    """Returns list of errors. Empty means layout usable.

    Only conditions leaving axis unreachable or stealing reserved key count. Nothing stylistic
    — merely-unusual layout is just applied. `order` is the layout being checked: the joint
    one by default, XYZ_ORDER for the XYZ jog's.
    """
    errors = []

    missing = [ACTION_LABEL[a] for a in order if not keymap.get(a)]
    if missing:
        errors.append("Not bound: " + ", ".join(missing))

    seen = {}
    for action in order:
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


def validate_shortcuts(shortcuts, *jog_layouts):
    """validate(), plus: a shortcut may not sit on a key ANY jog layout holds. The two jog
    layouts share keys because only one is bound at a time; a shortcut is bound in both."""
    errors = validate(shortcuts, SHORTCUT_ORDER)
    for layout in jog_layouts:
        for action, key in layout.items():
            for mine in SHORTCUT_ORDER:
                if key and shortcuts.get(mine) == key:
                    errors.append(f"{display_key(key)} is bound to both "
                                  f"{ACTION_LABEL[mine]} and {ACTION_LABEL[action]}.")
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
            f"{pair('Z_UP', 'Z_DOWN')} = ZM")


def to_xyz_hint(keymap):
    """The same line for the XYZ jog's layout."""
    def pair(a, b):
        return f"{display_key(keymap.get(a, '?'))}/{display_key(keymap.get(b, '?'))}"
    return (f"{pair('X_NEG', 'X_POS')} = X · {pair('Y_POS', 'Y_NEG')} = Y · "
            f"{pair('Z_POS', 'Z_NEG')} = Z")


def to_shortcut_hint(shortcuts):
    """The two one-press keys, for the end of either line above."""
    return (f"{display_key(shortcuts.get('RESET_POS', '?'))} = RESET POS · "
            f"{display_key(shortcuts.get('HOME', '?'))} = HOME")


_active = None
_active_xyz = None
_active_shortcuts = None


def _layout(data, order, default):
    """One layout out of the file, or its default WHOLE."""
    candidate = {a: str(data[a]) for a in order if data.get(a)}
    if len(candidate) == len(order) and not validate(candidate, order):
        return candidate
    return dict(default)


def _load_both():
    """Saved layouts, or defaults.

    Never raises. Corrupt/incomplete layout discarded whole, default used instead — merging
    partial file puts some axes on your keys, others on defaults, harder to spot than clean
    revert. The layouts fall back SEPARATELY: a file written before the XYZ jog (or the
    shortcuts) existed has none of those keys, and that must not cost the operator their
    joint layout.
    """
    global _active, _active_xyz, _active_shortcuts
    if None not in (_active, _active_xyz, _active_shortcuts):
        return
    try:
        with open(KEYBINDS_FILE, "r", encoding="utf-8") as fh:
            data = json.load(fh)
        if not isinstance(data, dict):
            data = {}
    except Exception:
        data = {}
    _active = _layout(data, ACTION_ORDER, DEFAULT_KEYMAP)
    _active_xyz = _layout(data, XYZ_ORDER, DEFAULT_XYZ_KEYMAP)
    _active_shortcuts = _layout(data, SHORTCUT_ORDER, DEFAULT_SHORTCUTS)


def load():
    _load_both()
    return dict(_active)


def save(keymap, xyz_keymap=None, shortcuts=None):
    """Persists the joint layout, and the XYZ one and the shortcuts with it when given (else
    the ones in use stay). One file holds all three. Returns False (and writes nothing) if
    any is invalid — including a shortcut on a key a jog layout holds."""
    global _active, _active_xyz, _active_shortcuts
    _load_both()
    xyz = _active_xyz if xyz_keymap is None else xyz_keymap
    keys = _active_shortcuts if shortcuts is None else shortcuts
    if validate(keymap) or validate(xyz, XYZ_ORDER) or validate_shortcuts(keys, keymap, xyz):
        return False
    _active, _active_xyz, _active_shortcuts = dict(keymap), dict(xyz), dict(keys)
    try:
        with open(KEYBINDS_FILE, "w", encoding="utf-8") as fh:
            json.dump({**_active, **_active_xyz, **_active_shortcuts}, fh, indent=2)
    except OSError:
        return False
    return True


def active_map():
    return load()


def active_xyz_map():
    _load_both()
    return dict(_active_xyz)


def active_shortcuts():
    _load_both()
    return dict(_active_shortcuts)
