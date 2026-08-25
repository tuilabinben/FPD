"""Where app's persisted files live.

Leaf module — no imports from rest of robot_sim, none beyond stdlib. palettes.py loads before
anything else (see its docstring), needs this to stay that way.

DEV RUN: next to this package, as always — machine_settings.json, keybinds.json,
limit_presets.json, appearance.json sit alongside .py files, convenient to find, matches every
existing install on disk.

FROZEN (PyInstaller etc): onefile build extracts whole package into FRESH temp dir every launch
(sys._MEIPASS) — file written next to __file__ there gone the moment process exits, settings/
taught boundaries/keybinds would silently reset every restart. %APPDATA%\\RobotMotionController
used instead: writable w/o admin rights regardless of exe install location (unlike Program
Files copy), SAME path across onefile/onedir builds — packaging mode not something rest of app
has to know.
"""

import os
import sys

APP_DIR_NAME = "RobotMotionController"


def user_data_dir():
    """Directory settings/keybinds/preset/appearance files live in.

    Created if missing — callers join filename onto this and open() directly, directory must
    already exist.
    """
    if getattr(sys, "frozen", False):
        base = os.environ.get("APPDATA") or os.path.expanduser("~")
        d = os.path.join(base, APP_DIR_NAME)
        os.makedirs(d, exist_ok=True)
        return d
    return os.path.dirname(os.path.abspath(__file__))
