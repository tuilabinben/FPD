"""Where the app's persisted files live.

A leaf module — no imports from the rest of robot_sim, and none of its own
beyond the standard library. palettes.py in particular loads before
anything else (see its own docstring) and needs this to stay that way.

DEV RUN: next to this package, exactly as it always was — `machine_settings
.json`, `keybinds.json`, `limit_presets.json` and `appearance.json` sit
alongside the .py files, which is convenient to find and is what every
existing install already has on disk.

FROZEN (PyInstaller or similar): a onefile build extracts the whole
package into a FRESH temporary directory every launch (`sys._MEIPASS`), so
a file written next to `__file__` there is gone the moment the process
exits — settings, taught boundaries and keybinds would silently reset on
every restart. `%APPDATA%\\RobotMotionController` is used instead: it is
writable without admin rights regardless of where the exe itself was
installed (unlike a Program Files copy of the app folder), and it is the
SAME path across onefile and onedir builds, so packaging mode is not
something the rest of the app has to know about.
"""

import os
import sys

APP_DIR_NAME = "RobotMotionController"


def user_data_dir():
    """Directory the settings/keybinds/preset/appearance files live in.

    Created if missing — callers join a filename onto this and open()
    it directly, so the directory has to already exist.
    """
    if getattr(sys, "frozen", False):
        base = os.environ.get("APPDATA") or os.path.expanduser("~")
        d = os.path.join(base, APP_DIR_NAME)
        os.makedirs(d, exist_ok=True)
        return d
    return os.path.dirname(os.path.abspath(__file__))
