"""Robot Motion Controller — modular STCR4000S simulator / ClearCore GUI.

Layout
------
    config.py           tuning constants, geometry, protocol vocabulary
    theme.py            colour palette + shade()
    kinematics.py       IK / FK maths (pure, testable)
    serial_backend.py   optional pyserial import + thin wrapper
    widgets/            canvas-drawn controls and UI factories
    core/               behaviour mixins (serial, heartbeat, jog, p2p, safety…)
    ui/                 window layout, panels and dialogs
    app.py              RobotControlApp = all of the above + main()
"""

__version__ = "6.0.0"
__all__ = ["RobotControlApp", "main", "__version__"]


def __getattr__(name):
    """Lazy GUI import: lets robot_sim.kinematics/config import (and be unit-tested) w/o tkinter."""
    if name in ("RobotControlApp", "main"):
        from . import app
        return getattr(app, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
