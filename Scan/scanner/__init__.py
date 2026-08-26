"""340 degree scanner — a small companion to the main robot console.

    config.py   constants and the wire strings shared with the firmware
    link.py     serial link, plus a simulator that speaks the same lines
    store.py    the points a scan produced, and the CSV writer
    plot.py     the polar view of one layer
    app.py      the window

The board side is NOT a separate sketch: the scan commands were added to
RobotMotionController_v9_ClearCore, so a sweep is driven through the same
jog primitives a key press uses and inherits every soft limit, every PLC
travel switch and the E-STOP path unchanged.
"""

from .app import ScannerApp, main

__all__ = ["ScannerApp", "main"]
