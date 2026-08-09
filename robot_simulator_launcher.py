"""Launcher for the Robot Motion Controller.

The implementation lives in the `robot_sim` package — see
robot_sim/__init__.py for the module map. This file only starts it.

Run with:  python robot_simulator_launcher.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_sim import RobotControlApp, main  # noqa: E402,F401  (re-exported)

if __name__ == "__main__":
    main()
