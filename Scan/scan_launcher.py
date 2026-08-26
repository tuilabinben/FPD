"""Launcher for the 340 degree scanner.

Run with:  python scan_launcher.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from scanner import main  # noqa: E402

if __name__ == "__main__":
    main()
