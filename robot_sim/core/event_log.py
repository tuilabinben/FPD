"""Timestamped event log with a bounded ring buffer.

Performance notes (v9.3)
------------------------
The original implementation called ``widget.index("end-1c")`` on every log
call to count lines.  That forces Tkinter to walk the entire B-tree of the
Text widget — O(n) in the number of lines — and was the single largest
contributor to UI lag once the log exceeded a few thousand entries.

This version keeps its own ``_log_line_count`` integer and never asks the
widget how many lines it holds.  Trimming is also batched: when lines must
be deleted, one ``widget.delete()`` call removes the entire excess instead
of one line at a time.

``widget.see("end")`` is another expensive call — it triggers a full layout
recalculation.  It is now skipped when the user has scrolled upward to read
earlier entries (auto-scroll only when already at the bottom).
"""

import datetime

from ..config import LOG_MAX_LINES


class EventLogMixin:
    # Initialised to 0; the first insert makes it 1.
    _log_line_count: int = 0

    def log(self, message: str, tag: str = "default"):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        widget = getattr(self, "log_text", None)
        if widget is None:          # log() called before the UI exists
            print(f"[{timestamp}] {message}")
            return

        widget.configure(state="normal")
        widget.insert("end", f"[{timestamp}] {message}\n", tag)
        self._log_line_count += 1

        # ── auto-scroll only when the view is already at the bottom ──
        # yview() returns (first_visible, last_visible) as fractions of the
        # total content.  A value of 1.0 at [1] means the bottom is visible.
        # A small tolerance (0.02 ≈ ~1 line on a 50-line view) avoids the
        # edge case where Tkinter's own rounding puts last_visible at 0.999.
        if widget.yview()[1] >= 0.98:
            widget.see("end")

        # ── trim oldest lines when over the cap ──
        # One bulk delete instead of one per call.  The internal counter is
        # adjusted by the same amount so it stays in sync without re-querying
        # the widget.
        if self._log_line_count > LOG_MAX_LINES:
            excess = self._log_line_count - LOG_MAX_LINES
            widget.delete("1.0", f"{excess + 1}.0")
            self._log_line_count -= excess

        widget.configure(state="disabled")

    def _reset_log_line_count(self):
        """Re-sync the internal counter from the widget.

        Called after a theme rebuild restores the log content in bulk, where
        the counter and the widget would otherwise disagree.
        """
        widget = getattr(self, "log_text", None)
        if widget is not None:
            self._log_line_count = int(widget.index("end-1c").split(".")[0])

    # Backwards-compatible alias for the original private name.
    _log = log
