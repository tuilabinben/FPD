"""Timestamped event log with a bounded ring buffer."""

import datetime

from ..config import LOG_MAX_LINES


class EventLogMixin:
    def log(self, message: str, tag: str = "default"):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        widget = getattr(self, "log_text", None)
        if widget is None:          # log() called before the UI exists
            print(f"[{timestamp}] {message}")
            return

        widget.configure(state="normal")
        widget.insert("end", f"[{timestamp}] {message}\n", tag)
        widget.see("end")

        # Trim the oldest lines once the buffer grows past the cap. The old
        # version deleted exactly one line per call, so a burst of messages
        # could push the buffer well past the limit; this drains the excess.
        line_count = int(widget.index("end-1c").split(".")[0])
        if line_count > LOG_MAX_LINES:
            excess = line_count - LOG_MAX_LINES
            widget.delete("1.0", f"{excess + 1}.0")

        widget.configure(state="disabled")

    # Backwards-compatible alias for the original private name.
    _log = log
