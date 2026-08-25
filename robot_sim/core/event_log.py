"""Timestamped event log with a bounded ring buffer."""

import datetime

from ..config import LOG_MAX_LINES


class EventLogMixin:
    def log(self, message: str, tag: str = "default"):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        widget = getattr(self, "log_text", None)
        if widget is None:          # log() called before UI exists
            print(f"[{timestamp}] {message}")
            return

        widget.configure(state="normal")
        widget.insert("end", f"[{timestamp}] {message}\n", tag)
        widget.see("end")

        # Trim oldest lines past cap. Old version deleted only 1 line/call,
        # so message burst could push buffer well past limit; drains excess.
        line_count = int(widget.index("end-1c").split(".")[0])
        if line_count > LOG_MAX_LINES:
            excess = line_count - LOG_MAX_LINES
            widget.delete("1.0", f"{excess + 1}.0")

        widget.configure(state="disabled")

    # back-compat alias for original private name
    _log = log
