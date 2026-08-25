"""Helpers for tracking and safely cancelling tkinter `after` jobs.

Original code cancelled jobs by hand in six different places, and in a
couple cancelled an already-fired id — harmless on CPython but raises
TclError on some Tk builds. Everything now goes through here.
"""

import tkinter as tk


class TimerMixin:
    def _schedule(self, attr, delay_ms, callback, *args):
        """Cancels any pending job stored in `attr`, then schedules a new one."""
        self._cancel_job(attr)
        job = self.root.after(delay_ms, callback, *args)
        setattr(self, attr, job)
        return job

    def _cancel_job(self, attr):
        """Cancels the job id stored in `attr` (if any) and clears it."""
        job = getattr(self, attr, None)
        if not job:
            return
        try:
            self.root.after_cancel(job)
        except (tk.TclError, ValueError):
            pass        # already fired, or interpreter shutting down
        setattr(self, attr, None)

    def _cancel_jobs(self, *attrs):
        for attr in attrs:
            self._cancel_job(attr)
