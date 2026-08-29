"""SCAN mode — start a sweep, read what comes back.

Third mode beside P2P and JOG, same firmware and same link: the board
sweeps through its own jog primitives, so soft limits, PLC switches and
E-STOP apply exactly as they do to a held key.
"""

import os
from collections import deque
from tkinter import filedialog, messagebox

from ..config import (
    PLC_SENSOR_ENFORCE_BY_AXIS,
    SCAN_MISS_WARN_FRACTION,
    SCAN_PLOT_MIN_RANGE_MM,
    SCAN_RECENT_WINDOW,
    SCAN_REDRAW_MS,
    SCAN_TAG_ABORT,
    SCAN_TAG_DONE,
    SCAN_TAG_SEEK,
    cmd_scan_sensor,
    cmd_scan_start,
)
from .. import scan_plan
from .scan_store import ScanStore

import re

POINT_RE = re.compile(r"\[SCAN_PT\]\s*(\d+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)")
LAYER_RE = re.compile(r"\[SCAN_LAYER\]\s*(\d+)\s*/\s*(\d+)\s+z=\s*(-?[\d.]+)")
READ_RE = re.compile(r"\[SCAN_READ\]\s*(-?[\d.]+)")


class ScanControlMixin:
    # ── state ────────────────────────────────────────────────────────
    def _init_scan_state(self):
        self.scan_store = ScanStore()
        self.scan_running = False
        self.scan_layer = 0
        self.scan_layers = 0
        self.scan_plan = None
        # THIS scan only: a carried-over hit rate reports an unplugged
        # sensor as healthy for fifty points.
        self.scan_recent = deque(maxlen=SCAN_RECENT_WINDOW)
        self.scan_last_mm = None
        self._scan_dirty = False
        self._scan_redraw_job = None

    # ── the derivation, live ─────────────────────────────────────────
    def scan_current_plan(self):
        """(plan, None) or (None, message). Never raises: hints follow every
        keystroke, so half-typed input is an answer, not a traceback."""
        try:
            plan = scan_plan.plan(
                self.scan_hz_v.get(), self.scan_points_v.get(),
                self.scan_slices_v.get(), self.scan_gap_v.get(),
                self.scan_sweep_v.get(),
                settings=self.settings, start_z_mm=self.sim_z)
        except scan_plan.ScanPlanError as exc:
            return None, str(exc)
        return plan, None

    # ── starting ─────────────────────────────────────────────────────
    def start_scan(self):
        if self.scan_running:
            return
        if self.motion_locked or self.is_running or self.is_homing:
            self.log("SCAN ignored — something is already moving. Stop first.",
                     tag="warn")
            return
        if self.jog_active:
            self.log("SCAN ignored — a jog axis is held. Release it first.",
                     tag="warn")
            return

        plan, error = self.scan_current_plan()
        if error:
            messagebox.showerror("Scan", error)
            return
        if not self._scan_plc_ready():
            return
        # Ceiling and speed clamp WARN, never refuse — operator's own
        # limits, their call. But they see the number before it runs.
        if plan["warnings"]:
            if not messagebox.askokcancel(
                    "Scan", "\n\n".join(plan["warnings"])
                    + "\n\nStart the scan anyway?"):
                return
            for line in plan["warnings"]:
                self.log("SCAN — " + line, tag="warn")

        self.scan_store.clear()
        self.scan_plan = plan
        self.scan_layer, self.scan_layers = 0, plan["slices"]
        self.scan_recent.clear()
        self.scan_last_mm = None
        self.scan_plot.set_range(SCAN_PLOT_MIN_RANGE_MM)

        self.send(cmd_scan_sensor(self.scan_sensor_v.get()))
        self.send(cmd_scan_start(plan["gap_mm"], plan["deg_step"],
                                 plan["slices"], plan["sweep_deg"],
                                 plan["rot_deg_s"]))

        self.scan_running = True
        self._set_motion_locked(True)
        self._refresh_scan_buttons()
        self.status_var.set(f"SCANNING — {plan['slices']} slices")
        self.scan_progress_v.set(f"Layer 0/{plan['slices']} — starting")
        self.log("SCAN started — " + scan_plan.summary(plan))
        if not self._hardware_live():
            self.log("No board is connected, so nothing will actually sweep — "
                     "the panel is showing what WOULD be sent. There is no "
                     "simulated sensor here; use the stand-alone Scan tool for "
                     "that.", tag="warn")
        self._scan_redraw_tick()

    def _scan_plc_ready(self):
        """Refuses only where the board is CERTAIN to refuse anyway, so the
        fix is named at the panel. Never refuses on UNKNOWN: that means
        this app has no news, not that the board has none."""
        if not self._hardware_live():
            return True                       # nothing to be ready
        if not self.settings.get(PLC_SENSOR_ENFORCE_BY_AXIS["ROT"], True):
            messagebox.showerror(
                "Scan",
                "RM's switch is switched off in Settings → Boundaries.\n\n"
                "It is the reference every slice is measured from, so the "
                "board will refuse the scan. Switch it back on first.")
            return False
        if self._plc_led_state in ("unreachable", "disabled"):
            messagebox.showerror(
                "Scan",
                "There is no PLC device data, so RM's switch cannot be seen "
                "and the scan has no frame to sweep in.\n\n"
                "Check the cable and the PLC's Ethernet module — PLC_TEST on "
                "the board reports which layer is failing.")
            return False
        if self._plc_led_state == "no_reply":
            # Board keeps its last status word and may well accept it —
            # but that reading could be minutes old. Ask.
            return messagebox.askokcancel(
                "Scan",
                "The PLC is not answering device reads, so RM's switch "
                "reading may be stale.\n\nStart the scan anyway?")
        return True

    # ── stopping ─────────────────────────────────────────────────────
    def stop_scan(self):
        if not self.scan_running:
            return
        self.send("SCAN_STOP")
        self._scan_ended("Stopped by the operator")

    def cancel_scan_locally(self, why):
        """End the GUI's idea of a scan, send nothing. For callers that
        already stopped the machine — board cancels the scan on ESTOP and
        STOP, so a second SCAN_STOP would race and log twice."""
        if not self.scan_running:
            return
        self._scan_ended(why)

    def _scan_ended(self, why):
        self.scan_running = False
        self._cancel_job("_scan_redraw_job")
        self._set_motion_locked(False)
        self._refresh_scan_buttons()
        self.scan_progress_v.set(why)
        self._scan_repaint()

    # ── one reading, no motion ───────────────────────────────────────
    def scan_test_read(self):
        """Does the sensor work, BEFORE committing to a sweep."""
        if not self._hardware_live():
            messagebox.showinfo(
                "Scan", "No board is connected, so there is nothing to read "
                        "from.")
            return
        self.send("SCAN_READ")

    def save_scan_csv(self):
        if not self.scan_store.points:
            messagebox.showinfo("Scan", "There are no points to save yet.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", initialfile="scan.csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        try:
            n = self.scan_store.to_csv(path)
        except OSError as exc:
            messagebox.showerror("Scan", f"Could not write the file: {exc}")
            return
        self.log(f"Saved {n} scan points to {os.path.basename(path)}.")

    # ── incoming lines ───────────────────────────────────────────────
    def _on_scan_line(self, text):
        """Every [SCAN_*] reply. NOTHING here logs — the RX pump already
        did, and [SCAN_PT] is telemetry (hundreds a layer). Store takes
        the points, a timer draws them."""
        m = POINT_RE.search(text)
        if m:
            layer, deg, mm = int(m.group(1)), float(m.group(2)), float(m.group(3))
            self.scan_store.add(layer, deg, mm)
            self._note_scan_reading(mm)
            self._scan_dirty = True
            return

        m = READ_RE.search(text)
        if m:
            # TEST READ: no redraw tick runs outside a scan.
            self._note_scan_reading(float(m.group(1)))
            self._refresh_scan_sensor_lamp()
            return

        m = LAYER_RE.search(text)
        if m:
            self.scan_layer, self.scan_layers = int(m.group(1)), int(m.group(2))
            self.scan_store.set_layer_z(self.scan_layer, float(m.group(3)))
            self.scan_progress_v.set(f"Layer {self.scan_layer}/{self.scan_layers}")
            return

        if text.startswith(SCAN_TAG_SEEK):
            # RM moving, nothing measured yet. Say so, or it reads as a
            # scan that started and produced nothing.
            self.scan_progress_v.set("Finding the RM switch…")
        elif text.startswith(SCAN_TAG_DONE):
            self._scan_ended("Finished")
            self.status_var.set("READY — scan finished.")
        elif text.startswith(SCAN_TAG_ABORT):
            self._scan_ended("Aborted")
            self.status_var.set("STOPPED — scan aborted.")

    def scan_refused_by_board(self):
        """[ERROR] while this app thought a scan ran. Drop the run too, or
        START stays greyed out with nothing running."""
        if self.scan_running:
            self._scan_ended("Refused by the board")

    def _note_scan_reading(self, mm):
        self.scan_last_mm = mm
        self.scan_recent.append(mm >= 0)

    # ── repaint, on a timer ──────────────────────────────────────────
    def _scan_redraw_tick(self):
        """Timer, not per point: a 1° step is 341 points a layer."""
        self._scan_redraw_job = None
        if self._scan_dirty:
            self._scan_dirty = False
            self._scan_repaint()
        if self.scan_running:
            self._schedule("_scan_redraw_job", SCAN_REDRAW_MS,
                           self._scan_redraw_tick)

    def _scan_repaint(self):
        store = self.scan_store
        # Scale frozen on layer 1: rescaling makes two heights look alike.
        if store.range_mm is None and store.max_radius() > 0:
            store.range_mm = max(SCAN_PLOT_MIN_RANGE_MM,
                                 round(store.max_radius() * 1.15 / 50.0) * 50.0)
            self.scan_plot.set_range(store.range_mm)

        pts = store.layer_points(self.scan_layer)
        ghost = store.layer_points(self.scan_layer - 1) if self.scan_layer > 1 else ()
        z = store.layer_z.get(self.scan_layer)
        title = (f"layer {self.scan_layer}/{self.scan_layers}"
                 + (f"   z = {z:.1f} mm" if z is not None else ""))
        self.scan_plot.redraw(pts, ghost, title)
        self.scan_counts_v.set(f"{store.total} points · {store.misses} missed")
        self.scan_scale_v.set(f"full scale {self.scan_plot.range_mm:.0f} mm · "
                              f"0° is straight ahead at RM 0")
        self._refresh_scan_sensor_lamp()

    def scan_miss_fraction(self):
        """RECENT readings only, so the lamp recovers with the sensor. One
        lost echo is not a fault; a wall of them is."""
        if not self.scan_recent:
            return 0.0
        return sum(1 for ok in self.scan_recent if not ok) / len(self.scan_recent)

    def scan_sensor_health(self):
        """(state, headline, detail). No Tk, so tests read it windowless."""
        if self.scan_last_mm is None:
            return ("none", "no reading yet",
                    "press TEST READ, or start a scan")
        miss = self.scan_miss_fraction()
        if self.scan_last_mm < 0:
            headline = "NO ECHO"
            state = "bad"
        else:
            headline = f"{self.scan_last_mm:.1f} mm"
            state = "ok"
        n = len(self.scan_recent)
        detail = f"{int(round(miss * n))} of the last {n} readings missed"
        if miss >= SCAN_MISS_WARN_FRACTION:
            state = "bad"
            detail += " — check the wiring, the sensor kind, and the calibration"
        return (state, headline, detail)
