"""The scanner window.

One screen, one job: sweep 340 degrees, rise by the step you typed, sweep
again. The only field that really needs a decision is the Z step; the rest
have working defaults and are here because the board needs a number for
them, not because the operator is expected to care every time.
"""

import os
import re
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from . import config as C
from .link import SerialLink, SimulatedBoard, available_ports, HAS_SERIAL
from .plot import PolarPlot
from .store import ScanStore

POINT_RE = re.compile(r"\[SCAN_PT\]\s*(-?\d+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)")
LAYER_RE = re.compile(r"\[SCAN_LAYER\]\s*(\d+)\s*/\s*(\d+)\s+z=(-?[\d.]+)")

POLL_MS = 30
REDRAW_MS = 120


class ScannerApp:
    def __init__(self, root):
        self.root = root
        root.title(C.APP_TITLE)
        root.configure(bg=C.BG)
        root.minsize(*C.WINDOW_MIN)

        self.store = ScanStore()
        self.link = SerialLink(self._on_line, self._log_error)
        self.sim = SimulatedBoard(self._on_line)
        self.scanning = False
        self.layer = 0
        self.layers = 0
        self._dirty = False          # points arrived since the last redraw

        self._build()
        self._refresh_ports()
        self._tick()
        self._redraw_tick()
        self.log("Not connected — running simulated. Press START SCAN to see it work.")

    # ==================================================================
    # layout
    # ==================================================================
    def _card(self, parent, **kw):
        return tk.Frame(parent, bg=C.PANEL, padx=12, pady=10, **kw)

    def _build(self):
        self._build_connection()
        body = tk.Frame(self.root, bg=C.BG)
        body.pack(fill="both", expand=True, padx=12, pady=(0, 8))
        self._build_controls(body)
        self._build_plot(body)
        self._build_log()

    def _build_connection(self):
        row = self._card(self.root)
        row.pack(fill="x", padx=12, pady=12)

        tk.Label(row, text="PORT", bg=C.PANEL, fg=C.MUTED,
                 font=C.FONT_CAPTION).pack(side="left", padx=(0, 6))
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(row, textvariable=self.port_var, width=12,
                                     state="readonly", values=[])
        self.port_box.pack(side="left")
        tk.Button(row, text="⟳", command=self._refresh_ports, bg=C.FIELD,
                  fg=C.INK, relief="flat", width=3).pack(side="left", padx=4)

        tk.Label(row, text="BAUD", bg=C.PANEL, fg=C.MUTED,
                 font=C.FONT_CAPTION).pack(side="left", padx=(10, 6))
        self.baud_var = tk.StringVar(value=C.DEFAULT_BAUD)
        ttk.Combobox(row, textvariable=self.baud_var, width=8, state="readonly",
                     values=C.BAUD_CHOICES).pack(side="left")

        self.connect_btn = tk.Button(row, text="CONNECT", command=self._toggle_connect,
                                     bg=C.ACCENT, fg="#0b0d10", relief="flat",
                                     font=C.FONT_BOLD, padx=16, pady=4)
        self.connect_btn.pack(side="left", padx=14)

        self.link_lamp = tk.Label(row, text="● SIMULATED", bg=C.PANEL, fg=C.WARN,
                                  font=C.FONT_BOLD)
        self.link_lamp.pack(side="left")

        tk.Button(row, text="EMERGENCY STOP", command=self._estop, bg=C.BAD,
                  fg="#0b0d10", relief="flat", font=C.FONT_BOLD,
                  padx=14, pady=4).pack(side="right")

    def _field(self, parent, label, var, hint=""):
        wrap = tk.Frame(parent, bg=C.PANEL)
        wrap.pack(fill="x", pady=(0, 8))
        tk.Label(wrap, text=label, bg=C.PANEL, fg=C.MUTED,
                 font=C.FONT_CAPTION).pack(anchor="w")
        tk.Entry(wrap, textvariable=var, bg=C.FIELD, fg=C.INK, relief="flat",
                 insertbackground=C.INK, font=C.FONT_MONO,
                 width=14).pack(anchor="w", ipady=3)
        if hint:
            tk.Label(wrap, text=hint, bg=C.PANEL, fg=C.MUTED,
                     font=C.FONT_CAPTION).pack(anchor="w")
        return wrap

    def _build_controls(self, body):
        col = self._card(body)
        col.pack(side="left", fill="y", padx=(0, 12))

        tk.Label(col, text="STEP UP EACH SCAN", bg=C.PANEL, fg=C.INK,
                 font=C.FONT_BOLD).pack(anchor="w")
        tk.Label(col, text="how far the lift rises between sweeps",
                 bg=C.PANEL, fg=C.MUTED, font=C.FONT_CAPTION).pack(anchor="w",
                                                                   pady=(0, 4))
        self.z_step_var = tk.StringVar(value=f"{C.DEFAULT_Z_STEP_MM:g}")
        big = tk.Frame(col, bg=C.PANEL)
        big.pack(anchor="w", pady=(0, 2))
        tk.Entry(big, textvariable=self.z_step_var, bg=C.FIELD, fg=C.ACCENT,
                 relief="flat", insertbackground=C.INK, font=C.FONT_BIG,
                 width=6, justify="right").pack(side="left", ipady=6)
        tk.Label(big, text="mm", bg=C.PANEL, fg=C.MUTED,
                 font=C.FONT_BOLD).pack(side="left", padx=(8, 0))

        self.height_hint = tk.Label(col, text="", bg=C.PANEL, fg=C.MUTED,
                                    font=C.FONT_CAPTION, justify="left")
        self.height_hint.pack(anchor="w", pady=(0, 12))
        self.z_step_var.trace_add("write", lambda *_a: self._refresh_hint())

        self.layers_var = tk.StringVar(value=str(C.DEFAULT_LAYERS))
        self.layers_var.trace_add("write", lambda *_a: self._refresh_hint())
        self._field(col, "LAYERS", self.layers_var)

        self.deg_step_var = tk.StringVar(value=f"{C.DEFAULT_DEG_STEP:g}")
        self._field(col, "ANGULAR STEP (°)", self.deg_step_var,
                    "1° = 341 points per sweep")

        tk.Label(col, text="SENSOR", bg=C.PANEL, fg=C.MUTED,
                 font=C.FONT_CAPTION).pack(anchor="w")
        self.sensor_var = tk.StringVar(value=C.DEFAULT_SENSOR)
        ttk.Combobox(col, textvariable=self.sensor_var, width=12, state="readonly",
                     values=list(C.SENSOR_KINDS)).pack(anchor="w", pady=(0, 12))

        self.start_btn = tk.Button(col, text="START SCAN", command=self._start,
                                   bg=C.OK, fg="#0b0d10", relief="flat",
                                   font=C.FONT_BOLD, pady=8)
        self.start_btn.pack(fill="x")
        self.stop_btn = tk.Button(col, text="STOP", command=self._stop,
                                  bg=C.FIELD, fg=C.INK, relief="flat",
                                  font=C.FONT_BOLD, pady=6, state="disabled")
        self.stop_btn.pack(fill="x", pady=(6, 12))

        self.progress_var = tk.StringVar(value="Idle")
        tk.Label(col, textvariable=self.progress_var, bg=C.PANEL, fg=C.INK,
                 font=C.FONT_MONO, justify="left").pack(anchor="w")
        self.counts_var = tk.StringVar(value="0 points")
        tk.Label(col, textvariable=self.counts_var, bg=C.PANEL, fg=C.MUTED,
                 font=C.FONT_CAPTION, justify="left").pack(anchor="w")

        self._refresh_hint()

    def _build_plot(self, body):
        right = self._card(body)
        right.pack(side="left", fill="both", expand=True)
        self.plot = PolarPlot(right, size=420)
        self.plot.pack(pady=(0, 8))
        self.scale_var = tk.StringVar(value="")
        tk.Label(right, textvariable=self.scale_var, bg=C.PANEL, fg=C.MUTED,
                 font=C.FONT_CAPTION).pack()
        tk.Button(right, text="SAVE CSV…", command=self._save_csv, bg=C.FIELD,
                  fg=C.INK, relief="flat", font=C.FONT_BOLD,
                  pady=6).pack(fill="x", pady=(10, 0))

    def _build_log(self):
        wrap = self._card(self.root)
        wrap.pack(fill="both", padx=12, pady=(0, 12))
        self.log_box = tk.Text(wrap, height=7, bg=C.FIELD, fg=C.INK,
                               relief="flat", font=C.FONT_MONO, wrap="word")
        self.log_box.pack(fill="both", expand=True)
        self.log_box.tag_config("err", foreground=C.BAD)
        self.log_box.tag_config("warn", foreground=C.WARN)
        self.log_box.tag_config("ok", foreground=C.OK)
        self.log_box.configure(state="disabled")

    # ==================================================================
    # helpers
    # ==================================================================
    def log(self, text, tag=None):
        self.log_box.configure(state="normal")
        self.log_box.insert("end", text + "\n", tag or ())
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def _log_error(self, text):
        self.log(text, "err")

    def _refresh_ports(self):
        ports = available_ports()
        self.port_box["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        if not HAS_SERIAL:
            self.log("pyserial is not installed — the port list will stay empty.",
                     "warn")

    def _refresh_hint(self):
        """Live: what height the last layer would end at, and whether that
        fits. Answered before START rather than as an [ERROR] afterwards."""
        try:
            step = float(self.z_step_var.get())
            layers = int(float(self.layers_var.get()))
        except (TypeError, ValueError):
            self.height_hint.config(text="", fg=C.MUTED)
            return
        # The lift only moves BETWEEN layers, so the top is reached after
        # layers - 1 steps, not layers.
        top = step * max(layers - 1, 0)
        text = f"{layers} layers → tops out {top:.1f} mm above the start"
        if top > C.Z_STROKE_MM:
            self.height_hint.config(
                text=text + f"  ✗ past the {C.Z_STROKE_MM:g} mm stroke", fg=C.BAD)
        else:
            self.height_hint.config(text=text, fg=C.MUTED)

    def _validate(self):
        """Returns (z_step, deg_step, layers) or None, having said why."""
        try:
            z_step = float(self.z_step_var.get())
            deg_step = float(self.deg_step_var.get())
            layers = int(float(self.layers_var.get()))
        except (TypeError, ValueError):
            messagebox.showerror(C.APP_TITLE, "Every field has to be a number.")
            return None
        if z_step < C.Z_STEP_MIN_MM:
            messagebox.showerror(C.APP_TITLE,
                                 f"The step up must be at least {C.Z_STEP_MIN_MM} mm.")
            return None
        if not (C.DEG_STEP_MIN <= deg_step <= C.DEG_STEP_MAX):
            messagebox.showerror(
                C.APP_TITLE,
                f"The angular step must be between {C.DEG_STEP_MIN} and "
                f"{C.DEG_STEP_MAX:g}°.")
            return None
        if not (C.LAYERS_MIN <= layers <= C.LAYERS_MAX):
            messagebox.showerror(C.APP_TITLE,
                                 f"Layers must be between {C.LAYERS_MIN} and "
                                 f"{C.LAYERS_MAX}.")
            return None
        if z_step * (layers - 1) > C.Z_STROKE_MM:
            messagebox.showerror(
                C.APP_TITLE,
                f"{layers} layers {z_step:g} mm apart need "
                f"{z_step * (layers - 1):.1f} mm of lift, past the "
                f"{C.Z_STROKE_MM:g} mm stroke.")
            return None
        return z_step, deg_step, layers

    def _send(self, text):
        if self.link.is_open:
            self.link.send(text)
        else:
            self.sim.send(text)

    # ==================================================================
    # actions
    # ==================================================================
    def _toggle_connect(self):
        if self.link.is_open:
            self.link.send(C.CMD_BYE)
            self.link.close()
            self.connect_btn.config(text="CONNECT")
            self.link_lamp.config(text="● SIMULATED", fg=C.WARN)
            self.log("Disconnected — back to the simulation.")
            return
        port = self.port_var.get()
        if not port:
            messagebox.showerror(C.APP_TITLE, "Pick a COM port first.")
            return
        if self.link.open(port, self.baud_var.get()):
            self.connect_btn.config(text="DISCONNECT")
            self.link_lamp.config(text="● CONNECTED", fg=C.OK)
            self.log(f"Connected on {port}.", "ok")
            self.link.send(C.CMD_PING)

    def _start(self):
        if self.scanning:
            return
        params = self._validate()
        if params is None:
            return
        z_step, deg_step, layers = params
        self.store.clear()
        self.plot.set_range(C.PLOT_MIN_RANGE_MM)
        self.layer, self.layers = 0, layers
        self._send(C.cmd_sensor(self.sensor_var.get()))
        self._send(C.cmd_scan_start(z_step, deg_step, layers))
        self.scanning = True
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        self.log(f"Scan started — {layers} layers, {z_step:g} mm apart, "
                 f"{deg_step:g}° steps.", "ok")

    def _stop(self):
        self._send(C.CMD_SCAN_STOP)
        self.log("Stop sent.", "warn")

    def _estop(self):
        # Straight out, no confirmation. A dialog between the operator and
        # a stop button is the thing that makes the button useless.
        self._send(C.CMD_ESTOP)
        self._scan_ended("EMERGENCY STOP")

    def _scan_ended(self, why):
        self.scanning = False
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.progress_var.set(why)

    def _save_csv(self):
        if not self.store.points:
            messagebox.showinfo(C.APP_TITLE, "There are no points to save yet.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", initialfile="scan.csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        try:
            n = self.store.to_csv(path)
        except OSError as exc:
            messagebox.showerror(C.APP_TITLE, f"Could not write the file: {exc}")
            return
        self.log(f"Saved {n} points to {os.path.basename(path)}.", "ok")

    # ==================================================================
    # incoming lines
    # ==================================================================
    def _on_line(self, line):
        m = POINT_RE.search(line)
        if m:
            layer, deg, mm = int(m.group(1)), float(m.group(2)), float(m.group(3))
            self.store.add(layer, deg, mm)
            self._dirty = True
            return                       # never logged: hundreds per layer

        m = LAYER_RE.search(line)
        if m:
            self.layer, self.layers = int(m.group(1)), int(m.group(2))
            self.store.set_layer_z(self.layer, float(m.group(3)))
            self.log(line)
            return

        if line.startswith(C.TAG_SEEK):
            # Nothing is being measured yet, and the turntable IS moving.
            # Saying so stops it looking like a scan that started and then
            # produced no points.
            self.progress_var.set("Finding the RM switch…")
            self.log(line, "warn")
        elif line.startswith(C.TAG_REF):
            self.log(line, "ok")
        elif line.startswith(C.TAG_DONE):
            self._scan_ended("Finished")
            self.log(line, "ok")
        elif line.startswith(C.TAG_ABORT):
            self._scan_ended("Aborted")
            self.log(line, "warn")
        elif line.startswith(C.TAG_ERROR):
            # The board refused it, so the GUI's own idea that a scan is
            # running has to go with it -- otherwise START stays greyed out
            # with nothing running.
            if self.scanning:
                self._scan_ended("Refused")
            self.log(line, "err")
        elif line.startswith(C.TAG_WARN):
            self.log(line, "warn")
        else:
            self.log(line)

    # ==================================================================
    # timers
    # ==================================================================
    def _tick(self):
        self.link.poll()
        self.sim.poll()
        self.root.after(POLL_MS, self._tick)

    def _redraw_tick(self):
        """Repaints on a timer, not per point.

        At 1° steps a layer is 341 points; redrawing the canvas for each of
        them would spend the whole scan drawing and none of it reading.
        """
        if self._dirty:
            self._dirty = False
            self._repaint()
        self.root.after(REDRAW_MS, self._redraw_tick)

    def _repaint(self):
        # Freeze the radial scale on the first layer. Rescaling per layer
        # would make two heights look the same when they are not.
        if self.store.range_mm is None and self.store.max_radius() > 0:
            biggest = self.store.max_radius()
            self.store.range_mm = max(C.PLOT_MIN_RANGE_MM,
                                      round(biggest * 1.15 / 50.0) * 50.0)
            self.plot.set_range(self.store.range_mm)

        pts = self.store.layer_points(self.layer)
        ghost = self.store.layer_points(self.layer - 1) if self.layer > 1 else ()
        z = self.store.layer_z.get(self.layer)
        title = (f"layer {self.layer}/{self.layers}"
                 + (f"   z = {z:.1f} mm" if z is not None else ""))
        self.plot.redraw(pts, ghost, title)

        self.progress_var.set(f"Layer {self.layer}/{self.layers}"
                              if self.scanning else self.progress_var.get())
        self.counts_var.set(f"{self.store.total} points · "
                            f"{self.store.misses} missed")
        self.scale_var.set(f"full scale {self.plot.range_mm:.0f} mm · "
                           f"0° is straight ahead at RM 0")


def main():
    root = tk.Tk()
    ScannerApp(root)
    root.mainloop()
