"""The scanner window.

One screen, one job: sweep 340 degrees, rise by the step you typed, sweep
again. The only field that really needs a decision is the Z step; the rest
have working defaults and are here because the board needs a number for
them, not because the operator is expected to care every time.
"""

import os
import re
import tkinter as tk
from collections import deque
from tkinter import filedialog, messagebox, ttk

from . import config as C
from .link import SerialLink, SimulatedBoard, available_ports, HAS_SERIAL
from .plot import PolarPlot
from .store import ScanStore

POINT_RE = re.compile(r"\[SCAN_PT\]\s*(-?\d+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)")
LAYER_RE = re.compile(r"\[SCAN_LAYER\]\s*(\d+)\s*/\s*(\d+)\s+z=(-?[\d.]+)")
READ_RE = re.compile(r"\[SCAN_READ\]\s*(-?[\d.]+)\s*mm")

# [PLC_STATE] link=UP socket=OPEN data=OK conn=2/3 word=0080 timeouts=1
#             | limit Z/R/A2=001 end Z/R/A2=-+- enforce Z/R/A2=111
#
# The three bit fields are ordered by AXIS -- Z, then R, then A2 -- and the
# devices behind them are NOT in tidy numeric order (M32 is ZM's, M30 is
# A2M's). This app only wants the MIDDLE character of each, which is RM's,
# so it never has to care which M number that is.
PLC_LINK_RE = re.compile(r"link=(\w+)\s+socket=(\w+)", re.IGNORECASE)
PLC_DATA_RE = re.compile(r"data=(NONE|STALE|OK)", re.IGNORECASE)
PLC_CONN_RE = re.compile(r"conn=(\d+)/(\d+)", re.IGNORECASE)
PLC_LIMIT_RE = re.compile(r"limit\s+Z/R/A2\s*=\s*([01?]{3})", re.IGNORECASE)
PLC_ENFORCE_RE = re.compile(r"enforce\s+Z/R/A2\s*=\s*([01?]{3})", re.IGNORECASE)

POLL_MS = 30
REDRAW_MS = 120

# How many recent readings the sensor indicator judges on. Short enough to
# react while the operator is still standing at the machine, long enough
# that the odd lost echo does not turn the lamp red -- a sensor that misses
# one reading in fifty is working, and a lamp that says otherwise gets
# ignored, which is the failure that matters.
SENSOR_WINDOW = 50


class ScannerApp:
    def __init__(self, root):
        self.root = root
        root.title(C.APP_TITLE)
        root.configure(bg=C.BG)
        root.minsize(*C.WINDOW_MIN)

        self.store = ScanStore()
        self.link = SerialLink(self._on_line, self._log_error)
        self.sim = SimulatedBoard(self._on_line)
        # Defaults ON so the tool demonstrates out of the box. Switch it off
        # and nothing but a real board can put a point on the screen.
        self.sim_enabled = True
        self.scanning = False
        self.layer = 0
        self.layers = 0
        self._dirty = False          # points arrived since the last redraw

        # The sensor indicator's state. Written from _on_line (cheap, and
        # it runs hundreds of times a second) and RENDERED in _repaint, for
        # the same reason the plot is: touching a StringVar per point makes
        # Tk redraw a label 250 times a second to no purpose.
        self.last_mm = None          # None = nothing has been read yet
        self.recent = deque(maxlen=SENSOR_WINDOW)   # True per hit, False per miss

        # The PLC link and RM's switch, as last reported by the board. Both
        # start UNKNOWN and must never fall back to a reassuring value: the
        # scan is refused without either, so a lamp that guesses "clear" or
        # "connected" is guessing about the one thing the operator opened
        # the panel to check.
        self.plc_state = "unknown"
        self.rm_state = "unknown"
        self.homing = False

        self._build()
        self._refresh_ports()
        self._tick()
        self._plc_tick()
        self._redraw_tick()
        self.log("Not connected — running simulated. Press START SCAN to see it work.")

    # ==================================================================
    # layout
    # ==================================================================
    def _card(self, parent, **kw):
        return tk.Frame(parent, bg=C.PANEL, padx=12, pady=10, **kw)

    def _scrollable(self, parent):
        """Wraps the rest of the window in a scrolling viewport.

        Returns the frame everything should be built into. On a short screen
        the plot and the log used to be cut off with no way to reach them;
        the window's minsize could not be lowered without hiding controls
        instead.
        """
        outer = tk.Frame(parent, bg=C.BG)
        outer.pack(fill="both", expand=True)
        canvas = tk.Canvas(outer, bg=C.BG, highlightthickness=0, bd=0)
        bar = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=bar.set)
        bar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        inner = tk.Frame(canvas, bg=C.BG)
        window = canvas.create_window((0, 0), window=inner, anchor="nw")
        # Two bindings, and BOTH are needed. The first keeps the scrollable
        # region equal to what the content actually occupies. The second
        # stretches the content to the canvas width -- without it the frame
        # keeps its own requested width and the whole layout sits in a
        # narrow column against the left edge, which looks like a broken
        # window rather than a scrolling one.
        inner.bind("<Configure>",
                   lambda _e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>",
                    lambda e: canvas.itemconfigure(window, width=e.width))

        # The wheel is bound application-wide only WHILE the pointer is over
        # this canvas, and released on the way out. Bound permanently it
        # would also turn the value in a focused combobox, which is a wheel
        # over a control silently changing a scan parameter.
        def wheel(event):
            canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")
        canvas.bind("<Enter>", lambda _e: canvas.bind_all("<MouseWheel>", wheel))
        canvas.bind("<Leave>", lambda _e: canvas.unbind_all("<MouseWheel>"))
        return inner

    def _build(self):
        # THE TOP BAR IS PINNED, OUTSIDE THE SCROLLER, because EMERGENCY
        # STOP lives on it. A stop control that can be scrolled off the
        # screen is its own hazard -- the same argument that keeps the
        # e-stop on both motion panels of the main console. The two lamps
        # ride with it, which is right for a different reason: they answer
        # "can a scan start at all", and that should not need scrolling to.
        self._build_connection(self.root)
        shell = self._scrollable(self.root)
        body = tk.Frame(shell, bg=C.BG)
        body.pack(fill="both", expand=True, padx=12, pady=(0, 8))
        self._build_controls(body)
        self._build_plot(body)
        self._build_log(shell)

    def _build_connection(self, parent):
        row = self._card(parent)
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

        # SIMULATION is a switch, not a silent fallback. With it OFF and no
        # board connected, START is refused -- because the alternative is
        # invented points that look exactly like measured ones, and the only
        # thing separating them is whether somebody remembered which mode
        # they were in.
        self.sim_btn = tk.Button(row, text="SIM: ON", command=self._toggle_sim,
                                 bg=C.WARN, fg="#0b0d10", relief="flat",
                                 font=C.FONT_BOLD, padx=12, pady=4)
        self.sim_btn.pack(side="left", padx=14)

        tk.Button(row, text="EMERGENCY STOP", command=self._estop, bg=C.BAD,
                  fg="#0b0d10", relief="flat", font=C.FONT_BOLD,
                  padx=14, pady=4).pack(side="right")

        # The two things that decide whether START can work at all, next to
        # the link lamp rather than buried in the log. The board refuses a
        # scan without PLC device data, and refuses one with RM's switch
        # switched off -- both were errors you only saw after pressing
        # START and watching the machine not move.
        self.rm_lamp = tk.Label(row, bg=C.PANEL, font=C.FONT_BOLD)
        self.rm_lamp.pack(side="right", padx=(0, 16))
        self.plc_lamp = tk.Label(row, bg=C.PANEL, font=C.FONT_BOLD)
        self.plc_lamp.pack(side="right", padx=(0, 16))
        self._refresh_plc_lamps()

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
        self.deg_step_var.trace_add("write", lambda *_a: self._refresh_sweep_hint())
        self._field(col, "ANGULAR STEP (°)", self.deg_step_var)

        # The sweep is 340 by default because that is the whole travel, not
        # because it is required. One wall is a real job, and a short sweep
        # is much faster.
        self.sweep_var = tk.StringVar(value=f"{C.DEFAULT_SWEEP_DEG:g}")
        self.sweep_var.trace_add("write", lambda *_a: self._refresh_sweep_hint())
        self._field(col, "SWEEP (°)", self.sweep_var)
        self.sweep_hint = tk.Label(col, text="", bg=C.PANEL, fg=C.MUTED,
                                   font=C.FONT_CAPTION, justify="left")
        self.sweep_hint.pack(anchor="w", pady=(0, 12))

        tk.Label(col, text="SENSOR", bg=C.PANEL, fg=C.MUTED,
                 font=C.FONT_CAPTION).pack(anchor="w")
        self.sensor_var = tk.StringVar(value=C.DEFAULT_SENSOR)
        ttk.Combobox(col, textvariable=self.sensor_var, width=12, state="readonly",
                     values=list(C.SENSOR_KINDS)).pack(anchor="w", pady=(0, 4))

        # THE SENSOR INDICATOR. A scan that produces nothing looks identical
        # to a scan whose sensor is unplugged until the points come back
        # empty, by which time the machine has swept a whole layer. This
        # says which it is BEFORE the run, and keeps saying so during it.
        self.sensor_lamp = tk.Label(col, text="● no reading yet", bg=C.PANEL,
                                    fg=C.MUTED, font=C.FONT_BOLD, anchor="w")
        self.sensor_lamp.pack(anchor="w", fill="x")
        self.sensor_hint = tk.Label(col, text="press TEST READ, or start a scan",
                                    bg=C.PANEL, fg=C.MUTED, font=C.FONT_CAPTION,
                                    justify="left", anchor="w")
        self.sensor_hint.pack(anchor="w", fill="x")
        # Works while the machine is idle: SCAN_READ takes one reading and
        # moves nothing, so the sensor can be aimed and checked by hand.
        self.read_btn = tk.Button(col, text="TEST READ", command=self._test_read,
                                  bg=C.FIELD, fg=C.INK, relief="flat",
                                  font=C.FONT_BOLD, pady=4)
        self.read_btn.pack(fill="x", pady=(6, 12))

        self.start_btn = tk.Button(col, text="START SCAN", command=self._start,
                                   bg=C.OK, fg="#0b0d10", relief="flat",
                                   font=C.FONT_BOLD, pady=8)
        self.start_btn.pack(fill="x")
        self.stop_btn = tk.Button(col, text="STOP", command=self._stop,
                                  bg=C.FIELD, fg=C.INK, relief="flat",
                                  font=C.FONT_BOLD, pady=6, state="disabled")
        self.stop_btn.pack(fill="x", pady=(6, 12))

        # HOME is here because a scan wants a reference and this is where
        # the operator is standing when they find out they have not got
        # one. It is the SAME PLC cycle the main console runs -- there is no
        # second homing path -- so a machine homed from here is homed for
        # both applications.
        self.home_btn = tk.Button(col, text="HOME", command=self._home,
                                  bg=C.FIELD, fg=C.INK, relief="flat",
                                  font=C.FONT_BOLD, pady=6)
        self.home_btn.pack(fill="x")
        tk.Label(col, text="asks the PLC to home all four axes",
                 bg=C.PANEL, fg=C.MUTED, font=C.FONT_CAPTION).pack(anchor="w",
                                                                   pady=(0, 12))

        self.progress_var = tk.StringVar(value="Idle")
        tk.Label(col, textvariable=self.progress_var, bg=C.PANEL, fg=C.INK,
                 font=C.FONT_MONO, justify="left").pack(anchor="w")
        self.counts_var = tk.StringVar(value="0 points")
        tk.Label(col, textvariable=self.counts_var, bg=C.PANEL, fg=C.MUTED,
                 font=C.FONT_CAPTION, justify="left").pack(anchor="w")

        self._refresh_hint()
        self._refresh_sweep_hint()

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

    def _build_log(self, parent):
        wrap = self._card(parent)
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

    def _refresh_sweep_hint(self):
        """How many points a layer will hold, live. The angular step alone
        does not answer it -- the sweep is the other half."""
        try:
            sweep = float(self.sweep_var.get())
            deg_step = float(self.deg_step_var.get())
        except (TypeError, ValueError):
            self.sweep_hint.config(text="", fg=C.MUTED)
            return
        if deg_step <= 0 or sweep <= 0:
            self.sweep_hint.config(text="", fg=C.MUTED)
            return
        if sweep > C.SWEEP_MAX_DEG:
            self.sweep_hint.config(
                text=f"✗ past the {C.SWEEP_MAX_DEG:g}° the turntable can turn",
                fg=C.BAD)
            return
        # The first point is taken AT the start angle, so a 340° sweep in 1°
        # steps is 341 points, not 340.
        self.sweep_hint.config(text=f"{int(sweep / deg_step) + 1} points per layer",
                               fg=C.MUTED)

    def _validate(self):
        """Returns (z_step, deg_step, layers, sweep) or None, having said why."""
        try:
            z_step = float(self.z_step_var.get())
            deg_step = float(self.deg_step_var.get())
            layers = int(float(self.layers_var.get()))
            sweep = float(self.sweep_var.get())
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
        if not (C.SWEEP_MIN_DEG <= sweep <= C.SWEEP_MAX_DEG):
            messagebox.showerror(
                C.APP_TITLE,
                f"The sweep must be between {C.SWEEP_MIN_DEG:g} and "
                f"{C.SWEEP_MAX_DEG:g}° — the whole travel of the turntable.")
            return None
        if sweep < deg_step:
            messagebox.showerror(
                C.APP_TITLE,
                f"A {sweep:g}° sweep is shorter than the {deg_step:g}° step, "
                f"so each layer would hold a single point.")
            return None
        return z_step, deg_step, layers, sweep

    def _send(self, text):
        """Board first, simulator only if it is switched on.

        Returns False when there is nowhere to send it, so the caller can
        say so rather than appearing to work.
        """
        if self.link.is_open:
            return self.link.send(text)
        if self.sim_enabled:
            return self.sim.send(text)
        return False

    @property
    def _source(self):
        return "board" if self.link.is_open else "simulation"

    def _refresh_link_lamp(self):
        if self.link.is_open:
            self.link_lamp.config(text="● CONNECTED", fg=C.OK)
        elif self.sim_enabled:
            self.link_lamp.config(text="● SIMULATED", fg=C.WARN)
        else:
            # Not a fault, but not a machine either. Distinct from
            # SIMULATED, because the difference decides whether pressing
            # START does anything at all.
            self.link_lamp.config(text="● NO BOARD", fg=C.BAD)

    def _toggle_sim(self):
        if self.scanning:
            messagebox.showinfo(C.APP_TITLE,
                                "Stop the scan before changing the source.")
            return
        self.sim_enabled = not self.sim_enabled
        self.sim_btn.config(text="SIM: ON" if self.sim_enabled else "SIM: OFF",
                            bg=C.WARN if self.sim_enabled else C.FIELD,
                            fg="#0b0d10" if self.sim_enabled else C.INK)
        self._refresh_link_lamp()
        if self.sim_enabled:
            self.log("Simulation ON — with no board connected, START produces "
                     "made-up points.", "warn")
        else:
            self.log("Simulation OFF — only a connected board can produce points.",
                     "ok")

    # ==================================================================
    # actions
    # ==================================================================
    def _toggle_connect(self):
        if self.link.is_open:
            self.link.send(C.CMD_BYE)
            self.link.close()
            self.connect_btn.config(text="CONNECT")
            self._refresh_link_lamp()
            self._plc_link_lost()
            self.log("Disconnected — " + ("back to the simulation."
                                          if self.sim_enabled
                                          else "no source of points now."))
            return
        port = self.port_var.get()
        if not port:
            messagebox.showerror(C.APP_TITLE, "Pick a COM port first.")
            return
        if self.link.open(port, self.baud_var.get()):
            self.connect_btn.config(text="DISCONNECT")
            self._refresh_link_lamp()
            self.log(f"Connected on {port}.", "ok")
            self.link.send(C.CMD_PING)
            # Ask once immediately. The board pushes [PLC_STATE] on every
            # change of the status word, but a link that is already steady
            # produces no change, so without this the lamps would sit on
            # NO LINK against a perfectly healthy PLC.
            self.link.send(C.CMD_PLC_STATUS)

    def _start(self):
        if self.scanning:
            return
        if self.homing:
            messagebox.showinfo(C.APP_TITLE,
                                "The machine is homing. Wait for it to finish.")
            return
        params = self._validate()
        if params is None:
            return
        z_step, deg_step, layers, sweep = params
        if not self.link.is_open and not self.sim_enabled:
            messagebox.showerror(
                C.APP_TITLE,
                "No board is connected, and simulation is switched off.\n\n"
                "Connect a board, or switch SIM back on to see how the scan "
                "behaves without one.")
            return
        if self.link.is_open and not self._plc_ready_for_scan():
            return
        self.store.clear()
        self.store.simulated = not self.link.is_open
        self.plot.set_range(C.PLOT_MIN_RANGE_MM)
        self.layer, self.layers = 0, layers
        # The indicator judges THIS scan. Carrying a hit rate over from the
        # last one would report a sensor that has since been unplugged as
        # healthy for the first fifty points.
        self.recent.clear()
        self._send(C.cmd_sensor(self.sensor_var.get()))
        self._send(C.cmd_scan_start(z_step, deg_step, layers, sweep))
        self.scanning = True
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        # HOME during a sweep would drive the turntable out from under the
        # scan, so it goes with START rather than being merely refused when
        # pressed.
        self.home_btn.config(state="disabled")
        self.log(f"Scan started on the {self._source} — {layers} layers, "
                 f"{z_step:g} mm apart, {deg_step:g}° steps over {sweep:g}°.",
                 "warn" if self.store.simulated else "ok")

    def _plc_ready_for_scan(self):
        """Says NO only where the board is CERTAIN to refuse anyway.

        The point is to name the fix here, standing at the panel, instead of
        letting the operator press START and read an [ERROR] afterwards.
        What it must not do is refuse on a state it merely has not heard
        about yet: UNKNOWN means this app has no news, not that the board
        has no data, and blocking on it would make a working machine
        unusable whenever a push was missed.
        """
        if self.rm_state == "disabled":
            messagebox.showerror(
                C.APP_TITLE,
                "RM's switch is switched off on the board.\n\n"
                "It is the reference every layer is measured from, so the "
                "board will refuse the scan. Send SET_PLC_SENSOR_ENFORCE:ROT,1 "
                "to put it back.")
            return False
        if self.plc_state in ("unreachable", "disabled"):
            messagebox.showerror(
                C.APP_TITLE,
                "There is no PLC device data, so RM's switch cannot be seen "
                "and the scan has no frame to sweep in.\n\n"
                "Check the cable and the PLC's Ethernet module. PLC_TEST on "
                "the board reports which layer is failing.")
            return False
        if self.plc_state == "no_reply":
            # NOT refused: the board keeps the last good status word, so it
            # may well accept the scan. But the switch reading it accepts it
            # against could be minutes old, which is worth knowing before a
            # long run rather than after it.
            return messagebox.askokcancel(
                C.APP_TITLE,
                "The PLC is not answering device reads, so RM's switch "
                "reading may be stale.\n\nStart the scan anyway?")
        return True

    def _test_read(self):
        """One reading, no motion. The point of it is to find out whether
        the sensor works BEFORE committing the machine to a sweep."""
        if not self._send(C.CMD_SCAN_READ):
            messagebox.showerror(
                C.APP_TITLE,
                "No board is connected, and simulation is switched off — "
                "there is nothing to read from.")

    # ------------------------------------------------------------------
    # the PLC link, and RM's switch
    # ------------------------------------------------------------------
    def _refresh_plc_lamps(self):
        label, colour = C.PLC_STATE_LABELS[self.plc_state]
        self.plc_lamp.config(text="● " + label, fg=colour)
        label, colour = C.RM_STATE_LABELS[self.rm_state]
        self.rm_lamp.config(text="● " + label, fg=colour)

    def _set_plc_state(self, plc_state, rm_state):
        """Logs only on a CHANGE. At a 5 s poll a line per reply buries the
        log, which is what the log is there to avoid."""
        changed = (plc_state != self.plc_state) or (rm_state != self.rm_state)
        was_plc = self.plc_state
        self.plc_state, self.rm_state = plc_state, rm_state
        self._refresh_plc_lamps()
        if not changed:
            return
        if plc_state != was_plc:
            if plc_state == "connected":
                self.log("PLC link up — the Mitsubishi is answering device reads.", "ok")
            elif plc_state == "no_reply":
                self.log("PLC socket is open but no device data is coming back. "
                         "Check that MC protocol is enabled on that port.", "warn")
            elif plc_state == "unreachable":
                self.log("PLC unreachable — check the cable, the subnet and the "
                         "PLC's Ethernet module. A scan cannot start without it.",
                         "err")
        if rm_state == "disabled":
            self.log("RM's switch is switched off on the board. It is the reference "
                     "every layer starts from, so a scan will be refused. "
                     "SET_PLC_SENSOR_ENFORCE:ROT,1 puts it back.", "err")

    def _plc_link_lost(self):
        """Serial link gone. UNKNOWN, not a stale reading: a CONNECTED lamp
        from three minutes ago is worse than no lamp at all."""
        self._set_plc_state("unknown", "unknown")

    def _on_plc_state(self, line):
        """Reads the board's [PLC_STATE] into the two lamps.

        The socket is deliberately NOT what drives the link lamp -- see the
        note on PLC_STATE_LABELS. `data=` is.
        """
        m = PLC_LINK_RE.search(line)
        if not m:
            return
        link, socket = m.group(1).upper(), m.group(2).upper()
        if link == "DISABLED":
            # Off on purpose. Checked first so it can never be reported as
            # a fault, and it takes the RM lamp with it: with the link down
            # the switch is not being read at all.
            self._set_plc_state("disabled", "unknown")
            return

        data = PLC_DATA_RE.search(line)
        conn = PLC_CONN_RE.search(line)
        ever_opened = bool(conn) and int(conn.group(1)) > 0
        got = data.group(1).upper() if data else None
        if got == "OK":
            plc = "connected"
        elif got == "STALE":
            plc = "no_reply"
        elif ever_opened:
            # TCP answers, MC protocol does not. Steady even while the
            # socket itself opens and closes on each timeout, which is the
            # flap a socket-driven lamp showed.
            plc = "no_reply"
        elif link != "UP" or socket != "OPEN":
            plc = "unreachable"
        else:
            plc = "no_reply"

        # RM is the MIDDLE of each Z/R/A2 field. "?" means the board has no
        # device data and is saying so, which is not the same as CLEAR.
        bits = PLC_LIMIT_RE.search(line)
        enforce = PLC_ENFORCE_RE.search(line)
        if enforce and enforce.group(1)[1] == "0":
            rm = "disabled"
        elif plc != "connected" or not bits or bits.group(1)[1] == "?":
            rm = "unknown"
        else:
            rm = "covered" if bits.group(1)[1] == "1" else "clear"
        self._set_plc_state(plc, rm)

    def _note_reading(self, mm):
        """Records one reading for the indicator. Called per point, so it
        stays arithmetic: the rendering happens in _repaint."""
        self.last_mm = mm
        self.recent.append(mm >= 0)

    def _refresh_sensor_lamp(self):
        if self.last_mm is None:
            self.sensor_lamp.config(text="● no reading yet", fg=C.MUTED)
            self.sensor_hint.config(text="press TEST READ, or start a scan",
                                    fg=C.MUTED)
            return
        misses = sum(1 for hit in self.recent if not hit)
        seen = len(self.recent)
        # A miss is the sensor's OWN answer -- "nothing came back" -- so the
        # lamp reports the last reading for what it was, and judges the
        # health separately on the rate. One lost echo in a wall of good
        # ones is not a fault; every reading missing is.
        if self.last_mm < 0:
            self.sensor_lamp.config(text="● NO ECHO", fg=C.BAD)
        else:
            self.sensor_lamp.config(text=f"● {self.last_mm:.1f} mm", fg=C.OK)
        if seen and misses == seen:
            self.sensor_hint.config(
                text=f"nothing came back in {seen} readings — check the wiring, "
                     f"the sensor kind, and (ANALOG) the calibration", fg=C.BAD)
        elif seen and misses * 2 > seen:
            self.sensor_hint.config(text=f"{misses} of the last {seen} missed",
                                    fg=C.WARN)
        else:
            self.sensor_hint.config(text=f"{misses} of the last {seen} missed",
                                    fg=C.MUTED)

    def _home(self):
        """Asks the board to run the PLC home cycle.

        BOARD ONLY. The simulator has no PLC -- it says so when asked for
        PLC_STATUS -- and a simulated home that reported success would zero
        nothing, while telling the operator their machine had a reference.
        That is the one lie this application is built not to tell.
        """
        if self.scanning:
            messagebox.showinfo(C.APP_TITLE,
                                "Stop the scan before homing the machine.")
            return
        if self.homing:
            return
        if not self.link.is_open:
            messagebox.showerror(
                C.APP_TITLE,
                "HOME moves the real machine, so it needs a connected board.\n\n"
                "The simulator has no PLC to ask, and a pretend home would "
                "report a reference the machine has not got.")
            return
        # Confirmed, unlike START: HOME drives all four axes to their
        # switches on the PLC's schedule, and the operator may be standing
        # at the panel rather than at the machine.
        if not messagebox.askokcancel(
                C.APP_TITLE,
                "HOME will drive every axis to its switch and reset the "
                "coordinates.\n\nMake sure the machine is clear. Continue?"):
            return
        self.homing = True
        self.start_btn.config(state="disabled")
        self.home_btn.config(state="disabled")
        self.progress_var.set("Homing…")
        self.link.send(C.CMD_HOME)
        self.log("HOME sent. ClearCore holds its IO-0 output into the PLC's X0 "
                 "input — a wire, not a network message — and waits for DONE.",
                 "warn")

    def _home_ended(self, ok, line):
        self.homing = False
        self.start_btn.config(state="normal")
        self.home_btn.config(state="normal")
        self.progress_var.set("Homed" if ok else "Home failed")
        self.log(line, "ok" if ok else "err")

    def _stop(self):
        self._send(C.CMD_SCAN_STOP)
        self.log("Stop sent.", "warn")

    def _estop(self):
        # Straight out, no confirmation. A dialog between the operator and
        # a stop button is the thing that makes the button useless.
        self._send(C.CMD_ESTOP)
        # Clears the HOME state too. A stop that left the panel believing a
        # home was still in progress would keep START refused with nothing
        # running -- the same trap an [ERROR] mid-scan used to set.
        self.homing = False
        self._scan_ended("EMERGENCY STOP")

    def _scan_ended(self, why):
        self.scanning = False
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.home_btn.config(state="normal")
        self.progress_var.set(why)

    def _save_csv(self):
        if not self.store.points:
            messagebox.showinfo(C.APP_TITLE, "There are no points to save yet.")
            return
        # Simulated points are the same numbers in the same columns as real
        # ones. The filename is the only thing that will still say which is
        # which a week from now.
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile="scan_SIMULATED.csv" if self.store.simulated else "scan.csv",
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
            self._note_reading(mm)
            self._dirty = True
            return                       # never logged: hundreds per layer

        m = READ_RE.search(line)
        if m:
            # A one-off TEST READ. Logged, unlike a scan point: it was asked
            # for by hand and there is exactly one of it.
            mm = float(m.group(1))
            self._note_reading(mm)
            self._refresh_sensor_lamp()
            self.log(line, "err" if mm < 0 else "ok")
            return

        m = LAYER_RE.search(line)
        if m:
            self.layer, self.layers = int(m.group(1)), int(m.group(2))
            self.store.set_layer_z(self.layer, float(m.group(3)))
            self.log(line)
            return

        if line.startswith(C.TAG_PLC_STATE):
            # Never logged raw: the board pushes one on every change of the
            # status word, and the lamps say the same thing better.
            self._on_plc_state(line)
            return

        if line.startswith(C.TAG_HOME):
            # Every step of the cycle reports under [HOME]; only two of them
            # mean it is over, so the end is matched on the text and not on
            # the tag.
            upper = line.upper()
            if C.HOME_DONE_TEXT in upper:
                self._home_ended(True, line)
            elif C.HOME_FAILED_TEXT in upper:
                self._home_ended(False, line)
            else:
                self.log(line)
            return

        if line.startswith((C.TAG_PLC_HOME, C.TAG_COORD_RESET)):
            self.log(line, "ok")
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
        if self.sim_enabled:
            self.sim.poll()
        self.root.after(POLL_MS, self._tick)

    def _plc_tick(self):
        """Backstop for the board's pushed [PLC_STATE].

        A link that is steady produces no change of status word and so no
        push, which is exactly the state the operator wants confirmed. Only
        asked of a real board: the simulator has no PLC to report on, and a
        made-up CONNECTED lamp would be the same lie the SIM switch exists
        to prevent.
        """
        if self.link.is_open:
            self.link.send(C.CMD_PLC_STATUS)
        self.root.after(C.PLC_POLL_MS, self._plc_tick)

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
        self._refresh_sensor_lamp()
        # Says so on the picture itself, not only in the log. A screenshot
        # of a simulated scan is otherwise indistinguishable from a real one.
        source = "SIMULATED DATA · " if self.store.simulated else ""
        self.scale_var.set(f"{source}full scale {self.plot.range_mm:.0f} mm · "
                           f"0° is straight ahead at RM 0")


def main():
    root = tk.Tk()
    ScannerApp(root)
    root.mainloop()
