"""Behaviour checks for the scanner. Run with:  python tests/scan_check.py

Uses the main project's headless Tk stub (../../tests/tkstub), for the same
reason the console's suite does: the window never opens, but every widget,
every StringVar and every parse path is still exercised.
"""

import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SCAN = os.path.dirname(HERE)
ROOT = os.path.dirname(SCAN)
sys.path.insert(0, os.path.join(ROOT, "tests", "tkstub"))
sys.path.insert(0, SCAN)

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

FAIL = []


def check(cond, msg):
    print(("  OK   " if cond else "  FAIL ") + msg)
    if not cond:
        FAIL.append(msg)


import tkinter as tk                                   # noqa: E402
import scanner.config as C                             # noqa: E402
from scanner.app import ScannerApp, LAYER_RE, POINT_RE  # noqa: E402
from scanner.link import SimulatedBoard                # noqa: E402
from scanner.plot import PolarPlot                     # noqa: E402
from scanner.store import ScanStore                    # noqa: E402

SimulatedBoard.POINT_INTERVAL_S = 0.0                  # no wall clock in tests


print("\n=== 1. the wire matches the firmware ===")
# Spelled out rather than rebuilt from the same helper: a test that calls
# the function it is testing to work out the expected answer proves only
# that the function is consistent with itself.
check(C.cmd_scan_start(5.0, 1.0, 10) == "SCAN_START:5.000,1.000,10,340.00",
      "SCAN_START carries zStep, degStep, layers and the sweep")
check(C.cmd_scan_start(2.5, 0.5, 4, 180.0) == "SCAN_START:2.500,0.500,4,180.00",
      "  ...and a shorter sweep goes in the fourth field")
check(C.cmd_sensor("ANALOG") == "SET_SCAN_SENSOR:ANALOG",
      "the sensor kind is sent by name, not by an index nobody can read")
check(C.cmd_cal(0.5, 10.0) == "SET_SCAN_CAL:0.500000,10.000",
      "the analog calibration is mm-per-count then offset")
check(C.DEG_STEP_MAX == 90.0 and C.Z_STROKE_MM == 285.0,
      "the GUI's limits mirror the board's, so a bad number is caught here first")

# The board is the authority on all of this; if these strings drift the
# scan silently stops working, so read them out of the .ino itself.
ino = open(os.path.join(ROOT, "RobotMotionController_v9_ClearCore",
                        "RobotMotionController_v9_ClearCore.ino"),
           encoding="utf-8").read()
for token in ("SCAN_START:", "SCAN_STOP", "SCAN_READ", "SCAN_STATUS",
              "SET_SCAN_SENSOR:", "SET_SCAN_CAL:", "PLC_STATUS"):
    check(token in ino, f"the firmware answers {token}")
check('"[PLC_STATE] link=" ' in ino or "[PLC_STATE] link=" in ino,
      "  ...and reports the PLC link in the shape the lamps parse")
check("limit Z/R/A2=" in ino and "enforce Z/R/A2=" in ino,
      "  ...with RM's switch as the MIDDLE bit of each Z/R/A2 field")
check("[SCAN_PT] " in ino and "[SCAN_LAYER] " in ino and "[SCAN_DONE] " in ino,
      "  ...and sends the three lines this app parses")
check("scanReadDistanceMm" in ino and "serviceScan" in ino,
      "the scan lives in the SAME firmware, not a second sketch")


print("\n=== 2. parsing what the board sends ===")
m = POINT_RE.search("[SCAN_PT] 3,127.50,412.25")
check(m is not None and m.group(1) == "3" and m.group(2) == "127.50"
      and m.group(3) == "412.25", "a point splits into layer, angle, distance")
m = POINT_RE.search("[SCAN_PT] 1,0.00,-1.00")
check(m is not None and float(m.group(3)) < 0,
      "a miss keeps its negative distance instead of being read as 0 mm")
m = LAYER_RE.search("[SCAN_LAYER] 2/10 z=15.00 mm")
check(m is not None and m.group(1) == "2" and m.group(2) == "10"
      and m.group(3) == "15.00", "a layer line gives the index, the total and the height")
check(POINT_RE.search("[SCAN_LAYER] 2/10 z=15.00 mm") is None,
      "  ...and the two patterns do not match each other's lines")


print("\n=== 3. the store keeps misses, and never plots them ===")
st = ScanStore()
st.add(1, 0.0, 300.0)
st.add(1, 1.0, -1.0)
st.add(1, 2.0, 310.0)
st.add(2, 0.0, 320.0)
st.set_layer_z(1, 0.0)
st.set_layer_z(2, 5.0)
check(st.total == 4 and st.misses == 1, "every point is kept, misses counted")
check(st.layer_points(1) == [(0.0, 300.0), (2.0, 310.0)],
      "the plot gets the HITS only - a miss at radius 0 would draw a false wall")
check(st.layers() == [1, 2], "layers are listed in order")
check(abs(st.max_radius() - 320.0) < 1e-9, "the radial extent ignores misses")

path = os.path.join(tempfile.gettempdir(), "scan_check.csv")
rows = st.to_csv(path)
body = open(path, encoding="utf-8").read().splitlines()
check(rows == 4 and len(body) == 5, "the CSV holds every point plus a header")
check(body[0].startswith("layer,z_mm,angle_deg,distance_mm,x_mm,y_mm,hit"),
      "  ...with the columns named")
check(body[2].endswith(",0") and ",-1.00," in body[2],
      "  ...and a miss is written out marked, not dropped")
check(body[1].split(",")[4].startswith("300.") ,
      "a hit at 0 deg is 300 mm along +X")
os.remove(path)


print("\n=== 4. the polar mapping ===")
root = tk.Tk()
pp = PolarPlot(root, size=400)
pp.set_range(400.0)
cx, cy = pp._centre
x0, y0 = pp._to_xy(0.0, 400.0)
check(x0 > cx and abs(y0 - cy) < 1e-6, "0 deg points along +X, where the arm points at RM 0")
x90, y90 = pp._to_xy(90.0, 400.0)
# Canvas Y grows DOWNWARD. Without negating the sine the plot is mirrored
# and a notch on the left of the machine appears on the right of the screen.
check(abs(x90 - cx) < 1e-6 and y90 < cy, "90 deg points UP the screen, not down")
check(pp._to_xy(0.0, 0.0) == (cx, cy), "zero distance sits on the centre")
pp.set_range(10.0)
check(pp.range_mm == C.PLOT_MIN_RANGE_MM,
      "the scale never collapses below its floor, whatever the data says")


print("\n=== 5. the window, driven through a whole scan ===")
app = ScannerApp(tk.Tk())
app.z_step_var.set("4")
app.layers_var.set("3")
app.deg_step_var.set("2")
check(app._validate() == (4.0, 2.0, 3, 340.0), "sane numbers validate")

app.z_step_var.set("0")
check(app._validate() is None, "a zero step up is refused before anything is sent")
app.z_step_var.set("4")
app.deg_step_var.set("120")
check(app._validate() is None, "an angular step past a quarter turn is refused")
app.deg_step_var.set("2")
app.layers_var.set("0")
check(app._validate() is None, "zero layers is refused")
app.layers_var.set("100")
check(app._validate() is None,
      "100 layers 4 mm apart need 396 mm of lift - refused against the 285 mm stroke")
app.layers_var.set("3")
app.z_step_var.set("abc")
check(app._validate() is None, "a field that is not a number is refused")
app.z_step_var.set("4")

# The sweep may be SHORTER than the travel -- one wall is a real job -- but
# never longer, because the turntable cannot turn further than its own stop.
app.sweep_var.set("120")
check(app._validate() == (4.0, 2.0, 3, 120.0), "a shorter sweep is accepted")
app.sweep_var.set("400")
check(app._validate() is None, "a sweep past the 340 deg of travel is refused")
app.sweep_var.set("0")
check(app._validate() is None, "a zero sweep is refused")
app.sweep_var.set("1")
check(app._validate() is None,
      "a sweep shorter than one step is refused - a layer would hold one point")
app.sweep_var.set("340")

app.deg_step_var.set("2")
app.sweep_var.set("340")
app._refresh_sweep_hint()
check("171 points per layer" in app.sweep_hint.cget("text"),
      "the hint counts the points a layer will hold: the first is taken AT the start angle")
app.sweep_var.set("400")
app._refresh_sweep_hint()
check("past the" in app.sweep_hint.cget("text"),
      "  ...and says so when the turntable cannot turn that far")
app.sweep_var.set("340")
app.deg_step_var.set("2")
app._refresh_sweep_hint()

# The live hint answers the same question before the operator presses START.
# Called directly: the real app hangs it off a StringVar trace, which the
# headless stub does not fire, and the arithmetic is the part worth testing.
app.layers_var.set("3")
app._refresh_hint()
check("8.0 mm above the start" in app.height_hint.cget("text"),
      "the hint reports the top of the LAST layer: 3 layers is two steps, not three")
app.layers_var.set("100")
app._refresh_hint()
check("past the" in app.height_hint.cget("text"),
      "  ...and says so when it will not fit")
app.layers_var.set("3")
app._refresh_hint()

app._start()
check(app.scanning is True, "START marks a scan running")
check(str(app.start_btn.cget("state")) == "disabled",
      "  ...and the button cannot be pressed twice")
for _ in range(80):
    app.sim.poll()
    if not app.sim.running:
        break
app._repaint()
check(app.store.layers() == [1, 2, 3], "all three layers arrived")
check(app.store.layer_z == {1: 0.0, 2: 4.0, 3: 8.0},
      "  ...at the heights the Z step asks for")
# The sweep starts at the RM switch and ALTERNATES: out, then back to the
# switch, then out again. The return leg collects a layer instead of being
# dead rewind travel, and ending on the switch re-references the turntable
# so the start angle cannot drift over a tall scan.
_l1 = [d for d, _r in app.store.layer_points(1)]
_l2 = [d for d, _r in app.store.layer_points(2)]
_l3 = [d for d, _r in app.store.layer_points(3)]
check(_l1[0] > _l1[-1], "layer 1 sweeps AWAY from the switch, so its angle counts down")
check(_l2[0] < _l2[-1], "layer 2 comes BACK to it, counting up")
check(_l3[0] > _l3[-1], "  ...and layer 3 turns round again")
check(abs(_l1[0] - _l3[0]) < 1e-6,
      "every outward layer starts at the same angle - the switch, not a drifting count")
check(app.store.total > 500, "a 2 deg step over 340 deg is about 171 points a layer")
check(app.scanning is False and app.progress_var.get() == "Finished",
      "[SCAN_DONE] ends the scan and re-enables START")
check(str(app.start_btn.cget("state")) == "normal", "  ...the button really is back")

frozen = app.store.range_mm
app._repaint()
check(app.store.range_mm == frozen,
      "the radial scale is frozen once chosen, so layers can be compared by eye")


print("\n=== 5b. simulation is a SWITCH, not a silent fallback ===")
sw = ScannerApp(tk.Tk())
check(sw.sim_enabled is True, "it starts simulated, so the tool demonstrates out of the box")
check(sw.sim_btn.cget("text") == "SIM: ON", "  ...and the button says so")
sw._start()
check(sw.scanning is True and sw.store.simulated is True,
      "with SIM ON and no board, a scan runs and is MARKED simulated")
sw._on_line("[SCAN_ABORT] stopped")

sw._toggle_sim()
check(sw.sim_enabled is False and sw.sim_btn.cget("text") == "SIM: OFF",
      "the switch turns it off")
check(sw.link_lamp.cget("text") == "\u25cf NO BOARD",
      "  ...and the lamp stops claiming SIMULATED - there is no source at all now")
sw._start()
check(sw.scanning is False,
      "START is REFUSED with no board and no simulation, rather than inventing points")
check(sw.store.total == 0, "  ...and nothing lands in the store")
check(sw._send("SCAN_STATUS") is False,
      "  ...nothing is sent anywhere either")

# Toggling mid-scan would change what the numbers mean half way through the
# file, so it is refused while one is running.
sw._toggle_sim()
sw._start()
check(sw.scanning is True, "back on, a scan runs again")
sw._toggle_sim()
check(sw.sim_enabled is True, "the switch is refused mid-scan")
sw._on_line("[SCAN_ABORT] stopped")


print("\n=== 5c. the sweep the operator typed is the sweep that is sent ===")
sv = ScannerApp(tk.Tk())
sent = []
sv._send = lambda text: (sent.append(text), True)[1]
sv.sweep_var.set("120")
sv.deg_step_var.set("10")
sv.layers_var.set("2")
sv._start()
check(any(l == "SCAN_START:5.000,10.000,2,120.00" for l in sent),
      "a 120 deg sweep reaches the wire, rather than the 340 deg default")

sw2 = ScannerApp(tk.Tk())
sw2.sweep_var.set("90")
sw2.deg_step_var.set("30")
sw2.layers_var.set("2")
sw2._start()
for _ in range(400):
    sw2.sim.poll()
    if not sw2.sim.running:
        break
_angles = [d for d, _r in sw2.store.layer_points(1)]
check(abs((max(_angles) - min(_angles)) - 90.0) < 30.1,
      "  ...and the simulated layer really only covers 90 deg, not 340")


print("\n=== 5d. the sensor indicator ===")
si = ScannerApp(tk.Tk())
check(si.last_mm is None and si.sensor_lamp.cget("text") == "● no reading yet",
      "it starts saying nothing has been read - NOT a healthy-looking 0 mm")

si._on_line("[SCAN_READ] 412.25 mm (ULTRASONIC)")
check(abs(si.last_mm - 412.25) < 1e-9, "a TEST READ reply is picked up")
check(si.sensor_lamp.cget("text") == "● 412.2 mm"
      and si.sensor_lamp.cget("fg") == C.OK,
      "  ...and shown on the lamp straight away, without waiting for a repaint")

si._on_line("[SCAN_READ] -1.00 mm (ULTRASONIC)")
check(si.sensor_lamp.cget("text") == "● NO ECHO"
      and si.sensor_lamp.cget("fg") == C.BAD,
      "a miss reads as NO ECHO, not as -1.0 mm, and not as a distance")

# A sensor that answers nothing at all is the failure worth naming: the scan
# runs, the machine sweeps, and every point is empty.
si2 = ScannerApp(tk.Tk())
for i in range(60):
    si2._on_line(f"[SCAN_PT] 1,{i}.00,-1.00")
si2._refresh_sensor_lamp()
check("nothing came back" in si2.sensor_hint.cget("text")
      and si2.sensor_hint.cget("fg") == C.BAD,
      "every reading missing is called out, wiring and calibration named")

# ...and the odd lost echo is NOT. A lamp that cries wolf gets ignored.
si3 = ScannerApp(tk.Tk())
for i in range(60):
    si3._on_line(f"[SCAN_PT] 1,{i}.00,{-1.0 if i % 20 == 0 else 300.0:.2f}")
si3._refresh_sensor_lamp()
check(si3.sensor_hint.cget("fg") == C.MUTED,
      "  ...but one lost echo in twenty is a working sensor, and stays quiet")
check(si3.sensor_lamp.cget("fg") == C.OK, "  ...with the last good reading shown")

check(len(si3.recent) <= 50,
      "the indicator judges RECENT readings, so it recovers when the sensor does")
si3._start()
check(len(si3.recent) == 0,
      "a new scan starts the count again - last scan's health is not this scan's")

si4 = ScannerApp(tk.Tk())
si4._toggle_sim()
check(si4._send(C.CMD_SCAN_READ) is False,
      "TEST READ has nowhere to go with no board and no simulation")


print("\n=== 5e. the PLC link and RM's switch ===")
# The scan is REFUSED by the board without PLC device data, and refused with
# RM's switch switched off. Both were errors you only saw after pressing
# START; they are on the panel now.
pl = ScannerApp(tk.Tk())
check(pl.plc_state == "unknown" and pl.rm_state == "unknown",
      "both lamps start UNKNOWN - never a reassuring default")
check(pl.plc_lamp.cget("text") == "● PLC: NO LINK"
      and pl.rm_lamp.cget("text") == "● RM SWITCH: ?",
      "  ...and say so on the panel")

OK_LINE = ("[PLC_STATE] link=UP socket=OPEN data=OK conn=2/3 word=0080 timeouts=1"
           " | limit Z/R/A2=010 end Z/R/A2=-+- enforce Z/R/A2=111")
pl._on_line(OK_LINE)
check(pl.plc_state == "connected" and pl.plc_lamp.cget("fg") == C.OK,
      "data=OK is what lights the link lamp")
check(pl.rm_state == "covered",
      "RM is the MIDDLE bit of Z/R/A2 - reading it positionally as M30 would "
      "put ZM's switch on RM's lamp")

pl._on_line(OK_LINE.replace("limit Z/R/A2=010", "limit Z/R/A2=101"))
check(pl.rm_state == "clear", "  ...and a 0 there is CLEAR, with the others set")

# A dead link showing CLEAR is the field bug this whole convention exists to
# stop: on a safety display the failure read as good news.
pl._on_line("[PLC_STATE] link=DOWN socket=CLOSED data=NONE conn=0/0 word=---- "
            "timeouts=0 | NO DEVICE DATA | limit Z/R/A2=??? end Z/R/A2=???")
check(pl.plc_state == "unreachable", "conn=0 with the socket shut is UNREACHABLE")
check(pl.rm_state == "unknown", "  ...and '?' is UNKNOWN, NOT clear")

# The socket cycles on every reply timeout, so a socket-driven lamp flapped.
# A socket that has opened even once proves cable and address are fine.
pl._on_line("[PLC_STATE] link=DOWN socket=CLOSED data=NONE conn=4/9 word=---- "
            "timeouts=9 | NO DEVICE DATA | limit Z/R/A2=??? end Z/R/A2=???")
check(pl.plc_state == "no_reply",
      "a socket that HAS opened is NO REPLY, not UNREACHABLE - different faults")
pl._on_line(OK_LINE.replace("data=OK", "data=STALE"))
check(pl.plc_state == "no_reply", "data=STALE is the same fault: reads stopped landing")
check(pl.rm_state == "unknown",
      "  ...and the switch reading is not trusted while the data is stale")

pl._on_line("[PLC_STATE] link=DISABLED socket=CLOSED data=NONE conn=0/0 word=---- "
            "timeouts=0 | LINK DISABLED | limit Z/R/A2=???")
check(pl.plc_state == "disabled",
      "SET_PLC_LINK:0 reads as DISABLED, never as a fault")

pl._on_line(OK_LINE.replace("enforce Z/R/A2=111", "enforce Z/R/A2=101"))
check(pl.rm_state == "disabled" and pl.rm_lamp.cget("fg") == C.BAD,
      "RM's switch switched off is flagged RED - it stops a scan outright")

logged = []
pl.log = lambda text, tag=None: logged.append(text)
pl._on_line(OK_LINE.replace("enforce Z/R/A2=111", "enforce Z/R/A2=101"))
check(logged == [],
      "an unchanged state logs NOTHING - a line per 5 s poll would bury the log")

# Losing the serial link must not leave a CONNECTED lamp standing.
pl2 = ScannerApp(tk.Tk())
pl2._on_line(OK_LINE)
pl2._plc_link_lost()
check(pl2.plc_state == "unknown" and pl2.rm_state == "unknown",
      "losing the board marks both UNKNOWN - a stale CONNECTED is worse than none")

# The board refuses these anyway; saying so here names the fix instead.
pl3 = ScannerApp(tk.Tk())
pl3._on_line(OK_LINE.replace("enforce Z/R/A2=111", "enforce Z/R/A2=101"))
check(pl3._plc_ready_for_scan() is False, "START is refused with RM's switch off")
pl3._on_line("[PLC_STATE] link=DOWN socket=CLOSED data=NONE conn=0/0 word=---- "
             "timeouts=0 | limit Z/R/A2=???")
check(pl3._plc_ready_for_scan() is False, "  ...and with no PLC device data")
pl3._on_line(OK_LINE)
check(pl3._plc_ready_for_scan() is True, "a healthy link lets it through")
pl3.plc_state, pl3.rm_state = "unknown", "unknown"
check(pl3._plc_ready_for_scan() is True,
      "UNKNOWN does NOT block: it means this app has no news, not that the "
      "board has no data")

# The simulator has no PLC and says so, rather than inventing a green lamp.
pl4 = ScannerApp(tk.Tk())
pl4._send(C.CMD_PLC_STATUS)
check(pl4.plc_state == "unreachable" and pl4.rm_state == "unknown",
      "the simulator reports NO DEVICE DATA - it will not fake a PLC")


print("\n=== 5f. HOME, and the scrolling viewport ===")
hm = ScannerApp(tk.Tk())
check(hm.homing is False and str(hm.home_btn.cget("state")) != "disabled",
      "HOME starts available")

# Board only. A simulated home would report a reference the machine has not
# got, which is the one lie this app is built not to tell.
sent = []
hm.link.send = lambda text, *_a, **_k: (sent.append(text), True)[1]
hm._home()
check(sent == [] and hm.homing is False,
      "HOME is REFUSED with no board - the simulator has no PLC to ask")

# With a board: confirmed, then the cycle owns the panel until it ends.
import scanner.app as _app
_real_ask, _real_open = _app.messagebox.askokcancel, type(hm.link).is_open
_app.messagebox.askokcancel = lambda *a, **k: True
type(hm.link).is_open = property(lambda self: True)
hm._home()
check(sent == ["HOME"], "with a board it sends HOME, and nothing else")
check(hm.homing is True and str(hm.home_btn.cget("state")) == "disabled",
      "  ...and the button cannot be pressed twice")
check(str(hm.start_btn.cget("state")) == "disabled",
      "  ...nor a scan started while the axes are moving")
hm._start()
check(hm.scanning is False, "  ...START says so rather than silently doing nothing")

# Every step reports under [HOME]; only two of them mean it is over.
hm._on_line("[HOME] ZM reached its switch.")
check(hm.homing is True, "a progress line does NOT end the cycle")
hm._on_line("[HOME] Homing complete. Coordinates reset to standard home.")
check(hm.homing is False and hm.progress_var.get() == "Homed",
      "'Homing complete' ends it and gives the buttons back")
check(str(hm.start_btn.cget("state")) == "normal", "  ...START really is back")

hm._home()
hm._on_line("[HOME] FAILED — never reached: RM within 30s")
check(hm.homing is False and hm.progress_var.get() == "Home failed",
      "a failure ends it too, reported as a failure")
check(str(hm.home_btn.cget("state")) != "disabled",
      "  ...otherwise HOME stays greyed out with nothing running")

# E-STOP has to clear it, or START stays refused against nothing.
hm._home()
hm._estop()
check(hm.homing is False and str(hm.start_btn.cget("state")) == "normal",
      "E-STOP clears the homing state, it does not strand the panel")
_app.messagebox.askokcancel = _real_ask
type(hm.link).is_open = _real_open

hm2 = ScannerApp(tk.Tk())
hm2._start()
check(str(hm2.home_btn.cget("state")) == "disabled",
      "HOME is disabled during a scan - it would drive RM out from under the sweep")
hm2._on_line("[SCAN_ABORT] stopped")
check(str(hm2.home_btn.cget("state")) == "normal", "  ...and comes back when it ends")

# The window scrolls, but the EMERGENCY STOP does not scroll away with it.
sc = ScannerApp(tk.Tk())
def _walk(w, out):
    out.append(w)
    for kid in w.winfo_children():
        _walk(kid, out)
tree = []
_walk(sc.root, tree)
check(any(isinstance(w, tk.Canvas) and "<Configure>" in w.bindings for w in tree),
      "there is a scrolling viewport, sized from its content")
check(any(isinstance(w, tk.Scrollbar) for w in tree), "  ...with a scrollbar on it")
estop = [w for w in tree
         if isinstance(w, tk.Button) and w.cget("text") == "EMERGENCY STOP"]
check(len(estop) == 1, "there is exactly one EMERGENCY STOP")
# Walk its ancestry: nothing between it and the root may be the scroller's
# inner frame. A stop control that can be scrolled off screen is its own
# hazard.
def _inside_canvas(widget):
    node = widget
    while node is not None and node is not sc.root:
        if isinstance(getattr(node, "master", None), tk.Canvas):
            return True
        node = getattr(node, "master", None)
    return False
check(not _inside_canvas(estop[0]),
      "  ...and it is PINNED outside the scroller, never scrollable off screen")
check(_inside_canvas(sc.start_btn) and _inside_canvas(sc.log_box),
      "  ...while the controls and the log do scroll")


print("\n=== 6. what happens when it goes wrong ===")
app2 = ScannerApp(tk.Tk())
app2._start()
check(app2.scanning is True, "a scan is running")
app2._on_line("[ERROR] SCAN refused - the machine is already moving.")
check(app2.scanning is False and app2.progress_var.get() == "Refused",
      "a refusal from the board clears the GUI's idea that a scan is running")
check(str(app2.start_btn.cget("state")) == "normal",
      "  ...otherwise START stays greyed out with nothing running")

app6 = ScannerApp(tk.Tk())
app6._on_line("[SCAN_SEEK] turning RM to its switch to reference the sweep...")
check(app6.progress_var.get() == "Finding the RM switch…",
      "seeking says so - the turntable IS moving, it is just not measuring yet")

app3 = ScannerApp(tk.Tk())
app3._start()
app3._on_line("[SCAN_ABORT] RM was stopped mid-sweep by a soft limit")
check(app3.scanning is False and app3.progress_var.get() == "Aborted",
      "an abort is reported as an abort, not as a finished scan")

app4 = ScannerApp(tk.Tk())
app4._start()
app4._estop()
check(app4.scanning is False, "E-STOP ends the scan in the GUI too")
check(app4.sim.running is False, "  ...and the board stops sweeping")

app5 = ScannerApp(tk.Tk())
logged = []
app5.log = lambda text, tag=None: logged.append(text)
for i in range(200):
    app5._on_line(f"[SCAN_PT] 1,{i}.00,300.00")
check(logged == [],
      "points are NEVER logged - 341 lines a layer would bury everything else")
app5._on_line("[SCAN_LAYER] 2/3 z=5.00 mm")
check(len(logged) == 1, "  ...but a layer change is worth one line")
check(app5.store.total == 200, "  ...but they all reach the store")


print("\n=== 7. the simulator speaks the firmware's language ===")
seen = []
sim = SimulatedBoard(seen.append)
sim.send("SCAN_START:5,10,2")
check(any(l.startswith("[SCAN_BEGIN]") for l in seen), "it opens with [SCAN_BEGIN]")
check(any(l.startswith("[SCAN_SEEK]") for l in seen),
      "  ...then goes to find the RM switch, like the board does")
check(any(l.startswith("[SCAN_REF]") for l in seen),
      "  ...and announces the switch as the reference")
check(any(l.startswith("[SCAN_LAYER] 1/2") and "dir=-" in l for l in seen),
      "  ...before sweeping AWAY from it")
for _ in range(200):
    sim.poll()
    if not sim.running:
        break
check(any(l.startswith("[SCAN_PT]") for l in seen), "it emits points")
check(any(l.startswith("[SCAN_LAYER] 2/2") and "dir=+" in l for l in seen),
      "the next layer turns back the other way")
check(sum(1 for l in seen if l.startswith("[SCAN_REF]")) == 2,
      "  ...and re-references when it arrives back on the switch")
check(any(l.startswith("[SCAN_DONE]") for l in seen), "and finishes with [SCAN_DONE]")

# The fake room is a RECTANGLE the machine stands off-centre in. Straight
# walls only come out straight if the polar mapping is right, so the shape
# is a running check on the plot; a circle would look identical however
# wrong the maths was. Opposite walls must add up to the room, which is the
# one invariant that does not depend on where the machine stands in it.
import statistics
def _wall(deg):
    hits = [sim._distance(deg, 0.0) for _ in range(41)]
    return statistics.median([h for h in hits if h >= 0])
check(abs((_wall(0) + _wall(180)) - 2 * sim.ROOM_HALF_X) < 3.0,
      "the simulated room is 800 mm across, measured through the machine")
check(abs((_wall(90) + _wall(270)) - 2 * sim.ROOM_HALF_Y) < 3.0,
      "  ...and 560 mm deep, whichever way it is measured")
check(abs(_wall(0) - _wall(180)) > 50.0,
      "  ...with the machine off-centre, so the four walls are four distances")
check(_wall(45) > _wall(0) and _wall(45) > _wall(90),
      "a corner is further away than either wall meeting at it")
seen.clear()
sim.send("SCAN_STOP")
check(any("phase=IDLE" in l for l in seen),
      "stopping an idle scanner says so rather than pretending to abort")

print("\n" + ("ALL SCANNER CHECKS PASSED" if not FAIL else "FAILURES: %s" % FAIL))
sys.exit(1 if FAIL else 0)
