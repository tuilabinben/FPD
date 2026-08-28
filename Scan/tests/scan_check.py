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
              "SET_SCAN_SENSOR:", "SET_SCAN_CAL:"):
    check(token in ino, f"the firmware answers {token}")
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
check(app._validate() == (4.0, 2.0, 3), "sane numbers validate")

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
seen.clear()
sim.send("SCAN_STOP")
check(any("phase=IDLE" in l for l in seen),
      "stopping an idle scanner says so rather than pretending to abort")

print("\n" + ("ALL SCANNER CHECKS PASSED" if not FAIL else "FAILURES: %s" % FAIL))
sys.exit(1 if FAIL else 0)
