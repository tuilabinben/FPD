<div align="center">

# 340° Scanner

**Sweep the turntable, read a distance sensor, step the lift up, sweep again.**

A small companion to the main Robot Motion Controller — same machine, same board, same cable.

</div>

---

## Quick start

```bash
python scan_launcher.py
```

Type how far the lift should **step up between sweeps**, press **START SCAN**.
With no board connected it runs simulated, so the whole thing can be demonstrated
away from the machine.

---

## What it does

One layer is a **340° sweep** of the turntable with the distance sensor sampled every
*n* degrees. At the end of a sweep the turntable rewinds to 0°, the lift rises by your
step, and the next layer starts. Stack the layers and you have the shape of whatever
surrounds the machine.

| Field | Meaning | Default |
| :--- | :--- | ---: |
| **Step up each scan** | how far ZM rises between layers — **the input this tool is for** | 5 mm |
| Layers | how many sweeps | 10 |
| Angular step | degrees between samples; 1° gives 341 points per layer | 1° |
| Sensor | ultrasonic or analog laser | ULTRASONIC |

The height hint under the step field is live: it tells you where the **last** layer tops
out and flags it in red before you press START if that is past the 285 mm stroke. Three
layers is two steps, not three — the lift only moves *between* sweeps.

---

## The board does the sweeping, not this app

The scan commands were added to **`RobotMotionController_v9_ClearCore`** — the same
firmware the main console drives. There is no second sketch, and that is the important
decision here:

> The sweep is driven through **the same jog primitives a key press uses**. Every soft
> limit, every PLC travel switch and the E-STOP path apply to a scan exactly as they
> apply to an operator holding a key. A separate sketch would have had to reimplement
> all of that, and would have got it wrong.

If a soft limit or a PLC switch stops the axis mid-sweep, the scan **aborts and says so**
rather than quietly finishing a layer that only covered a third of the circle.

The turntable **rewinds** between layers instead of sweeping back the other way.
Alternating would halve the scan time and put every other layer on the far side of the
drivetrain's backlash — which is the half a degree the scan exists to resolve.

Scanning runs RM at **20% of jog speed**. At full speed an ultrasonic read that times out
blocks for 30 ms, which is 3° of travel — wider than the features you are looking for.

---

## Wiring

| Signal | Pin | Note |
| :--- | :--- | :--- |
| Ultrasonic trigger | **IO-0** | 10 µs pulse out |
| Ultrasonic echo | **IO-1** | pulse width in, 30 ms timeout ≈ 5 m |
| Analog distance | **A-9** | 0–10 V or 4–20 mA sensor into an analog input |

IO-3/4/5 are the PLC limit lamps and are not available. IO-1 and IO-2 belong to the
opt-in rotary limit sensors — check `ENABLE_ROT_Z_LIMIT_SENSORS` in the firmware is
still `0` before wiring the echo there.

**A reading that failed comes back negative, never 0.** Zero millimetres is a legitimate
distance and "the echo never returned" is not, so a miss is plotted as a gap, counted
separately, and written into the CSV marked rather than dropped.

### Calibrating the analog sensor

An analog sensor reports **−1 until it is calibrated** — deliberately, rather than a
plausible-looking number nobody has reason to trust. Take two readings at known
distances, solve the straight line, and send:

```
SET_SCAN_CAL:<mm_per_count>,<offset_mm>
```

`SCAN_READ` takes a single reading without starting a sweep, which is how you aim the
sensor and check the calibration.

---

## Output

**SAVE CSV…** writes every point:

```
layer,z_mm,angle_deg,distance_mm,x_mm,y_mm,hit
1,0.00,0.00,391.68,391.680,0.000,1
1,0.00,1.00,-1.00,,,0
```

`x_mm`/`y_mm` are the point in the machine's own frame, so the file drops straight into
a plot or a point-cloud tool. Misses carry `hit=0` and no coordinates.

The polar view shows the current layer with the previous one ghosted behind it — a step
in the wall between two heights reads as the two outlines separating. **The radial scale
is chosen from the first layer and then frozen**, because a plot that rescales itself
every layer cannot be compared with the one above it by eye.

0° is straight ahead, where the arm points at RM 0. The dim wedge is the 20° the
turntable cannot sweep through; a gap there is the machine's shape, not a sensor fault.

---

## Commands added to the firmware

| Command | |
| :--- | :--- |
| `SCAN_START:<zStepMm>,<degStep>,<layers>[,<sweepDeg>]` | begin |
| `SCAN_STOP` | abort, stopping the axes |
| `SCAN_STATUS` | phase, sensor, layer, points, calibration |
| `SCAN_READ` | one reading, no motion |
| `SET_SCAN_SENSOR:ULTRASONIC\|ANALOG` | pick the sensor |
| `SET_SCAN_CAL:<mmPerCount>,<offsetMm>` | analog calibration |

| Reply | |
| :--- | :--- |
| `[SCAN_BEGIN] sensor=… layers=… zStep=… degStep=… sweep=… fromRot=… fromZ=…` | accepted |
| `[SCAN_LAYER] <n>/<N> z=<mm> mm` | a layer started |
| `[SCAN_PT] <layer>,<deg>,<mm>` | one sample; negative mm is a miss |
| `[SCAN_DONE] <N> layers, <n> points` | finished |
| `[SCAN_ABORT] <why>` | stopped early, and why |

`SCAN_START` is refused outright if the machine is already moving, if a scan is already
running, if any parameter is out of range, or if the last layer would need more lift
than the stroke has.

---

## Tests

```bash
python tests/scan_check.py
```

Covers the wire format against the firmware source, the parsing, miss handling, the CSV,
the polar mapping, validation, and a whole simulated three-layer scan. The firmware side
is covered by the main project's `tests/run_tests.sh`, section S.

---

## Layout

| Path | |
| :--- | :--- |
| `scan_launcher.py` | entry point |
| `scanner/config.py` | constants and the wire strings shared with the board |
| `scanner/link.py` | serial link, and the simulator that speaks the same lines |
| `scanner/store.py` | the points, and the CSV writer |
| `scanner/plot.py` | the polar view |
| `scanner/app.py` | the window |
