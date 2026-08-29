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

> **What the simulation draws.** A rectangular chamber, 800 x 560 mm, with the machine
> standing off-centre in it — so the plot shows four straight walls and four corners.
> That shape is deliberate: straight walls only come out straight if the polar mapping
> is right, so the picture checks itself. Anything you see there is the fake room, not
> a sensor.

### SIM: ON / OFF

Simulation is a **switch in the connection row**, not a silent fallback.

| | Lamp | START with no board |
| :--- | :--- | :--- |
| **SIM: ON** *(default)* | `● SIMULATED` | runs, and every point is made up |
| **SIM: OFF** | `● NO BOARD` | **refused** — nothing is sent, nothing is stored |

Turn it off for real work. Simulated points are the same numbers in the same columns as
measured ones, so the app marks them everywhere it can: the plot caption reads
`SIMULATED DATA`, and SAVE offers `scan_SIMULATED.csv` rather than `scan.csv`. The
switch is refused mid-scan — flipping it half way through would change what the numbers
in one file mean.

---

## What it does

Pressing START turns the turntable until it reaches the **RM travel switch**, and that is
where the scan is referenced from. Then:

1. **Sweep away from the switch** by the sweep angle, sampling the distance sensor every
   *n* degrees
2. **Lift** by your step
3. **Sweep back**, until the switch is reached again — sampling all the way
4. **Lift**, and repeat, alternating direction

Stack the layers and you have the shape of whatever surrounds the machine.

| Field | Meaning | Default |
| :--- | :--- | ---: |
| **Step up each scan** | how far ZM rises between layers — **the input this tool is for** | 5 mm |
| Layers | how many sweeps | 10 |
| Angular step | degrees between samples; 1° gives 341 points per layer | 1° |
| **Sweep** | how far round each layer goes, 1–340° | 340° |
| Sensor | ultrasonic or analog laser | ULTRASONIC |

The height hint under the step field is live: it tells you where the **last** layer tops
out and flags it in red before you press START if that is past the 285 mm stroke. Three
layers is two steps, not three — the lift only moves *between* sweeps.

**The sweep may be shorter than 340°, never longer.** Scanning one wall is a real job and
a 90° sweep takes a quarter of the time. Longer is refused by both the panel and the
board, because the turntable cannot turn past its own stop — the extra degrees would be
spent driving into the RM soft limit and the layer would abort part-way. The hint under
the field counts the points a layer will hold, which the angular step alone cannot tell
you.

### HOME

**HOME** asks the board to run the PLC home cycle — the same one the main console runs,
not a second homing path, so a machine homed from here is homed for both applications. It
is here because a scan wants a reference and this is where you are standing when you find
out you have not got one.

It is **confirmed**, unlike START: it drives all four axes to their switches on the PLC's
schedule, and you may be at the panel rather than at the machine. It is **board-only** —
the simulator has no PLC to ask, and a pretend home would report a reference the machine
has not got. It is disabled during a scan, because it would drive RM out from under the
sweep, and E-STOP clears it.

### The window scrolls — except the top bar

On a short screen the plot and the log were cut off with no way to reach them. Everything
below the top bar now sits in a scrolling viewport; the wheel works while the pointer is
over it, and is released on the way out so it cannot silently change a value in a focused
combobox.

**The top bar is pinned deliberately.** EMERGENCY STOP lives on it, and a stop control
that can be scrolled off the screen is its own hazard — the same argument that keeps the
e-stop on both motion panels of the main console. The PLC and RM lamps ride with it for a
different reason: they answer "can a scan start at all", which should not need scrolling
to.

### The sensor indicator

Under the sensor picker: the **last reading**, and how many of the last 50 were misses.

* `● no reading yet` — nothing has been read. Not a healthy-looking `0 mm`.
* `● 412.2 mm` — the last reading, in green.
* `● NO ECHO` — that reading came back negative.

**TEST READ** takes a single reading with the machine standing still, which is how you
aim the sensor and check an analog calibration before committing to a sweep.

The miss counter is what separates *one lost echo* from *a sensor that is not plugged
in*. A wall of misses says so in red and names the three things it is ever caused by
(wiring, sensor kind, calibration); one in twenty stays quiet, because a lamp that cries
wolf gets ignored. It counts **recent** readings only, so it recovers as soon as the
sensor does, and it starts fresh at every START — last scan's health is not this scan's.

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
rather than quietly finishing a layer that only covered a third of the circle. A return
leg is the exception — being stopped there is arrival, not a fault.

**Every layer is referenced to the switch, not to a degree count.** A return leg ends
when the switch trips, whatever the counter says, so the turntable is re-squared every
other layer and the start angle cannot drift over a tall scan.

Sweeping in **alternating** directions means the return leg collects a layer instead of
being dead rewind travel, so a scan takes half as long. The cost is a fixed backlash
offset between odd and even layers — a constant, measurable and removable afterwards,
unlike drift, which is not.

The scan is **refused outright** if there is no PLC device data or if RM's switch has
been disabled: without the switch there is no frame to sweep in, and two scans taken on
different days would have angle columns that mean different things.

### The PLC lamp and the RM switch lamp

Both refusals above are now **on the panel**, beside the link lamp, instead of arriving
as an `[ERROR]` after START with the machine standing still.

| PLC lamp | Means |
| :--- | :--- |
| `NO LINK` | this app has heard nothing from a board yet |
| `CONNECTED` | device reads are landing — a scan can start |
| `NO REPLY` | socket opens, MC protocol does not answer. Almost always MC protocol not enabled on that port |
| `UNREACHABLE` | the socket has never opened at all: cable, subnet or address |
| `DISABLED` | `SET_PLC_LINK:0`, off on purpose — never shown as a fault |

**The lamp reports device data, not the socket.** A lamp following the socket flapped
green/red every few seconds on the machine, because the board drops and reopens the
socket on every reply timeout — that is a resynchronisation, not a fault. `NO REPLY` and
`UNREACHABLE` are kept apart because they send you to different places: a socket that has
opened even once proves the cable and the address are fine.

| RM switch lamp | Means |
| :--- | :--- |
| `?` | no device data — **not** the same as clear |
| `ON` | the switch is covered; this is the scan's reference position |
| `CLEAR` | the switch is not covered |
| `OFF` | enforcement switched off on the board. **Red** — it stops a scan outright |

`?` never renders as `CLEAR`. On the main console a dead link once showed four clear
lamps while a switch was physically covered, and on a safety display the failure read as
good news. Losing the serial link marks both lamps unknown rather than leaving a stale
`CONNECTED` standing.

START refuses locally only where the board is **certain** to refuse anyway — RM's switch
off, or no device data — and names the fix. It does **not** block on `?`: unknown means
this app has no news, not that the board has none. A stale link asks before starting
rather than refusing, because the board keeps its last good status word and may well
accept the scan.

The board **pushes** `[PLC_STATE]` whenever the status word changes, so the lamps are
event-driven; the app also asks once on connect and every 5 s after, because a link that
is healthy and steady produces no change to push. The simulator answers `NO DEVICE DATA`
rather than faking a green lamp — the same rule the SIM switch follows.

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
| `[SCAN_BEGIN] sensor=… layers=… zStep=… degStep=… sweep=… fromZ=…` | accepted |
| `[SCAN_SEEK] …` | turning to find the RM switch; nothing measured yet |
| `[SCAN_REF] RM on its switch at <deg>` | referenced — every layer is measured from here |
| `[SCAN_LAYER] <n>/<N> z=<mm> mm dir=<+\|-> from=<deg>` | a layer started, and which way it turns |
| `[SCAN_PT] <layer>,<deg>,<mm>` | one sample; negative mm is a miss |
| `[SCAN_DONE] <N> layers, <n> points` | finished |
| `[SCAN_ABORT] <why>` | stopped early, and why |

`SCAN_START` is refused outright if the machine is already moving, if a scan is already
running, if any parameter is out of range, if the last layer would need more lift than
the stroke has, if there is no PLC device data, or if RM's switch is disabled.

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
