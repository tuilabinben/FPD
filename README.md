<div align="center">

# Robot Motion Controller

**Desktop control software for the STCR4000S frog-leg wafer-handling robot.**

Python · Tkinter · ClearCore firmware · Mitsubishi MC protocol

`v1.0` — [Download the Windows build](https://github.com/tuilabinben/FPD/releases/tag/v1.0)

</div>

---

## Quick start

**Just run it** — download `RobotMotionController-v1.0.exe`, double-click. No install, no Python.
Windows SmartScreen will warn on first launch (the build is unsigned): *More info → Run anyway*.

**From source:**

```bash
python robot_simulator_launcher.py
```

| Requirement | |
| :--- | :--- |
| Python 3.10+ with Tkinter | required |
| `pyserial` | required for a real board — the GUI runs without it, simulated |
| `pillow` | optional — anti-aliased rounded corners via 4× supersampling |

The app declares DPI awareness before the first widget, so it draws on the monitor's real pixel
grid instead of being bitmap-stretched by Windows. See [`robot_sim/hidpi.py`](robot_sim/hidpi.py).

> **No hardware? It still works.** With nothing connected the GUI simulates the machine —
> jog, IK, P2P programs, boundaries and the XY board all behave. Every screen is reachable.

---

## Contents

[Overview](#overview) · [The machine](#the-machine) · [Control modes](#control-modes) ·
[Speed](#speed) · [Arm angles](#arm-angles) · [Boundaries](#boundaries-and-the-coordinate-reference) ·
[PLC link](#the-plc-link) · [Calibration](#calibration) · [Keys](#keys) ·
[Settings](#settings) · [Serial protocol](#serial-protocol) · [Safety](#safety-design) ·
[Tests & building](#tests-and-building) · [Repo map](#repo-map)

---

## Overview

One operator console, two ways to drive the machine, sharing a single serial connection:

- **P2P** — absolute X/Y/Z targets, full inverse kinematics, four-leg programs
- **Joystick** — held-key jog on RM / A1M / A2M / ZM

Both modes share the live pose, the connection, the boundaries and the event log. Switching
mode auto-stops any motion, and the firmware refuses conflicting commands on its own as a
backstop.

Three status lamps sit in the connection row:

| Lamp | Answers |
| :--- | :--- |
| **COM PORT** | is the serial port open |
| **CLEARCORE IO0** | is the board answering |
| **PLC LINK** | is Mitsubishi device data arriving — `NO REPLY` and `UNREACHABLE` are different faults |

---

## The machine

Four axes. **HOME is the minimum of all four**, and it is the origin for everything.

| Axis | Travel | Notes |
| :--- | :--- | :--- |
| **RM** turntable | 0 – 340° | 0° is the CCW stop and is HOME. The 20° wedge between 340° and 360° is unreachable |
| **A1M / A2M** elbows | separate motors | frog-leg pair, one per deck |
| **ZM** lift | 0 – 285 mm | measured from HOME, never negative |

Reach of the wafer centre from the turntable axis:

| Pose | Reach |
| :--- | ---: |
| HOME, arm folded | **240 mm** |
| Rated working reach | **575 mm** |
| Arm straight (singularity) | **605 mm** |

Cartesian **X, Y** are measured from the turntable axis and are signed; **Z** is height above
HOME. `X 0, Y 0` is the centre of rotation — a reference point, not a reachable target.
The two decks sit 9 mm apart, applied per arm inside the conversion, so both arms take the
**same** Z.

---

## Control modes

| | P2P | Joystick | Scan |
| :--- | :--- | :--- | :--- |
| **Target** | X/Y/Z or joint angles | direction, per axis | a sweep per slice, stacked |
| **Runs** | `HOME → A → B → HOME` | while the key is held | until the last slice, or STOP |
| **PLC limit switch** | **refuses** a leg driving further in | **warns**, does not block | RM's switch is the *reference* — refused without it |
| **Path** | joint-space (bows off the straight line) | — | RM sweeps, ZM steps up, RM sweeps back |

### Scan

The turntable sweeps, a distance sensor reads, the lift steps up, and it sweeps back —
stack the slices and you have the shape of whatever surrounds the machine. It is a **mode
in this console**, driving the same board over the same link; the stand-alone `Scan/` tool
still exists and is the one that runs *simulated*, away from the machine.

**You give four numbers, not a speed:**

| Field | |
| :--- | :--- |
| **Sensor sample rate** | what the sensor can deliver, Hz |
| **Points per slice** | how finely one slice is read |
| **Slices** | how many heights |
| **Slice spacing** | how far ZM rises between them |

```
seconds a slice  = points / rate
RM sweep speed   = sweep / seconds a slice
angular step     = sweep / points
ZM travel        = spacing × (slices − 1)
```

So at 50 Hz, 50 points over a 330° sweep is **330° in one second**; 100 points is two
seconds. The board is sent the step *and* the speed (`SCAN_START`'s fifth field) and clamps
a speed past what RM can do — the points still land at the same angles, the sweep just takes
longer, and both the panel and the board say so.

Two things **warn and ask** rather than refusing, because both are the operator's own
limits: a total ZM travel past the ceiling in **Settings → Scan** (default **180 mm**), and a
speed past RM's. The hard refusals stay on the board — the 285 mm stroke, and no PLC device
data.

`SAVE CSV…` writes `layer,z_mm,angle_deg,distance_mm,x_mm,y_mm,hit`; a miss is written
marked, never dropped, and never drawn at radius 0.

The **Oxy board** in P2P plots the reachable annulus, the unreachable RM wedge, the taught RM
band, HOME, A, B and the live pose. The A→B line is drawn straight because that is the
operator's *intent*; the machine's real path bows away from it, and the caption says so.

---

## Speed

One fixed reference speed. Each axis gets a percentage of it — that is the only knob.

```
axis motor RPM = 150 × (axis % / 100) × AXIS_SCALE
```

| Axis | Scale | Default | Motor RPM | Real speed |
| :--- | ---: | ---: | ---: | :--- |
| **A1M / A2M** | 1.00 | 125% | 187.5 | quoted in motor RPM — see [Arm angles](#arm-angles) |
| **RM** | 1.00 | 75% | 112.5 | 103.8 °/s (`i_RM` = 6.5) |
| **ZM** | 0.75 | 50% | 56.25 | 18.75 mm/s (20 mm per rev) |

- Settings shows the real RPM and derived speed beside each field as you type.
- **There is no percentage cap and no warning** — what you type is what is sent. Engineering
  backstops still clamp underneath: `ROT_VEL_MAX_DEG_S` 120, `Z_VEL_MAX_MM_S` 140,
  `ARM_MOTOR_RPM_MAX` 400. The preview prints `▸ capped` when one bites.
- The 1% floor stays: 0% would freeze an axis and a negative would reverse it.
- Acceleration is a **separate** family of percentages against 375 RPM/s, so the ramp can be
  tuned without touching cruise speed.

---

## Arm angles

Three numbers describe the same elbow. Knowing which one you are looking at matters.

| Frame | HOME | Rated reach | Straight | Where it appears |
| :--- | ---: | ---: | ---: | :--- |
| **Base angle** | 0° | 90° | 110.4° | the panels — what the operator reads |
| **Frog-leg (fold)** | 0° | 146.68° | 180° | the `fold …° · R = … mm` line |
| **Motor degrees** | 0° | 1144° | 1404° | the wire, and every taught elbow boundary |

`ARM_GEAR_RATIO = 7.80` motor degrees per frog-leg degree. The board counts step pulses and
nothing else, so **motor degrees are the only figure it knows exactly** — the other two are
derived. That is why taught elbow limits are stored in motor degrees: re-calibrating the ratio
must never invalidate a boundary somebody walked the machine to.

> **Pending change.** The base angle is being re-zeroed to read **−30° at HOME and +60° at full
> extension** — the same 90° of travel, measured from a different zero, matching the MATLAB
> model. It is blocked on one bench measurement (reach at full extension). Until then the panels
> read 0 – 90°.

Settings files carry a `_schema` (currently **4**). When a stored value changes *meaning*, the
schema bumps and the affected keys are **dropped with a warning** rather than silently
converted — re-teaching an elbow boundary is two SET HERE presses.

---

## Boundaries and the coordinate reference

The factory envelope is what the structure allows. Your installed envelope — cassette, chamber
port, cable loop — is narrower, and it is yours to set.

**Settings → Boundaries.** Type a number for ZM and RM, or jog to the physical stop and press
**SET HERE**.

- **Elbow rows are SET HERE only.** The reported angle rides on a gear ratio; typing `90°`
  types against a scale you have not measured. *"Wherever the arm is standing"* is exact.
- **The elbow pair is unordered** — teach either end first. Values are stored raw and sorted
  where they are read.
- **Each arm has its own pair.** Sharing one is how A2M once got driven past its stop while
  A1M's angle was the one being checked.
- ZM and RM keep an envelope and an ordering rule, because for those two the scale is known.

Two switches decide whether a boundary bites, and they **AND** — turning the master back on
never re-arms an axis someone switched off on its own:

| Control | Scope |
| :--- | :--- |
| per-axis `ENFORCED` | this boundary |
| master `LIMITS ENABLED` | all four at once |

Every clamp is **directional**: an axis already outside a boundary can always be jogged back
in. No pose can pin the machine.

**RESET COORDINATES** zeroes one axis counter where it stands. It measures nothing — it trusts
that you jogged to the reference. It confirms first and is refused while anything moves. The
buttons live in the motion panels, in **both** modes, because declaring a reference is a
jogging job.

> **Teach after homing, not before.** HOME re-zeroes the counters, and a boundary taught
> beforehand keeps its number while losing its meaning. This is deliberately not auto-corrected:
> after a PLC home the offset is genuinely unknown to the board, and inventing one would move a
> safety boundary to a place nobody chose.

**Saved limit sets** — name and store whole envelopes (tight for production, wide for
maintenance) in `limit_presets.json`. LOAD fills the form; you still press APPLY to send it.

---

## The PLC link

A Mitsubishi PLC at **192.168.3.101:1025**. ClearCore is an **MC protocol 3E** client and it is
**read-only** — it writes nothing, ever.

| Device | Meaning | Sits at |
| :--- | :--- | :--- |
| `M32` | ZM travel limit | minimum — bottom of the stroke |
| `M31` | RM travel limit | maximum — the axis is mounted inverted |
| `M30` | A2M travel limit | **both ends** — see below |
| `X0` | HOME request | a **physical wire** from ClearCore IO-0 |

**HOME is a wire, not a packet.** `X0` is an input device: the PLC refreshes X from the
terminals every scan, so a network write to it was overwritten within 10 ms. It worked only
while X0 was unwired, and the failure looked like a ClearCore fault.

**A2M's switch is wired at both ends of its travel** — one device, two switches, so the bit
cannot say which end tripped it. The board records the direction the axis was travelling on the
rising edge and refuses only that direction; the opposite always stays open, so the arm is never
pinned on its own switch. A trip at the far end is explicitly **not** the home reference.

**HOME STATE** is all three devices covered. It is the one condition allowed to zero the
counters unasked — edge-triggered, and refused while anything moves.

> Unknown never renders as CLEAR. A dead link shows `NO DATA` in purple, not four clear lamps.
> That distinction was a real field bug: a switch was physically ON while the panel said CLEAR.

---

## Calibration

Two constants turn counts into real units and **neither has been measured on the bench**. Both
are settable at runtime — no re-flash.

| What | Assumed | How to measure | Apply |
| :--- | :--- | :--- | :--- |
| **ZM lead** | 20 mm/rev | rule on the carriage, command 100 mm (not 10), measure | `SET_Z_LEAD:<mm>` |
| **Elbow ratio** | 7.80 motor°/fold° | mark the elbow, command known revolutions, divide by the angle swept | `SET_ARM_RATIO:<r>` |
| **RM ratio** | 6.5 | command 90°, measure the angle actually swept | `SET_ROT_RATIO:<r>` |

- A **non-power-of-two** error in Z points at the mechanics, not the driver — microstep switches
  can only ever be wrong by powers of two.
- A wrong Z lead also moves where every ZM soft limit physically sits, because those are stored
  in millimetres.
- The elbow ratio does **not** affect jog speed — that is bounded in motor RPM, which is
  ratio-free. It scales every reported angle, every reach figure, and every absolute arm move.

---

## Keys

`A` `D` — RM  ·  `I` `K` — A1M  ·  `O` `L` — A2M  ·  `W` `S` — ZM

Rebind any of them in **Settings → Controls**: click a box, press the key. If it is already
taken the two rows **swap**, so no axis is ever left unbound. Saved to `keybinds.json`; a
corrupt file is discarded whole rather than half-merged.

**Reserved, cannot be rebound:** `SPACE` E-STOP · `ESC` Settings · `BACKSPACE` HOME.

Jog is keyboard and the on-screen pads. There is no controller support — it was removed on
request.

---

## Settings

Six tabs — **Speed · Boundaries · Scan · Controls · PID · Appearance** — each with its own APPLY and
DEFAULTS acting **only on that tab**. A global reset that wiped taught boundaries because
someone undid a speed change is the failure this avoids.

- Nothing applies on keystroke. Edits stage until APPLY.
- `ESC` opens and closes the window; refused while a program runs.
- PID gains are individually lockable — locking Kd freezes Kd alone, and DEFAULTS skips it.
- **RESTART** (connection row) relaunches in place via `os.execv`, after `BYE` and a clean port
  close. Confirmed, and refused while anything moves.

<details>
<summary><b>Seven colour schemes</b></summary>

| | Scheme | |
| :-- | :--- | :--- |
| 1 | **Graphite** *(default)* | near-monochrome dark, near-white accent |
| 2 | Slate Blue | cool greys, steel blue |
| 3 | Nordic Light | light theme, muted blue — for bright rooms |
| 4 | Warm Paper | warm off-white, muted clay |
| 5 | Carbon Orange | very dark, industrial orange |
| 6 | Pure Mono | greyscale chrome |
| 7 | Mint | the previous default, kept for reversibility |

Applies immediately — the window rebuilds in place, keeping the serial connection, every field
value and the event log. **Refused while the machine moves**: the rebuild briefly destroys the
jog pads and the E-STOP button.

</details>

---

## Serial protocol

USB serial, **115200 baud** by default, two-way ASCII lines.

<details>
<summary><b>Commands — host → board</b></summary>

| Command | |
| :--- | :--- |
| **Link** | |
| `PING` · `BYE` | heartbeat probe · graceful disconnect |
| `STATUS` · `PROFILE` · `LIMITS` | report state · derived speeds · active limits |
| **Speed & PID** | |
| `SET_SPEED:rpm,acc,rotPct,armPct,zPct` | master RPM and per-axis percentages |
| `SET_PID:kp,ki,kd[,N]` · `PID_ON` · `PID_OFF` · `PID_RESET` | gains and switching |
| **Boundaries** | |
| `SET_LIMIT:axis,MIN\|MAX,value` | axis = `Z` `ROT` `A1` `A2` |
| `SET_LIMIT_HERE:axis,MIN\|MAX` | capture the current position |
| `SET_LIMIT_ENFORCE:axis,0\|1` · `SET_LIMITS_ENABLED:0\|1` | per-axis · master |
| `RESET_LIMITS` | back to the inset defaults |
| **Reference** | |
| `RESET_COORD:<Z\|ROT\|A1\|A2>` | zero one counter; does **not** claim a full reference |
| `HOME` | PLC homing, via the IO-0 → X0 wire |
| **P2P** | |
| `LOAD:…` · `LOAD_BOTH:…` | joint-space program, single or dual arm |
| `MOVE_XYZ:arm,X,Y,Z` · `LOAD_XYZ:…` | Cartesian — board runs IK. **HOME = X0 Y0 Z0** |
| `LOAD_XYZ_BOTH:Xa,Ya,Za,Xb,Yb,Zb` | dual arm — **Za must equal Zb** (one carriage) |
| `IK:arm,X,Y,Z` · `FK:d1,rot,a1,a2,arm` | compute only, no motion |
| `RUN` · `STOP` · `ESTOP` | execute · halt · emergency stop |
| **Jog** | |
| `ROT_CW` `ROT_CCW` `ROT_STOP` | turntable |
| `A1_FWD` `A1_BACK` `A1_STOP` | arm 1 |
| `A2_FWD` `A2_BACK` `A2_STOP` | arm 2 |
| `ARM_FWD` `ARM_BACK` `ARM_STOP` | both elbows |
| `Z_UP` `Z_DOWN` `Z_STOP` | lift |
| `JOG_HB` | dead-man keep-alive, ~150 ms |
| `MOVE_A1:<deg>` · `MOVE_R1:<mm>` | absolute elbow angle · radial reach |
| `SET_BOOST:<×>` | temporary jog multiplier |
| **Calibration** | |
| `SET_Z_LEAD:<mm>` · `SET_ARM_RATIO:<r>` · `SET_ROT_RATIO:<r>` | no re-flash |
| **PLC** | |
| `PLC_STATUS` · `PLC_TEST` · `PLC_RECONNECT` | poll · one blocking diagnostic read · drop and redial |
| `SET_PLC_LINK:0\|1` · `SET_PLC_POLL:<ms>` · `PLC_DEBUG:0\|1` | disable · poll rate · echo frames |
| `SET_PLC_SENSOR_ENFORCE:axis,0\|1` | switch off **one broken** limit switch |

</details>

<details>
<summary><b>Feedback — board → host</b></summary>

| Line | |
| :--- | :--- |
| `PONG` · `[CONNECTED]` | heartbeat reply · IO0 handshake |
| `[CLEARCORE POS] …` · `[JOG POS] …` | live pose, P2P and jog |
| `[SPEED] …` · `[PROFILE] … (CLAMPED)` | configuration · derived speeds, `CLAMPED` when a ceiling bites |
| `[LIMITS] …` · `[LIMIT_SET] …` · `[LIMIT_ENFORCE] …` | active band · accepted · switched |
| `[LIMIT] Z_UP …` | an axis hit a soft limit and stopped |
| `[PLC_STATE] … data=OK conn=n/m limit Z/R/A2=… end Z/R/A2=…` | link health, switch bits, which end each is refusing |
| `[PLC_LIMIT] …` | an axis was stopped by a PLC switch |
| `[PLC_HOME] …` · `[HOME] …` | homing handshake |
| `[IK] …` · `[FK] …` · `[SINGULARITY] …` | kinematics results · advisory near straight-arm |
| `[RUN] TARGET REACHED` · `[ESTOP] EMERGENCY STOP` | move complete · stop acknowledged |
| `[WATCHDOG] …` | jog stopped — no keep-alive from the host |
| `[WARN] …` · `[ERROR] …` | interlock fired · malformed or out-of-range |

`[LIMITS]` is logged and never parsed back. **The GUI is the sole system of record** for
boundaries — the board holds them in RAM only and is re-sent them on every handshake.

</details>

---

## Safety design

Defence in depth, enforced independently in the GUI and in the firmware.

- **One E-STOP path.** `emergency_stop_all` — the button on *both* motion panels and `SPACE`
  all call it. One audited function beats several similar ones. Neither button is disabled
  while the machine moves, which is exactly when it has to work.
- **Mode switching auto-stops** any active motion; the firmware refuses conflicting commands on
  its own if GUI and board ever desync.
- **`PING` always gets an immediate `PONG`**, even mid-motion, so the heartbeat cannot time out
  because the board is busy.
- **Jog is dead-man.** It moves only while held, and stops on a missed keep-alive.
- **P2P enforces, jog warns.** A program runs unattended with nobody watching that axis, so it
  must not start. Jog is how you come off a switch — blocking it would pin the machine.
- **Unknown is never CLEAR.** A dead PLC link marks the sensors unknown, and nothing
  auto-resets on a dead connection.
- **Nothing the board reports ever writes a boundary.** Boundaries come from the operator only.

The board's IO0 LED is an **activity** light, not a steady "connected" light: every command in
flashes short, every line out flashes longer. A live link flickers; a dead one goes dark. All
timing is `millis()`-based, never `delay()`, so motion and serial are never blocked.

---

## Tests and building

```bash
tests/run_tests.sh
```

| Suite | Covers | How |
| :--- | :--- | :--- |
| `tests/firmware_check.cpp` | IK, limits, PLC frames, sensors, homing | compiles the `.ino` against a desktop ClearCore shim, asserts on the serial text |
| `tests/python_check.py` | kinematics, boundaries, speed, panels, protocol parsing | drives the app through a headless Tk stub |

Run them before and after every change. They live inside the repo deliberately — an earlier
generation lived in `/tmp` and vanished with a machine restart, taking the only evidence the
firmware worked with it.

**Building the executable:**

```bash
python -m PyInstaller --noconfirm --clean --onefile --windowed --name "RobotMotionController-v1.0" --hidden-import serial.tools.list_ports_windows robot_simulator_launcher.py
```

A frozen build writes its settings to `%APPDATA%\RobotMotionController`, so taught boundaries
and keybinds survive both a restart and swapping in a newer exe. A source run keeps them beside
the package instead.

---

## Repo map

| Path | |
| :--- | :--- |
| `robot_sim/` | the GUI — `config.py` holds every constant, `kinematics.py` the maths |
| `robot_sim/core/` | one mixin per concern: protocol, jog, P2P, safety, serial, keyboard |
| `robot_sim/ui/` | panels and the settings dialog |
| `RobotMotionController_v9_ClearCore/` | the firmware — one `.ino`, plus `FIRMWARE_NOTES.md` |
| `Scan/` | the stand-alone scanner — same firmware commands, and the only one that can run simulated |
| `ClearCore_PLC_Test/` | standalone PLC link probe, for bringing the Mitsubishi up on its own |
| `tests/` | both suites and their stubs |
| `CLAUDE.md` | why the non-obvious decisions were made — read before changing behaviour |
