# frfrfrdrrd — Robot Motion Controller

```
python robot_simulator_launcher.py
```

> **Optional:** `pip install pillow` — enables anti-aliased rounded corners via 4× supersampling. Without it the app still runs, just with aliased arc-based corners.

> **DPI-aware.** The app declares DPI awareness before the first widget, so it draws on your monitor's real pixel grid instead of being bitmap-stretched by Windows. See `robot_sim/hidpi.py`.

---

## Table of Contents

1. [Overview](#overview)  
2. [Control Modes](#control-modes)  
3. [Speed System](#speed-system)  
4. [Arm Angle Convention](#arm-angle-convention)  
5. [Boundaries & Coordinate Reference](#boundaries--coordinate-reference)  
6. [Calibration](#calibration)  
7. [Jog Keys & Gamepad](#jog-keys--gamepad)  
8. [Appearance](#appearance)  
9. [Settings](#settings)  
10. [Serial Protocol Reference](#serial-protocol-reference)  
11. [Safety Design](#safety-design)  
12. [Tests](#tests)  

---

## Overview

A desktop control application for the STCR4000S frog-leg robot, built on Python/Tkinter with a ClearCore firmware backend. It merges two control modes on one shared connection:

- **Point-to-Point (P2P):** Absolute X/Y/Z target motion with full inverse kinematics
- **Joystick:** Held-key (or gamepad) jog on RM / A1M / A2M / ZM axes

Both modes share:
- One serial connection panel with 3-LED status (COM Port / ClearCore IO0 / Heartbeat)
- PING/PONG heartbeat with missed-beat tracking
- Unified Event Log (`>>` sent / `<<` received, color-tagged)
- Software-only simulation fallback when no hardware is connected — both modes remain fully testable

---

## Control Modes

| Mode | What it does | How targets are specified |
|:-----|:-------------|:------------------------|
| **P2P** | Moves to an absolute position | X, Y, Z coordinates (Cartesian) or direct joint angles |
| **Joystick** | Continuous jog while a key/button is held | Per-axis direction (CW/CCW, FWD/BACK, UP/DOWN) |

Switching modes auto-stops any active motion. The firmware independently refuses conflicting commands as a backstop.

---

## Speed System

The reference speed is a **fixed 150 motor RPM** (not editable). Each axis has one speed knob: its **percentage**.

```
axisMotorRpm = 150 × (axisPercent / 100) × AXIS_RPM_SCALE
```

`AXIS_RPM_SCALE` is a calibration constant that accounts for different gearing across axes:

| Axis | Drivetrain | Scale | Default % | Resulting Speed |
|:-----|:-----------|------:|----------:|:----------------|
| A1M / A2M | Gear ratio unmeasured | 1.000 | 125% | 187.5 motor RPM |
| RM | `i_RM = 4.375 × 6.5 = 28.4375:1` | 1.000 | 75% | 23.74 °/s |
| ZM | 20 mm per motor rev | 0.750 | 50% | 18.75 mm/s |

- Settings shows the real motor RPM beside each percentage as you type.
- Above the tested default, the field turns **amber** with a warning about potential lost steps, overshoot, and vibration on an open-loop drive. Higher values are still accepted after confirmation.
- There is no hard percentage cap. Each axis still has a real backstop underneath: `ROT_VEL_MAX_DEG_S`, `Z_VEL_MAX_MM_S`, or `ARM_MOTOR_RPM_MAX`.
- The 1% floor stays — 0% would freeze an axis, and negatives would reverse it.

---

## Arm Angle Convention

`A1M_POS` / `A2M_POS` read **0° at home** and count up as the elbow extends outward:

| Pose | Old (`th3_cad`) | Current (from home) |
|:-----|----------------:|--------------------:|
| Home / fully retracted | 60° | **0°** |
| Straight arm (singularity) | 180° | **120°** |
| JEL drawing limit (575 mm reach) | 151.72° | **91.72°** |
| Singularity warning | 170° | **110°** |
| Factory elbow band | 60°–180° | **0°–120°** |

**Why the change:** The board counts motor steps from wherever it was referenced and scales them by `ARM_GEAR_RATIO`. It could never produce a real `th3_cad` — the `60°` it used to print at home was zero rotation wearing a CAD label. Reporting rotation-from-home is the same number with an honest name, and it reads the way an operator thinks: home is zero, and it counts up.

The `th3_cad` frame still exists internally inside `fold_angle_to_reach()` / `reach_to_fold_angle()`, which add and remove `ARM_ZERO_CAD_DEG = 60`. No other code references it.

> **Settings migration:** Saved files carry a `_schema` version. Old elbow limits (written in the `th3_cad` frame) are **dropped with a warning** — they cannot be safely converted because they were produced with an unmeasured gear ratio. Re-teaching them is two SET HERE presses per arm. Speed and ZM/RM boundaries are kept, since their units didn't change.

---

## Boundaries & Coordinate Reference

The factory envelope is what the structure allows. The operator's installed environment is usually narrower, so those limits are yours to set:

- **Settings → Boundaries** has min/max boxes for ZM and RM, plus a **SET HERE** button that captures the machine's current position — jog to the physical limit, press it, done.
- **Elbow rows are SET HERE only.** Because the reported angle depends on `ARM_GEAR_RATIO` (unmeasured), typing `90°` means nothing. *"Wherever the arm is standing right now"* is exact regardless.
- **The elbow pair is unordered.** Jog to one stop, SET HERE; jog to the other, SET HERE. The software sorts them where they are read, not where they are stored.
- **Each arm has its own pair.** Sharing one limit is how A2M used to be driven past its stop while A1M's angle was checked.
- ZM and RM pairs cannot invert or collapse — that would make every position illegal with no way to jog back out.

**RESET COORDINATES** zeroes every axis counter at the current position and activates soft limits. It does not measure anything — it trusts that you jogged to the reference pose. It asks for confirmation and is refused while anything is moving.

### Saved Limit Sets

One machine often needs multiple envelopes (tight for production, wide for maintenance). **Settings → Boundaries → SAVED LIMIT SETS** stores named presets in `robot_sim/limit_presets.json`:

| Button | Action |
|:-------|:-------|
| **SAVE** | Stores the current boundary values under the entered name. Validates first — an invalid set is refused. |
| **LOAD** | Fills the form from a saved set. You still press APPLY to send it to the machine. |
| **DELETE** | Removes the saved preset only. The machine's live limits are untouched. |

---

## Calibration

The software ships with theoretical constants from the MATLAB/Simscape CAD model. **These are placeholders — they must be measured on your physical machine.** If the robot over-shoots or under-shoots its target, this is almost certainly why.

### Z-Axis (Height) — Ball Screw Lead
- **Default assumption:** 20 mm per motor revolution
- **How to measure:** Put a ruler on the carriage, command a known distance, measure actual travel.
- **To apply:** Send `SET_Z_LEAD:<mm_per_rev>` over serial. No re-flash needed.

### RM (Rotation) — Gear Ratio
- **Default assumption:** 28.4375 (from `4.375 × 6.5` in the Simscape model)
- **How to measure:** Command a 90° rotation, measure actual angle swept. True ratio = `28.4375 × (actual / 90)`.
- **To apply:** Send `SET_ROT_RATIO:<ratio>` over serial. No re-flash needed.

### AM (Elbows) — Gear Ratio
- **Default assumption:** 2.0 (motor degrees per frog-leg degree, from the Simscape linkage model)
- **How to measure:** Mark the elbow, command a known number of motor revolutions, divide by the joint angle actually swept.
- **To apply:** Update `ARM_GEAR_RATIO` in `robot_sim/config.py` **and** send `SET_ARM_RATIO:<ratio>` to the firmware.

> **What the arm ratio affects:** It does NOT affect jog speed (the ratio cancels out in the pulse-rate math). But it controls every **angle** the board reports and every **absolute position** it drives to — so all IK targets, reach figures, and `MOVE_A1`/`MOVE_A2` commands are wrong by exactly this factor until measured.

---

## Jog Keys & Gamepad

### Keyboard
Default layout: `A/D` = RM · `I/K` = A1M · `O/L` = A2M · `W/S` = ZM.

- Fully customizable in **Settings → Controls**. Click a key box, press the new key. If it's already in use, the two rows **swap** (no axis is ever left unbound).
- Reserved keys: `SPACE` (E-STOP), `ESC` (Settings), `H` (Home) — cannot be reassigned.
- Saved to `robot_sim/keybinds.json` and restored on startup.
- A corrupt or incomplete keybinds file is discarded whole rather than half-merged.

### Xbox Gamepad
| Input | Axis |
|:------|:-----|
| RT / RB | A1M forward / backward |
| LT / LB | A2M forward / backward |
| X / B | RM CW / CCW |
| A / Y | ZM up / down |

Both arms can be driven simultaneously and independently. The gamepad goes through the same `jog_start()`/`jog_stop()` path as the keyboard — it never talks to the board directly.

---

## Appearance

Seven color schemes, selectable in **Settings → Appearance**:

| # | Scheme | Description |
|:--|:-------|:------------|
| 1 | **Graphite** *(default)* | Near-monochrome dark, near-white accent |
| 2 | Pure Mono | Greyscale chrome |
| 3 | Slate Blue | Cool greys, steel blue accent |
| 4 | Nordic Light | Light theme, muted blue — for bright rooms |
| 5 | Warm Paper | Warm off-white, muted clay accent |
| 6 | Carbon Orange | Very dark, industrial orange |
| 7 | Mint | Previous default scheme, kept for reversibility |

- Saved to `robot_sim/appearance.json` and applies **immediately** — the window rebuilds in place, keeping the serial connection, all field values, and the event log.
- **Refused while the machine is moving.** The rebuild briefly destroys and recreates the jog pads and E-STOP button, which would remove the operator's ability to stop the machine.

---

## Settings

Five tabs — **Speed**, **Boundaries**, **Controls**, **PID**, **Appearance** — each with its own APPLY and DEFAULTS that act **only on that tab**. A global reset that wiped your boundaries because you wanted to undo a speed change is the failure this avoids.

- `ESC` opens and closes the window (refused while a program is running).
- Each tab scrolls independently. The action bar is always visible at the bottom.
- PID gains are **individually lockable** — locking Kd freezes just Kd; the other terms stay editable, and DEFAULTS skips locked gains.

**RESTART** (in the connection row) relaunches the application in-place via `os.execv`. It sends `BYE`, closes the port cleanly, asks for confirmation, and is **refused while anything is moving**.

---

## Serial Protocol Reference

USB Serial, **115200 baud**, custom two-way ASCII string protocol.

### Commands (Python → ClearCore)

| Command | Description |
|:--------|:------------|
| **Connection** | |
| `PING` | Heartbeat probe |
| `BYE` | Graceful disconnect |
| **Status & Reporting** | |
| `STATUS` | Report current state |
| `PROFILE` | Report active speed profile |
| `LIMITS` | Report active soft limits |
| **Speed & PID** | |
| `SET_SPEED:rpm,acc,rotPct,armPct,zPct` | Set master RPM + per-axis percentages |
| `SET_PID:kp,ki,kd[,N]` | Store PID gains |
| `PID_ON` / `PID_OFF` / `PID_RESET` | Enable / disable / restore PID preset |
| **Boundaries** | |
| `SET_LIMIT:axis,MIN\|MAX,value` | Set a travel limit (`axis` = Z / ROT / A1 / A2) |
| `SET_LIMIT_HERE:axis,MIN\|MAX` | Capture current position as that limit |
| `RESET_LIMITS` | Restore factory envelope |
| `SET_LIMIT_ENFORCE:axis,0\|1` | Enable/disable one axis's boundary check |
| `SET_LIMITS_ENABLED:0\|1` | Master on/off for all boundaries |
| **Coordinate Reference** | |
| `RESET_COORD` (alias `SET_REF`) | Zero all axis counters; activates soft limits |
| `CLEAR_REF` | Drop the reference, suspend soft limits |
| **Motion — P2P** | |
| `LOAD:...` / `LOAD_BOTH:...` | Load a joint-space program (single or dual-arm) |
| `MOVE_XYZ:arm,X,Y,Z` / `LOAD_XYZ:...` | Cartesian target — board runs IK. **HOME = X 0, Y 0, Z 0** |
| `LOAD_XYZ_BOTH:Xa,Ya,Za,Xb,Yb,Zb` | Dual-arm Cartesian. **Za must equal Zb** (one carriage; 9 mm deck offset handled internally) |
| `IK:arm,X,Y,Z` / `FK:d1,rot,a1,a2,arm` | Compute-only queries, no motion |
| `RUN` | Execute loaded program |
| `STOP` / `ESTOP` | Halt motion / emergency stop |
| `HOME` | PLC homing via IO-0 → X0 wire (Ethernet link is read-only) |
| **Motion — Jog** | |
| `ROT_CW` / `ROT_CCW` / `ROT_STOP` | Jog turntable |
| `A1_FWD` / `A1_BACK` / `A1_STOP` | Jog arm 1 elbow |
| `A2_FWD` / `A2_BACK` / `A2_STOP` | Jog arm 2 elbow |
| `ARM_FWD` / `ARM_BACK` / `ARM_STOP` | Both elbows together |
| `Z_UP` / `Z_DOWN` / `Z_STOP` | Jog lift |
| `JOG_HB` | Jog dead-man keep-alive (~150 ms interval) |
| `MOVE_A1:th3` / `MOVE_R1:mm` | Absolute elbow angle / radial reach |
| **Calibration** | |
| `SET_Z_LEAD:<mm>` | Set ZM ball screw lead (no re-flash) |
| `SET_ROT_RATIO:<ratio>` | Set RM gear ratio (no re-flash) |
| `SET_ARM_RATIO:<ratio>` | Set AM gear ratio (no re-flash) |
| `SET_BOOST:multiplier` | Temporary jog speed multiplier |

### Feedback (ClearCore → Python)

| Response | Meaning |
|:---------|:--------|
| `PONG` / `[CONNECTED]` | Heartbeat reply / IO0 handshake success |
| `[ALIVE] uptime: Xs` | Idle status ping with board uptime |
| `[CLEARCORE POS] D1: F mm \| ROT: F° \| A1M: F° \| A2M: F° (P%)` | Live P2P position + completion % |
| `[JOG POS] ROT: F° \| A1M: F° \| A2M: F° \| Z: F mm` | Live jog position |
| `[SPEED] master F RPM, F RPM/s \| RM F% \| ARM F% \| ZM F%` | Active speed configuration |
| `[PROFILE] RM F °/s ... (CLAMPED)` | Real derived speeds; `CLAMPED` = ceiling hit |
| `[LIMITS] Z a..b mm \| ROT a..b° \| A1 a..b° \| A2 a..b°` | Active soft limits |
| `[LIMIT_SET] axis END = value` | Limit accepted |
| `[COORD_RESET] ...` | Counters zeroed, soft limits active |
| `[LIMIT] ROT_CW / Z_UP / A1_FWD ...` | Axis hit a limit and was stopped |
| `[IK] arm=N ...` / `[FK] arm=N ...` | Kinematics query result |
| `[SINGULARITY] th3=F°` | Advisory: frog-leg near straight-arm |
| `[WATCHDOG] ...` | Jog stopped — no keep-alive from host |
| `DA DEN DIEM DICH THANH CONG` | P2P move completed successfully |
| `DUNG KHAN CAP` | Stop/ESTOP acknowledged |
| `[HOME] Homing started.` / `[HOME] Homing complete.` | PLC homing handshake |
| `[WARN] ...` | Interlock auto-canceled a conflicting motion |
| `[ERROR] ...` | Malformed, out-of-range, or out-of-sequence command |

---

## Safety Design

Defense in depth — enforced in both software and firmware independently:

- **Mode switching** auto-stops any active jog or P2P motion.
- **Firmware interlocks** independently refuse P2P while jogging (and vice versa), as a backstop if GUI and firmware ever desync.
- **One E-STOP code path** (`emergency_stop_all`): both the on-screen button and SPACE call it. One audited function beats several similar ones.
- **PING always gets an immediate PONG**, even mid-motion, so the heartbeat never times out because the board is busy.
- **PLC sensors** (M5–M8) sit at opposite ends of their axes. Jog **warns** when driving into a covered sensor but does not block (the operator may need to back off). P2P **refuses** a move that would drive further into a covered sensor.
- **Dead PLC link** reads as sensors UNKNOWN (not CLEAR), so nothing auto-resets on a dead connection.

### LED Activity Indicator
The board's status LED (IO0) is an **activity light**, not a steady "connected" light:
- Every command **received** triggers a short flash.
- Every feedback line **sent** triggers a slightly longer flash.
- A live connection shows as a slow, regular flicker; a dead one goes fully dark.
- All timing is `millis()`-based — never `delay()` — so motion, serial, and heartbeats are never blocked.

---

## Tests

```bash
tests/run_tests.sh
```

Two test suites:

| Suite | What it tests | How it works |
|:------|:--------------|:-------------|
| `tests/firmware_check.cpp` | Firmware logic (IK, limits, PLC, sensors, homing) | Compiles the `.ino` against a desktop ClearCore shim and asserts on serial output |
| `tests/python_check.py` | GUI logic (kinematics, boundaries, speed, panels) | Drives the app through a headless Tk stub |

Both live inside the repo deliberately — an earlier generation lived in `/tmp` and vanished with a machine restart. See `tests/README.md` for stub details.
