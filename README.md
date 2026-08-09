Robot Motion Controller — P2P + Joystick
=============================================================================

    python robot_simulator_launcher.py

**Optional but recommended: `pip install pillow`.** The custom controls
are canvas-drawn, and the Tk canvas has no anti-aliasing — a rounded
corner comes out as hard stair-steps. With Pillow installed, every rounded
surface is instead rendered at 4× and downsampled with LANCZOS, which
gives genuinely smooth edges. Without it the app still runs and falls back
to arc-based corners (correct geometry, just aliased).

**The app is DPI-aware.** Tk declares itself DPI-unaware by default, so on
any Windows display above 100% scaling the OS renders the window at 96 DPI
and then *bitmap-stretches* it to the real size — which is what made
everything look low-resolution regardless of how carefully it was drawn.
`robot_sim/hidpi.py` declares awareness before the first widget exists,
sets Tk's font scaling, and scales the hard-coded pixel dimensions so the
UI draws on the monitor's real pixel grid at its intended physical size.

Merges two control modes on one shared connection layer:
  - POINT TO POINT : absolute X/Y/Z target motion 
  - JOYSTICK        : held-key jog on ROT/ARM/Z axes

SHARED ACROSS BOTH MODES:
  - One connection panel + 3-LED status (COM PORT / CLEARCORE IO0 / HEARTBEAT)
  - PING/PONG heartbeat every HEARTBEAT_INTERVAL_MS with missed-beat tracking
  - One unified Event Log (>> sent / << received, color-tagged)
  - A software-only simulation fallback when no hardware is confirmed, so
    both modes remain fully testable/demoable with nothing plugged in

SAFETY DESIGN (defense in depth — enforced in BOTH layers independently):
  - Switching mode in the GUI auto-stops any active jog axis + any P2P move
  - Firmware also refuses to run a P2P move while a jog axis is held (and
    vice versa) — a backstop in case the GUI and firmware ever desync
  - There is exactly ONE emergency-stop code path (emergency_stop_all):
    the P2P panel's button and the Space key both call it. One audited
    function beats several similar ones — which is also why the jog
    panel's duplicate button and the P2P panel's plain STOP were removed.
  - PING always gets a PONG reply immediately, even mid-motion, so the
    heartbeat never times out just because the board is busy

LED ACTIVITY INDICATOR:
    The board's status LED (LED_PIN / ClearCore IO0) is now a pure
    ACTIVITY light instead of a steady "connected" light:
    - Every command RECEIVED from Python triggers a very short,
        fast flash (see LED_FLASH_RX_MS) — the faster the commands
      arrive (e.g. held jog keys), the faster it flickers.
    - Every feedback line SENT to Python triggers a slightly longer
      flash (see LED_FLASH_TX_MS), so you can visually confirm the
      board is actively reporting back.
      
  Both flashes are done with millis()-based timing (ledPulse() /
the LED section of loop()) — never delay() — so motion stepping,
serial reads, and heartbeats are never blocked by the LED.

  A steady "connected" LED is no longer used: with feedback flowing
every ALIVE_INTERVAL_MS while idle, a live connection now shows
itself as a slow, regular flicker; a dead one goes fully dark.
  
## Speed: one fixed reference, one percentage per motor

The reference speed is a **constant 150 motor RPM**. It is not editable —
it used to be, which made two knobs doing the same job (raising the master
and raising a percentage produced identical motion). There is now exactly
one way to change a speed: that axis's percentage.

```
axisMotorRpm = 150 * (axisPercent / 100) * AXIS_RPM_SCALE
```

`AXIS_RPM_SCALE` is calibration, not a setting. The axes are geared very
differently, so a raw percentage of one shared RPM would mean nothing:

| Axis | Drivetrain (from the MATLAB model) | Scale | Default | Real speed |
| :--- | :--- | :--- | :--- | :--- |
| A1M / A2M | ratio **unmeasured** | 1.000 | **125%** | 187.5 motor RPM |
| RM  | `i_RM_total = 4.375 × 6.5 = 28.4375:1` | 1.000 | **75%** | 23.74 °/s |
| ZM  | 20 mm per motor rev | 0.750 | **50%** | 18.75 mm/s |

Settings shows the real motor RPM beside each percentage as you type.

**Above the default, the field turns amber and warns.** The defaults are
the fastest settings tested stable on this machine, so "above the default"
and "past what has been validated" are the same statement. Higher values
are still accepted after a confirmation — the warning is about lost steps,
overshoot and vibration on an open-loop drive, not a hard block.

**There is no hard percentage cap.** Safe for a specific reason rather
than by luck: a percentage is a multiplier, not a speed, and every axis it
feeds still has a real backstop underneath — `ROT_VEL_MAX_DEG_S` and
`Z_VEL_MAX_MM_S` for the two axes whose gearing is known,
`ARM_MOTOR_RPM_MAX` for the one whose gearing isn't. Set RM to 900% and it
still stops at 120 °/s. The 1% floor stays, because 0% freezes an axis and
a negative value would reverse it.

> **`ARM_GEAR_RATIO` is still a placeholder and should be measured.**
> It no longer affects arm *speed* — the ratio cancels between the °/s
> conversion and the pulses-per-degree conversion, so the pulse rate
> depends only on the RPM asked for. But it still controls every **angle**
> the board reports and every **absolute position** it drives to, so
> `MOVE_A1`/`MOVE_A2`, the reported elbow angle, the reach figures and IK
> targets are wrong by exactly this factor until it is measured. Jog is
> unaffected. To measure: mark the elbow, command a known number of motor
> revolutions, divide by the joint angle actually swept.

## The arm angle: rotation from home

**`A1M_POS` / `A2M_POS` read 0° at home and count up as the elbow turns
out.** Fully retracted is 0°, straight out is 120°, and the reach that
corresponds to each is unchanged: 133.2 mm at 0°, 613.2 mm at 120°.

This replaces `th3_cad`, the CAD elbow angle from `mophong_init.m`, where
retracted was 60° and straight was 180°. The reason is not presentation:

> **The board could never produce a real `th3_cad`.** It counts steps from
> wherever it was last referenced and scales them by `ARM_GEAR_RATIO`, an
> unmeasured placeholder. The `60°` it used to print at home was zero
> rotation wearing a CAD label — a number that looked like a measured
> angle and wasn't. Reporting rotation from home is the *same* number with
> an honest name, and it reads the way an operator thinks: home is
> nothing, and it counts up.

`th3_cad` still exists, because the frog-leg geometry is genuinely written
in it, but only inside `fold_angle_to_reach()` / `reach_to_fold_angle()`
(and `reachFromFoldAngle()` / `foldAngleFromReach()` on the board), which
add and remove `ARM_ZERO_CAD_DEG = 60`. No other code mentions it, and
`tests/python_check.py` fails the build if it leaks out again.

Everything that was expressed in the old frame moved with it:

| | old (`th3_cad`) | new (from home) |
| :--- | ---: | ---: |
| Home / retracted | 60° | **0°** |
| Straight arm (singularity) | 180° | **120°** |
| JEL drawing limit, 575 mm | 151.72° | **91.72°** |
| Singularity warning | 170° | **110°** |
| Factory elbow band | 60°…180° | **0°…120°** |

> **Saved settings are migrated, not converted.** `machine_settings.json`
> now carries a `_schema`. A file written before this change has taught
> elbow limits of `60…180` that would silently be read as *60–180 degrees
> of rotation* — a band starting 60° away from home, so the arm couldn't
> retract and you'd be hunting a mechanical fault that isn't there. Those
> four values are dropped with a warning telling you to re-teach them.
> They are not converted, because they were produced by the same
> unmeasured ratio and were never real angles to convert. Speeds and the
> ZM/RM boundaries are untouched — mm and RM degrees didn't change meaning.

## Boundaries and the coordinate reference

The factory envelope is what the *structure* allows. What the machine may
actually use is narrower and depends on what is installed around it, so
those limits belong to the operator:

- **Settings → Boundaries** has a min/max box for ZM and RM, and each one
  has a **SET HERE** button that captures the machine's current position —
  jog to the lowest point the lift may go, press it, done.
- **The elbow rows are SET HERE only — their boxes cannot be typed in.**
  The board's elbow angle is scaled by `ARM_GEAR_RATIO`, which has not
  been measured, so a typed `90°` means nothing; *"wherever the arm is
  standing right now"* is exact regardless.
- **The elbows have no envelope at all.** Whatever `A1M_POS` / `A2M_POS`
  reads is accepted — four figures included. Because the reported angle
  rides on that unmeasured ratio, the arm really can read `1000°` at a
  pose CAD calls `140°`, so any ceiling written here would be a guess, and
  a guess that rejects a pose the arm is physically standing at stops the
  operator teaching the machine at all. ZM and RM keep their real
  envelopes, because for those two the scale **is** known and a number
  outside it really is impossible.
- **The elbow pair is unordered.** Jog to one stop, SET HERE; jog to the
  other, SET HERE. Which you reached first does not matter — both numbers
  are stored exactly as captured and sorted where they are *read*
  (`armBand()` on the board, `_limit_pair()` in the GUI). Sorting on write
  would fold your first taught position against whatever stale value sat
  in the other box, so your second SET HERE would quietly overwrite your
  first. The only arrangement refused is both ends landing on the *same*
  position, which pins the axis where it stands.
- **Each arm has its own pair.** Sharing one arm limit is how A2M used to
  be driven past its stop while A1M's angle was the one being checked.
- A ZM or RM pair can never invert or collapse; a MIN above its MAX would
  make every position illegal and the axis couldn't be jogged back out.

- **RESET COORDINATES** zeroes every axis counter at the current position and
  is what turns the soft limits on. It measures nothing — it trusts that
  you jogged the machine to the reference pose — so it asks first and is
  refused while anything is moving.

> **Firmware note.** An unbounded elbow range means a taught band can span
> past the straight-arm point, and reach is a cosine of
> `θ + ARM_ZERO_CAD_DEG` — monotonic only across half a period. Taking the
> min/max of the two *endpoints* is therefore wrong once the band crosses
> an extreme: with a band of −260°…480° both ends land on the same part of
> the curve, so the endpoint-only version reported a reachable radius of
> 593.9–613.2 mm and refused every ordinary target. `reachBandFor()`
> checks the angles where `θ + 60` is a multiple of 180 (θ = 120, 300,
> −60, …), which is where the cosine actually reaches its extremes.

The board holds limits in **RAM only**, so the GUI is the system of
record: it saves them to `robot_sim/machine_settings.json` and re-sends
everything the moment the PING/PONG handshake succeeds.

### Saved limit sets

One machine often needs more than one envelope — a tight one for running
against a cassette, a wider one for maintenance. The **SAVED LIMIT SETS**
row stores them by name in `robot_sim/limit_presets.json`:

| Button | Does |
| :--- | :--- |
| **SAVE** | Saves the boxes as currently shown, under the name in the box |
| **LOAD** | Fills the form from a saved set — you still press APPLY |
| **DELETE** | Deletes the saved set only; the machine's live limits are untouched |

The name field is an editable dropdown: type a new name to create a set,
pick an existing one to target it. A few deliberate behaviours:

- **SAVE validates first.** A preset is something you'll recall later and
  trust without re-reading, so an invalid or inverted set is refused
  rather than stored as a fault waiting to be found.
- **A preset must carry every boundary.** A partial one would load some
  axes and silently leave others on whatever was in the boxes.
- Overwriting and deleting both ask first, and a corrupt presets file
  degrades to "no presets" instead of taking the app down.

## Jog keys

The default layout is `A/D` = RM · `I/K` = A1M · `O/L` = A2M · `W/S` = ZM.

Bindings are editable in **Settings → Controls** and saved to
`robot_sim/keybinds.json`. Click a key box, press the key you want; Escape
cancels. If the key is already in use the two rows **swap**, rather than
leaving an axis unbound. `SPACE` (e-stop), `ESC` (settings) and `H` (home)
are reserved and cannot be taken.

There are **no preset layouts and no advice about which keys sit near
which**. What is comfortable depends on the hands using it, and a
controller that argues with the operator over their own keyboard is noise
between them and the machine. Only three things are refused, all of them
conditions that would leave the app broken rather than merely unusual:

- an action with no key at all — that axis silently becomes unreachable
  from the keyboard, with nothing on screen to explain why;
- one key driving two actions, for the same reason;
- taking `SPACE`, `ESC` or `H` — losing the emergency stop, the settings
  window or HOME to a rebinding is not a trade worth offering.

Everything else applies without a question being asked.

**Your layout is remembered.** APPLY writes it to `keybinds.json` and the
app reads it back at startup, so a custom layout survives restarts;
DEFAULTS is the way back to `A/D · I/K · O/L · W/S`. A file that is corrupt
or missing an action is discarded **whole** rather than merged — a
half-loaded keymap would leave some axes on your keys and others on the
defaults, which is much harder to notice than a clean revert.

Applying a layout takes effect immediately: the binder clears its old
bindings first, and every on-screen key name is live, so the pads and the
hint strip never advertise a key that no longer does anything.

## Appearance

Seven colour schemes ship with the app, picked in **Settings →
Appearance**. The default is **Graphite** — near-monochrome dark, with
colour used only where it carries meaning.

| # | Scheme | |
| :-- | :--- | :--- |
| 1 | **Graphite** *(default)* | Near-monochrome dark, near-white accent |
| 2 | Pure Mono | Greyscale chrome; axes separated by lightness only |
| 3 | Slate Blue | Cool greys, one restrained steel blue |
| 4 | Nordic Light | Light theme, muted blue — for bright rooms |
| 5 | Warm Paper | Warm off-white, muted clay accent |
| 6 | Carbon Orange | Very dark, industrial orange |
| 7 | Mint | The previous scheme, kept so the change is reversible |

The choice is saved to `robot_sim/appearance.json` and **applies
immediately** — the window is rebuilt in place, keeping the serial
connection, your settings, every field value and the event log.

It is refused while the machine is moving. The rebuild destroys and
recreates the jog pads and the E-STOP button, and doing that with an axis
under power — even for the few milliseconds it takes — would remove your
ability to stop the machine.

Under the hood: every widget module does `from ..theme import PANEL_BG`,
which copies the *value* at import time, so rebinding `theme.PANEL_BG`
alone is not enough. `apply_palette()` walks every imported `robot_sim.*`
module and rebinds any name that still holds the **old** value —
comparing against the old value rather than matching by name is what
makes that sweep safe, since a module with its own unrelated `BORDER`
simply will not match.

Schemes are not free-form. `palettes.py` documents five rules every one
must satisfy, and they are enforced by a test rather than by eye:

- Surface levels must be separable from their neighbours (flat design has
  no shadows, so those steps do the work shadows used to).
- Fields must read as recessed and read-only wells deeper still — that is
  the operator's only cue for what is editable.
- **Red means stop and amber means warning in every scheme.** No palette
  may use amber as its primary accent, or a warning stops standing out.
- **Motor colours come in two groups, not four hues.** `A1M` and `A2M`
  share the primary accent; `RM` and `ZM` share the secondary. The arms
  are a pair and the base axes are a pair, so the colour answers "which
  kind of axis" and the label answers "which one" — which is how a row of
  readouts actually gets scanned. They were red / purple / blue / teal,
  which made that row look like a different application.

  The two tones are solved for target luminances ~45 apart, not blended
  by a fixed ratio: a fixed 30% blend moves brightness by an amount that
  depends on where the accent started, which collapsed some schemes to a
  2-unit separation — invisible, and worse than the clashing hues it
  replaced. Separation by lightness also means they survive red-green
  colour blindness.
- Body text needs ≥ 110 luminance contrast against its card.

## Settings

Five tabs — **Speed**, **Boundaries**, **Controls**, **PID**,
**Appearance** — each with its own APPLY and DEFAULTS that act **only on
that tab**. A global reset that also wiped your boundaries because you
wanted to undo a speed change is the failure this avoids. `Esc` opens and
closes the window; it is refused while a program is running.

Each tab body **scrolls on its own** when its content is taller than the
window, and the scrollbar only appears when it is actually needed. The
action bar is packed against the bottom *before* the body, so APPLY is
never the thing that scrolls out of reach.

## Restart

`RESTART`, between PING and SETTINGS in the connection row, relaunches the
application in place (`os.execv` — the same process, not a second instance
fighting for the COM port). It sends `BYE` and closes the port on the way
out so the board sees a clean disconnect, asks for confirmation, and is
**refused while anything is moving**: the process is about to be replaced,
and an axis under power with no controller attached is a runaway nothing
in the app can stop.

PID gains are **individually lockable**. Locking Kd freezes just Kd — the
other terms stay editable and keep being sent, and DEFAULTS skips anything
locked rather than silently overriding it.

## Serial Communication Protocol

The STCR4000S uses a custom, two-way ASCII string protocol over USB Serial (Baud: 115200).

### ➔ Python to ClearCore (Commands)
| Command | Action |
| :--- | :--- |
| `PING` / `BYE` | Heartbeat probe / graceful disconnect |
| `STATUS` / `PROFILE` / `LIMITS` | Report state, speed profile, active limits |
| `SET_SPEED:rpm,acc,rotPct,armPct,zPct` | Universal RPM + per-motor percentages |
| `SET_PID:kp,ki,kd[,N]` | Store the PID gains (one preset, no form field) |
| `PID_ON` / `PID_OFF` / `PID_RESET` | Enable / disable / restore the preset |
| `SET_LIMIT:axis,MIN\|MAX,value` | Set a travel limit; axis = `Z`/`ROT`/`A1`/`A2` |
| `SET_LIMIT_HERE:axis,MIN\|MAX` | Take the **current position** as that limit |
| `RESET_LIMITS` | Restore the factory envelope |
| `SET_LIMIT_ENFORCE:axis,0\|1` | Switch **one** axis's boundary off / on; values kept |
| `SET_LIMITS_ENABLED:0\|1` | The same for every axis at once (ANDs with the above) |
| `RESET_COORD` (alias `SET_REF`) | Zero every axis counter here; enables soft limits |
| `CLEAR_REF` | Drop the reference, suspend soft limits |
| `LOAD:...` / `LOAD_BOTH:...` | Load a joint-space program (A→B, or dual-arm) |
| `MOVE_XYZ:arm,X,Y,Z` / `LOAD_XYZ:...` | Cartesian targets — the board runs the IK. **HOME is X 0, Y 0, Z 0**: X/Y from the turntable axis and signed, Z above HOME and never negative |
| `LOAD_XYZ_BOTH:Xa,Ya,Za,Xb,Yb,Zb` | Simultaneous dual-arm. One carriage, so **Za must equal Zb** — the 9 mm deck offset is the board's job |
| `IK:arm,X,Y,Z` / `FK:d1,rot,a1,a2,arm` | Compute and report only, no motion |
| `RUN` / `STOP` / `HOME` / `ESTOP` | Run loaded program / halt / PLC home / e-stop. HOME drives ClearCore's **IO-0 terminal into the PLC's X0 input** — a wire; the Ethernet link is read-only and cannot start a home |
| `ROT_CW` / `ROT_CCW` / `ROT_STOP` | Jog the turntable |
| `A1_FWD` / `A1_BACK` / `A1_STOP` | Jog arm 1's elbow alone |
| `A2_FWD` / `A2_BACK` / `A2_STOP` | Jog arm 2's elbow alone |
| `ARM_FWD` / `ARM_BACK` / `ARM_STOP` | Both elbows together (what LINK sends) |
| `Z_UP` / `Z_DOWN` / `Z_STOP` | Jog the lift |
| `MOVE_A1:th3` / `MOVE_R1:mm` | One elbow to an absolute angle / radial reach |
| `JOG_HB` | Jog dead-man keep-alive (host must send every ~150 ms) |
| `SET_BOOST:multiplier` | Temporary jog speed multiplier |

### ⬅ ClearCore to Python (Feedback)
| Response | Meaning |
| :--- | :--- |
| `PONG` / `[CONNECTED]` | Heartbeat response / IO0 handshake success |
| `[ALIVE] uptime: Xs` | Status ping with board uptime |
| `[CLEARCORE POS] D1: F mm \| ROT: F deg \| A1M: F deg \| A2M: F deg (P%)` | Live P2P telemetry and completion percentage |
| `[JOG POS] ROT: F deg \| A1M: F deg \| A2M: F deg \| Z: F mm` | Live jog telemetry, both elbows |
| `[SPEED] master F RPM, F RPM/s \| RM F% \| ARM F% \| ZM F%` | Active universal speed and percentages |
| `[PROFILE] RM F deg/s ... (CLAMPED)` | Resulting real speeds; `CLAMPED` = ceiling hit |
| `[LIMITS] Z a..b mm \| ROT a..b deg \| A1 a..b deg \| A2 a..b deg` | Active soft limits |
| `[LIMIT_SET] axis END = value` | A limit was accepted |
| `[COORD_RESET] ...` | Counters zeroed; soft limits now active |
| `[LIMIT] ROT_CW / Z_UP / A1_FWD ...` | An axis reached a limit and was stopped |
| `[IK] arm=N ...` / `[FK] arm=N ...` | Kinematics query result |
| `[SINGULARITY] th3=F deg` | Advisory: frog-leg near straight |
| `[WATCHDOG] ...` | Board stopped a jog — no keep-alive from the host |
| `DA DEN DIEM DICH THANH CONG` | P2P move successfully completed |
| `DUNG KHAN CAP` | Stop/ESTOP command acknowledged |
| `[HOME] Homing started.` / `[HOME] Homing complete.` | PLC homing handshake |
| `[WARN] ...` | An interlock auto-canceled a conflicting motion |
| `[ERROR] ...` | Malformed, out-of-range or out-of-sequence command |

## Tests

```
tests/run_tests.sh
```

Two suites. `tests/firmware_check.cpp` compiles the `.ino` against a
desktop ClearCore shim and asserts on the text the board sends back;
`tests/python_check.py` drives the GUI's logic through a headless Tk stub.
Both live inside the repo deliberately — an earlier generation of these
harnesses lived in `/tmp` and vanished with a machine restart, taking the
only evidence the firmware worked with them. See `tests/README.md` for the
two stub details that are load-bearing.
