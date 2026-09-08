# Working on this project

Context for a Claude session picking this up cold. Read this before
touching anything; it is mostly a list of decisions that look wrong until
you know why they were made.

---

## What this is

An FPD wafer-handling robot (frog-leg twin arm, STCR4000S-class). Four
pieces, which must agree with each other:

| Piece | Path | Role |
| :--- | :--- | :--- |
| **Firmware** | `RobotMotionController_v9_ClearCore/*.ino` (~3700 lines) | ClearCore board. Owns motion, soft limits, IK, the PLC read link, HOME and the scan sweep. |
| **GUI** | `robot_sim/` (~10 000 lines, Python + Tkinter) | Operator console. Serial link, three control modes, settings. |
| **Scanner** | `Scan/` | Stand-alone scanner for the same board. The only one that runs **simulated**, away from the machine. |
| **Simulation** | `../MATLAB_v4_final` | Simscape model + `mophong_init.m`. **Historical reference only** — see the geometry note below. |

`v8` and `clearcore/` are older firmware, kept for reference. **v9 is the
live one.** MATLAB is read-only by request, and is no longer the source of
the geometry — the arm was measured on the bench and the .m disagrees.

Four axes: **RM** turntable, **A1M** / **A2M** the two elbows (separate
motors), **ZM** lift.

**Three control modes**, section 2 of the window: **POINT TO POINT**,
**JOYSTICK**, **SCAN**. Section 3 shows one panel at a time.

---

## Run the tests before and after every change

```
tests/run_tests.sh
```

Two suites — ~300 firmware assertions and ~650 Python ones — both must
be green:

* `tests/firmware_check.cpp` — compiles the `.ino` against
  `tests/stub/ClearCore.h` and asserts on the text the board sends back.
* `tests/python_check.py` — drives the GUI's logic through
  `tests/tkstub/`, a headless Tk stand-in.

They live **inside the repo on purpose**. An earlier generation lived in
`/tmp` and was destroyed by a machine restart, taking the only evidence
the firmware worked with it. Do not recreate that situation.

Two stub details are load-bearing, both paid for by real bugs:

* `Serial.println` **captures into `OUT`**. When it was a no-op, every
  `saw("...")` assertion passed vacuously — worse than no tests.
* `Widget.__getitem__` **raises `KeyError`**. Returning `None` made
  `"x" in widget` fall back to iteration that never raised `IndexError`,
  and the suite hung forever.

`tests/tkstub` is not a renderer. It records the widget tree and holds
`StringVar` values, which is enough to exercise every dialog path.

---

## The arm frame: HOME is base −30°, the working maximum is base +60°

Read this first. Most of the older notes in this file were written under a
different frame, and this section is the one that decides the numbers.

**The link set is `mophong_init.m`'s**: `a3..a6 = 45 / 160 / 160 / 248.2`,
so `a3+a6 = 293.2` and `a4+a5 = 320`, and

```
R = (a3 + a6) - (a4 + a5) * cos(th3_cad)
```

Only the two SUMS are used, so if the links are ever measured individually,
preserve them.

**Three names for one pose**, and only the first is stored anywhere:

| | at HOME | working max | straight |
| :--- | ---: | ---: | ---: |
| **motor deg** — what the board counts | 0 | 702 | 936 |
| **fold deg** — frog-leg from home | 0 | 90 | 120 |
| **base deg** — what the operator reads | **−30** | **+60** | +90 |
| **th3_cad** — the CAD frame | 60 | 150 | 180 |
| **R** | **133.2 mm** | **570.3 mm** | 613.2 mm |

```
base  = th3_cad - 90 = fold + ARM_ZERO_CAD_DEG - 90 = fold - 30
motor = fold * ARM_GEAR_RATIO
```

* **`ARM_ZERO_CAD_DEG = 60.0`.** HOME is the CAD frame's `th3_cad` 60 pose,
  133.2 mm from the turntable axis — confirmed on the machine.
* **The base angle is 1:1 with the arm**, offset 90°. It is NOT a scale
  factor. The previous frame stretched fold onto 0..90 base through
  `90/146.68`, so one base degree was not one arm degree and the two
  numbers drifted apart across the travel.
* **`FOLD_ANGLE_SPEC_MAX_DEG = 90` (base +60, 570.3 mm) IS A NOTE, NOT A
  LIMIT.** Nothing refuses a target past it. The arm may go on to base
  +90° / 613.2 mm, where it is straight and singular. What stops it is the
  operator's taught elbow band, as always — the note exists so the panel
  and the boot banner can say where the machine is meant to work.
  `arm_frame_note()` in `config.py` is the one place that sentence is
  written, as `arm_home_note()` + `arm_max_note()`. **P2P shows both poses;
  JOYSTICK shows the HOME half alone** — jogging is about where the arm is,
  and a working ceiling is a question about a target. The note carries the
  poses and nothing else; that MAX is not enforced is said here, in the
  code and in the banner, not on a line read every day.
* **`DEFAULT_POINT_A`'s X is `ARM_MIN_REACH_MM`**, so the P2P panel opens on
  the pose the arm is actually in. Derived, not a literal: a frame change
  moves it instead of leaving a stale 240 behind.
* **Travel is fold 0..120**, motor 0..936, default taught band 0..926.
* `FOLD_ANGLE_SINGULARITY_WARN_DEG = 110` — 10° short of straight, as it
  has always been.

**What this replaced**, so an older comment is recognisable: a bench-solved
pair `a3+a6 = 422.5, a4+a5 = 182.5` that read 240 mm at HOME and 605 mm
straight, with `ARM_ZERO_CAD_DEG = 0` and base `0..90` scaled through
146.68. Those two measurements were taken in the from-home frame; the
machine's own home pose is the 133 mm one, and the frame moved to match.

**THE MATLAB PARITY SWEEP IS STILL GONE.** The links agree with the .m
again, but the sweep is not coming back: `python_check.py` section 7 checks
a **round trip** instead — every pose IK solves must come back out of FK in
the same place, to machine precision, over a few thousand poses. That
catches the drift the sweep existed to catch with no external file to
disagree with, and it does not go red when the frame moves again.

The .m stays the reference for the **frame**: the Z chain
(`Z_offset(arm 1) = 514.3 mm`) and the `d1` 0..285 stroke, neither of which
depends on link length.

**RM's gearing is `I_RM_TOTAL = 6.5`.** It used to be written `4.375 × 6.5`
= 28.4375. The extra 4.375 is gone: at 6.5 the defaults work out as the
speeds the machine actually ran at (RM 50% = 75 RPM = **69.2 °/s**, the
figure in section 5). Changing it rescales every RM speed, so it is a
decision to take with the machine in front of you.

Two departures from the .m are still deliberate and still tested:

* MATLAB **clamps** an unreachable target silently; here it raises, because
  the operator would otherwise press RUN believing the point was accepted.
  `clamp_like_matlab=True` reproduces the .m for comparison.
* MATLAB **pins AM2 at `th3_home_cad`** for the whole trajectory. The idle
  arm holds where it is instead — parking it home is a move of up to 120
  deg the operator never commanded, on an arm that may be carrying a
  substrate.

The **path shape** is joint-space, not MATLAB's 40-sample Cartesian
straight line. Chosen explicitly; a straight-line tool path needs a
streamed segment list or on-board interpolation.

---

## The five decisions most likely to be "fixed" by mistake

### 1. `ARM_GEAR_RATIO = 7.80` is MEASURED. The model's 2 was wrong.

It does **not** affect arm speed — that is bounded in motor RPM and motor
°/s, both ratio-free. It **does** scale every reported frog-leg angle,
every reach figure, and every absolute arm move (`MOVE_A1`, `MOVE_A2`, IK
targets). Jog and the taught limits are unaffected, because both work in
motor degrees.

**Where 7.80 came from:** the arm reaches **575 mm** at full extension. The
earlier 10.0 put that same motor position at 498 mm, so
`10.0 × fold(498) / fold(575)` = `10.0 × 114.45 / 146.68` = **7.80**. It is
a *reach* measurement rather than an angle one, because reach is what a tape
measure can actually read on this machine.

**That arithmetic ran on the PREVIOUS reach curve.** `fold()` there was
`reach_to_fold_angle()` on the 240..605 mm pair; on the curve now in
`config.py` the same two measurements imply **7.61**. 7.80 is kept because
it is what the machine has been running and nobody has re-measured — it is
the number to check first if reported reaches read consistently long or
short. `SET_ARM_RATIO:<r>` changes it with no re-flash and nothing taught
has to be re-taught.

**Do not restore 2 from `mophong_init.m`.** The Simscape diagram drives each
arm's two revolutes from one AM signal — shoulder `×1`, knee `×−2` — and the
closed-form FK agrees, links at `th2 ± th3_math`. That describes the
**linkage**, and the bench says the linkage is not what the .m thinks
either. Whatever it describes, it says nothing about the gearbox in front
of it, which is where the rest of the 7.80 lives. A model being
self-consistent is not evidence about the drive train.

`SET_ARM_RATIO:<r>` still changes it live — no re-flash, and nothing taught
has to be re-taught, because taught elbow limits are stored in motor
degrees. See 1b.

### 1a. RM zero is the CCW stop, and it IS home

RM travels **0..340**, not -170..+170. Zero is the fully counter-clockwise
stop, which is where HOME is, so RM now agrees with every other axis: home
is zero and the number counts up.

**Cartesian +X moved with it.** At RM 0 the arm points along +X, so HOME is
a true X0 Y0 Z0 reference and a target straight ahead of home is `(R, 0)`.
Any X,Y taught under the old centred frame is rotated 170 deg from what it
used to mean.

Two consequences that bite:

* `atan2` returns (-180, 180], so a bearing just clockwise of home would
  read -5 and be refused as "below the CCW limit". `rot_from_bearing()` in
  the GUI and the `th2 += 360` in `solveIkFrogleg()` wrap it into [0, 360).
* The **20 deg wedge between 340 and 360 is unreachable** from either side.
  It is the gap the turntable cannot sweep through, and IK refuses it.

**HOME is the MINIMUM of all four axes** (d1 0, RM 0, both elbows 0). That
is why `LIMIT_SAFETY_MARGIN` is applied at the **far end only**: insetting
the lower end put HOME itself outside the working envelope, which refused
the home pose and with it every P2P program, since a run is
HOME -> A -> B -> HOME. **All three physical switches sit at that same
lower end** — `M32` at the bottom of ZM's stroke, `M30` retracted, `M31` at
RM's CCW end — which is what makes `M30 && M31 && M32` a coherent home
state at all. `M31` was written as the CW/maximum end for a while; see the
switch table below for what that cost.

### 1b. The elbow reports MOTOR degrees; the frog-leg angle is derived

**`A1M` / `A2M` are motor shaft degrees from home.** The board counts step
pulses and nothing else, so that is the only elbow figure it knows exactly.
The frog-leg angle and the reach are derived:

```
fold_deg = motor_deg / ARM_GEAR_RATIO       ARM_GEAR_RATIO = 7.80
base_deg = fold_deg - 30                    -30 at home, +60 the working max
R        = 293.2 - 320 * cos(fold_deg + 60) 133.2 mm home, 613.2 straight
```

**7.80 is measured — see section 1 for the arithmetic and for why the
model's 2 does not apply.** `SET_ARM_RATIO:<r>` changes it with no re-flash
if the drive train is ever altered.

Consequences that are easy to get wrong:

* **Taught elbow limits are stored in MOTOR degrees**, factory band
  `0..936` (fold 0..120 × 7.80), defaults `0..926`. That is the raw
  count, so re-calibrating the ratio never
  invalidates a boundary somebody taught. Storing fold degrees would
  rescale every taught number the moment the ratio moved.
  **The Boundaries tab SHOWS them as BASE degrees** — `−30.00..88.72` for
  that default band — because base is the frame every other readout uses;
  the store, the wire and the presets are all still motor degrees. The
  translation lives in `_limit_to_display()` / `_limit_from_display()` in
  `settings_dialog.py` and NOWHERE else, exactly like RM's `0..-340`
  entries. Converting anywhere downstream would put a taught boundary at
  the mercy of the gear ratio, which is the one thing this storage frame
  exists to prevent.
* `reachBandFor()`, `forward_kinematics()` and `is_near_singularity()` all
  take **fold** degrees. Every caller that holds motor degrees must convert
  — `armFoldFromMotor()` on the board, `fold_angle_from_motor_deg()` in the
  GUI. Feeding motor degrees straight into FK put the reported end effector
  at twice the elbow angle it had.
* Arm speed is quoted in **motor** °/s (exact) with the fold °/s beside it.
* `machine_settings.json` went to `_schema` **3** for this; v2 files' elbow
  limits were fold degrees and are dropped. It is on **4** now — see
  Persistence.

**HOME is 0 MOTOR degrees and base −30°, and `ARM_ZERO_CAD_DEG` is 60.**
The stored figure never moved — the board still counts from zero at the
reference — but what the operator reads is the base angle, and that reads
−30 at HOME. Nothing displays `th3_cad` itself. The jog panel used to
*initialise* its readout to `60.00 deg` and the boot banner announced the
th3_cad convention as the operator's frame; both looked like the machine
jumping to 60°, and both are gone.

### 1c. Per-axis enforcement, the master enable, and inset defaults

| Control | Effect |
| :--- | :--- |
| `lim_<axis>_enforced` | is **this** boundary stopping the axis? Values kept either way. |
| `limits_enabled` | the same, **every axis** at once. |
| `LIMIT_SAFETY_MARGIN` | how far the **defaults** sit inside the envelope |

The two switches **AND**, they do not override:
`axisLimited(axis) == limitsEnabled && limXxxEnforced`. Turning the master
back on must never re-arm an axis somebody switched off on its own, and the
GUI mirrors this in `_axis_enforced()` so its offline simulation clamps
where the board clamps.

**`isHomed` is NOT part of that.** A taught boundary applies from the
moment it is taught. It used to wait for a reference — `isHomed &&
limitsEnabled`, plus a band widened by a full travel either side in
`_axis_bounds()` — on the argument that the counters mean nothing without
one. That argument does not survive contact with how a boundary is set:
the operator jogs to the stop and presses SET HERE, so the boundary is
captured against **the same counters it is later compared with**. It is
meaningful in exactly the frame it was taught in. Waiting for a reference
meant a limit taught at `-300` let the axis run past `-420` — a machine
with no protection at all, while the panel said ENFORCED.

The boot-time deadlock that widening was invented to fix is solved
properly instead, by the **escape rule**: every clamp is directional.

* Firmware: `serviceArmSoftLimit()` and the ZM/RM jog checks stop `dir`
  only when that direction takes the axis further out — they always did,
  so removing the `isHomed` gate was enough.
* GUI: `_apply_axis_limit(value, previous, lo, hi)` needs `previous`,
  because the simulation integrates a position rather than holding a
  direction. Crossing out from inside stops **on** the line; already
  outside, further out freezes and back toward the band is allowed.

So an axis sitting outside a boundary taught in an earlier session can
always be jogged back in, and no pose can pin it. What a missing reference
still costs is the **meaning of the numbers** — absolute moves — which is
what the status suffix now says. It used to say "soft limits off", which
is no longer true.

**The known gap: HOME re-anchors the frame, and boundaries do not follow.**
`finishHoming()` zeroes all four counters, so a boundary taught *before*
the reference existed points at a different physical place afterwards —
the number survives, its meaning does not. The two ways of setting a
reference are not equally recoverable:

* **RESET COORDINATES** zeroes in place, so the offset is exactly the
  reading at that instant and the boundaries *could* be shifted to keep
  their physical position.
* **HOME** used to be genuinely unconvertible: the PLC drove the axes while
  ClearCore had released them, so the board never counted those steps. That
  is no longer true — the board homes itself now and counts every step it
  drives — but nothing converts the boundaries either way.

This is **deliberately not handled in code** — the user's working practice
is to home first and teach afterwards, and they chose to leave it rather
than carry per-boundary frame tracking. Do not add silent conversion later
without asking: moving a safety boundary to a place nobody chose is worse
than a number the operator knows to re-teach.

The per-axis control **used to be a value LOCK** (`lim_<axis>_locked`,
`SET_LIMIT_LOCK`), which froze the number while leaving the limit
enforced. It was removed, not renamed: it was the only per-axis control,
so the panel could say `UNLOCKED` — an answer about the *number* — while
"is this boundary actually on?" had no per-axis answer anywhere. Values
are now guarded by APPLY alone, like every other tab. Consequences:

* **Enforcement does not gate writes.** A switched-off boundary is still
  editable and still takes `SET HERE` — teaching a boundary while it is
  not yet policing anything is the normal order of work. So `_send_limits()`
  no longer needs the unlock/write/re-lock dance; it sends **values first,
  then the per-axis switches, then the master one**, because the board keeps
  limits in RAM and arming an axis before its numbers land would enforce
  whatever the board happened to be holding.
* **`SET_LIMIT_LOCK` is refused by the board, not aliased.** Mapping
  `lock=1` onto `enforce=1` would let an un-updated GUI arm an axis while
  believing it had frozen a value.
* The button has **three** captions, not two: `ENFORCED`, `NOT ENFORCED`,
  and `ON (MASTER OFF)`. Without the third, four buttons would read
  ENFORCED on a machine enforcing nothing — so `_toggle_limits_enabled()`
  must call `_refresh_limit_enforce()` as well as its own refresh.
* Switching one **off** is confirmed and logged as a warning; switching it
  **on** is neither. Making the machine safer is not an event that needs to
  interrupt anybody.
* No schema bump. Nothing stored changed meaning — the old key is ignored,
  the new one defaults to `True` — and a bump would fire
  `ARM_FRAME_V2_RESET_KEYS` and cost the operator their taught elbow
  boundaries for nothing.

Defaults are inset at the FAR END ONLY (ZM `0..280`, RM `0..335`, elbows
`0..926` motor°) — see 1a for why the lower end is left on the stop. They
used to *be* the factory envelope, which meant the soft limit and the
mechanical stop were the same position and the soft limit protected
nothing.

**`RESET_POSITION` is a different thing from both HOME and RESET
COORDINATES.** It DRIVES the machine to (0,0,0,0) under the board's own
motor control — no PLC handshake, and it skips the M30..M32 block a P2P leg
respects, though taught soft limits still apply. It never sets `isHomed`,
because it re-anchors nothing. Confirmed before it runs.

`RESET_COORD:<Z|ROT|A1|A2>` zeroes one axis. It deliberately does **not**
set `isHomed` — claiming a full reference from one axis would enforce
limits against three counters that are still meaningless.

The reset buttons live in **section 3, MOTION CONTROL**, not Settings: it is
an action used while jogging to the reference pose. They are in
`motion_lock_widgets`, so a counter cannot be zeroed mid-move.

**All three motion panels carry the row** — P2P, JOYSTICK and SCAN.
P2P-only was the first layout and it was the wrong half: declaring the
reference is a jogging job, so the operator had to switch mode to finish
it, and switching mode auto-stops motion. One builder,
`_build_coord_reset_row()` in
`ui/coord_reset.py`, called from each panel; do not hand-roll a second copy,
or a change to the axis list or the confirm path lands on one panel only.
`coord_reset_buttons` is **extended, never reassigned** (both rows must end
up in it, or one panel's buttons stay live during a move), and it is cleared
alongside `motion_lock_widgets` in `_init_state()` **and** on a theme
rebuild — otherwise `set_enabled()` walks destroyed widgets and raises.

### 1d. A stale board cannot inject the old 60 deg frame

v9.2+ telemetry appends `FOLD1`/`FOLD2`/`R1`/`R2`. Their **absence** is how
`_board_reports_motor_degrees()` detects pre-v9.2 firmware, which reported
the elbow in the old frame — 60 at home. Those two values are refused (RM
and ZM still update, their frame did not change) and the operator is told to
re-flash, once per connection.

Without this, connecting to an un-flashed board makes A1M/A2M read 60 and
stay there however you jog, which looks exactly like a fault in the GUI. If
someone reports a phantom 60, check the board's firmware first.

### 2. The arm angle is rotation from home, not `th3_cad`

`A1M_POS` reads **0° at home** and counts up, in MOTOR degrees: straight
out is fold **120°**, which is `120 × 7.80` = **936 motor°**. The working
maximum is fold 90° = **702 motor°** = base +60.

**The panel does not show motor degrees.** The `A1M_BASE` / `A2M_BASE`
cards show the BASE angle — −30 at HOME, +60 at the working maximum — with
the fold angle and the reach in the line underneath. Motor degrees are what
the wire and the taught boundaries carry, and they are not a number anybody
reads off the machine.

The CAD frame (`th3_cad`) survives *only* as `ARM_ZERO_CAD_DEG` inside
`fold_angle_to_reach()` / `reach_to_fold_angle()` in `kinematics.py` and
`reachFromFoldAngle()` / `foldAngleFromReach()` in the firmware. Nothing
prints it: the operator's frame is the base angle, which is `th3_cad − 90`.

Why not report `th3_cad` directly: the board cannot produce one. It counts
steps from its last reference, so the `60°` it used to print at home was
zero rotation wearing a CAD label.

`python_check.py` **fails the build if `th3_cad` leaks into any module
other than `kinematics.py` and `config.py`.** That check is deliberate.

### 3. The elbow boundaries have no envelope and are unordered

* **No envelope** (`floor = ceil = None`, `ARM_LIMITS_UNBOUNDED`). Any
  number is accepted, four figures included. The ratio is measured now, so
  the numbers mean something — but the elbow's ZERO still does not: the
  counter reads 0 wherever the board powered up, so any ceiling is a
  ceiling on an offset nobody knows. A guess that rejects a pose the arm is
  physically standing at stops the operator teaching the machine at all.
* **Unordered.** Teach either end first. Both numbers are stored **raw**
  and sorted where they are *read* — `armBand()` on the board,
  `_limit_pair()` in `jog_control.py`. **Do not sort on write:** that
  folds the first taught position against the stale value in the other
  slot, so the operator's second SET HERE silently overwrites their first.
* Only one arrangement is refused: both ends on the *same* position, which
  pins the axis with no jog out.
* **ZM and RM keep both an envelope and an ordering rule**, because for
  those two the scale is known and a number outside it really is
  impossible.

The two elbow rows are **capture-only** — read-only entries, SET HERE
only. Typing is not the objection any more (the ratio is measured, and the
boxes read base degrees); the elbow's ZERO is. A typed number is typed
against an origin nobody knows, so the position has to come from the
machine.

### 3a. There is no structural REACH envelope either, for the same reason

A `133.2 mm` FLOOR used to be enforced inside `solve_ik()`. The number is
back as the reach at HOME, but the **floor** is not: it assumed the elbow's
zero really is the folded home pose, and it refused `X 0, Y 0` and every
short radius on a machine that may well reach them.

The arithmetic span is wider than the travel in this frame, and that is the
part to watch: `293.2 ± 320` means **−26.8 .. 613.2 mm**, so a radius
*shorter* than HOME still solves — to a NEGATIVE fold angle, which the
panel shows as a base angle below −30. What refuses it is the taught elbow
band, not the geometry.

What is enforced now:

| Check | Where | Switchable? |
| :--- | :--- | :--- |
| `R` within `a3+a6 ± (a4+a5)` = 293.2 ± 320, i.e. **−26.8..613.2 mm** | `_check_reach()`, `solveIkFrogleg()` | **No** — arithmetic. `acos` would clamp and return a pose nobody asked for |
| `d1` within **ZM's stroke** 0..285 | `solveIkFrogleg()`, `jointTargetIsLegal()` | **No** — the top stop is not a setting |
| `rot` within **RM's travel** 0..340 | `solveIkFrogleg()`, `jointTargetIsLegal()` | **No** — no bearing past it is reachable from either side |

A radius the arithmetic cannot reach still raises; what is gone is the
extra structural floor on top of it. Angles outside `0…180` fold are
expected and are not an error — the counter is zeroed wherever the operator
declared the reference, so a taught band may sit anywhere.

### TAUGHT boundaries no longer gate a P2P move at all

**Removed on request.** A P2P point outside the boundary the operator
taught now solves, loads and runs. Three gates went, and they had to go
together:

| Was | Now |
| :--- | :--- |
| `_limit_violation()` refused at LOAD | **warns** in the log, loads anyway |
| `solveIkFrogleg()` refused the solve (Z, ROT, taught reach band) | physical travel only |
| `jointTargetIsLegal()` refused the store (Z, ROT, A1, A2) | physical travel only; elbows unchecked |

Removing only the GUI one is the trap: the panel then believes a program is
loaded while the board has refused to store it, RUN does nothing, and the
only clue is an `[ERROR] Point A rejected` in the log. If you ever put one
of these back, put all three back.

**What still stops a P2P move**, and none of it is switchable:

* the **arithmetic** reach span, −26.8..613.2 mm — outside it there is no elbow
  angle at all
* **ZM's stroke** and **RM's travel** — machine facts, not settings
* the **PLC travel switches**, unchanged: `runLegBlockedByLimit()` still
  refuses a leg that would drive an axis further into a covered switch,
  because a program runs unattended

**JOG is untouched.** `axisLimited()` / `serviceArmSoftLimit()` and the
GUI's `_apply_axis_limit()` still honour the taught bands and the per-axis
switches, so `_axis_enforced()` is still live and still means something —
just not for a P2P target. The firmware change needs a **re-flash**.

Consequences that were easy to miss and are now tested:

* `reach_band_from_motor_deg()` in `kinematics.py` mirrors the firmware's
  `reachBandFor()` — **not** min/max of the endpoints, for the reason in
  section 4. The panel and the board would otherwise advertise and enforce
  different bands.
* The P2P workspace line is **live**, from `_refresh_workspace_hint()`, and
  quotes the taught band. A hard-coded envelope there would name a limit
  nothing applies. It repaints on APPLY and on every enforcement toggle.
* `_limit_violation()` still **describes** a breach — it is what the LOAD
  warning quotes — and still reads the pair through `_limit_pair()`, which
  **sorts** (elbow boundaries are stored exactly as taught and may be in
  either order) and skips an axis whose enforcement is off. It no longer
  *decides* anything; see the section above.
* `solve_ik()` no longer clamps `idle_deg`. It is a measured position, not
  a request; clamping it to `0…120` commanded a move on the arm the
  operator did *not* select, which is the exact thing `idle_deg` exists to
  prevent.
* The board's ZM and RM checks inside `solveIkFrogleg()` now go through
  `axisEnforced()` too, so a switched-off axis is switched off everywhere.

### 3b. HOME is the P2P reference: X 0, Y 0, Z 0

Everything Cartesian on the wire and in the panel is measured from HOME.

| Axis | Origin | Sign |
| :--- | :--- | :--- |
| X, Y | the **turntable axis** | signed — RM can put the arm behind the machine |
| Z | HOME, the bottom of the lift stroke | **never negative**, 0…285 |

`X 0, Y 0` is the centre of rotation, not the tool's position at HOME —
at HOME the arm is retracted and its centre sits **133.2 mm** out, so `0,0,0`
is the reference point and still not a reachable *target*. A frame pinned
to the tool would rotate with RM and stop being a frame at all.

**Z is carriage travel, so it is the same number for both arms.** The
9 mm deck offset is applied per arm inside the conversion, not by the
operator. That killed the old `Z0 - Z1 = 9 mm` rule: `LOAD_XYZ_BOTH` and
the GUI's BOTH mode now want the two Z values **equal**. Subtracting the
drop in `_sync_z_for_both_mode()` as well would apply it twice.

**The maths underneath is still absolute** — arm 1's deck at 514.3 mm with
the lift down — and must stay that way: it is the frame the Z chain and the
firmware both speak, and the IK→FK round trip is checked in it. So the
translation lives at the **edges**, one function each side:

* `z_abs_from_home()` / `z_home_from_abs()` in `kinematics.py`, called by
  `p2p_load_parameters()` and the telemetry readout.
* `solveIkFromHome()` in the firmware, wrapping `solveIkFrogleg()`. Every
  Cartesian command handler goes through it; `solveIkFrogleg()` itself is
  untouched and still absolute.

Both readouts print **both** figures — "…mm above HOME (real … mm)" —
because the from-HOME number is what matches the entry boxes and the
absolute one is what a tape measure would say. The P2P panel also carries
a live **real height** line under each Z entry, from
`_refresh_real_heights()`: it follows the typed value on every keystroke,
is computed **per deck** (so switching arms repaints it even though the
number did not change), shows nothing at all while the field is not yet a
number, and flags a Z outside the stroke before you press LOAD.

Things that had to move with the frame, and will bite if they are missed:
`DEFAULT_POINT_A/B` (were 560 and 650, absolute heights; left alone they
would open every session with two unreachable points), the `_check_d1`
message, the board's `[ERROR] Z=… out of ZM travel` message, and `FK`,
which now answers from HOME with `Zabs` alongside — an FK that answered in
a different frame from the one IK accepts is a round trip that does not
round-trip.

`lim_z_min` / `lim_z_max` needed **no** change: they were always carriage
millimetres, and carriage millimetres are exactly what the typed Z is now.

### 4. `reachBandFor()` does not take min/max of the endpoints

Reach is a cosine of `θ + ARM_ZERO_CAD_DEG`, monotonic only across half a
period. Once a taught band crosses an extreme, the extreme radius is
**inside** the interval, and an endpoint-only answer reports a narrower
band than the arm can actually sweep — which refused ordinary targets. The
function checks every angle in the band where `θ + ARM_ZERO_CAD_DEG` is a
multiple of 180. With the offset at 60 those are fold `120, 300, −60…` —
note that fold 0 (HOME) is **not** one of them, so the band's minimum at
the home end really is its endpoint. The code writes the offset rather than
the numbers, so a frame change needs no edit here.

### 5. Speed is one fixed master RPM × per-axis percentage

`MASTER_RPM = 150` is a **constant, not a field**. Only the percentages
are editable. There is **no percentage cap** — above the recommended value
the field turns amber and warns, and still applies. Engineering ceilings
(`ROT_VEL_MAX_DEG_S`, `Z_VEL_MAX_MM_S`, `ARM_MOTOR_RPM_MAX`) remain as
backstops.

| Axis | Speed % | Accel % | Works out as |
| :--- | ---: | ---: | :--- |
| RM | 50 | 100 | 75 RPM · 69.2 °/s |
| A1M / A2M | 62.5 | 70 | 93.8 motor RPM |
| ZM | 200 | 200 | 225 RPM · 75 mm/s |

**These six numbers came off the machine as the combination that ran
stably.** They are a bench result, not a calculation — do not re-derive
them, and do not tune one without the other five in front of you.

**The accel percentages are their own numbers, not copies of the speed
ones.** They used to be copies — `rotAccPct = ROT_PCT_DEF` and so on — and
that is what made the arm feel broken. Acceleration is what decides how far
an axis carries on after the key is released:

```
coast = v² / 2a
```

At 125% speed and 125% accel the arm ramped for 0.40 s and coasted **225
MOTOR degrees** — every time the operator
let go. RM and ZM never showed it because their gearing divides it out; the
arm has **no gear reduction in the velocity calculation**, so at equal
percentages it runs two orders of magnitude faster in output terms while
sharing the same acceleration budget. That asymmetry is the thing to
remember, and it is why `DEFAULT_ARM_ACC_PCT` must not be "tidied up" back
to `DEFAULT_ARM_PCT`.

At the settings above the coast is ~100 motor° on the arm, ~6.9° on RM and
**~15 mm on ZM** — the lift is the one to watch by hand, though a scan is
unaffected because `SCAN_SPEED_SCALE` cuts the velocity and not the
acceleration, leaving ~0.6 mm.

Both sides carry the same six numbers — `robot_sim/config.py`'s
`DEFAULT_*_PCT` / `DEFAULT_*_ACC_PCT` and the firmware's `*_PCT_DEF` /
`*_ACC_PCT_DEF` — and `python_check.py` reads the `.ino` to assert they
agree. A drift would have the panel advertising a ramp the board does not
use until the GUI happens to send `SET_SPEED`.

The arm is bounded in **motor RPM**, not °/s. The old 100 °/s ceiling was
removed because the arm gearing makes it far too slow.

> Note for anyone tempted to "correct" the defaults: at real gearing, "AM
> percentage higher than RM" is only coherent because the percentages are
> of motor RPM, not of output speed. This was raised with the user with
> numbers and they chose it deliberately.

**A saved `machine_settings.json` beats these defaults**, which is correct —
the operator's stored choice wins — but it means changing a default here is
invisible on a machine that already has a settings file. Press DEFAULTS then
APPLY on the Speed tab to actually adopt them.

---

### 6. The PLC link is MC Protocol, it is READ-ONLY, and HOME no longer uses it

The PLC is a Mitsubishi at **192.168.3.101:1025**, and ClearCore is an MC
Protocol **3E BINARY** client (`#define PLC_MC_ASCII 0`). It batch-reads
**three words, `M0..M47`**, so the three bits it needs land in one round
trip. **It writes nothing, ever.**

> **BINARY, not ASCII.** The code must match the PLC's own *Communication
> Data Code* on the Ethernet Configuration screen. A mismatch is not a
> partial failure: the PLC silently drops every frame in the wrong format,
> which is exactly what the machine did before this was corrected.

| Device | Meaning | Direction |
| :--- | :--- | :--- |
| `M32` | ZM travel limit switch — bottom of the stroke | read |
| `M31` | RM travel limit switch — the CW end | read |
| `M30` | A2M travel limit switch — **wired at both ends** | read |

**Those three are the only devices read.** `M1` (DONE), `M5`–`M8` (the old
home sensors) and `M10`–`M13` (run) are gone from **both** sides — deleted,
not muted, and both suites assert their absence. They lit a lamp and
decided nothing, while `M30` was the bit actually refusing a jog and had no
lamp anywhere: an operator watched "M5 ZM lift = CLEAR" while ZM would not
move down.

**HOME IS DRIVEN BY THIS BOARD, and the PLC is not asked.** `beginHoming()`
drives each axis in `HOME_DIR_*` at `HOME_SPEED_SCALE` (25%) through the
ordinary jog velocity path, and stops each one the instant **its own**
switch bit reads covered; completion is `M30 && M31 && M32` through the
per-axis enforce flags. There is no request line, no DONE bit and no
PLC-side sequence to wait on.

Why it changed: HOME used to assert a wire (ClearCore `IO-0` → PLC `X0`)
and wait for the PLC's own home sequence. **Nothing on the PLC side ever
ran that sequence**, so HOME simply sat there and timed out. The board
already owns the motors and already reads the switches.

Consequences, all asserted:

* **`plcAssertHomeRequest`, `plcClearHomeRequest`, `plcHomeDoneAsserted`
  and `PLC_HOME_REQ_PIN` are gone.** So are `plcFrameWriteBit()`,
  `PLC_MC_CMD_WRITE` and `PLC_MC_SUB_BIT` — there is no write path at all,
  not even an unused one. If a future feature must set a PLC device, use an
  **internal relay (M, decimal numbering)** — never an X, which the PLC
  refreshes from its physical terminals every scan — and write down why.
* **`HOME_DIR_*` is NOT `PLC_LIMIT_END_*`.** Which way a covered switch
  refuses, and which way HOME goes looking for it, are separate facts that
  happen to agree per axis. All three are **negative** — ZM down, A2M
  retract, RM counter-clockwise. Sharing one constant meant a wrong end
  sent HOME the wrong way with nowhere separate to correct it.

* **`PLC_LIMIT_END_ROT` read `+1` and that pinned RM outright.** HOME
  drives RM onto `M31` and `finishHoming()` calls `PositionRefSet(0)`
  there, so **RM's zero IS its switch**. Calling that zero the axis
  *maximum* made `_sensor_violation()` refuse every P2P point with
  `rot > 0` as "further into" a covered switch — the operator saw
  `RM turntable is sitting on M31 at its MAX end … (0.00 -> 45.00)` on an
  ordinary point — while the one direction left, CCW, ran straight under
  `lim_rot_min = 0`. An axis cannot stand on its minimum and on its
  maximum switch at once. If a similar refusal ever appears on another
  axis, check that its switch end agrees with where its zero is set.
* **The jog watchdog must ignore a home.** HOME is not a jog, so no
  `JOG_HB` arrives and the 700 ms watchdog cancelled the move — which made
  HOME look like it did nothing at all. `serviceJogWatchdog()` returns
  early while `isHoming`, and the same exemption covers a scan.
* **HOME is REFUSED without device data.** The switches are how it knows
  when to stop, so `beginHoming()` fails immediately and says so rather
  than driving four axes blind. Timeout is 30 s.
* **A1M has no switch fitted** and is not moved by HOME. An axis already
  sitting on its switch, or one whose switch is switched off, is not driven
  either — driving further into a covered switch is the one direction that
  must never be commanded.
* `tests/stub/ClearCore.h` **records** `digitalWrite` into `PIN_LEVEL`. A
  swallowing stub would let a pin test pass while the terminal never moved
  — the same trap `Serial.println` was in.

**Vestige, deliberately left:** `config.py` still carries
`PLC_HOME_REQUEST_DEVICE = "X0"`, `PLC_HOME_REQUEST_SOURCE` and the `X0`
row in `PLC_DEVICE_MAP`, and `python_check.py` still asserts them. They
document the wire that *was* there and are read by nothing else; the
firmware is the authority and it homes itself. Do not build anything new on
them.

Do not go back to v9.0's `"M2\n"` → `"DONE"` line protocol. It needed a
SOCOPEN/RECV ladder written on the PLC to parse it; MC protocol answers
device reads with no ladder code at all, and the handshake is visible in
GX Works while it runs.

Timing, and why each number is what it is:

* **The poll has TWO rates, and the fast one is load-bearing.**
  `PLC_POLL_IDLE_DEF_MS = 20`, `PLC_POLL_HOMING_MS = 10`, chosen on
  `isHoming`. During a home the switch state IS the stop signal, so the
  poll interval is how long an axis keeps moving after it arrives. Do not
  "simplify" this back to one rate. `SET_PLC_POLL:<ms>` changes the idle
  rate at runtime and is not persisted.
* **The socket timeout (`PLC_TXN_TIMEOUT_MS` 800 ms) must stay longer than
  the PLC's CPU monitoring timer (500 ms).** Otherwise the board abandons a
  reply the PLC is still going to send, the late reply arrives against the
  *next* request, and a boundary pattern is applied one cycle late. On a
  timeout the socket is dropped deliberately — that is the only certain
  resynchronisation. `PLC_RECONNECT_MS` is 3000.
* **If HOME never starts, check the link first.** `PLC_TEST` does one
  blocking read and reports the PHY link, the TCP connect, the exact frame
  sent and what came back — which separates "no cable", "socket open but
  not speaking MC protocol", "wrong data code" and "PLC refused the
  device". `PLC_DEBUG:1` echoes every frame; `PLC_STATUS` adds
  `[PLC_COUNTS]`.

### A2M's switch is wired at BOTH ends, and the BOARD decides which

**Read this before touching `PLC_LIMIT_END_A2`.** A2M has one PLC device,
`M30`, and two physical switches on it: one at the retracted end, one at
full extension. The bit therefore cannot say which end tripped it. The
**direction the axis was travelling on the rising edge** can, and that is
the whole mechanism -- `plcServiceLimitLatch()` on the board records it,
and only that direction is refused. The opposite stays available, always,
or the arm would be pinned on its own switch with no way off.

Four consequences, each paid for:

* **The latch runs BEFORE `plcServiceLimitStops()`**, which is what zeroes
  the direction the latch reads. Swap them and the end is always 0.
* **The direction is REMEMBERED, not read live**, and that ordering rule
  above was not enough on its own. `plcServiceLimitStops()` is only one of
  the things that zeroes a direction; the taught **soft** limit and the jog
  watchdog do too, they run every loop pass, and the PLC bit only arrives
  on a poll. For a fully extended arm the soft limit sits at essentially
  the same place as the physical far switch, so it won every time: the
  latch saw a rising edge with nothing moving, assumed the home end, and
  reported a far-end trip as **COVERED MIN**. That is not cosmetic —
  `plcLimitSensorSatisfied()` then accepts a fully EXTENDED arm as the home
  reference and zeroes the counters at the wrong end of the travel.
  `plcRememberTravelDir()` runs **first in `loop()`**, before anything that
  can zero a direction, and keeps the last non-zero one for
  `PLC_TRAVEL_DIR_MEMORY_MS` (1000). Bounded on purpose: a direction from a
  second ago is not evidence, so past the window it goes back to assuming
  the home end and **says** it assumed.
* **`PLC_LIMIT_END_A2` stays `-1`.** It is no longer "the end" -- it is the
  HOME-side end: the fallback when nothing is latched, the end HOME drives
  toward, and the only end that may count toward the home state.
* **A far-end trip is NOT the reference.** `plcLimitSensorSatisfied()`
  requires the effective end to be the home end, and the GUI mirrors it in
  `plc_sensor_at_home_end()`. Without that, an arm parked fully EXTENDED
  satisfies the home state and zeroes the counters 90 deg from where the
  machine actually is.
* **HOME drives off a far-end trip first.** `homeWaitForClear[]` makes that
  axis ignore its bit until the switch clears, because the bit is already
  on and means the opposite of arrival.

**The GUI never derives the end itself.** It cannot see the edge between
two polls, and a wrong guess refuses the one direction that comes off the
switch. The board reports it in `PLC_STATUS` as `end Z/R/A2=-+-`;
`_read_plc_limit_ends()` adopts it, and a board too old to send the field
leaves the home-side end in place -- which is what that firmware enforced
anyway.

**Known gap, deliberate:** a board that BOOTS with the bit already on has
no edge to latch from and assumes the home end. Powering up parked at home
is the normal case and the assumption is right there. The alternatives are
worse: refusing to guess either pins the axis or blocks HOME forever, and
HOME is what would produce the edge.

Jog still only **warns** at either end. Jog is how you come off a switch.

### THREE travel-limit switches, and they do NOT sit at the same end

The switch row is built by **all three** motion panels from
`ui/sensor_panel.py`, like the coordinate-reset row.

| Bit | Axis | End it sits at | Jog command that drives INTO it |
| :--- | :--- | :--- | :--- |
| `M32` | ZM | **minimum** — bottom of the stroke | `Z_DOWN` |
| `M31` | RM | **minimum** — the CCW end, where HOME parks it | `ROT_CCW` |
| `M30` | A2M | **minimum** — retracted — *and the far end too* | `A2_BACK` |

**A1M has no switch fitted.** There is deliberately no `PLC_M_LIMIT_A1`.

**ZM and A2M are SWAPPED from the tidy numeric order**, measured on the
machine: `M32` follows ZM, `M30` follows A2M. The board originally assumed
`M30 = ZM`, so ZM watched a bit that sits at 1 and — with its switch at the
minimum end — every `Z_DOWN` was refused wherever the carriage actually
was. That fault was chased through soft limits, gear ratios and poll rates
for a whole session; none of those could have fixed it, because the bit
being read was never ZM's. `PLC_M_LIMIT_*` on the board and
`PLC_SENSOR_PANEL` in the GUI are the one place each side states it, and
`python_check.py` asserts the two agree.

`PLC_LIMIT_END_*` / the last field of `PLC_SENSOR_PANEL` say which END each
sits at. "Covered" therefore means the opposite thing for RM than for the
other two, which is why the lamp caption names the end — reading the lamp
without it is guesswork.

**These DO stop an axis**, unlike the `M5`–`M8` home sensors they replaced.
While covered, an axis may not drive FURTHER INTO its switch; the opposite
direction stays available, always, or the machine would be pinned on its
own limit with no way off.

**P2P ENFORCES, JOG ONLY WARNS.** This asymmetry is deliberate:

* A P2P leg that would drive an axis further into a covered switch is
  **refused** — `runLegBlockedBySensor()` on the board,
  `_sensor_violation()` in the GUI. A program runs unattended and the
  operator is not watching that axis, so it must not start.
* Jog **warns and proceeds**. Jog is a dead-man control: it moves only while
  held, the operator is looking at the machine, and jogging is how you come
  OFF a tripped switch. Blocking it would also risk pinning the machine.

Both checks compare against the **live pose**, so they answer "would this
move make it worse", not "is a switch covered". An axis that is not moving
is never refused.

If you find yourself re-adding a jog block, the previous revision had one
and it was removed on request. `PLC_SENSOR_BLOCKS_*` is gone from the
firmware, and both suites assert its absence.

**HOME STATE = M30 and M31 and M32 all covered** — the same three bits
HOME itself completes on. It is the one condition allowed to zero the
counters unasked, **edge-triggered** (holding at home would otherwise
re-zero every poll and eat real motion) and **refused while anything is
moving**, re-arming once stopped. A bit that is on because A2M is fully
EXTENDED does not count — see the both-ends note above.

Nothing a switch reports ever writes a working boundary. Boundaries come
from the operator only.

### P2P runs HOME -> A -> B -> HOME

Four legs, on the board (`PHASE_TO_HOME_FIRST` .. `PHASE_TO_HOME_LAST`, all
started through `beginRunLeg()`) and in the offline simulation. Starting and
ending at the reference is what makes the cycle repeatable, and it matches
`mophong_init.m`'s `P_home -> A -> B -> P_home`.

### SCAN is a third MODE, and its speed is DERIVED

`robot_sim/ui/scan_panel.py` + `robot_sim/core/scan_control.py`, beside P2P
and JOYSTICK. It sends `SCAN_START` to the **same** firmware over the
**same** link — the board sweeps through its jog primitives, so soft
limits, PLC switches and E-STOP apply to a scan exactly as to a held key.
`Scan/` still exists and still runs *simulated*; this mode does not
simulate, because the console has no fake sensor and a made-up point is
worse than none.

**The panel asks for four numbers and derives the rest** (`scan_plan.py`):

```
t    = points / sample_hz     seconds a slice
w    = sweep / t              deg/s RM must turn at
step = sweep / points         degrees between samples
lift = spacing * (slices - 1) total ZM travel
```

50 Hz and 50 points over 330° is 330° in **one second**; 100 points is two.
That is why the point count is the input and the speed is the output —
the operator knows what the sensor can deliver and how finely they want a
slice, not what deg/s that implies.

Consequences that will bite:

* **`SCAN_START` gained a fifth field, `rotDegS`, optional and LAST.** A
  board flashed before it reads four fields and ignores the fifth, falling
  back to `SCAN_SPEED_SCALE`. Do not reorder the fields.
* **The clamp is on RM only.** `scanRotScale()` scales the turntable; ZM
  keeps `SCAN_SPEED_SCALE`, whose small coast is what makes the lift safe.
  The scan's speed suits the *sensor* and says nothing about the lift.
* **Asking for more than RM can do is a WARNING, not a refusal.** The board
  samples by POSITION, not by clock, so a clamp costs time and not data —
  the points land at the same angles. Both sides say so.
* **A slice holds `points + 1`**, because the first sample is taken at the
  reference angle before the turntable moves. `deg_step = sweep / points`,
  NOT `/ (points - 1)`: that makes the count exact and the speed wrong.
* **`scan_max_z_mm` (Settings → Scan, default 180 mm) WARNS and asks.** It
  is the operator's own working ceiling; `D1_MAX_MM` on the board is the
  hard refusal. Total lift is `spacing * (slices - 1)` — using `slices`
  would refuse scans that fit.
* **`[SCAN_PT]` is telemetry** (`TELEMETRY_PREFIXES`), so it is parsed and
  never logged, and the plot repaints on a timer. At a 1° step a layer is
  341 points; a line and a canvas redraw each would spend the scan drawing.
* **Nothing in `_on_scan_line()` logs.** The RX pump has already written
  every non-telemetry line; a `self.log()` there prints each reply twice.
* Mode switch and E-STOP both end a scan — the mode switch sends
  `SCAN_STOP` first, E-STOP does not, because the board's own `ESTOP`
  handler already calls `cancelScan()`.

---

### The Motion tab picks a RAMP SHAPE, and the board executes it

`robot_sim/motion_profile.py` is a port of
`Compare_Angular_Motion_Profiles.m` — the same three profiles, the same
maths, so the curve in Settings and the figure in the report agree. The .m
stays the reference; symbol names mirror it (`Theta`, `omegaMax`, `Vp`,
`tJ`, `tA`, `tV`, `J`) for eyeball diffing.

Speed and accel say *how fast* and *how hard*. A profile says what
acceleration does **between** them:

| | Ramp | Jerk | 180° at RM's defaults |
| :--- | :--- | :--- | ---: |
| Trapezoidal | steps between three values | **infinite** at each corner | 2.80 s |
| S-curve, `rS = 0.5` | half the ramp eases in/out, 7 phases | bounded | 2.90 s |
| Pure S-curve, `rS = 1` | no constant-accel phase at all | bounded, half the above | 3.00 s |

**Smoothness costs time** — bounded jerk cannot reach the same average
acceleration — and the panel prints the total so it cannot look free.

Four things worth not undoing:

* **`NONE` is kept distinct from `TRAPEZOIDAL`** even though the shape is
  the same. `NONE` means "not in play, the board ramps as it always did";
  `TRAPEZOIDAL` is a positive choice of that shape. A menu whose off
  position is spelled like one of its options cannot say which was meant.
* **Trapezoidal returns `j = None`, not zeros.** Its jerk is infinite at
  the corners, and a list of zeros is a quieter lie than no answer.
* **The preview uses RM's own speed and accel**, live from the Speed tab,
  not the .m's textbook 60/120 — a curve drawn against numbers this
  machine does not use answers nothing. It falls back to the stored values
  on half-typed input and never raises.
* **An unknown stored profile falls back to `NONE`**, never to whichever
  is first, so a settings file from a newer build cannot silently select a
  different shape.

**THE BOARD EXECUTES IT — by INTERPOLATION, not by asking.** ClearCore's
step generator has exactly two knobs, `VelMax` and `AccelMax`, so it can
produce a trapezoid and nothing else. A profiled leg is therefore driven
as a moving setpoint: `commandRunLeg()` plans the profile, and every
`serviceRun()` pass commands each axis to `start + u·(target − start)`,
where `u` comes from `profileAt()`. The generator only ever chases a
setpoint that is already the right shape, which it can do because the
setpoint never asks for more than the axis's own limits.

`SET_MOTION_PROFILE:<NONE|TRAPEZOIDAL|SCURVE|PURE_SCURVE>`. Held in RAM
like the limits, so `_push_settings_to_board()` re-sends it on every
handshake — otherwise a board that rebooted mid-session would go back to
its own trapezoid while the panel still showed an S-curve.

Five things that will bite:

* **`PROFILE_NONE` is not a shape, it is the old code path.** One
  `Move(MOVE_TARGET_ABSOLUTE)` per axis, exactly as before. Nobody who
  ignores this tab gets different motion, and a firmware check asserts it.
* **ONE TIME BASE FOR ALL FOUR AXES.** `u` is shared, and the profile's
  limits are the tightest any axis imposes — `min` over axes of
  `vmax/|Δ|` and `amax/|Δ|`. So a profiled leg also **coordinates** the
  axes: they start together and finish together. The unprofiled path
  issues four independent `Move()` calls that finish whenever they finish.
  That is a behaviour change beyond smoothness, worth knowing before
  comparing the two by eye.
* **The interpolation runs BEFORE the `allMotorsSettled()` test** in
  `serviceRun()`. The setpoint is only ever slightly ahead of the axes, so
  they *are* momentarily settled between updates; testing first ended the
  leg on its first pass.
* **The last setpoint is the exact target**, commanded once when the clock
  runs out. Float arithmetic on a millisecond tick would otherwise leave
  the leg a fraction short of the number the operator typed.
* **`SET_MOTION_PROFILE` is refused while moving**, and `cancelRun()`
  clears `runProfileActive`. Swapping the plan under a running
  interpolation is a step in the setpoint — the exact discontinuity the
  feature exists to remove.

**JOG gets the USEFUL HALF now — ease up, ease down — never the whole
profile.** A full profile needs `Theta` up front (`tV = Theta/Vp − Ta`),
and a held key has never decided how far it is going; that half stays out
of jog by construction, always will. What a key-down DOES know the moment
it fires is the axis's own jog speed and accel, which is exactly the
s-curve's ramp-up math (`tJ`, `tA`, `J`) with `Theta` and the cruise `tV`
term left out — `armJogRamp()` / `jogRampV()` in the firmware. `NONE` and
`TRAPEZOIDAL` are untouched: `TRAPEZOIDAL`'s jerk is already infinite at
the corners, which is exactly the old instant `MoveVelocity()` step, so
there is nothing to add for it. Only `SCURVE`/`PURE_SCURVE` ease.

Two things kept deliberately narrow:

* **Only a voluntary `*_STOP` / `stopArmJog` eases down.** Every safety
  stop — soft limit, PLC limit, watchdog, `cancelJog()` (ESTOP/STOP) —
  still calls `MoveVelocity(0)` directly and unconditionally, and
  `cancelJog()` clears every axis's ramp state too, so a stale ease can
  never re-issue a nonzero command after it. Coasting further into a
  limit on the way out would defeat the stop.
* **A release BEFORE the ramp-up reached full speed is a hard stop, not a
  mirrored ease.** Solving that would need the ramp's current position
  worked backward, which is exactly the closed-loop problem `Theta` lets
  the ramp-up side skip. `releaseJogRamp()` says so and the caller falls
  back to the old `MoveVelocity(0)`.

**SCAN gets the WHOLE profile, both ends — because a scan leg has a
length.** That is the entire difference from a jog: a sweep is
`scanSweepDeg` and a lift is `scanZStepMm`, both known before the axis
moves, so `Theta` exists and the deceleration can be **planned**. The axis
arrives at the target already at rest instead of being stopped there.
`scanPlanMove()` / `scanMoveTick()` / `serviceScanMoves()`, planned in
PULSES so the velocity that comes out is what `MoveVelocity()` wants.

**SEEK is the exception and keeps the ease-up half only** —
`scanSeekRotMove()`. It is looking for a switch, so its length is not
known, and you cannot plan a move whose end you have not found yet. Same
reason jog only ever gets that half.

**Still VELOCITY-driven, never position-driven, and this is the load-
bearing part.** A run leg is interpolated as a moving position setpoint; a
scan must not be, because every soft limit, every PLC travel switch and
E-STOP stop this machine by zeroing a jog direction and calling
`MoveVelocity(0)`. A position setpoint would be re-commanded on the very
next service pass and drive straight back through the stop. So the profile
is applied as a velocity SHAPE over the same `rotDir`/`jzDir`, and
`scanMoveTick()` gives the axis up the instant it sees the direction
zeroed under it.

**The CREEP is the honest part.** An open-loop velocity plan run on a
clock does not land exactly — a blocking sensor read stretches a service
pass, the generator lags the commanded velocity — so the plan hands over
to a slow creep at its tail and the leg ends on the condition that
actually matters: the sweep angle, or the RM switch. Bounded by
`SCAN_CREEP_MAX_DEG` / `_MM`; past that the plan and the machine disagree
by more than slop explains, and it says so.

The profiled approach is *better* at the switch, not worse: the return leg
now decelerates into it and arrives at creep speed, where the old flat
sweep ran at full speed until the bit tripped.

**It costs almost nothing in time.** Accel is not scaled by the scan (only
speed is), so the ramps are short against a 16 s sweep: **+0.06 s**
trapezoidal, **+0.09 s** S-curve, **+0.12 s** pure S-curve on 340°.

**A scan stop must never go through `applyJogVelocities()`.** That
function deliberately SKIPS an axis that is mid-ease, because
`serviceJogRamps()` owns it, so `rotDir = 0; applyJogVelocities();` would
leave a half-ramped axis running with nothing left to command it to zero.
That was live for a while: an abort, a `SCAN_STOP` or an E-STOP during the
ramp-up left the turntable turning. The `scanStop*()` helpers exist to make
the stop unconditional, and the firmware suite asserts each one.

`PROFILE_NONE` plans nothing and a scan is exactly what it always was —
one flat `MoveVelocity()` per leg. **`TRAPEZOIDAL` DOES apply to a scan**,
unlike to a jog: jog skips it because its corner jerk *is* the step a
plain `MoveVelocity()` already gives, but a leg with a known length is a
different question and the trapezoid is a positive choice of that shape.

The scan's SPEED still comes from `scanRotScale()`, for a different reason
again (the sensor, not smoothness); the profile decides only how it gets
there.

The measured coast on an UNPROFILED release is still ~100 motor° on the arm
and ~15 mm on ZM; the profiled ease is a bit longer again (bounded jerk
costs distance same as it costs time — see the run-leg table above), which
is one more reason `LIMIT_SAFETY_MARGIN` insets the far end rather than the
boundary being flush with the stop.

`tests/stub/ClearCore.h` **records `Move()`'s target** (`lastMoveTarget`,
`moveCalls`). It used to swallow it, which was fine while a leg was one
call; a profiled leg is a sequence of setpoints, and a swallowing `Move()`
would let every test of that sequence pass while nothing moved — the same
trap `Serial.println` and `digitalWrite` were in. **`MoveVelocity()`
records now too** (`lastVelocityCmd`, `velocityCalls`) — same trap, one
call late: a jog ramp is a sequence of `MoveVelocity` commands walking a
velocity, and a swallowing one would let every ease/release assertion
pass while nothing was ever actually commanded in between.

**Known gap, not closed: the GUI's offline jog simulation does not mirror
this.** `core/jog_control.py`'s simulated jog still snaps straight to full
speed and back — it integrates a position per tick rather than walking a
ramp, and nothing there reads `SCURVE`/`PURE_SCURVE`. The board's real jog
now eases under those two profiles; the on-screen preview does not. Same
category as the P2P soft-limit mirroring in `_axis_bounds()` — if this is
ever closed, that is the precedent to follow, not a second one-off.

### The Oxy board draws a CHORD, not the tool path

`ui/xy_board.py` plots the reachable annulus, the unreachable RM wedge, the
taught RM band, HOME, A, B and the live pose. Two things are deliberate:

* The **scale is fixed** to the outer reach. A plot that rescales itself
  cannot be compared between runs by eye.
* The A->B line is **straight because that is the operator's intent**. The
  machine's real path is a joint-space move that bows away from it, and the
  caption says so — somebody checking clearance needs the swept arc, not
  the chord.

It reads the entry boxes on every keystroke, so it must never raise on
half-typed input (`-`, `1e`, empty). `_xy_points()` returns None instead,
and the tests feed it exactly those strings.

### ZM lead and the arm ratio are BOTH settled now

`zMmPerRev` and `armGearRatio` are the two numbers that turn counts into
real units, and **both have been confirmed on the machine**:

| | Value | How |
| :--- | ---: | :--- |
| `zMmPerRev` | **20** | commanded millimetres are real millimetres |
| `armGearRatio` | **7.80** | 575 mm reach at full extension — see section 1 |

They stay runtime-settable — `SET_Z_LEAD:<mm>`, `SET_ARM_RATIO:<r>` — so a
changed lead screw, pulley or gearbox needs no re-flash. Nothing taught has
to be re-taught either way, because every taught elbow boundary is stored in
motor degrees and every ZM boundary in millimetres.

If Z ever travels **3x** the commanded distance the true lead is 3 x 20 = 60
mm/rev. A non-power-of-2 error points at the mechanics; the driver's
microstep switches can only ever err by powers of two. Measure over 100 mm,
not 10 — a wrong ZM lead moves where every ZM soft limit physically is.

### All three modes share ONE live pose

`current_joints` is the single store; `sim_z` / `sim_rot` / `sim_a1` /
`sim_a2` are **properties onto it**. They used to be a second copy — jog
integrated `sim_*`, P2P integrated `current_joints`, and nothing kept them
together, so jogging and then switching to P2P ran the program from
wherever P2P last left off rather than from where the arm actually was.
Every panel repaints from either update path, and SCAN reads the same
store for the height its first slice starts at.

### UNKNOWN sensor data must never render as CLEAR

The switch lamps used to be built showing `CLEAR` and only changed when a
poll landed. So a dead MC-protocol link showed a row of `CLEAR` lamps,
indistinguishable from "nothing is covered" — on a safety display the
failure read as good news. That was a real field bug: a switch was
physically ON and the panel said CLEAR.

Three parts to the fix, and all three are load-bearing:

* `plcStatusSummary()` sends `NO DEVICE DATA | limit Z/R/A2=??? end
  Z/R/A2=???` when it has none. It used to return a bare `"no data"` with
  **no bit field at all**,
  so the GUI's regex matched nothing and simply never updated. Silence is
  what read as CLEAR.
* `plc_sensor_data_seen` gates every consumer — the lamps, `plc_home_state()`
  and therefore the automatic coordinate reset. The stored bits are NOT
  zeroed when data goes stale; the flag is the authority, so a future reader
  that forgets the gate fails loudly rather than reading a fake `False`.
* Losing the serial link calls `_plc_link_lost()`, which marks the sensors
  unknown as well as clearing the lamp. A stale `CLEAR` from three minutes
  ago is worse than no reading.

### The PLC lamp reports DATA, not the socket

A lamp that followed the socket **flapped** CONNECTED / UNREACHABLE every
few seconds on the machine. It was not lying — the socket really was
cycling:

```
TCP connect OK -> poll -> no reply -> 800 ms timeout -> socket dropped to
resynchronise -> PLC_RECONNECT_MS later it reconnects -> repeat
```

It was answering "is a socket open" when the operator is asking "is device
data arriving", which is what HOME and the sensors actually depend on. The
board now reports two extra fields and the lamp is driven from them:

| Field | Meaning |
| :--- | :--- |
| `data=NONE\|STALE\|OK` | whether device reads are landing |
| `conn=<ok>/<tries>` | how many TCP connects **succeeded** |

`conn` matters on its own: a socket that has opened even once proves the
cable and address are fine, so a currently-closed socket is **NO REPLY**,
not UNREACHABLE. Without it the lamp still alternated between those two as
the socket cycled — the same flap one level down. `UNREACHABLE` now means
`conn=0/N`: never opened at all.

**That flapping is itself a diagnosis.** It means TCP connects and MC
protocol does not answer, so the board says so in the log rather than
leaving it to be guessed, and the connect line reads "TCP socket open",
never "connected". The connect message is rate limited, because reconnecting
every 3 s forever otherwise buries everything else.

### A dead device read stops HOME before it starts

HOME stops each axis on **its own switch bit**, which arrives only by
device read. With no successful read the board would be driving four axes
blind, so `beginHoming()` **refuses outright** — `[HOME] FAILED — no PLC
device data` — instead of starting a move it cannot end. Same cause, two
symptoms: stale switch lamps and a HOME that will not run. Check the link
before suspecting the PLC.

**`PLC_TEST` is the command to run when the sensors read stale.** One
blocking read, reporting the PHY link, the TCP connect, the exact frame sent,
and what came back — which separates "no cable", "socket open but not
speaking MC protocol", "wrong data code", and "PLC refused the device" from
each other. `PLC_DEBUG:1` echoes every frame; `PLC_STATUS` adds
`[PLC_COUNTS]` with connect/send/read/timeout totals.

`SET_PLC_POLL:<ms>` changes the idle rate without a re-flash. It is not
persisted, so a power cycle returns to `PLC_POLL_IDLE_DEF_MS`. A slow poll
was never the cause of a stale reading: a working link updates within one
interval, a dead one never updates at all.

The board also **pushes** `[PLC_STATE]` whenever the status word changes, so
the GUI is event-driven instead of waiting to ask.

---

### The third status lamp is the PLC link, not the heartbeat

`HEARTBEAT (3s)` is gone. COM PORT and CLEARCORE already show serial
health, so the lamp said nothing new; whether the Mitsubishi is reachable
was shown nowhere and decides whether HOME can work at all.

`plc_led_card` has four states — `NO LINK` / `CONNECTED` / `NO REPLY` /
`UNREACHABLE` — driven from the board's `[PLC_STATE]` reply. `NO REPLY`
(socket open, no device data) is kept distinct from `UNREACHABLE` (socket
closed) because they are different faults: the first is almost always MC
protocol not enabled on the port, the second is cable or address.

The heartbeat **mechanism stays** — it is what notices a dead board and
fires the all-stop. It just no longer lights anything. Its 3 s tick also
sends `PLC_STATUS`, so there is one cadence rather than two.

`_set_plc_led()` logs **only on a change of state**. At a 3 s poll, a line
per reply would bury the log, so the raw `[PLC_STATE]` is never logged.

---

### `M5`-`M8` are GONE, and no PLC bit ever writes a boundary

The old home sensors were read, lit a lamp and decided nothing. They are
deleted from both sides — `plcServiceHomeSensors()` and
`plcAllHomeSensors()` with them — and the three bits that replaced them,
`M30`-`M32`, are travel limits that really do stop an axis.

What has NOT changed is the rule underneath: **nothing the board reports
ever writes a limit.** `[LIMITS]` is logged and never parsed back, there is
no `[PLC_LIMIT_SET]` message and no `_on_plc_limit_set()` handler, and the
GUI is the sole system of record. If you find yourself re-adding
`PLC_LIMIT_DIR_*` or a boundary-adoption path, that is the mistake.

Working boundaries come from the operator only — typed, or taught with
`SET HERE`. Physical protection beyond the soft limits is the PLC's own
ladder.

---

## Conventions the user has asked for

* **English throughout** in all user-facing strings. The two Vietnamese
  wire strings are gone: the board now sends `[RUN] TARGET REACHED` and
  `[ESTOP] EMERGENCY STOP`. `protocol.py` still *accepts* the old ones so a
  board running v8 keeps working — that back-compatibility is the only
  reason they appear anywhere, and `python_check.py` enforces the
  distinction. `RobotMotionController_v8_ClearCore/` and `clearcore/` were
  left in Vietnamese on purpose: they are historical records of what
  shipped, not live code. `MATLAB_v4_final/` is read-only by request.
* **No presets where the operator's own choice is the point.** Jog
  keybinds have no preset layouts and no advice about which keys sit near
  which; PID has one gain set and no controller-form selector. Both were
  explicitly removed after being built. Do not reintroduce them.
* **EMERGENCY STOP is on EVERY motion panel** — P2P, JOYSTICK and SCAN. It was removed from JOG
  once — jog is a dead-man control and SPACE fires the same path — and
  put back by request. Both arguments were true and neither helps someone
  with a hand on the mouse looking at the machine; a stop control whose
  location depends on the current mode is its own hazard. Both buttons
  call the one audited `emergency_stop_all`; a second stop implementation
  is the thing to prevent, not a second button. Neither is in
  `motion_lock_widgets` — that list is disabled while the machine moves,
  which is when the button has to work. SCAN's STOP button is out of that
  list for the same reason.
* **The Xbox/gamepad input is GONE, not merely unwired.** Jog is keyboard
  and on-screen pads only; `core/gamepad_control.py` is deleted and both
  suites assert its absence. The eight jog commands are untouched — the pad
  was only ever another input onto `jog_start()` / `jog_stop()`.
* **Telemetry is PARSED, never logged.** `[JOG POS]`, `[CLEARCORE POS]` and
  `[SCAN_PT]` are in `TELEMETRY_PREFIXES`, so they update the readouts and
  never touch the event log — 20 inserts a second, each forcing a scroll
  and a repaint, cost more than the jog itself. `JOG_HB` goes out with
  `log_tx=False` for the same reason.
* **Nothing applies on keystroke.** Edits stage until APPLY — except the
  two live readouts (real height, and the boundary enforcement captions),
  which report state rather than stage a change.
* **Per-section APPLY / DEFAULTS.** A global reset that wiped taught
  boundaries because someone undid a speed change costs an afternoon of
  re-teaching. The seven tabs — **Speed · Motion · Boundaries · Scan · Controls ·
  PID · Appearance** — each own their buttons, acting only on that tab.
* **Round corners everywhere**, anti-aliased via Pillow supersampling
  (`widgets/draw.py`). Tk's `create_polygon(smooth=True)` is a spline with
  no AA and looked jagged — do not go back to it.
* **Live theme switching**, no restart. `theme.apply_palette()` walks
  `sys.modules` and rebinds names still holding the old value.
* `hidpi.font()` is **deliberately an identity function.** Scaling there
  as well as via `tk scaling` double-scaled every label and overflowed the
  buttons.

---

## Persistence, and the schema trap

| File | Holds |
| :--- | :--- |
| `robot_sim/machine_settings.json` | Speeds, accel, boundaries + enforcement, PID gains + locks, the scan ZM ceiling, the motion profile |
| `robot_sim/keybinds.json` | Jog key layout |
| `robot_sim/limit_presets.json` | Named boundary sets |
| `robot_sim/appearance.json` | Colour scheme |

**The board holds limits in RAM only**, so the GUI is the system of record
and re-sends them on every handshake.

`machine_settings.json` carries `_schema` — **currently 4**. Two bumps so
far, and `_load_settings_file()` still applies both:

* **< 3** drops the four taught elbow boundaries (`ARM_FRAME_V2_RESET_KEYS`).
  They were fold degrees; they are motor degrees now.
* **< 4** drops the two RM boundaries (`ROT_FRAME_V4_RESET_KEYS`). They were
  centred `-170..+170`; RM reads `0..340` now, and a stored `±150` read in
  the new frame refuses every bearing past 150°, including any negative X.

A key the app does not already hold is ignored, so a NEW setting needs no
bump — it simply defaults. If you change what a stored value *means*, bump
and drop the affected keys with a warning. Do not convert values that were
produced by a *superseded* gear ratio; they were never real angles to
convert. Reading a stale value silently is how someone ends up hunting a
mechanical fault that does not exist.

Partial or corrupt files are discarded **whole**, never merged. A
half-loaded keymap that leaves some axes on your keys and others on the
defaults is far harder to notice than a clean revert.

---

## Reserved keys

HOME is **BackSpace**, from `keybinds.HOME_KEY` — `core/keyboard.py` binds
that constant rather than a literal, so the key that is reserved and the key
that homes are the same one by construction. They had drifted:
`RESERVED_KEYS` said backspace while the binder still listened for `h`/`H`,
so homing fired on a letter that was no longer protected and could also be
taken by a jog axis. `H` sits mid-keyboard and was too easy to hit.

Reserved keys must be spelled as **Tk keysyms** (`BackSpace`, capital S).
A lowercase `"backspace"` never matches a captured keypress, so the key
looks reserved in Settings while an axis can still take it.

`SPACE` (e-stop), `ESC` (settings), `BACKSPACE` (home) and `ENTER` (RUN
PROGRAM, P2P) cannot be rebound —
the live list is `keybinds.RESERVED_KEYS`, and `python_check.py` now derives
its assertions from it rather than naming keys. It used to hard-code `h`,
which meant that when HOME moved to `backspace` the test carried on passing
while checking an ordinary letter. Jog defaults: `A/D` = RM, `I/K` = A1M,
`O/L` = A2M, `W/S` = ZM.

`ESC` is bound with `bind_all`, which is application-wide. The dialog must
**not** also bind `<Escape>` or it opens and closes in the same keypress —
that bug shipped once. The handler returns `"break"`.

---

## Working style the user expects

Concise, direct, high-signal. Lead with the answer. Flag assumptions,
risks and blind spots rather than presenting a clean story. When a
physical constraint makes a request impossible as stated, say so with
numbers and offer the real options — that has happened more than once here
and was wanted both times.

Respond in Vietnamese unless the user writes in English.

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

Rules:
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).
