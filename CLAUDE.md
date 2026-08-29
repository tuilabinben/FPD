# Working on this project

Context for a Claude session picking this up cold. Read this before
touching anything; it is mostly a list of decisions that look wrong until
you know why they were made.

---

## What this is

An FPD wafer-handling robot (frog-leg twin arm, STCR4000S-class). Three
pieces, which must agree with each other:

| Piece | Path | Role |
| :--- | :--- | :--- |
| **Firmware** | `RobotMotionController_v9_ClearCore/*.ino` (~2300 lines) | ClearCore board. Owns motion, soft limits, IK. |
| **GUI** | `robot_sim/` (~6700 lines, Python + Tkinter) | Operator console. Serial link, jog, point-to-point, settings. |
| **Simulation** | `../MATLAB_v4_final` | Simscape model + `mophong_init.m`, the source of the geometry. |

`v8` and `clearcore/` are older firmware, kept for reference. **v9 is the
live one.** MATLAB is treated as read-only reference — the user asked
explicitly that it not be edited.

Four axes: **RM** turntable, **A1M** / **A2M** the two elbows (separate
motors), **ZM** lift.

---

## Run the tests before and after every change

```
tests/run_tests.sh
```

Two suites, ~300 assertions, both must be green:

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

## P2P vs the MATLAB model — already identical, and now enforced

`solve_ik()` and the firmware's `solveIkFrogleg()` reproduce
`solve_ik_frogleg` from `mophong_init.m` to **machine precision** (0.00e+00
over a 4080-pose sweep). Both suites now assert this against a transcribed
copy of the MATLAB helper, so a drift fails the build instead of being
noticed on the machine. Do not "fix" the maths; check the tests first.

Three departures are deliberate and tested as such:

* MATLAB **clamps** an unreachable target silently; here it raises, because
  the operator would otherwise press RUN believing the point was accepted.
  `clamp_like_matlab=True` reproduces the .m exactly for comparison.
* MATLAB uses **arm 1's `Z_offset` for both arms**; here each deck has its
  own, 9 mm apart.
* MATLAB **pins AM2 at `th3_home_cad`** for the whole trajectory. The idle
  arm holds where it is instead — parking it home is a move of up to 120°
  the operator never commanded, on an arm that may be carrying a substrate.
  `ikToJoints()` used to snap it home, which also meant `LOAD` and
  `LOAD_XYZ` produced different poses from the same coordinates.

**`i_RM_total` still differs on purpose**: the .m writes `4.375 * 6.4` = 28,
the controller uses `4.375 * 6.5` = 28.4375. The Simscape diagram and the
machine agree on 6.5. Changing it moves every RM angle by 1.5% — that is a
decision to take with the machine in front of you, not a tidy-up.

The **path shape** is joint-space, not MATLAB's 40-sample Cartesian
straight line. That was chosen explicitly; a straight-line tool path needs
a streamed segment list or on-board interpolation.

---

## The five decisions most likely to be "fixed" by mistake

### 1. `ARM_GEAR_RATIO = 2.0` is derived from the model, not measured

It does **not** affect arm speed — that is bounded in motor RPM and motor
°/s, both ratio-free. It **does** scale every reported frog-leg angle,
every reach figure, and every absolute arm move (`MOVE_A1`, `MOVE_A2`, IK
targets). Jog and the taught limits are unaffected, because both work in
motor degrees.

To confirm: mark the elbow, command a known number of motor revolutions,
divide by the frog-leg angle actually swept. Then `SET_ARM_RATIO:<r>` — no
re-flash, and nothing taught has to be re-taught. See 1b for the
derivation.

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

The MATLAB parity sweep now compares RM **modulo a whole turn** — the
geometry is unchanged, only the frame's zero moved, and both suites assert
that the difference is exactly a multiple of 360.

**HOME is the MINIMUM of all four axes** (d1 0, RM 0, both elbows 0). That
is why `LIMIT_SAFETY_MARGIN` is applied at the **far end only**: insetting
the lower end put HOME itself outside the working envelope, which refused
the home pose and with it every P2P program, since a run is
HOME -> A -> B -> HOME. The lower end is also the one end the machine has
physical sensors on (M5, M6), so it is already protected.

### 1b. The elbow reports MOTOR degrees; the frog-leg angle is derived

**`A1M` / `A2M` are motor shaft degrees from home.** The board counts step
pulses and nothing else, so that is the only elbow figure it knows exactly.
The frog-leg angle and the reach are derived:

```
fold_deg = motor_deg / ARM_GEAR_RATIO       ARM_GEAR_RATIO = 2.0
R        = 293.2 - 320 * cos(fold_deg + 60)
```

**The ratio of 2 comes from the model, not the bench.** `mophongv2.slx`
drives each arm's two revolutes from one AM signal — the shoulder at `×1`,
the knee at `×−2` — and `mophong_init.m`'s FK says the same thing by
putting the links at `th2 ± th3_math`, symmetric about the radial line. The
elbow motor is on the knee, so one fold degree costs two motor degrees.
**Confirm on the bench**; `SET_ARM_RATIO:<r>` changes it with no re-flash.

Consequences that are easy to get wrong:

* **Taught elbow limits are stored in MOTOR degrees**, factory band
  `0..240`. That is the raw count, so re-calibrating the ratio never
  invalidates a boundary somebody taught. Storing fold degrees would
  rescale every taught number the moment the ratio moved.
* `reachBandFor()`, `forward_kinematics()` and `is_near_singularity()` all
  take **fold** degrees. Every caller that holds motor degrees must convert
  — `armFoldFromMotor()` on the board, `fold_angle_from_motor_deg()` in the
  GUI. Feeding motor degrees straight into FK put the reported end effector
  at twice the elbow angle it had.
* Arm speed is quoted in **motor** °/s (exact) with the fold °/s beside it.
* `machine_settings.json` went to `_schema` **3** for this; v2 files' elbow
  limits were fold degrees and are dropped.

**HOME is 0, in both frames.** The `60` is `th3_cad` and is geometry, not a
label — `cos` needs it to make R at home 133.2 mm — so `ARM_ZERO_CAD_DEG`
stays. Nothing displays it. The jog panel used to *initialise* its readout
to `60.00 deg` and the boot banner announced the th3_cad convention; both
looked like the machine jumping to 60°, and both are gone.

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
* **HOME** cannot be converted at all. The PLC drives the axes while
  ClearCore has released them, so the board never counts those steps and
  the offset is genuinely unknown to it.

This is **deliberately not handled in code** — the user's working practice
is to home first and teach afterwards, and they chose to leave it rather
than carry per-boundary frame tracking. Do not add silent conversion
later: after a PLC home there is no correct offset to apply, and inventing
one would move a safety boundary to a place nobody chose.

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

Defaults are inset (ZM `5..280`, RM `±165`, elbows `10..230` motor°). They
used to *be* the factory envelope, which meant the soft limit and the
mechanical stop were the same position and the soft limit protected
nothing.

`RESET_COORD:<Z|ROT|A1|A2>` zeroes one axis. It deliberately does **not**
set `isHomed` — claiming a full reference from one axis would enforce
limits against three counters that are still meaningless.

The reset buttons live in **section 3, MOTION CONTROL**, not Settings: it is
an action used while jogging to the reference pose. They are in
`motion_lock_widgets`, so a counter cannot be zeroed mid-move.

**Both motion panels carry the row** — P2P *and* JOYSTICK. P2P-only was the
first layout and it was the wrong half: declaring the reference is a jogging
job, so the operator had to switch mode to finish it, and switching mode
auto-stops motion. One builder, `_build_coord_reset_row()` in
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

`A1M_POS` reads **0° at home** and counts up; straight out is **120°**.
The CAD frame (`th3_cad`: 60° retracted, 180° straight) survives *only*
inside `fold_angle_to_reach()` / `reach_to_fold_angle()` in
`kinematics.py` and `reachFromFoldAngle()` / `foldAngleFromReach()` in the
firmware, which add and remove `ARM_ZERO_CAD_DEG = 60`.

Why: the board cannot produce a real `th3_cad`. It counts steps from its
last reference and scales them by the placeholder ratio, so the `60°` it
used to print at home was zero rotation wearing a CAD label. Rotation from
home is the same number with an honest name.

`python_check.py` **fails the build if `th3_cad` leaks into any module
other than `kinematics.py` and `config.py`.** That check is deliberate.

### 3. The elbow boundaries have no envelope and are unordered

* **No envelope** (`floor = ceil = None`, `ARM_LIMITS_UNBOUNDED`). Any
  number is accepted, four figures included. Because the reported angle
  rides on the unmeasured ratio, the arm genuinely reads `1000°` at a pose
  CAD calls `140°`; any ceiling would be a guess, and a guess that rejects
  a pose the arm is physically standing at stops the operator teaching the
  machine at all.
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
only. A typed `90°` would be typed against a scale that is wrong.

### 3a. There is no structural REACH envelope either, for the same reason

`133.2–613.2 mm` used to be enforced inside `solve_ik()`. It is gone. The
floor was `R(fold = 0°)` — it assumed the elbow's zero really *is* the
folded home pose, and that fold degrees are motor degrees over an
`ARM_GEAR_RATIO` nobody has measured. Two assumptions stacked, so the
floor was a guess, and it refused `X 0, Y 0` and every short radius on a
machine that may well reach them.

What is enforced now:

| Check | Where | Switchable? |
| :--- | :--- | :--- |
| `R` within `a3+a6 ± (a4+a5)` = 293.2 ± 320 | `_check_reach()`, `solveIkFrogleg()` | **No** — arithmetic. `acos` would clamp and return a pose nobody asked for |
| `R` within the **taught elbow band** | `_limit_violation()`, `solveIkFrogleg()` | Yes, per axis |

So a radius below 133.2 mm now **solves**, to a negative fold angle. That
is not an error: the reported angle rides on the unmeasured ratio, so
angles outside `0…120` are expected.

Consequences that were easy to miss and are now tested:

* `reach_band_from_motor_deg()` in `kinematics.py` mirrors the firmware's
  `reachBandFor()` — **not** min/max of the endpoints, for the reason in
  section 4. The panel and the board would otherwise advertise and enforce
  different bands.
* The P2P workspace line is **live**, from `_refresh_workspace_hint()`, and
  quotes the taught band. A hard-coded envelope there would name a limit
  nothing applies. It repaints on APPLY and on every enforcement toggle.
* `_limit_violation()` reads the pair through `_limit_pair()`, which
  **sorts** — elbow boundaries are stored exactly as taught and may be in
  either order — and skips an axis whose enforcement is off.
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
at HOME the arm is retracted and its centre sits 133.2 mm out, so `0,0,0`
is the reference point and still not a reachable *target*. A frame pinned
to the tool would rotate with RM and stop being a frame at all.

**Z is carriage travel, so it is the same number for both arms.** The
9 mm deck offset is applied per arm inside the conversion, not by the
operator. That killed the old `Z0 - Z1 = 9 mm` rule: `LOAD_XYZ_BOTH` and
the GUI's BOTH mode now want the two Z values **equal**. Subtracting the
drop in `_sync_z_for_both_mode()` as well would apply it twice.

**The maths underneath is still absolute** — arm 1's deck at 514.3 mm with
the lift down — and must stay that way: that is `mophong_init.m`'s frame,
and the 4080-pose parity sweep only means something while both sides speak
it. So the translation lives at the **edges**, one function each side:

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

Reach is a cosine of `θ + 60`, monotonic only across half a period. Once a
taught band crosses an extreme, the extreme radius is **inside** the
interval. With a band of −260°…480° both endpoints land on the same part
of the curve, and the endpoint-only version reported a reachable radius of
593.9–613.2 mm and refused every ordinary target. The function checks the
angles where `θ + 60` is a multiple of 180 (θ = 120, 300, −60, …).

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
MOTOR degrees** — most of its 0..240 taught band — every time the operator
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

### 6. The PLC link is MC Protocol, and it is READ-ONLY

The PLC is a Mitsubishi at **192.168.3.101:1025**, and ClearCore is an MC
Protocol **3E ASCII** client. It batch-reads the single word `M0..M15`
every 50 ms — one round trip covers the whole handshake, because every
device it needs lives in that word. **It writes nothing.**

| Device | Meaning | Direction |
| :--- | :--- | :--- |
| `X0` | HOME request | **a wire** from ClearCore `IO-0`, held until DONE |
| `M1` | DONE | read |
| `M5`–`M8` | MinZ / OutR / OutR1 / OutR2 → ZM, RM, A1M, A2M **home sensors** | read |
| `M10`–`M13` | Run ZM / RM / A1M / A2M | read |

**The HOME request is a wire, not a packet**, and that is a fix rather
than a preference. `X0` is an INPUT device: the PLC refreshes X from the
physical terminals at the top of every scan, so an MC-protocol write to
`X0` was overwritten within one scan (< 10 ms) the moment `X0` had a wire
on it. It worked *only while X0 was unwired*, and the failure — HOME never
starting, `M10..M13` never coming on — looked like a ClearCore fault. A
physical output into that same physical input is what the scan expects.

Consequences worth keeping:

* **No ladder change.** `X0` is the same HOME input it always was, in
  parallel with the panel pushbutton. No SOCOPEN/RECV rungs.
* **Clearing the request cannot fail.** The old Ethernet clear could be
  refused by a dead socket, leaving the PLC latched and re-homing the
  machine when the link came back. `digitalWrite` always lands.
* **There is no `plcFrameWriteBit()`, and no `PLC_MC_CMD_WRITE`.** They
  were deleted rather than left unused, and both suites assert their
  absence. If a future feature must set a PLC device, use an **internal
  relay (M, decimal numbering)** — never an X — and write down why.
* `PLC_HOME_REQ_PIN` is defined **outside** the `#if PLC_LINK_MODE` block.
  The two modes had drifted on to different terminals (`IO5` against a
  comment saying IO-0) and only one of them was on the machine.
* `tests/stub/ClearCore.h` **records** `digitalWrite` into `PIN_LEVEL`.
  A swallowing stub would let every HOME test pass while the terminal
  never moved — the same trap `Serial.println` was in.

Do not go back to v9.0's `"M2\n"` → `"DONE"` line protocol. It needed a
SOCOPEN/RECV ladder written on the PLC to parse it; MC protocol answers
device reads with no ladder code at all, and the handshake is then visible
in GX Works while it runs.

Three things here are load-bearing:

* **DONE alone does not end a home.** `M1` stays latched after a cycle, so
  a second HOME would be answered by the previous one's bit before the
  machine moved. The gate is: `M10..M13` seen ON at least once this cycle,
  now all OFF, *and* `M1` set.
* **The poll has TWO rates, and the fast one is load-bearing.**
  `PLC_POLL_IDLE_DEF_MS = 20`, `PLC_POLL_HOMING_MS = 10`, chosen on
  `isHoming`. During a home it must be fast **because of the gate above**:
  a home sequence shorter than one poll interval would finish entirely
  between two polls, `plcSawRunDuringHome` would never be set, and a home
  that physically succeeded would fail on the 30 s timeout —
  intermittently, depending on where the poll landed. Do not "simplify"
  this back to one rate.
  (These were 5000/200 at one point and this file said so long after the
  machine had been moved to 20/10. `SET_PLC_POLL:<ms>` changes the idle
  rate at runtime and is not persisted.)
* **The socket timeout (800 ms) must stay longer than the CPU monitoring
  timer (500 ms).** Otherwise the board abandons a reply the PLC is still
  going to send, the late reply arrives against the *next* request, and a
  boundary pattern is applied one cycle late. On a timeout the socket is
  dropped deliberately — that is the only certain resynchronisation.
* **If HOME never starts, check the wire first.** `PLC_STATUS` showing
  `M10..M13` never coming on used to mean the X0 write had been
  overwritten. It now means the IO-0 → X0 wire, its 24 V return, or the
  PLC's own home sequence. The network cannot start a home and no longer
  pretends it can.

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

### All four sensors work, and they sit at OPPOSITE ends

The sensor row is built by **both** motion panels from
`ui/sensor_panel.py`, like the coordinate-reset row.

| Bit | Axis | End it sits at | Jog command that drives INTO it |
| :--- | :--- | :--- | :--- |
| `M5` MinZ | ZM | **minimum** — bottom of the stroke | `Z_DOWN` |
| `M6` OutR | RM | **minimum** — the CCW stop, 0 deg | `ROT_CCW` |
| `M7` OutR1 | A1M | **maximum** — fully extended | `A1_FWD` |
| `M8` OutR2 | A2M | **maximum** — fully extended | `A2_FWD` |

M5/M6 mark the HOME end of their axis; M7/M8 mark the FAR end. "Covered"
therefore means the opposite thing for each pair, which is why the lamp
caption says `(home)` or `(far)` — reading the lamp without that is
guesswork. `PLC_SENSOR_END_*` on the board and the last field of
`PLC_SENSOR_PANEL` in the GUI are the one place each side states it.

**P2P ENFORCES, JOG ONLY WARNS.** This asymmetry is deliberate:

* A P2P leg that would drive an axis further into a covered sensor is
  **refused** — `runLegBlockedBySensor()` on the board,
  `_sensor_violation()` in the GUI. A program runs unattended and the
  operator is not watching that axis, so it must not start.
* Jog **warns and proceeds**. Jog is a dead-man control: it moves only while
  held, the operator is looking at the machine, and jogging is how you come
  OFF a tripped sensor. Blocking it would also risk pinning the machine on
  its own switch. Physical protection while jogging is the PLC's ladder.

Both checks compare against the **live pose**, so they answer "would this
move make it worse", not "is a sensor covered". A move AWAY from a covered
sensor is exactly what the operator needs, and an axis that is not moving
is never refused.

If you find yourself re-adding a jog block, the previous revision had one
and it was removed on request. `PLC_SENSOR_BLOCKS_*` is gone from the
firmware, and both suites assert its absence.

**HOME STATE = M5 and M6 covered while M7 and M8 are CLEAR.** At home the
lift is down, the turntable is at 0 and both arms are pulled IN — the
opposite end from where M7/M8 sit. It is the one condition allowed to zero
the counters unasked, **edge-triggered** (holding at home would otherwise
re-zero every poll and eat real motion) and **refused while anything is
moving**, re-arming once stopped.

Nothing a sensor reports ever writes a working boundary. Boundaries come
from the operator only.

### P2P runs HOME -> A -> B -> HOME

Four legs, on the board (`PHASE_TO_HOME_FIRST` .. `PHASE_TO_HOME_LAST`, all
started through `beginRunLeg()`) and in the offline simulation. Starting and
ending at the reference is what makes the cycle repeatable, and it matches
`mophong_init.m`'s `P_home -> A -> B -> P_home`.

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

### ZM lead and the arm ratio are BOTH unmeasured

`zMmPerRev` (20 assumed) and `armGearRatio` (2 derived) are the two numbers
that turn counts into real units, and neither has been measured on the
bench. Both are runtime-settable — `SET_Z_LEAD:<mm>`, `SET_ARM_RATIO:<r>` —
so calibration needs no re-flash.

If Z travels **3x** the commanded distance the true lead is 3 x 20 = 60
mm/rev. A non-power-of-2 error points at the mechanics; the driver's
microstep switches can only ever err by powers of two. Measure over 100 mm,
not 10.

A wrong ZM lead also moves where every ZM soft limit physically is, because
those are stored in millimetres.

### Jog and P2P share ONE live pose

`current_joints` is the single store; `sim_z` / `sim_rot` / `sim_a1` /
`sim_a2` are **properties onto it**. They used to be a second copy — jog
integrated `sim_*`, P2P integrated `current_joints`, and nothing kept them
together, so jogging and then switching to P2P ran the program from
wherever P2P last left off rather than from where the arm actually was.
Both panels repaint from either update path.

### UNKNOWN sensor data must never render as CLEAR

The sensor lamps used to be built showing `CLEAR` and only changed when a
poll landed. So a dead MC-protocol link showed four `CLEAR` lamps, which is
indistinguishable from "nothing is covered" — on a safety display the
failure read as good news. That was a real field bug: M6 was physically ON
and the panel said CLEAR.

Three parts to the fix, and all three are load-bearing:

* `plcStatusSummary()` sends `home Z/R/A1/A2=????` when it has no device
  data. It used to return a bare `"no data"` with **no bit field at all**,
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

### One dead device read breaks HOME too

`plcHomeDoneAsserted()` needs the run bits and DONE, both of which arrive by
device read. **With no successful read HOME can never complete**, however
well the PLC homes the machine — the two symptoms have one cause, so check
the link before suspecting the PLC.

`beginHoming()` warns up front when `plcGoodReads == 0`, and the timeout
names which of the three faults it was: no device read at all, reads working
but `M10..M13` never came on (the IO-0 → X0 wire or the PLC's sequence), or
the axes ran but `M1` never set.

**`PLC_TEST` is the command to run when the sensors read stale.** One
blocking read, reporting the PHY link, the TCP connect, the exact frame sent,
and what came back — which separates "no cable", "socket open but not
speaking MC protocol", "wrong encoding", and "PLC refused the device" from
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

### `M5`–`M8` are HOME SENSORS, not limit switches

They are read to know when an axis has reached its reference, and for
nothing else. They do **not** stop a jog or a run, and they do **not** set
a working boundary.

An earlier revision of this file did both. It is wrong twice over: a home
sensor sits **at** the reference, so stopping on it would make it
impossible to jog off home, and writing its trip point into a limit would
overwrite a taught boundary with a position that is not a boundary. If you
find yourself re-adding `PLC_LIMIT_DIR_*` or a `[PLC_LIMIT_SET]` message,
that is the mistake.

Working boundaries come from the operator only — typed, or taught with
`SET HERE`. Physical protection is the PLC's own ladder.

So **nothing the board reports ever writes a limit**: `[LIMITS]` is logged
and never parsed back, and the GUI is the sole system of record.

`plcAllHomeSensors()` is used one way only: if the PLC returns DONE while a
sensor is still uncovered, the board **warns** and completes anyway.
Refusing would hang the machine on a miswired sensor; silence would hide
it.

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
* **EMERGENCY STOP is on BOTH motion panels.** It was removed from JOG
  once — jog is a dead-man control and SPACE fires the same path — and
  put back by request. Both arguments were true and neither helps someone
  with a hand on the mouse looking at the machine; a stop control whose
  location depends on the current mode is its own hazard. Both buttons
  call the one audited `emergency_stop_all`; a second stop implementation
  is the thing to prevent, not a second button. Neither is in
  `motion_lock_widgets` — that list is disabled while the machine moves,
  which is when the button has to work.
* **Nothing applies on keystroke.** Edits stage until APPLY — except the
  two live readouts (real height, and the boundary enforcement captions),
  which report state rather than stage a change.
* **Per-section APPLY / DEFAULTS.** A global reset that wiped taught
  boundaries because someone undid a speed change costs an afternoon of
  re-teaching. Settings tabs each own their own buttons, acting only on
  that tab.
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
| `robot_sim/machine_settings.json` | Speeds, boundaries, PID gains + locks |
| `robot_sim/keybinds.json` | Jog key layout |
| `robot_sim/limit_presets.json` | Named boundary sets |
| `robot_sim/appearance.json` | Colour scheme |

**The board holds limits in RAM only**, so the GUI is the system of record
and re-sends them on every handshake.

`machine_settings.json` carries `_schema` (currently **2**). If you change
what a stored value *means* — as the arm-angle re-zero did — bump it and
drop the affected keys with a warning. Do not convert values that were
produced by the unmeasured gear ratio; they were never real angles to
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

`SPACE` (e-stop), `ESC` (settings), `BACKSPACE` (home) cannot be rebound —
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
