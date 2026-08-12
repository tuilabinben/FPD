# Firmware design notes

Extracted prose from `RobotMotionController_v9_ClearCore.ino`. Each section is referenced from the source by number.

Comments that the test suites assert on were deliberately left inline in the source and are not reproduced here.

## 0. STCR4000S Motion Controller — ClearCore Firmware v9

============================================================
STCR4000S Motion Controller — ClearCore Firmware v9
============================================================
WHAT CHANGED FROM v8, AND WHY
-----------------------------
v8 was a pure joint-space board: the Python GUI did all inverse
kinematics and the firmware only ever saw d1/rot/a1/a2. It also
carried a kinematic model that was simply wrong:

    v8 / old GUI:  reach = 2 * 157.5 * cos(theta),  theta 0..90 deg
                   -> reach 0..315 mm, Z 0..200 mm, no per-arm offset

The real machine, per MATLAB_v4_final/mophong_init.m (the Simscape
model generated from the SolidWorks assembly):

    R  = a3 + a6 + (a4 + a5) * cos(th3_math)
       = 293.2 + 320 * cos(pi - th3_cad)
       = 293.2 - 320 * cos(th3_cad)
    d1 = Z - Z_offset,  clamped 0..285
    th2 = atan2(Y, X)

So the arm carries a FIXED 293.2 mm radial offset (a3 + a6) that v8
did not model at all: the wafer centre can never come closer than
133.2 mm to the rotation axis, and reaches 613.2 mm when straight.
Commanding "reach = 0" as v8 allowed is not a retracted arm, it is a
point the machine physically cannot represent.

v9 therefore implements solve_ik_frogleg() ON THE BOARD, so the
firmware is self-contained and can be driven from a plain serial
terminal with Cartesian targets, with no GUI in the loop.

GEOMETRY — every constant traced to mophong_init.m
--------------------------------------------------
  a3 = 45     turntable centre -> shoulder pivot
  a4 = 160    upper frog-leg link      \  drawing MTCR4160-300-AM
  a5 = 160    lower frog-leg link      /  reads "160  160" — AGREES
  a6 = 248.2  wrist pivot -> wafer centre
  d_base = 388, d3_arm1 = 50, d3_arm2 = 41, d4 = 46.5, d5 = 24.8, d6 = 5
  Z_offset(arm1) = 388+50+46.5+24.8+5 = 514.3
  Z_offset(arm2) = 388+41+46.5+24.8+5 = 505.3   (9 mm lower deck)
  i_RM_total = 4.375 * 6.5 = 28.4375  (motor revs per turntable rev)
      NOTE: mophong_init.m still writes 4.375 * 6.4 = 28. The Simscape
      block diagram and the machine both use 1/4.375 then 1/6.5, so
      28.4375 is the value that matches hardware. The .m file is left
      alone deliberately — it is the simulation's own record.

ANGLE CONVENTION — read this before touching any limit
------------------------------------------------------
All arm angles in this file, in the GUI, and on the wire are
The elbow angle this board REPORTS is rotation from home:
     fold =   0 deg -> retracted, R = 133.2 mm   (HOME)
     fold = 120 deg -> straight,  R = 613.2 mm   (singularity)
th3_cad (60 retracted, 180 straight) survives only inside
reachFromFoldAngle() / foldAngleFromReach(). See ARM_ZERO_CAD_DEG.
Reach GROWS with the angle. This is the opposite of v8's convention,
where 0 was full reach. If you flash v9 over v8 without re-homing,
the arm's zero reference is different — RE-HOME BEFORE MOVING.

SINGULARITY WARNING
-------------------
FOLD_ANGLE_MAX_DEG is set to the full mathematical 120 deg by
deliberate choice. At 180 deg the two frog-leg links are colinear:
radial stiffness collapses and dR/dth3 -> 0, so encoder/step error at
the elbow becomes large radial error at the wafer. JEL's own drawing
stops at 575 mm (fold = 91.72 deg). To use the conservative
limit, set FOLD_ANGLE_MAX_DEG = FOLD_ANGLE_SPEC_MAX_DEG below.

INDEPENDENT ARMS
----------------
v8 held one `armDir` and drove AM1 and AM2 from it, so the two elbows
could never be positioned separately — and its soft-limit check read
AM1's angle to decide whether to stop BOTH motors, meaning AM2 could be
driven past its own stop without ever being noticed. v9 gives each
elbow its own direction, its own soft limit checked against its own
measured angle, and its own jog/move commands. ARM_FWD/ARM_BACK remain
as the explicit "both together" gesture.

MOTOR MAPPING (unchanged from v8):
  M-0  ->  ZM   Z lift        (prismatic, d1, mm)
  M-1  ->  RM   rotation      (revolute, th2, deg, through i_RM = 28)
  M-2  ->  AM1  arm 1 elbow   (revolute, deg from home)
  M-3  ->  AM2  arm 2 elbow   (revolute, deg from home)

SAFETY DEFAULTS CARRIED OVER FROM v8 — DO NOT "OPTIMISE" AWAY:
  - Boot motion profile is conservative and reproduces the MATLAB
    trajectory, NOT some round number that looks fast in a dialog.
  - Limit-sensor handling is OPT-IN and compiled out by default. A
    floating digital input commonly reads HIGH, which would make the
    board believe an axis is at its limit the instant it moves.
  - ESTOP decelerates hard rather than MoveStopAbrupt(): an abrupt
    stop on an open-loop step/dir motor skips steps with nothing to
    detect it, silently corrupting the position reference.

SPEED MODEL — ONE UNIVERSAL RPM, ONE PERCENTAGE PER MOTOR (NEW)
--------------------------------------------------------------
There is now a single universal speed, in MOTOR RPM, and each motor
runs at its own percentage of it:

    axisMotorRpm = masterRpm * (axisPercent / 100) * AXIS_RPM_SCALE

AXIS_RPM_SCALE is a CALIBRATION constant, not a user setting. It exists
because the three axes are geared completely differently, so a raw
percentage of one shared RPM would be meaningless:

    RM  28.4375:1  -> 140 motor RPM gives  29.5 deg/s   scale 1.00
    ZM  20 mm/rev  -> 105 motor RPM gives  35.0 mm/s    scale 0.75
    AM  ratio UNMEASURED -> runs at the master RPM       scale 1.00

AM's scale was 0.03, from "25 deg/s = 4.17 motor RPM", which only held
if the arm gear ratio were really 1.0. On the machine 100 deg/s was still
visibly slow — proof the elbow has a real reduction and the motor was
being throttled to ~17 RPM while every other axis ran at 100+. Its
scale is now 1.0 and its old 100 deg/s ceiling is GONE.

RM and ZM are still clamped to a real engineering ceiling, because for
those two the gearing is known. The arm is clamped in MOTOR RPM
instead (ARM_RPM_MAX), which is the only unit on that axis that
currently means anything, and which guards the hazard that actually
exists: an open-loop stepper skipping steps at high RPM with no
encoder to notice.

PID NOTE: Kp/Ki/Kd/N are stored and echoed only. This is an OPEN loop
(no encoder feedback on this board); there is nothing for a PID loop
to close over unless your driver does its own closed-loop tuning.
The values are the single PID preset from the Stepper MATLAB report,
Table 2. v9.1 dropped the P/PI/PD presets and the PARALLEL/I-PD/PREFILTER
form selector: only one preset is used in practice, and the form field
was configuration with no effect on an open-loop board. PID_OFF stops
the gains being sent/echoed at all.

PROTOCOL — Host -> Board
  PING
  BYE
  SET_PID:kp,ki,kd[,N]               N = derivative filter coefficient
  PID_ON / PID_OFF                   enable/disable the stored gains
  PID_RESET                          restore the single report preset
  SET_SPEED:masterRpm,masterAccRpmS,rotPct,armPct,zPct,
            rotAccPct,armAccPct,zAccPct
      THE speed command. masterRpm is motor RPM; rotPct/armPct/zPct are
      per-motor shares of it (see SPEED MODEL above); the trailing 3
      are independent per-motor shares of masterAccRpmS.
  SET_MOTION:rotVel,rotAcc,armVel,armAcc,zVel,zAcc
      Legacy engineering-unit form; converted into the percentages.
  SET_PARAMS:...                     legacy v8 form; PID honoured,
                                     speed/accel fields ignored
  PROFILE / STATUS                   report the active profile and PID
  SET_BOOST:multiplier
  -- joint space (v8-compatible) --
  LOAD:d1A,rotA,a1A,a2A,d1B,rotB,a1B,a2B
  LOAD_BOTH:d1,rot,a1,a2
  -- Cartesian, NEW in v9: board runs the IK --
  -- Cartesian. HOME is the reference: X 0, Y 0, Z 0. X/Y are from the
     turntable axis and may be negative; Z is height ABOVE HOME and may
     not. The 9 mm deck offset is applied by the board, not by you.
  MOVE_XYZ:arm,X,Y,Z                 arm = 1 or 2, immediate move
  LOAD_XYZ:arm,Xa,Ya,Za,Xb,Yb,Zb     A -> B for one arm
  LOAD_XYZ_BOTH:Xa,Ya,Za,Xb,Yb,Zb    simultaneous, Za must EQUAL Zb
  IK:arm,X,Y,Z                       compute and report only, no motion
  FK:d1,rot,a1,a2,arm                report Cartesian, no motion
  -- independent per-arm control, NEW in v9 --
  A1_FWD / A1_BACK / A1_STOP         jog arm 1's elbow alone
  A2_FWD / A2_BACK / A2_STOP         jog arm 2's elbow alone
  MOVE_A1:th3  /  MOVE_A2:th3        one elbow to an absolute angle
  MOVE_R1:mm   /  MOVE_R2:mm         one elbow to an absolute reach
  -- common --
  RUN / STOP / HOME / ESTOP
  ROT_CW / ROT_CCW / ROT_STOP
  ARM_FWD / ARM_BACK / ARM_STOP      BOTH elbows together (v8-compatible)
  Z_UP / Z_DOWN / Z_STOP
  -- operator-defined reference and travel limits, NEW in v9.1 --
  RESET_COORD                        zero every axis counter HERE
  SET_REF                            alias of RESET_COORD (v9 name)
  CLEAR_REF                          drop the reference (limits still apply)
  SET_LIMIT:axis,end,value           axis = Z|ROT|A1|A2, end = MIN|MAX
  SET_LIMIT_HERE:axis,end            take the CURRENT position as that
                                     limit — "set here as bottom/top"
  RESET_LIMITS                       restore the factory envelope
  SET_LIMIT_ENFORCE:axis,<0|1>       switch ONE axis's boundary on/off
                                     (values are kept either way)
  SET_LIMITS_ENABLED:<0|1>           the same, every axis at once
  LIMITS                             dump the active soft limits

PROTOCOL — Board -> Host  (unchanged lines are v8-compatible)
  PONG
  [ALIVE] uptime: Xs
  [PARAMS_OK] ...
  [LOADED] ...
  [RUN] ...
  [CLEARCORE POS] D1: F mm | ROT: F deg | A1M: F deg | A2M: F deg (P%)
  [JOG POS] ROT: F deg | A1M: F deg | A2M: F deg | Z: F mm
            (v8 sent a single "ARM:" field; the GUI accepts both)
  [IK] arm=N d1=F rot=F th3=F R=F
  [FK] arm=N X=F Y=F Z=F
  [LIMITS] ...
  [SINGULARITY] th3=F deg — advisory
  [RUN] TARGET REACHED              (v9.2; was a Vietnamese string)
  [ESTOP] EMERGENCY STOP            (v9.2; was a Vietnamese string)
  [HOME] Homing started. / [HOME] Homing complete. ...
  [LIMIT] ROT_CW / ROT_CCW / Z_UP / Z_DOWN   (only if sensors enabled)
  [WARN] ... / [ERROR] ...
============================================================

## 1. TYPES — MUST STAY ABOVE THE FIRST FUNCTION DEFINITION.

The Arduino IDE auto-generates prototypes for every function in a
.ino and injects them at the line of the FIRST function definition.
If a user-defined type is declared lower down the file, the generated
prototype references it before it exists and the build fails with:

    error: 'IkResult' does not name a type

pointing at the function's own definition line, which is confusing
because that line is fine — it is the injected prototype that is out
of order. Keeping every struct/enum up here, above the first
function, is what makes the sketch build. Do not move this block down.

## 2. A sequential program is HOME -> A -> B -> HOME, four legs.

It starts and ends at the reference so the cycle is repeatable: every run
begins from the same pose whatever the operator did by hand beforehand,
and leaves the machine parked where the next one can start. This mirrors
mophong_init.m, whose trajectory is P_home -> A -> B -> P_home.

## 3. Joint travel

THE ELBOW ANGLE IS ROTATION FROM HOME, NOT th3_cad.

0 deg is the retracted home pose and the number counts up by however
far the elbow has turned. th3_cad (60 retracted, 180 straight) is the
CAD frame the geometry is written in, and it now appears ONLY inside
reachFromFoldAngle() / foldAngleFromReach(), which add and remove
ARM_ZERO_CAD_DEG.

This board cannot produce a real th3_cad anyway: it counts steps from
wherever it was last referenced, so what it can report exactly is MOTOR
degrees from home. The frog-leg angle is derived from that through
armGearRatio (see the elbow section below), and the reach follows from
the frog-leg angle. The "60 deg" this board used to print at home was
zero motor rotation wearing a CAD label — the honest version is A1M in
motor degrees plus FOLD1 and R1 alongside it.

## 4. 91.72, NOT 151.72. This constant is in the from-home frame like every

other angle in this file, and foldAngleFromReach(575) = 91.72. The old
151.72 was th3_cad (91.72 + ARM_ZERO_CAD_DEG) left behind by the frame
change — a th3_cad leak of exactly the kind config.py's test forbids.
Dormant, because nothing reads it unless somebody switches
FOLD_ANGLE_MAX_DEG over to the conservative drawing limit; had they
done so they would have got a 151.72 deg ceiling, i.e. no restriction
at all, while believing they had tightened the envelope to 575 mm.

## 5. ── RM ZERO IS THE CCW STOP, NOT MID-TRAVEL ──────────────────────

RM reads 0 at its fully counter-clockwise stop and counts up to 340 at
the clockwise one. It was -170..+170, centred.

HOME is that CCW stop, so RM now agrees with every other axis here:
home is zero and the number counts up. Cartesian +X moves with it, so
at RM = 0 the arm points along +X and HOME is a true X0 Y0 Z0
reference. NOTE the 20 deg wedge between 340 and 360 is unreachable
from either side — it is the gap the turntable cannot sweep through.

## 6. OPERATOR-DEFINED WORKING LIMITS  (NEW in v9.1)

The factory envelope above is what the STRUCTURE allows. What the
machine may actually use is narrower and depends on what is installed
around it — a cassette, a chamber port, a cable loop. Those limits
belong to the operator, not to this file, so they are variables that
SET_LIMIT / SET_LIMIT_HERE write at runtime.

Each arm carries its OWN pair. v8's single shared arm limit is exactly
how AM2 used to get driven past its stop while AM1's angle was checked.

These are RAM only. ClearCore has no battery-backed store wired here,
so the GUI keeps them in its settings file and re-sends them on every
connect. If you drive the board from a bare terminal, re-send them
after a power cycle or you are back on the factory envelope.

## 7. The elbow limits are held in MOTOR DEGREES, not frog-leg degrees.

That is the raw count, exact whatever the gear ratio turns out to be,
which means correcting armGearRatio never invalidates a boundary the
operator already taught. Storing them as frog-leg degrees would rescale
every taught number the moment the ratio changed, and the whole point
of teaching is that it survives calibration.

## 8. PER-AXIS ENFORCEMENT, AND THE MASTER ENABLE

Both switches now answer the same question — "is this boundary
stopping the axis?" — at two different scopes:

  limXxxEnforced — enforcement for THIS axis only. Its taught values
                   are kept and still reported; nothing stops the axis
                   at them while it is off. For commissioning one axis
                   whose boundaries are not taught yet, or a
                   maintenance move on one axis that has to go outside
                   the working envelope on purpose.

  limitsEnabled  — the same thing for EVERY axis at once. It is an AND,
                   not an override: an axis is protected only when both
                   are on, so the master switch can never quietly
                   re-arm an axis the operator turned off.

This REPLACES the old per-axis value LOCK (limXxxLocked,
SET_LIMIT_LOCK). The lock froze the boundary's number while leaving it
enforced, which is a real thing but not the thing an operator reads off
a button labelled for a safety limit — the panel said LOCKED / UNLOCKED
and answered a question nobody was asking, while "is this boundary
actually on?" had no per-axis answer at all. Changing values is now
guarded by APPLY alone, as on every other tab.

## 9. "Has the operator switched this axis's boundary on?" — the flag alone,

with no reference check. Target VALIDATION uses this: refusing a target
is free and happens before anything moves, so it stays available even
while the machine is unreferenced. Anything that stops a MOVING axis
uses axisLimited() instead, which also demands a reference.

## 10. The working band of one elbow, always low..high.

The two taught limits are stored exactly as they were captured and may
sit in either order (see applyLimit). Every consumer goes through here,
so no consumer has to care which SET HERE the operator pressed first.

## 11. A TAUGHT elbow boundary has NO envelope, and the pair is UNORDERED.

No envelope: the angle this board reports for an elbow is scaled by
armGearRatio, which is derived from the model rather than measured, so a
position captured off the real machine legitimately reads far outside
the CAD envelope. Any ceiling written here would be a guess, and a guess
that rejects a position the arm is physically standing at defeats the
whole point of teaching. The boundaries are MOTOR degrees, which is why
re-calibrating the ratio does not disturb them.

Unordered: SET_LIMIT_HERE takes whatever is on the screen, so the
operator jogs to one stop, presses SET HERE, jogs to the other, presses
SET HERE. Which of the two they reached first is not something they
should have to keep straight, so the pair is sorted after each write
rather than the second write being rejected for "inverting" it.

ZM and ROT keep their real envelopes and their ordering, because for
those two the scale IS known and a number outside it really is
impossible.

## 12. Radial reach spanned by an elbow moving between two angles.

NOT simply min/max of the two endpoints. R is a cosine of
(th + ARM_ZERO_CAD_DEG), so it only rises monotonically across one
half-period — and taught boundaries are no longer confined to that
window (see ARM_LIMITS_UNBOUNDED). Over an interval that crosses an
extreme, the extreme radius happens INSIDE the interval, not at either
end.

Getting this wrong is not cosmetic: with a band of -260..480 (the
from-home spelling of the old -200..540) both endpoints land on the
same part of the cosine, so the endpoint-only version reported a
reachable band of 593.9..613.2 mm and refused every ordinary target.

cos hits its extremes where (th + ARM_ZERO_CAD_DEG) is a multiple of
180, i.e. at th = 120, 300, -60, ... — checking those is both exact and
cheap, since the widest legal band contains only a handful.

## 13. AM1/AM2 elbow gearing — *** STILL A PLACEHOLDER. MEASURE IT. ***

The Simscape diagram's +/-1 and -2 gains on AM1/AM2 are the frog-leg
LINKAGE relationship (shoulder theta, elbow -2 theta), not a gearbox:
that model takes a joint angle as its input and contains no motor at
all. It says nothing about the motor-to-joint reduction, and reading
1:1 out of it was a mistake.

The machine says otherwise. Commanding "100 deg/s" felt slow, which at
1:1 would be 17 motor RPM — so there is a substantial reduction here.

This constant NO LONGER AFFECTS SPEED (the gear ratio cancels on the
way to the step generator — see ARM_RPM_SCALE below). What it still
controls is every ANGLE this board reports and every absolute elbow
position it drives to. Until it is measured, MOVE_A1/MOVE_A2, the
reported th3, the reach figures and the IK targets are all wrong by
exactly this factor. Jog is unaffected.

To measure: mark the elbow, command a known number of motor revolutions
and divide by the joint angle actually swept.
THE ELBOW: MOTOR DEGREES vs FROG-LEG DEGREES

These are two different numbers and this firmware used to conflate
them. Everything the board MEASURES is motor shaft rotation — it counts
step pulses and nothing else. The frog-leg link angle is DERIVED from
it, and the derivation needs a ratio.

WHERE THE RATIO COMES FROM (MATLAB_v4_final: mophongv2.slx + the .m)
-------------------------------------------------------------------
The Simscape root diagram drives each arm's TWO revolute joints from
the single AM1/AM2 signal:

    AM1 --x(-1)--> Revolute3   [banxoay : canhtay1]   the SHOULDER
        --x(-2)--> Revolute    [canhtay1 : canhtay2]  the KNEE

and mophong_init.m's forward kinematics says the same thing in closed
form — the upper link sits at (th2 + th3_math) and the lower at
(th2 - th3_math), symmetric about the radial line, so the knee turns
through twice the angle the driven link does:

    P2 = P1 + a4 * [cos(th2 + th3_math), sin(th2 + th3_math), d4]
    P3 = P2 + a5 * [cos(th2 - th3_math), sin(th2 - th3_math), d5]

The elbow motor is coupled to the knee, so ONE FROG-LEG DEGREE COSTS
TWO MOTOR DEGREES:

    fold_deg  = motor_deg / armGearRatio
    motor_deg = fold_deg  * armGearRatio

>>> CONFIRM THIS ON THE BENCH. <<<
It is derived from the model, not measured off the machine. Mark the
elbow, command a known number of motor revolutions, and divide by the
frog-leg angle actually swept. If it is not 2, send
SET_ARM_RATIO:<value> — no re-flash, and every reported angle, reach
figure and IK target follows immediately.

RM is the counter-example that shows this is the right shape: its ratio
has been in the model all along as 1/4.375 then 1/6.5, and
pulsesPerDegRot() has always used it (also SET_ROT_RATIO-adjustable now).
Pulses per MOTOR degree. Exact and ratio-free — only steps/rev and the
microstep setting, both known. This is the quantity the board can
actually count, so it is what the maths is built on.

## 14. MOTION PROFILE — ONE UNIVERSAL RPM, ONE PERCENTAGE PER MOTOR

v8 set a single VelMax in raw motor RPM and applied it identically to
all four motors. That could not work, because the axes are geared
completely differently:

    300 RPM  ->  elbow 1800 deg/s  (its whole 120 deg travel in 0.07 s!)
                 turntable 63 deg/s   (reasonable)
     10 RPM  ->  elbow 60 deg/s    (reasonable)
                 turntable 2.1 deg/s  (unusably slow)

Early v9 fixed that by abandoning RPM entirely and giving each axis its
own number in its own unit. Correct, but it means six fields to keep in
step and no single "go slower" knob.

v9.1 keeps the one knob AND the correctness: a universal motor RPM,
a percentage per motor, and a per-axis CALIBRATION scale that absorbs
the gearing difference so the percentages are comparable:

    axisMotorRpm = masterRpm * (pct/100) * AXIS_RPM_SCALE

The scales are chosen so 100% on RM and ZM reproduces the MATLAB
trajectory (mophong_init.m, 12 s home -> A -> B -> home):
    ZM  peak 34.5 mm/s   (mean 22.6)
    RM  mean 23.3 deg/s  (peak 85.8, inflated by the atan2 wrap
                          where the path passes near the centre)

DO NOT "simplify" ROT_RPM_SCALE or Z_RPM_SCALE to 1.0. They are what
carry the 28.4375:1 reduction and the 20 mm/rev lead into the
percentage; without them one master RPM would mean wildly different
real speeds on those two axes.

Percentages have NO upper bound. See AXIS_PCT_MIN below for why that
is safe rather than merely permissive.
Motor RPM that corresponds to 100% on each axis, at MASTER_RPM_NOMINAL.
  RM: 29.53 deg/s * 28.4375 / 6  = 140 RPM
  ZM: 35.0 mm/s  * 60 / 20       = 105 RPM
  AM: runs at the master RPM directly — see below

## 15. AM's scale was 0.0298, derived from "25 deg/s = 4.17 motor RPM" — which

only holds if ARM_GEAR_RATIO really is 1.0. On the machine, 100 deg/s
was still visibly slow, which is proof that it is not: the elbow has a
real reduction, so the motor was being asked for ~17 RPM while every
other axis ran at 100+.

The scale is therefore 1.0: the arm's percentage maps STRAIGHT to the
master motor RPM, exactly like RM's does.

Worth understanding, because it is what makes this safe to change: the
gear ratio CANCELS on the way to the step generator.
    armVelDegS   = rpm * 360 / (60 * G)
    armVelPulses = armVelDegS * PULSES_PER_MOTOR_REV * G / 360
                 = rpm * PULSES_PER_MOTOR_REV / 60
So the pulse rate the motor actually receives depends only on the RPM
asked for, whatever G is. ARM_GEAR_RATIO no longer has any influence on
how fast the arm moves — it only affects the ANGLES this board reports
and the absolute positions it drives to. Those are still wrong until it
is measured. See the ARM_GEAR_RATIO note above.

## 16. Defaults

RM and ZM sit at the percentages that reproduce the MATLAB trajectory
through their real gear ratios (i_RM_total = 28.4375, 20 mm/rev):
    RM 100% of 140 RPM -> 29.5 deg/s   — exactly mophong_init.m
    ZM  90% of 140 RPM -> 31.5 mm/s    — just under it
The arm sits far above them because its drivetrain is the one that
needs it: extend/retract is the stroke that gates cycle time, and the
elbow reduction means a high motor RPM is a moderate joint speed.
The host treats the master as a FIXED reference and only changes the
percentages, so these defaults are the ones the GUI ships with.
    RM  75% of 150 RPM -> 112.5 motor RPM -> 23.74 deg/s
    AM 125% of 150 RPM -> 187.5 motor RPM
    ZM  50% of 150 RPM ->  56.25 motor RPM -> 18.75 mm/s

## 17. NO upper bound on a percentage.

Safe for a specific reason, not by luck: a percentage is a multiplier,
not a speed, and every axis it feeds still has a real backstop
underneath it — ROT_VEL_MAX and Z_VEL_MAX for the two axes whose
gearing is known, ARM_RPM_MAX for the one whose gearing is not.
Capping the percentage as well was belt-and-braces that mostly got in
the way: 250% is a perfectly reasonable thing to ask of the arm.

The lower bound stays. 0% silently freezes an axis and a negative
percentage would invert its direction; neither is a speed setting.

## 18. HARD engineering ceilings, ~4x the nominal trajectory. These are the

backstop: whatever masterRpm and the percentages work out to, no axis
is ever handed more than this, and the clamp is reported rather than
applied silently.

RM and ZM keep theirs, because for those two the gearing is KNOWN
(28.4375:1 and 20 mm/rev), so a deg/s or mm/s ceiling is a statement
about something real.

THE ARM NO LONGER HAS ONE. Its old 100 deg/s ceiling was computed from
ARM_GEAR_RATIO = 1.0, a placeholder that turned out to be wrong, so the
ceiling was not protecting the arm from anything — it was throttling it
to roughly 17 motor RPM while claiming that was 100 deg/s. A limit
derived from an unmeasured constant is worse than no limit: it is a
number that looks like a safety margin and isn't one.

What replaces it is a bound in the unit that IS known and does mean
something on this axis: motor RPM. Past a few hundred RPM an open-loop
step/dir stepper stops following and starts skipping steps, silently,
with no encoder to notice — which on this board corrupts the position
reference. That, not an invented deg/s figure, is the real hazard.

## 19. PID — ONE PRESET, from the Stepper MATLAB report, Table 2

Plant identified in that report (ClearCore -> TB6600 -> stepper -> 1:50):
     G(s) = 12.5 / (s * (s + 12.5))     [rad per STEP pulse]
Pole placement for POT < 5% (zeta = 0.7071), ts in [0.4, 0.8] s gives
the PID row, which is the only one this controller uses:
     Kp = 24.97   Ki = 120.00   Kd = 1.33   N = 50     ts 0.57 s

N is the derivative filter coefficient (report: 50..100, to reject the
encoder's quantisation noise without altering the designed dynamics).

v9.1 REMOVED the P / PI / PD alternatives and the PARALLEL / I-PD /
PREFILTER form selector. Only the PID row was ever used, and the form
field configured a controller structure that does not exist on this
board — it was a menu that changed nothing but could still be got
wrong. PID_OFF replaces it: one switch that says whether these gains
are in play at all.

*** THIS BOARD DOES NOT CLOSE THE LOOP ***  The drive is open-loop
step/dir; the encoder is a monitoring device only, exactly as the
report states. These values are stored and echoed so the controller
configuration matches the documented Simulink model — they are not
running a PID here.

## 20. PLC LINK — MELSEC MC PROTOCOL 3E, ASCII FRAMES, TCP 192.168.3.101:1025

The PLC, not this board, owns the reference position and the physical
limit switches. ClearCore is a READ-ONLY MC-protocol client: it
batch-reads one word of bit devices and writes nothing at all. The one
thing it asks the PLC to do — HOME — goes out on a wire from IO-0 into
X0, not over this socket.

WHY MC PROTOCOL AND NOT A CUSTOM TEXT PROTOCOL
----------------------------------------------
v9.0 sent the line "M2\n" and waited for the substring "DONE". That
only works if somebody writes a SOCOPEN/RECV/SEND ladder on the PLC to
parse it. Port 1025 with a device comment list is the stock MELSEC
Ethernet configuration, which already answers device read/write
requests with no ladder code at all — so the handshake is bits in the
PLC's own device memory, visible in GX Works while it runs. Nothing to
write on the PLC side except the home sequence itself.

ASCII rather than binary frames on purpose: an ASCII frame is
legible in a packet capture and in this board's own log, so a wrong
device number is a five-second diagnosis instead of a hex dump. It
costs twice the bytes of binary, which at ~30 bytes per poll is
irrelevant. Set PLC_MC_ASCII to 0 only if the PLC's Ethernet module is
already fixed to binary; the frame builders below would then need
rewriting, so this is deliberately NOT a switch that half-works.

>>> THE PLC's Ethernet module must be configured for MC Protocol on
>>> port 1025, ASCII, TCP. That is the screen the IP/port came from.

## 21. PLC DEVICE MAP — transcribed from the GX Works comment list.

Only the devices this board actually touches are wired up; the rest
are recorded so the next person does not have to go and find the
screenshot again. READ means ClearCore polls it, WRITE means ClearCore
sets it.

  X0   HOME request     WIRE   driven by ClearCore's IO-0 terminal, NOT
                               over Ethernet — see the block below
  M0   RUN                     (PLC-side, not used here)
  M1   DONE             READ   homing finished, machine is on reference
  M2   rHOME                   (PLC-side)
  M3   STOP                    (PLC-side)
  M4   rJOG                    (PLC-side)
  M5   MinZ             READ   ZM  physical boundary reached
  M6   OutR             READ   RM  physical boundary reached
  M7   OutR1            READ   A1M physical boundary reached
  M8   OutR2            READ   A2M physical boundary reached
  M10  Run ZM           READ   PLC is driving ZM
  M11  Run RM           READ   PLC is driving RM
  M12  Run A1M          READ   PLC is driving A1M
  M13  Run A2M          READ   PLC is driving A2M
  M15/16/17  rLY/rLR/rLG      (lamps, PLC-side)
  M20  AUTO / M21 HOME / M22 fLED / M23 sHOME   (PLC-side)
  M25/26/27  ILY/ILR/ILG      (lamps, PLC-side)

M1 through M13 all live inside the single word M0..M15, so ONE batch
read per poll fetches the whole handshake. Three separate reads would
be three round trips for the same information.

## 22. The HOME request: A WIRE, not a packet

ClearCore's IO-0 terminal is wired directly to the PLC's X0 input, and
that is the whole HOME request. **Ethernet is READ-ONLY.** Nothing in
this sketch writes a PLC device any more; the socket exists only to
batch-read M0..M15.

This is the right way round, and it fixes a real defect rather than
being a matter of taste. X devices are refreshed from the physical
input terminals at the top of every PLC scan, so anything MC protocol
wrote into X0 was overwritten within one scan (< 10 ms) the moment X0
had a wire on it. The old code worked ONLY while X0 was unwired, and
the failure — HOME never starting, M10..M13 never coming on — looked
like a ClearCore fault. A physical output into a physical input is
exactly what the PLC's scan expects, so there is nothing left to race.

It is also faster and fails better: no socket, no round trip, no
timeout to sit through, and the request survives an Ethernet dropout
that would have stranded a written bit ON at the PLC.

The PLC ladder needs no SOCOPEN/RECV and no change at all — X0 is the
same HOME input it always was, in parallel with the panel pushbutton.

IO-0 is a general-purpose digital I/O on ClearCore and is used here as
an OUTPUT. Do not also configure it as an input elsewhere.

## 23. Poll cadence: slow when idle, fast while homing

TWO rates, and the second one is not an optimisation — it is what keeps
the HOME handshake correct.

Idle, there is nothing urgent in the status word: the home sensors and
the run bits are only interesting during a home, so 5 s is plenty and
leaves the Mitsubishi's Ethernet module almost entirely alone.

During a home it MUST be fast. plcHomeDoneAsserted() completes a cycle
only after it has SEEN M10..M13 come on and then go off again — that
gate is what stops a stale latched M1 from ending the next home before
the machine has moved. At 5 s, a home sequence shorter than one poll
interval finishes entirely between two polls: the run bits are never
observed, plcSawRunDuringHome stays false, and a home that physically
succeeded fails on the 30 s timeout. Intermittently, depending on where
the poll happened to land, which is the worst kind of fault to chase.

200 ms is short enough to catch any real PLC home sequence, and it only
applies for the few seconds a home is actually running.
5 s idle was chosen deliberately — it leaves the Mitsubishi alone when
nothing in the status word is urgent. It is NOT the reason a sensor reads
stale: a working link at 5 s shows a covered sensor within 5 s, whereas a
dead link never shows it at all.

It is now the DEFAULT rather than a constant, because 5 s is slow when you
are standing at the machine waving a hand over a sensor to test it.
SET_PLC_POLL:<ms> speeds it up with no re-flash; the value is not
persisted, so a power cycle returns to the deliberate default.

## 24. The socket timeout must be LONGER than the CPU monitoring timer below,

or this board abandons a request the PLC is still going to answer. The
late reply would then arrive while the next request is outstanding and
be read as ITS answer — a boundary bit pattern applied one cycle late,
which is the kind of fault that shows up once a week and never
reproduces. Monitoring timer 500 ms, socket 800 ms.

## 25. COMMUNICATION DATA CODE MUST MATCH THE PLC'S OWN SETTING, EXACTLY.

MC protocol 3E header fields. "03FF" + station 00 addresses the CPU the
Ethernet module is mounted on, which is the normal single-CPU case.


GX Works3: Module Parameter -> Ethernet Port -> Basic Settings ->
Communication Data Code. ASCII and Binary are two unrelated wire formats
with no overlap — a socket opened for one silently drops every frame
sent in the other, no error, nothing. That is not a guess: this board
sent ASCII, the PLC's own "Own Node Settings" screen was confirmed set
to Binary, and the field symptom was exactly what that mismatch produces
— TCP connects clean, plcGoodReads stays 0 forever, RX buffer always
empty. This machine's PLC is Binary, so PLC_MC_ASCII is 0.

The two frame formats below are NOT a half-measure switch: field widths,
byte order, and even which helper functions apply differ completely
between them (see the binary byte helpers further down). Flipping this
macro must reproduce whatever the PLC's screen actually says — check
there first if this board and the PLC ever disagree again.

## 26. M5..M8 ARE HOME SENSORS, NOT LIMIT SWITCHES

They are read to know when each axis has reached its reference, and for
nothing else. In particular they do NOT:
  - stop a jog or a run
  - register the position they trip at as a working boundary

An earlier revision did both. It is wrong twice over: a home sensor sits
AT the reference, so stopping on it would make it impossible to jog off
home, and writing its trip point into a limit would overwrite a taught
boundary with a position that is not a boundary at all.

Working boundaries come from the operator only — typed, or taught with
SET HERE. Physical protection is the PLC's own ladder.

WHAT THEY DO DO
---------------
All four are WIRED and working, and they sit at OPPOSITE ENDS:

  M5 MinZ   ZM  at the BOTTOM of the stroke   -> the DOWN end
  M6 OutR   RM  at the CCW stop                -> the 0 deg end
  M7 OutR1  A1M fully EXTENDED                 -> the far end
  M8 OutR2  A2M fully EXTENDED                 -> the far end

So M5/M6 mark the minimum of their axis and M7/M8 the maximum of theirs.
HOME STATE is therefore M5 AND M6 covered while M7 and M8 are CLEAR: at
home the lift is down, the turntable is at 0, and both arms are pulled
IN, which is the opposite end from where M7/M8 sit.

WHERE THEY ARE ENFORCED — P2P YES, JOG NO
-----------------------------------------
A covered sensor REFUSES a point-to-point leg that would drive that axis
further into it. A program runs unattended and the operator is not
watching the axis, so the move must not start.

In JOG the same condition only WARNS. Jog is a dead-man control: it moves
only while the button or key is held, the operator is looking at the
machine, and jogging is how you get OFF a tripped sensor in the first
place. Blocking it there also risks pinning the machine on its own switch.
Physical protection while jogging is the PLC's own ladder.
Which direction each sensor sits at. -1 = the axis minimum, +1 = maximum.

## 27. The HOME request line is the same terminal in EVERY link mode: it is a

wire to the PLC's X0 input, and the Ethernet socket has nothing to do
with it. Defining it once, outside the #if, is what stops the two modes
drifting on to different terminals — they did, IO5 against a comment
that said IO-0, and only one of them was on the machine.

## 28. Jog dead-man watchdog

The host holds a jog by sending one start command, so if that host
crashes, unplugs, or simply fails to send the stop, the axis would run
until ESTOP. (Exactly that happened: a GUI bug swallowed the release
event and the arm kept moving.) The host must now refresh the jog with
JOG_HB; if nothing arrives within the timeout the board stops itself.
Set to 0 to disable when driving the board by hand from a terminal.

## 29. INVERSE / FORWARD KINEMATICS

Direct port of solve_ik_frogleg() from mophong_init.m. The MATLAB
version silently CLAMPS an out-of-range target; this version reports
the violation instead, because a machine must never quietly move
somewhere other than where it was told.
(struct IkResult is declared near the top of this file — see the
 "TYPES" block there for why it cannot live down here.)

## 30. d1 = Z - Z_offset

Checked against the OPERATOR's limits, not the factory envelope: a
target the operator has excluded must be refused here, at the point
it is commanded, rather than half-executed and then stopped by the
jog limit part-way through the move.

## 31. radial reach -> elbow angle

Each arm has its own elbow limits, so the reachable radius band is
per-arm too. Using arm 1's band for arm 2 is the v8 mistake.
The taught band is in MOTOR degrees; the reach curve is a function of
the FROG-LEG angle. Converting here is what keeps a re-calibration of
armGearRatio from silently moving the reachable envelope.
Arithmetic first: outside a3+a6 +/- (a4+a5) there is NO elbow angle at
all, on any machine, and foldAngleFromReach() would clamp and hand
back a pose that is not the one asked for. This refusal is not an
opinion about the envelope and is not switchable.

## 32. Then the OPERATOR's envelope, and only if this axis is enforced.

There is no structural floor of 133.2 mm any more: that was
R(fold = 0) and it assumed the elbow's zero really is the folded home
pose, measured through an armGearRatio nobody has verified. Rejecting
a radius the arm is physically standing at is worse than having no
floor — see the same decision for the elbow boundaries themselves.

## 33. The arm is bounded in MOTOR RPM, before any conversion, because motor

RPM is the only unit on this axis that currently means anything —
The arm's deg/s below is MOTOR deg/s, which needs no gear ratio;
from it is a guess. Stepper stall, not an invented angular speed, is
the thing worth guarding against here.

## 34. Position readback in engineering units.

NOTE: HOME is d1 = 0, rot = 0, and BOTH ELBOWS AT 0 — zero motor
degrees, zero fold degrees. (th3_cad calls the same pose 60 deg; that
label is not used or reported anywhere.) The elbow's
pulse zero therefore corresponds to 60 deg, not 0 deg — that offset is
applied here so every reported angle is a true th3_cad.

## 35. SOFT-LIMIT VALIDATION — defence in depth. The GUI already rejects

bad targets, but the board must never assume the GUI is the only
thing that can talk to it.
An axis whose enforcement the operator has switched OFF is skipped
here as well as in the jog clamp. Validating against a boundary that
will not be applied would refuse a target the machine is willing to
drive to — the operator would be told the point is illegal by the same
system that has been told to stop policing that axis.

## 36. OPERATOR LIMIT EDITING

Every write goes through applyLimit(), which is the single place that
enforces the two rules a limit pair must always satisfy:
  1. it stays inside the factory envelope (structure wins over opinion)
  2. MIN stays below MAX by at least a usable span, so an axis can
     never be boxed into a range it cannot be jogged out of

The two TAUGHT elbow axes are exempt from both: they have no known
envelope to check against, and their pair is unordered and simply
sorted after each write. Rule 2's real guarantee — that the band is
never empty — is still upheld, by refusing a write that would put both
ends on the same position.
There is deliberately no "this boundary is locked" refusal any more.
The old per-axis LOCK froze the value, and it was the only thing the
per-axis button did — so the panel could tell you a boundary was
UNLOCKED while saying nothing about whether it was switched on. The
button now reports enforcement, and a switched-off axis is still fully
editable: you want to be able to teach a boundary while it is not yet
policing anything.

## 37. Store the captured number RAW, in the slot that was asked for.

It is deliberately NOT sorted here. Sorting on write would fold the
first taught position against whatever stale value sat in the other
slot, and the operator's second SET HERE would then silently
overwrite their first. The pair is sorted where it is READ instead
— armBand() — so both taught numbers survive intact and order stops
being something anyone has to think about.

The one arrangement that cannot work is both ends on the same
position: that pins the axis where it stands and no jog gets it
out again.

## 38. REPORTING

A1M/A2M carry MOTOR degrees. The FOLD/R fields are the derived
frog-leg angle and reach — appended rather than substituted so a host
that only knows the old five fields still parses the line, and so
somebody on a bare terminal can see both numbers at once.

## 39. EVERY Z ON THE WIRE IS MEASURED FROM HOME.

CARTESIAN COMMAND HANDLERS (new in v9)


HOME is the P2P reference point — X 0, Y 0, Z 0. X and Y are measured
from the turntable axis and are signed, because RM can put the arm
behind the machine; Z is the lift's travel UP from HOME and cannot be
negative, because HOME is the bottom of the stroke.

solveIkFrogleg() still works in the ABSOLUTE frame (arm 1's deck at
514.3 mm with the lift down) and must keep doing so: that is the frame
mophong_init.m is written in, and the parity sweep in firmware_check
compares the two to machine precision. So the conversion happens at
this edge, in ONE function, and is per arm — one carriage, two decks
9 mm apart, so the same Z-from-home is a different absolute height for
arm 1 and arm 2.

## 40. The idle arm HOLDS WHERE IT IS. It does not park at home.

mophong_init.m pins AM2 at th3_home_cad for the whole trajectory, and
this function used to copy that by writing FOLD_ANGLE_HOME_DEG into
the idle slot. On a plot that is free; on the machine it is a move of
up to 120 deg that the operator never commanded, on an arm that may
well be carrying a substrate. The GUI already computed the idle angle
as "leave it alone" (solve_ik's idle_deg), so the board snapping it
home also meant LOAD and LOAD_XYZ produced different poses from the
same coordinates depending on which one you used.
r.th3 is a FROG-LEG angle; the motors are commanded in MOTOR degrees.

## 41. One carriage lifts both decks, so in the FROM-HOME frame there is one

Z and the two commanded values must be EQUAL.

This used to demand Za - Zb = 9 mm, and that was right while Z was an
absolute height: the two decks are 9 mm apart, so one carriage
position was two different absolute numbers. From HOME it is the
carriage's own travel, identical for both, and the 9 mm is applied
inside solveIkFromHome() instead. Keeping the old rule as well would
apply the drop twice.

## 42. ── Soft limits are only meaningful once the machine has a reference ──

At power-on the step counters read 0, which this firmware maps to the
HOME pose: elbow = 0 (the retracted stop) and d1 = 0 (the bottom
of the lift). Those are exactly the MINIMUMS, so an EARLIER version
that clamped absolutely refused to retract the arms or lower Z from
the moment it booted, no matter where the machine physically was.

The fix for that was to suspend the limits entirely until a reference
existed. That was the wrong half of the problem to solve: it meant a
boundary the operator had just taught at -300 let the axis run past
-420, which is a machine with no protection at all.

What is done now: the taught boundaries apply IMMEDIATELY, and every
clamp is DIRECTIONAL — it stops `dir` only when that direction takes
the axis further outside its band. An axis sitting outside a boundary
at power-on can therefore always be jogged back in, and the boot-time
deadlock cannot happen. A reference is still needed before any
ABSOLUTE move, which is what RESET_COORD and HOME are for.
Enforcement needs BOTH a reference to measure against and the master
switch on. Without the reference the numbers are meaningless; without
the switch the operator has deliberately asked for a free machine.
A TAUGHT BOUNDARY APPLIES IMMEDIATELY — isHomed is NOT part of this.

This used to be `isHomed && limitsEnabled`, and jog was free in both
directions until the machine had a reference. The argument was that the
counters mean nothing before then. It does not survive contact with how
boundaries are actually set: the operator jogs to the stop and presses
SET HERE, so the boundary is captured against the SAME counters it is
later compared with. It is meaningful in exactly the frame it was
taught in, reference or not. Waiting for a reference meant a limit
somebody had just taught at -300 let the axis run to -427.

Being outside the band at power-on cannot trap an axis, because every
check below is DIRECTIONAL: it stops `dir` only when that direction
takes the axis further out. Jogging back toward the band is always
allowed.

isHomed still matters, and still gates the things it should: absolute
Cartesian moves and anything that claims to know where the machine is.

## 43. Per-elbow soft limit. Each arm is clamped against its OWN measured

angle — reaching AM1's stop must not halt AM2.

`whichArm` is 1 or 2 rather than a MotorDriver reference: the Arduino
prototype generator copies a signature verbatim to the top of the
sketch, and a namespace-qualified reference parameter there is a
second easy way to break the build. Plain ints are always safe.

## 44. PLC TRANSPORT — MC PROTOCOL 3E, ASCII

HOMING

FRAME LAYOUT (request), all fields ASCII hex unless noted:

  5000        subheader
  00          network no.
  FF          PC no.
  03FF        request destination module I/O no.
  00          request destination multidrop station no.
  NNNN        request data length: the character count of everything
              from the monitoring timer to the end of this frame
  0010        CPU monitoring timer, units of 250 ms
  0401/1401   command: batch read / batch write
  0000/0001   subcommand: word units / bit units
  M*000000    device code (2 chars) + device number (6 chars)
  0001        number of points
  [data]      write only: one char per bit, or 4 chars per word

Response:

  D000 00 FF 03FF 00  NNNN  EEEE  [data]

where NNNN is the character count of (end code + data) and EEEE is the
end code — "0000" is success, anything else is a PLC error number that
is worth logging verbatim because the manual indexes them.

The request data length is COMPUTED from the assembled string rather
than hand-counted. A wrong length makes the PLC either hang waiting for
bytes that never arrive or answer with an error that looks like a
device problem, and both cost an afternoon.

## 45. Hex/decimal text helpers

Shared by both wire formats: ASCII framing uses these to BUILD the wire
text itself; binary framing only uses them for LOG DISPLAY (turning an
already-decoded value into readable hex for sendFeedback), never for the
wire. Uppercase, zero padded, fixed width. MC protocol ASCII framing is
not tolerant about either: a lower-case 'a' and a missing leading zero
are both rejected.

## 46. Binary byte helpers

String is reused as a raw byte buffer here, NOT as text. That is safe
for the operations below: String::operator+=(char) and .length() track
an explicit length rather than calling strlen(), so an embedded 0x00
(the subheader's own second byte, on every single frame) does not
truncate anything. What is NOT safe, and is deliberately never used on
a binary buffer anywhere in this file, is any String method that calls
into the C string library under the hood (startsWith, indexOf, the
String(const char*) constructor) — those stop at the first embedded
0x00 and would silently mis-parse almost every real frame. Byte access
below always goes through plcByteAt()/plcU16At(), never those methods.

## 47. Batch read in WORD units, binary. The device number has no separate

hex/decimal numbering rule the way ASCII's plcDeviceNum() does — that
distinction is purely a text-encoding concept — it is always a raw
3-byte little-endian integer. Device code is one byte from the MELSEC
table; only M is wired up because M is the only device this board reads.

## 48. *** THERE IS NO WRITE FRAME BUILDER, ON PURPOSE ***

The link is READ-ONLY. The only thing this board ever asked the PLC to
do was HOME, and that is now a wire from IO-0 into X0. Leaving a
plcFrameWriteBit() here "in case it is useful" would be an invitation to
reintroduce exactly the defect the wire fixes — X devices are refreshed
from their terminals every scan, so a written X0 is overwritten within
one scan and the request silently never arrives.

If a future feature really does need to set a PLC device, use an
internal relay (M-device, decimal numbering), not an X, and add the
builder back deliberately with that constraint written down.

## 49. How many connects SUCCEEDED. Reported, because "we have never opened a

socket" and "we open one repeatedly and get no answer" are different
faults and want different words: the first is cable or address, the second
is the protocol. Without this the host could only see the socket's current
state, which cycles, so its lamp cycled with it.

## 50. LINK STATE IS ABOUT DATA, NOT ABOUT THE SOCKET

"Connected" used to mean "a TCP socket opened". That made the host's lamp
FLAP green/red every few seconds whenever the PLC accepted a connection
but did not answer device reads:

  connect OK -> lamp green -> poll -> no reply -> 800 ms timeout ->
  socket dropped for resync -> lamp red -> 3 s later reconnect -> green

The socket really was going up and down, so the lamp was not lying about
the socket — it was answering a question nobody asks. What the operator
needs to know is whether device data is arriving, which is the thing HOME
and the sensors depend on.

plcDataState() reports that, and the flapping stops because a link that
opens and never answers now sits steadily on NO_REPLY.
STALENESS IS DERIVED FROM THE POLL RATE, NOT HARD-CODED.

It was a constant 4000 ms, written when the idle poll was 500 ms. The poll
later became 5000 ms and the constant did not follow, so the window was
SHORTER than the interval between reads: a perfectly healthy link reported
OK for 4 s, then STALE for the 1 s until the next reply landed. The host
asks for PLC_STATUS every 3 s, so roughly one poll in five caught that
window and the lamp dropped CONNECTED -> NO REPLY and back — the same flap
this whole section exists to remove, one level up.

Three intervals of slack: one missed reply is a hiccup, three in a row is
a link that has stopped. The +1000 covers the 800 ms socket timeout that
precedes a retry, so a single timeout cannot age the data out on its own.
It must be a FUNCTION, not a constant: SET_PLC_POLL can move plcPollIdleMs
anywhere from 20 ms to 60 s at runtime, and homing polls at a different
rate again.

## 51. Fire and forget: the reply is collected by plcServiceRx() on later loop

passes. Nothing here waits, so a slow or absent PLC cannot stall the
motion loop — that is the whole reason this is a state machine and not
a blocking request/response call.
No `isRead` parameter: every transaction on this socket is a read now,
so a flag saying which kind it is could only ever be wrong.

## 52. PUSH the decoded word whenever it changes rather than waiting to be

asked. The host polls PLC_STATUS every few seconds; on its own that made
a sensor change take up to one host poll PLUS one PLC poll to appear.
Only on a change, so it cannot become a log flood. Shared by both parsers
below — the only thing that differs between ASCII and binary is how the
word got decoded, not what happens once it has.

## 53. Byte-indexed twin of the ASCII parser above. Every offset here is BYTES

into plcRxBuf, not characters — the two units are not interchangeable,
see PLC_MC_RES_HEADER_UNITS's definition above. A garbled length field
cannot be detected as "malformed" the way ASCII's hex parse failure can
— any 2 bytes decode to SOME uint16 — so a bad length just never
completes and the transaction times out and resyncs the normal way;
there is no separate error branch for it here.

## 54. The stream is now out of step: if that reply is merely late it will

arrive while the NEXT request is outstanding and be parsed as its
answer. Dropping the socket is the only way to resynchronise for
certain, and it costs one reconnect on a link that is already
misbehaving.

## 55. Parses the one-shot PLC_TEST reply and reports what it means. Separate

from plcConsumeResponse() on purpose — PLC_TEST is a blocking diagnostic
run with the machine stopped, not the incremental state machine the
normal poll uses, and it needs to say something useful about a reply
that never showed up at all, which plcConsumeResponse() never sees
(it is only called once bytes already exist).

## 56. Reacts to the four boundary bits. Called after every successful poll.

Two jobs, and the order matters: STOP the axis first, then register the
position. Registering first would spend a few hundred microseconds in
String building while the axis is still driving into its stop.
All four optical home sensors covered?

## 57. Reports home-sensor transitions. Read-only: it stops nothing and writes

no limits (see the header block on M5..M8).
JOG: warn only, never stop. See the header block — jog is a dead-man
control and is also how the operator gets off a tripped sensor.

Rate-limited to once per axis per entry into the condition, or a held jog
against a covered sensor would emit a line every 50 ms poll.

## 58. HOME STATE: M5 and M6 covered, M7 and M8 clear.

This is the machine physically sitting on its reference, so it is the
one condition that may zero the counters without anybody asking. It
fires on the RISING EDGE only — while the machine sits at home the
condition stays true, and re-zeroing every 50 ms poll would silently
swallow any real motion away from home.

## 59. True when the PLC says homing has finished.

DONE (M1) alone is not enough. The PLC leaves M1 latched after a home,
so a second HOME would be answered instantly by the previous cycle's
bit before the machine had moved at all. The gate is therefore: the run
bits M10..M13 must have been seen ON at least once during this cycle,
they must now all be OFF, and M1 must be set.

## 60. RESET THE COORDINATE SYSTEM TO THE STANDARD HOME POSE.

The PLC has put the machine on its reference position, so this
board's step counters must be re-anchored to the home pose or
every later absolute move would be offset by the drift.

The PLC has just put the machine on its reference position, so this
is the one moment the step counters can be anchored to a known pose.
All four go to zero, which by definition IS the standard home:
  d1  = 0 mm      lift at the bottom of its stroke
  ROT = 0 deg     turntable centred
  A1M = A2M = 0   motor degrees from home, i.e. fold 0 deg, R 133.2 mm
Skipping this leaves every later absolute move offset by however far
the machine had drifted from where the board thought it was.

## 61. Speed: ONE universal RPM + one percentage per motor, PLUS an

---- independent acceleration percentage per motor ----
  SET_SPEED:masterRpm,masterAccRpmS,rotPct,armPct,zPct,
            rotAccPct,armAccPct,zAccPct
The 3 accel fields were appended, never inserted -- a stale 5-field
sender fails the count check below and is refused loudly, rather than
having its 5th field silently misread as something else.
Every field is range-checked BEFORE any is applied, so one bad value
cannot leave the axes half-configured.

## 62. PLC diagnostics. Worth having on the wire rather than only in the GUI:

when HOME does not work the question is always "what does the PLC
actually say", and this answers it from a plain serial terminal.
Calibrating the elbow drivetrain WITHOUT a re-flash. This is the one
number that turns the motor rotation the board counts into a real
frog-leg angle, so it has to be settable from the bench.

## 63. The HOME request line is configured in EVERY link mode — it is a wire

to X0, not a feature of the Ethernet path. It is also driven to its
inactive level before anything else runs: a floating or latched-high
terminal at power-on would ask the PLC to home a machine nobody has
looked at yet.
