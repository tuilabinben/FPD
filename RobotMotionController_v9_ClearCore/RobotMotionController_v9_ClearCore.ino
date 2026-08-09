/*
 * ============================================================
 * STCR4000S Motion Controller — ClearCore Firmware v9
 * ============================================================
 * WHAT CHANGED FROM v8, AND WHY
 * -----------------------------
 * v8 was a pure joint-space board: the Python GUI did all inverse
 * kinematics and the firmware only ever saw d1/rot/a1/a2. It also
 * carried a kinematic model that was simply wrong:
 *
 *     v8 / old GUI:  reach = 2 * 157.5 * cos(theta),  theta 0..90 deg
 *                    -> reach 0..315 mm, Z 0..200 mm, no per-arm offset
 *
 * The real machine, per MATLAB_v4_final/mophong_init.m (the Simscape
 * model generated from the SolidWorks assembly):
 *
 *     R  = a3 + a6 + (a4 + a5) * cos(th3_math)
 *        = 293.2 + 320 * cos(pi - th3_cad)
 *        = 293.2 - 320 * cos(th3_cad)
 *     d1 = Z - Z_offset,  clamped 0..285
 *     th2 = atan2(Y, X)
 *
 * So the arm carries a FIXED 293.2 mm radial offset (a3 + a6) that v8
 * did not model at all: the wafer centre can never come closer than
 * 133.2 mm to the rotation axis, and reaches 613.2 mm when straight.
 * Commanding "reach = 0" as v8 allowed is not a retracted arm, it is a
 * point the machine physically cannot represent.
 *
 * v9 therefore implements solve_ik_frogleg() ON THE BOARD, so the
 * firmware is self-contained and can be driven from a plain serial
 * terminal with Cartesian targets, with no GUI in the loop.
 *
 * GEOMETRY — every constant traced to mophong_init.m
 * --------------------------------------------------
 *   a3 = 45     turntable centre -> shoulder pivot
 *   a4 = 160    upper frog-leg link      \  drawing MTCR4160-300-AM
 *   a5 = 160    lower frog-leg link      /  reads "160  160" — AGREES
 *   a6 = 248.2  wrist pivot -> wafer centre
 *   d_base = 388, d3_arm1 = 50, d3_arm2 = 41, d4 = 46.5, d5 = 24.8, d6 = 5
 *   Z_offset(arm1) = 388+50+46.5+24.8+5 = 514.3
 *   Z_offset(arm2) = 388+41+46.5+24.8+5 = 505.3   (9 mm lower deck)
 *   i_RM_total = 4.375 * 6.5 = 28.4375  (motor revs per turntable rev)
 *       NOTE: mophong_init.m still writes 4.375 * 6.4 = 28. The Simscape
 *       block diagram and the machine both use 1/4.375 then 1/6.5, so
 *       28.4375 is the value that matches hardware. The .m file is left
 *       alone deliberately — it is the simulation's own record.
 *
 * ANGLE CONVENTION — read this before touching any limit
 * ------------------------------------------------------
 * All arm angles in this file, in the GUI, and on the wire are
 * The elbow angle this board REPORTS is rotation from home:
 *      fold =   0 deg -> retracted, R = 133.2 mm   (HOME)
 *      fold = 120 deg -> straight,  R = 613.2 mm   (singularity)
 * th3_cad (60 retracted, 180 straight) survives only inside
 * reachFromFoldAngle() / foldAngleFromReach(). See ARM_ZERO_CAD_DEG.
 * Reach GROWS with the angle. This is the opposite of v8's convention,
 * where 0 was full reach. If you flash v9 over v8 without re-homing,
 * the arm's zero reference is different — RE-HOME BEFORE MOVING.
 *
 * SINGULARITY WARNING
 * -------------------
 * FOLD_ANGLE_MAX_DEG is set to the full mathematical 120 deg by
 * deliberate choice. At 180 deg the two frog-leg links are colinear:
 * radial stiffness collapses and dR/dth3 -> 0, so encoder/step error at
 * the elbow becomes large radial error at the wafer. JEL's own drawing
 * stops at 575 mm (fold = 91.72 deg). To use the conservative
 * limit, set FOLD_ANGLE_MAX_DEG = FOLD_ANGLE_SPEC_MAX_DEG below.
 *
 * INDEPENDENT ARMS
 * ----------------
 * v8 held one `armDir` and drove AM1 and AM2 from it, so the two elbows
 * could never be positioned separately — and its soft-limit check read
 * AM1's angle to decide whether to stop BOTH motors, meaning AM2 could be
 * driven past its own stop without ever being noticed. v9 gives each
 * elbow its own direction, its own soft limit checked against its own
 * measured angle, and its own jog/move commands. ARM_FWD/ARM_BACK remain
 * as the explicit "both together" gesture.
 *
 * MOTOR MAPPING (unchanged from v8):
 *   M-0  ->  ZM   Z lift        (prismatic, d1, mm)
 *   M-1  ->  RM   rotation      (revolute, th2, deg, through i_RM = 28)
 *   M-2  ->  AM1  arm 1 elbow   (revolute, deg from home)
 *   M-3  ->  AM2  arm 2 elbow   (revolute, deg from home)
 *
 * SAFETY DEFAULTS CARRIED OVER FROM v8 — DO NOT "OPTIMISE" AWAY:
 *   - Boot motion profile is conservative and reproduces the MATLAB
 *     trajectory, NOT some round number that looks fast in a dialog.
 *   - Limit-sensor handling is OPT-IN and compiled out by default. A
 *     floating digital input commonly reads HIGH, which would make the
 *     board believe an axis is at its limit the instant it moves.
 *   - ESTOP decelerates hard rather than MoveStopAbrupt(): an abrupt
 *     stop on an open-loop step/dir motor skips steps with nothing to
 *     detect it, silently corrupting the position reference.
 *
 * SPEED MODEL — ONE UNIVERSAL RPM, ONE PERCENTAGE PER MOTOR (NEW)
 * --------------------------------------------------------------
 * There is now a single universal speed, in MOTOR RPM, and each motor
 * runs at its own percentage of it:
 *
 *     axisMotorRpm = masterRpm * (axisPercent / 100) * AXIS_RPM_SCALE
 *
 * AXIS_RPM_SCALE is a CALIBRATION constant, not a user setting. It exists
 * because the three axes are geared completely differently, so a raw
 * percentage of one shared RPM would be meaningless:
 *
 *     RM  28.4375:1  -> 140 motor RPM gives  29.5 deg/s   scale 1.00
 *     ZM  20 mm/rev  -> 105 motor RPM gives  35.0 mm/s    scale 0.75
 *     AM  ratio UNMEASURED -> runs at the master RPM       scale 1.00
 *
 * AM's scale was 0.03, from "25 deg/s = 4.17 motor RPM", which only held
 * if the arm gear ratio were really 1.0. On the machine 100 deg/s was still
 * visibly slow — proof the elbow has a real reduction and the motor was
 * being throttled to ~17 RPM while every other axis ran at 100+. Its
 * scale is now 1.0 and its old 100 deg/s ceiling is GONE.
 *
 * RM and ZM are still clamped to a real engineering ceiling, because for
 * those two the gearing is known. The arm is clamped in MOTOR RPM
 * instead (ARM_RPM_MAX), which is the only unit on that axis that
 * currently means anything, and which guards the hazard that actually
 * exists: an open-loop stepper skipping steps at high RPM with no
 * encoder to notice.
 *
 * PID NOTE: Kp/Ki/Kd/N are stored and echoed only. This is an OPEN loop
 * (no encoder feedback on this board); there is nothing for a PID loop
 * to close over unless your driver does its own closed-loop tuning.
 * The values are the single PID preset from the Stepper MATLAB report,
 * Table 2. v9.1 dropped the P/PI/PD presets and the PARALLEL/I-PD/PREFILTER
 * form selector: only one preset is used in practice, and the form field
 * was configuration with no effect on an open-loop board. PID_OFF stops
 * the gains being sent/echoed at all.
 *
 * PROTOCOL — Host -> Board
 *   PING
 *   BYE
 *   SET_PID:kp,ki,kd[,N]               N = derivative filter coefficient
 *   PID_ON / PID_OFF                   enable/disable the stored gains
 *   PID_RESET                          restore the single report preset
 *   SET_SPEED:masterRpm,masterAccRpmS,rotPct,armPct,zPct,
 *             rotAccPct,armAccPct,zAccPct
 *       THE speed command. masterRpm is motor RPM; rotPct/armPct/zPct are
 *       per-motor shares of it (see SPEED MODEL above); the trailing 3
 *       are independent per-motor shares of masterAccRpmS.
 *   SET_MOTION:rotVel,rotAcc,armVel,armAcc,zVel,zAcc
 *       Legacy engineering-unit form; converted into the percentages.
 *   SET_PARAMS:...                     legacy v8 form; PID honoured,
 *                                      speed/accel fields ignored
 *   PROFILE / STATUS                   report the active profile and PID
 *   SET_BOOST:multiplier
 *   -- joint space (v8-compatible) --
 *   LOAD:d1A,rotA,a1A,a2A,d1B,rotB,a1B,a2B
 *   LOAD_BOTH:d1,rot,a1,a2
 *   -- Cartesian, NEW in v9: board runs the IK --
 *   -- Cartesian. HOME is the reference: X 0, Y 0, Z 0. X/Y are from the
 *      turntable axis and may be negative; Z is height ABOVE HOME and may
 *      not. The 9 mm deck offset is applied by the board, not by you.
 *   MOVE_XYZ:arm,X,Y,Z                 arm = 1 or 2, immediate move
 *   LOAD_XYZ:arm,Xa,Ya,Za,Xb,Yb,Zb     A -> B for one arm
 *   LOAD_XYZ_BOTH:Xa,Ya,Za,Xb,Yb,Zb    simultaneous, Za must EQUAL Zb
 *   IK:arm,X,Y,Z                       compute and report only, no motion
 *   FK:d1,rot,a1,a2,arm                report Cartesian, no motion
 *   -- independent per-arm control, NEW in v9 --
 *   A1_FWD / A1_BACK / A1_STOP         jog arm 1's elbow alone
 *   A2_FWD / A2_BACK / A2_STOP         jog arm 2's elbow alone
 *   MOVE_A1:th3  /  MOVE_A2:th3        one elbow to an absolute angle
 *   MOVE_R1:mm   /  MOVE_R2:mm         one elbow to an absolute reach
 *   -- common --
 *   RUN / STOP / HOME / ESTOP
 *   ROT_CW / ROT_CCW / ROT_STOP
 *   ARM_FWD / ARM_BACK / ARM_STOP      BOTH elbows together (v8-compatible)
 *   Z_UP / Z_DOWN / Z_STOP
 *   -- operator-defined reference and travel limits, NEW in v9.1 --
 *   RESET_COORD                        zero every axis counter HERE
 *   SET_REF                            alias of RESET_COORD (v9 name)
 *   CLEAR_REF                          drop the reference (limits still apply)
 *   SET_LIMIT:axis,end,value           axis = Z|ROT|A1|A2, end = MIN|MAX
 *   SET_LIMIT_HERE:axis,end            take the CURRENT position as that
 *                                      limit — "set here as bottom/top"
 *   RESET_LIMITS                       restore the factory envelope
 *   SET_LIMIT_ENFORCE:axis,<0|1>       switch ONE axis's boundary on/off
 *                                      (values are kept either way)
 *   SET_LIMITS_ENABLED:<0|1>           the same, every axis at once
 *   LIMITS                             dump the active soft limits
 *
 * PROTOCOL — Board -> Host  (unchanged lines are v8-compatible)
 *   PONG
 *   [ALIVE] uptime: Xs
 *   [PARAMS_OK] ...
 *   [LOADED] ...
 *   [RUN] ...
 *   [CLEARCORE POS] D1: F mm | ROT: F deg | A1M: F deg | A2M: F deg (P%)
 *   [JOG POS] ROT: F deg | A1M: F deg | A2M: F deg | Z: F mm
 *             (v8 sent a single "ARM:" field; the GUI accepts both)
 *   [IK] arm=N d1=F rot=F th3=F R=F
 *   [FK] arm=N X=F Y=F Z=F
 *   [LIMITS] ...
 *   [SINGULARITY] th3=F deg — advisory
 *   [RUN] TARGET REACHED              (v9.2; was a Vietnamese string)
 *   [ESTOP] EMERGENCY STOP            (v9.2; was a Vietnamese string)
 *   [HOME] Homing started. / [HOME] Homing complete. ...
 *   [LIMIT] ROT_CW / ROT_CCW / Z_UP / Z_DOWN   (only if sensors enabled)
 *   [WARN] ... / [ERROR] ...
 * ============================================================
 */

#include "ClearCore.h"
#include <math.h>

// ---- Motor connector mapping ----
#define MOTOR_Z   ConnectorM0   // ZM  — Z lift (d1)
#define MOTOR_ROT ConnectorM1   // RM  — rotation (th2)
#define MOTOR_A1  ConnectorM2   // AM1 — arm 1 elbow (deg from home)
#define MOTOR_A2  ConnectorM3   // AM2 — arm 2 elbow (deg from home)

#define LED_PIN LED_BUILTIN


// ══════════════════════════════════════════════════════════════
// TYPES — MUST STAY ABOVE THE FIRST FUNCTION DEFINITION.
//
// The Arduino IDE auto-generates prototypes for every function in a
// .ino and injects them at the line of the FIRST function definition.
// If a user-defined type is declared lower down the file, the generated
// prototype references it before it exists and the build fails with:
//
//     error: 'IkResult' does not name a type
//
// pointing at the function's own definition line, which is confusing
// because that line is fine — it is the injected prototype that is out
// of order. Keeping every struct/enum up here, above the first
// function, is what makes the sketch build. Do not move this block down.
// ══════════════════════════════════════════════════════════════
struct IkResult {
  bool   ok;
  double d1;        // mm, carriage stroke
  double th2;       // deg, turntable
  double th3;       // deg, CAD elbow angle of the selected arm
  double R;         // mm, resulting radius (for reporting)
  String error;     // populated when ok == false
};

// A sequential program is HOME -> A -> B -> HOME, four legs.
//
// It starts and ends at the reference so the cycle is repeatable: every run
// begins from the same pose whatever the operator did by hand beforehand,
// and leaves the machine parked where the next one can start. This mirrors
// mophong_init.m, whose trajectory is P_home -> A -> B -> P_home.
enum RunPhase { PHASE_NONE, PHASE_TO_HOME_FIRST, PHASE_TO_A, PHASE_TO_B,
                PHASE_TO_HOME_LAST, PHASE_DUAL, PHASE_RESET_HOME };

// HOME is no longer a move this board commands — it is a request handed
// to the PLC, which owns the reference position. See the PLC HOME
// HANDSHAKE section further down.
enum HomeState { HOME_IDLE, HOME_REQUESTED, HOME_COMPLETE, HOME_FAILED };

// Forward declarations. The Arduino IDE would normally generate these
// itself, but declaring them explicitly means the sketch also compiles
// with a plain C++ toolchain — which is what makes it testable off-target.
void plcAssertHomeRequest();
bool plcHomeDoneAsserted();
void plcClearHomeRequest();
void plcNetworkInit();
void servicePlc();
double armFoldFromMotor(double motorDeg);
double armMotorFromFold(double foldDeg);
double pulsesPerMmZ();
float currentA1Fold();
float currentA2Fold();
String plcStatusSummary();
bool plcAnyRunBit();
bool plcBit(int number);
bool plcAllHomeSensors();
bool plcHomeStateActive();
bool runLegBlockedBySensor(float d1, float rot, float a1, float a2, String &why);
void applyMotionParams();
void applyJogVelocities();
void reportMotionProfile();
void reachBandFor(double thMin, double thMax, double &rMin, double &rMax);

void reportLimits();
void resetLimitsToFactory();
void cancelJog();
void cancelRun();
void cancelHoming();
bool anyJogActive();
void sendFeedback(const String &line);
String pidSummary();
bool applyLimit(const String &axis, bool isMax, double value, String &why);
bool currentValueForAxis(const String &axis, double &out);
bool *limEnforceFor(const String &axis);
bool axisEnforced(const String &axis);
bool axisLimited(const String &axis);


// ══════════════════════════════════════════════════════════════
// GEOMETRY — mirrors robot_sim/config.py and mophong_init.m.
// Change these in ONE place only and re-flash; the GUI reads the
// same numbers from config.py.
// ══════════════════════════════════════════════════════════════
const double A3_MM = 45.0;
const double A4_MM = 160.0;
const double A5_MM = 160.0;
const double A6_MM = 248.2;

const double D_BASE_MM  = 388.0;
const double D3_ARM1_MM = 50.0;
const double D3_ARM2_MM = 41.0;    // 9 mm below arm 1
const double D4_MM = 46.5;
const double D5_MM = 24.8;
const double D6_MM = 5.0;

const double Z_OFFSET_ARM1_MM = D_BASE_MM + D3_ARM1_MM + D4_MM + D5_MM + D6_MM;  // 514.3
const double Z_OFFSET_ARM2_MM = D_BASE_MM + D3_ARM2_MM + D4_MM + D5_MM + D6_MM;  // 505.3
const double ARM2_Z_DROP_MM   = D3_ARM1_MM - D3_ARM2_MM;                          // 9.0

const double ARM_LINK_SUM_MM      = A4_MM + A5_MM;   // 320.0
const double ARM_RADIAL_OFFSET_MM = A3_MM + A6_MM;   // 293.2

// Motor revs per turntable rev: 1/4.375 then 1/6.5 in the Simscape model.
const double I_RM_TOTAL = 4.375 * 6.5;               // 28.4375

// ---- Joint travel ----
//
// THE ELBOW ANGLE IS ROTATION FROM HOME, NOT th3_cad.
//
// 0 deg is the retracted home pose and the number counts up by however
// far the elbow has turned. th3_cad (60 retracted, 180 straight) is the
// CAD frame the geometry is written in, and it now appears ONLY inside
// reachFromFoldAngle() / foldAngleFromReach(), which add and remove
// ARM_ZERO_CAD_DEG.
//
// This board cannot produce a real th3_cad anyway: it counts steps from
// wherever it was last referenced, so what it can report exactly is MOTOR
// degrees from home. The frog-leg angle is derived from that through
// armGearRatio (see the elbow section below), and the reach follows from
// the frog-leg angle. The "60 deg" this board used to print at home was
// zero motor rotation wearing a CAD label — the honest version is A1M in
// motor degrees plus FOLD1 and R1 alongside it.
const double ARM_ZERO_CAD_DEG = 60.0;   // the CAD angle we call 0

// These are the FACTORY envelope — the physical extremes the structure
// allows. The operator's own working limits are the mutable variables
// further down; they are always clamped inside these.
const double FOLD_ANGLE_HOME_DEG     = 0.0;     // retracted, zero rotation
// 91.72, NOT 151.72. This constant is in the from-home frame like every
// other angle in this file, and foldAngleFromReach(575) = 91.72. The old
// 151.72 was th3_cad (91.72 + ARM_ZERO_CAD_DEG) left behind by the frame
// change — a th3_cad leak of exactly the kind config.py's test forbids.
// Dormant, because nothing reads it unless somebody switches
// FOLD_ANGLE_MAX_DEG over to the conservative drawing limit; had they
// done so they would have got a 151.72 deg ceiling, i.e. no restriction
// at all, while believing they had tightened the envelope to 575 mm.
const double FOLD_ANGLE_SPEC_MAX_DEG = 91.72;   // -> R = 575 mm, JEL drawing
const double FOLD_ANGLE_MIN_DEG      = FOLD_ANGLE_HOME_DEG;
const double FOLD_ANGLE_MAX_DEG      = 120.0;   // -> R = 613.2 mm, full math
const double FOLD_SINGULARITY_WARN_DEG = 110.0;

const double ARM_GEAR_RATIO_DEF = 2.0;    // motor degrees per frog-leg degree
const double ARM_GEAR_RATIO_MIN = 0.01, ARM_GEAR_RATIO_MAX = 1000.0;
double armGearRatio = ARM_GEAR_RATIO_DEF;

// The ONLY two places the ratio is applied, so it cannot be half-applied.
double armFoldFromMotor(double motorDeg) {
  return (armGearRatio == 0.0) ? motorDeg : motorDeg / armGearRatio;
}
double armMotorFromFold(double foldDeg) { return foldDeg * armGearRatio; }

// The standard HOME pose, in the units moveJointsAbsolute() takes.
// Named so the four-leg program cannot drift from the reset that HOME and
// the PLC home state perform.
const float Z_HOME_MM_BOARD      = 0.0f;   // d1 at the bottom of the stroke
const float ROT_HOME_DEG_BOARD   = 0.0f;   // the CCW stop
const float ARM_HOME_MOTOR_DEG   = 0.0f;   // both elbows fully retracted

const double D1_MIN_MM = 0.0, D1_MAX_MM = 285.0;   // ZM carriage stroke
// ── RM ZERO IS THE CCW STOP, NOT MID-TRAVEL ──────────────────────
//
// RM reads 0 at its fully counter-clockwise stop and counts up to 340 at
// the clockwise one. It was -170..+170, centred.
//
// HOME is that CCW stop, so RM now agrees with every other axis here:
// home is zero and the number counts up. Cartesian +X moves with it, so
// at RM = 0 the arm points along +X and HOME is a true X0 Y0 Z0
// reference. NOTE the 20 deg wedge between 340 and 360 is unreachable
// from either side — it is the gap the turntable cannot sweep through.
const double ROT_MIN_DEG = 0.0, ROT_MAX_DEG = 340.0;   // 340 deg total

// ══════════════════════════════════════════════════════════════
// OPERATOR-DEFINED WORKING LIMITS  (NEW in v9.1)
//
// The factory envelope above is what the STRUCTURE allows. What the
// machine may actually use is narrower and depends on what is installed
// around it — a cassette, a chamber port, a cable loop. Those limits
// belong to the operator, not to this file, so they are variables that
// SET_LIMIT / SET_LIMIT_HERE write at runtime.
//
// Each arm carries its OWN pair. v8's single shared arm limit is exactly
// how AM2 used to get driven past its stop while AM1's angle was checked.
//
// These are RAM only. ClearCore has no battery-backed store wired here,
// so the GUI keeps them in its settings file and re-sends them on every
// connect. If you drive the board from a bare terminal, re-send them
// after a power cycle or you are back on the factory envelope.
// ══════════════════════════════════════════════════════════════
double limD1Min  = D1_MIN_MM,          limD1Max  = D1_MAX_MM;
double limRotMin = ROT_MIN_DEG,        limRotMax = ROT_MAX_DEG;
// The elbow limits are held in MOTOR DEGREES, not frog-leg degrees.
//
// That is the raw count, exact whatever the gear ratio turns out to be,
// which means correcting armGearRatio never invalidates a boundary the
// operator already taught. Storing them as frog-leg degrees would rescale
// every taught number the moment the ratio changed, and the whole point
// of teaching is that it survives calibration.
double limA1Min  = FOLD_ANGLE_MIN_DEG * ARM_GEAR_RATIO_DEF,
       limA1Max  = FOLD_ANGLE_MAX_DEG * ARM_GEAR_RATIO_DEF;
double limA2Min  = FOLD_ANGLE_MIN_DEG * ARM_GEAR_RATIO_DEF,
       limA2Max  = FOLD_ANGLE_MAX_DEG * ARM_GEAR_RATIO_DEF;
// ══════════════════════════════════════════════════════════════
// PER-AXIS ENFORCEMENT, AND THE MASTER ENABLE
//
// Both switches now answer the same question — "is this boundary
// stopping the axis?" — at two different scopes:
//
//   limXxxEnforced — enforcement for THIS axis only. Its taught values
//                    are kept and still reported; nothing stops the axis
//                    at them while it is off. For commissioning one axis
//                    whose boundaries are not taught yet, or a
//                    maintenance move on one axis that has to go outside
//                    the working envelope on purpose.
//
//   limitsEnabled  — the same thing for EVERY axis at once. It is an AND,
//                    not an override: an axis is protected only when both
//                    are on, so the master switch can never quietly
//                    re-arm an axis the operator turned off.
//
// This REPLACES the old per-axis value LOCK (limXxxLocked,
// SET_LIMIT_LOCK). The lock froze the boundary's number while leaving it
// enforced, which is a real thing but not the thing an operator reads off
// a button labelled for a safety limit — the panel said LOCKED / UNLOCKED
// and answered a question nobody was asking, while "is this boundary
// actually on?" had no per-axis answer at all. Changing values is now
// guarded by APPLY alone, as on every other tab.
// ══════════════════════════════════════════════════════════════
bool limZEnforced = true, limRotEnforced = true;
bool limA1Enforced = true, limA2Enforced = true;

// Enforcement is ON by default: a fresh board must not be less safe than
// the last one somebody configured.
bool limitsEnabled = true;

bool *limEnforceFor(const String &axis) {
  if (axis == "Z")   return &limZEnforced;
  if (axis == "ROT") return &limRotEnforced;
  if (axis == "A1")  return &limA1Enforced;
  if (axis == "A2")  return &limA2Enforced;
  return NULL;
}

// "Has the operator switched this axis's boundary on?" — the flag alone,
// with no reference check. Target VALIDATION uses this: refusing a target
// is free and happens before anything moves, so it stays available even
// while the machine is unreferenced. Anything that stops a MOVING axis
// uses axisLimited() instead, which also demands a reference.
bool axisEnforced(const String &axis) {
  bool *on = limEnforceFor(axis);
  return on == NULL ? true : *on;
}

// The working band of one elbow, always low..high.
//
// The two taught limits are stored exactly as they were captured and may
// sit in either order (see applyLimit). Every consumer goes through here,
// so no consumer has to care which SET HERE the operator pressed first.
void armBand(int arm, double &lo, double &hi) {
  double a = (arm == 2) ? limA2Min : limA1Min;
  double b = (arm == 2) ? limA2Max : limA1Max;
  lo = (a < b) ? a : b;
  hi = (a < b) ? b : a;
}

// A limit pair must never invert or collapse: a MIN above its MAX would
// make every position illegal, and the axis could not be jogged out.
const double LIMIT_MIN_SPAN_DEG = 1.0;
const double LIMIT_MIN_SPAN_MM  = 1.0;

// A TAUGHT elbow boundary has NO envelope, and the pair is UNORDERED.
//
// No envelope: the angle this board reports for an elbow is scaled by
// armGearRatio, which is derived from the model rather than measured, so a
// position captured off the real machine legitimately reads far outside
// the CAD envelope. Any ceiling written here would be a guess, and a guess
// that rejects a position the arm is physically standing at defeats the
// whole point of teaching. The boundaries are MOTOR degrees, which is why
// re-calibrating the ratio does not disturb them.
//
// Unordered: SET_LIMIT_HERE takes whatever is on the screen, so the
// operator jogs to one stop, presses SET HERE, jogs to the other, presses
// SET HERE. Which of the two they reached first is not something they
// should have to keep straight, so the pair is sorted after each write
// rather than the second write being rejected for "inverting" it.
//
// ZM and ROT keep their real envelopes and their ordering, because for
// those two the scale IS known and a number outside it really is
// impossible.
const bool ARM_LIMITS_UNBOUNDED = true;

// Derived reach envelope — computed, never hand-typed, so it can't drift
// out of sync with the link lengths above.
double reachFromFoldAngle(double foldDeg) {
  // The +ARM_ZERO_CAD_DEG is the whole of the from-home -> CAD conversion.
  return ARM_RADIAL_OFFSET_MM
       - ARM_LINK_SUM_MM * cos((foldDeg + ARM_ZERO_CAD_DEG) * DEG_TO_RAD);
}
// Radial reach spanned by an elbow moving between two angles.
//
// NOT simply min/max of the two endpoints. R is a cosine of
// (th + ARM_ZERO_CAD_DEG), so it only rises monotonically across one
// half-period — and taught boundaries are no longer confined to that
// window (see ARM_LIMITS_UNBOUNDED). Over an interval that crosses an
// extreme, the extreme radius happens INSIDE the interval, not at either
// end.
//
// Getting this wrong is not cosmetic: with a band of -260..480 (the
// from-home spelling of the old -200..540) both endpoints land on the
// same part of the cosine, so the endpoint-only version reported a
// reachable band of 593.9..613.2 mm and refused every ordinary target.
//
// cos hits its extremes where (th + ARM_ZERO_CAD_DEG) is a multiple of
// 180, i.e. at th = 120, 300, -60, ... — checking those is both exact and
// cheap, since the widest legal band contains only a handful.
void reachBandFor(double thMin, double thMax, double &rMin, double &rMax) {
  double a = reachFromFoldAngle(thMin);
  double b = reachFromFoldAngle(thMax);
  rMin = (a < b) ? a : b;
  rMax = (a < b) ? b : a;

  double k = ceil((thMin + ARM_ZERO_CAD_DEG) / 180.0);
  for (double th = k * 180.0 - ARM_ZERO_CAD_DEG;
       th <= thMax + 1e-9; th += 180.0) {
    double r = reachFromFoldAngle(th);
    if (r < rMin) rMin = r;
    if (r > rMax) rMax = r;
  }
}

double foldAngleFromReach(double rMM) {
  double c = (rMM - ARM_RADIAL_OFFSET_MM) / ARM_LINK_SUM_MM;
  if (c >  1.0) c =  1.0;
  if (c < -1.0) c = -1.0;
  return 180.0 - (acos(c) * RAD_TO_DEG) - ARM_ZERO_CAD_DEG;
}

// ══════════════════════════════════════════════════════════════
// MOTOR CALIBRATION — PLACEHOLDERS EXCEPT THE RM GEAR RATIO.
// Measure these on your real drivetrain before trusting any distance
// or angle this board reports.
// ══════════════════════════════════════════════════════════════
const double MOTOR_STEPS_PER_REV  = 200.0;   // 1.8 deg/step
const double MICROSTEPS_PER_STEP  = 16.0;    // driver microstep setting
const double PULSES_PER_MOTOR_REV = MOTOR_STEPS_PER_REV * MICROSTEPS_PER_STEP;

// RM gear ratio: modelled default 28.4375 (1/4.375 then 1/6.5, Simscape).
// Runtime-calibratable via SET_ROT_RATIO -- confirm on the bench if RM
// turns more or less than commanded.
const double ROT_GEAR_RATIO_DEF = I_RM_TOTAL;
const double ROT_GEAR_RATIO_MIN = 0.01, ROT_GEAR_RATIO_MAX = 1000.0;
double rotGearRatio = ROT_GEAR_RATIO_DEF;
double pulsesPerDegRot() { return (PULSES_PER_MOTOR_REV * rotGearRatio) / 360.0; }

// AM1/AM2 elbow gearing — *** STILL A PLACEHOLDER. MEASURE IT. ***
//
// The Simscape diagram's +/-1 and -2 gains on AM1/AM2 are the frog-leg
// LINKAGE relationship (shoulder theta, elbow -2 theta), not a gearbox:
// that model takes a joint angle as its input and contains no motor at
// all. It says nothing about the motor-to-joint reduction, and reading
// 1:1 out of it was a mistake.
//
// The machine says otherwise. Commanding "100 deg/s" felt slow, which at
// 1:1 would be 17 motor RPM — so there is a substantial reduction here.
//
// This constant NO LONGER AFFECTS SPEED (the gear ratio cancels on the
// way to the step generator — see ARM_RPM_SCALE below). What it still
// controls is every ANGLE this board reports and every absolute elbow
// position it drives to. Until it is measured, MOVE_A1/MOVE_A2, the
// reported th3, the reach figures and the IK targets are all wrong by
// exactly this factor. Jog is unaffected.
//
// To measure: mark the elbow, command a known number of motor revolutions
// and divide by the joint angle actually swept.
// ══════════════════════════════════════════════════════════════
// THE ELBOW: MOTOR DEGREES vs FROG-LEG DEGREES
//
// These are two different numbers and this firmware used to conflate
// them. Everything the board MEASURES is motor shaft rotation — it counts
// step pulses and nothing else. The frog-leg link angle is DERIVED from
// it, and the derivation needs a ratio.
//
// WHERE THE RATIO COMES FROM (MATLAB_v4_final: mophongv2.slx + the .m)
// -------------------------------------------------------------------
// The Simscape root diagram drives each arm's TWO revolute joints from
// the single AM1/AM2 signal:
//
//     AM1 --x(-1)--> Revolute3   [banxoay : canhtay1]   the SHOULDER
//         --x(-2)--> Revolute    [canhtay1 : canhtay2]  the KNEE
//
// and mophong_init.m's forward kinematics says the same thing in closed
// form — the upper link sits at (th2 + th3_math) and the lower at
// (th2 - th3_math), symmetric about the radial line, so the knee turns
// through twice the angle the driven link does:
//
//     P2 = P1 + a4 * [cos(th2 + th3_math), sin(th2 + th3_math), d4]
//     P3 = P2 + a5 * [cos(th2 - th3_math), sin(th2 - th3_math), d5]
//
// The elbow motor is coupled to the knee, so ONE FROG-LEG DEGREE COSTS
// TWO MOTOR DEGREES:
//
//     fold_deg  = motor_deg / armGearRatio
//     motor_deg = fold_deg  * armGearRatio
//
// >>> CONFIRM THIS ON THE BENCH. <<<
// It is derived from the model, not measured off the machine. Mark the
// elbow, command a known number of motor revolutions, and divide by the
// frog-leg angle actually swept. If it is not 2, send
// SET_ARM_RATIO:<value> — no re-flash, and every reported angle, reach
// figure and IK target follows immediately.
//
// RM is the counter-example that shows this is the right shape: its ratio
// has been in the model all along as 1/4.375 then 1/6.5, and
// pulsesPerDegRot() has always used it (also SET_ROT_RATIO-adjustable now).
// ══════════════════════════════════════════════════════════════
// Pulses per MOTOR degree. Exact and ratio-free — only steps/rev and the
// microstep setting, both known. This is the quantity the board can
// actually count, so it is what the maths is built on.
const double PULSES_PER_DEG_ARM_MOTOR = PULSES_PER_MOTOR_REV / 360.0;

// ZM ballscrew: 20 mm of carriage travel per motor revolution.
// ══════════════════════════════════════════════════════════════
// ZM LEAD — MEASURE THIS. It is the one number that decides whether a
// commanded millimetre is a real millimetre.
//
//     pulses = mm * PULSES_PER_MOTOR_REV / zMmPerRev
//
// 20 mm/rev was assumed, never measured. If the carriage travels FURTHER
// than commanded, the real lead is LARGER than the figure here, in exact
// proportion: a 10 mm command that moves 30 mm means the true lead is
// 3 * 20 = 60 mm/rev.
//
// HOW TO MEASURE: put a rule or a dial indicator on the carriage, send
// MOVE_Z_TEST:100 (or jog a known number of motor revolutions), and divide
// the distance actually travelled by the number of revolutions commanded.
// Then SET_Z_LEAD:<mm> — no re-flash.
//
// A wrong lead is not just a scaling error. Every Z soft limit is in
// millimetres, so it also moves where those boundaries physically are, and
// the from-HOME Z the operator types stops meaning what the panel says.
// ══════════════════════════════════════════════════════════════
const double Z_MM_PER_REV_DEF = 20.0;
const double Z_MM_PER_REV_MIN = 0.1, Z_MM_PER_REV_MAX = 500.0;
double zMmPerRev = Z_MM_PER_REV_DEF;

// Derived, so it can never be stale relative to zMmPerRev.
double pulsesPerMmZ() { return PULSES_PER_MOTOR_REV / zMmPerRev; }

// Kept as the DEFAULT only. Nothing should compute with it — use
// pulsesPerMmZ() — but the name is referenced in messages and tests.
const double Z_MM_PER_MOTOR_REV = Z_MM_PER_REV_DEF;

const bool INVERT_Z    = false;
const bool INVERT_ROT  = false;
const bool INVERT_ARM1 = false;
const bool INVERT_ARM2 = false;

// ══════════════════════════════════════════════════════════════
// LIMIT SENSORS — opt-in, see the header note. Enable ONLY after
// wiring real sensors and confirming pin numbers and active state.
// ══════════════════════════════════════════════════════════════
#define ENABLE_ROT_Z_LIMIT_SENSORS 0

#if ENABLE_ROT_Z_LIMIT_SENSORS
  #define ROT_LIMIT_CW_PIN   IO1
  #define ROT_LIMIT_CCW_PIN  IO2
  #define Z_LIMIT_UP_PIN     IO3
  #define Z_LIMIT_DOWN_PIN   IO4
  const int LIMIT_ACTIVE_STATE = HIGH;
#endif

// ══════════════════════════════════════════════════════════════
// MOTION PROFILE — ONE UNIVERSAL RPM, ONE PERCENTAGE PER MOTOR
//
// v8 set a single VelMax in raw motor RPM and applied it identically to
// all four motors. That could not work, because the axes are geared
// completely differently:
//
//     300 RPM  ->  elbow 1800 deg/s  (its whole 120 deg travel in 0.07 s!)
//                  turntable 63 deg/s   (reasonable)
//      10 RPM  ->  elbow 60 deg/s    (reasonable)
//                  turntable 2.1 deg/s  (unusably slow)
//
// Early v9 fixed that by abandoning RPM entirely and giving each axis its
// own number in its own unit. Correct, but it means six fields to keep in
// step and no single "go slower" knob.
//
// v9.1 keeps the one knob AND the correctness: a universal motor RPM,
// a percentage per motor, and a per-axis CALIBRATION scale that absorbs
// the gearing difference so the percentages are comparable:
//
//     axisMotorRpm = masterRpm * (pct/100) * AXIS_RPM_SCALE
//
// The scales are chosen so 100% on RM and ZM reproduces the MATLAB
// trajectory (mophong_init.m, 12 s home -> A -> B -> home):
//     ZM  peak 34.5 mm/s   (mean 22.6)
//     RM  mean 23.3 deg/s  (peak 85.8, inflated by the atan2 wrap
//                           where the path passes near the centre)
//
// DO NOT "simplify" ROT_RPM_SCALE or Z_RPM_SCALE to 1.0. They are what
// carry the 28.4375:1 reduction and the 20 mm/rev lead into the
// percentage; without them one master RPM would mean wildly different
// real speeds on those two axes.
//
// Percentages have NO upper bound. See AXIS_PCT_MIN below for why that
// is safe rather than merely permissive.
// ══════════════════════════════════════════════════════════════
// Motor RPM that corresponds to 100% on each axis, at MASTER_RPM_NOMINAL.
//   RM: 29.53 deg/s * 28.4375 / 6  = 140 RPM
//   ZM: 35.0 mm/s  * 60 / 20       = 105 RPM
//   AM: runs at the master RPM directly — see below
const float MASTER_RPM_NOMINAL = 140.0f;
const float ROT_RPM_SCALE = 140.0f   / MASTER_RPM_NOMINAL;   // 1.000
const float Z_RPM_SCALE   = 105.0f   / MASTER_RPM_NOMINAL;   // 0.750

// AM's scale was 0.0298, derived from "25 deg/s = 4.17 motor RPM" — which
// only holds if ARM_GEAR_RATIO really is 1.0. On the machine, 100 deg/s
// was still visibly slow, which is proof that it is not: the elbow has a
// real reduction, so the motor was being asked for ~17 RPM while every
// other axis ran at 100+.
//
// The scale is therefore 1.0: the arm's percentage maps STRAIGHT to the
// master motor RPM, exactly like RM's does.
//
// Worth understanding, because it is what makes this safe to change: the
// gear ratio CANCELS on the way to the step generator.
//     armVelDegS   = rpm * 360 / (60 * G)
//     armVelPulses = armVelDegS * PULSES_PER_MOTOR_REV * G / 360
//                  = rpm * PULSES_PER_MOTOR_REV / 60
// So the pulse rate the motor actually receives depends only on the RPM
// asked for, whatever G is. ARM_GEAR_RATIO no longer has any influence on
// how fast the arm moves — it only affects the ANGLES this board reports
// and the absolute positions it drives to. Those are still wrong until it
// is measured. See the ARM_GEAR_RATIO note above.
const float ARM_RPM_SCALE = 1.0f;

// ---- Defaults ----
// RM and ZM sit at the percentages that reproduce the MATLAB trajectory
// through their real gear ratios (i_RM_total = 28.4375, 20 mm/rev):
//     RM 100% of 140 RPM -> 29.5 deg/s   — exactly mophong_init.m
//     ZM  90% of 140 RPM -> 31.5 mm/s    — just under it
// The arm sits far above them because its drivetrain is the one that
// needs it: extend/retract is the stroke that gates cycle time, and the
// elbow reduction means a high motor RPM is a moderate joint speed.
// The host treats the master as a FIXED reference and only changes the
// percentages, so these defaults are the ones the GUI ships with.
//     RM  75% of 150 RPM -> 112.5 motor RPM -> 23.74 deg/s
//     AM 125% of 150 RPM -> 187.5 motor RPM
//     ZM  50% of 150 RPM ->  56.25 motor RPM -> 18.75 mm/s
const float MASTER_RPM_DEF     = 150.0f;   // universal speed, motor RPM
const float MASTER_ACC_DEF     = 375.0f;   // universal accel, RPM/s (~0.4 s ramp)
const float ARM_PCT_DEF        = 125.0f;
const float ROT_PCT_DEF        = 75.0f;
const float Z_PCT_DEF          = 50.0f;

float masterRpm     = MASTER_RPM_DEF;
float masterAccRpmS = MASTER_ACC_DEF;
float rotPct        = ROT_PCT_DEF;
float armPct        = ARM_PCT_DEF;
float zPct          = Z_PCT_DEF;

// Acceleration percentages, independent of the speed ones above -- until
// now accel silently reused rotPct/armPct/zPct. Default to the SAME values
// as the speed defaults, so a fresh board's real acceleration is unchanged.
float rotAccPct     = ROT_PCT_DEF;
float armAccPct     = ARM_PCT_DEF;
float zAccPct       = Z_PCT_DEF;

const float MASTER_RPM_MIN = 1.0f,   MASTER_RPM_MAX = 400.0f;
const float MASTER_ACC_MIN = 1.0f,   MASTER_ACC_MAX = 2000.0f;

// NO upper bound on a percentage.
//
// Safe for a specific reason, not by luck: a percentage is a multiplier,
// not a speed, and every axis it feeds still has a real backstop
// underneath it — ROT_VEL_MAX and Z_VEL_MAX for the two axes whose
// gearing is known, ARM_RPM_MAX for the one whose gearing is not.
// Capping the percentage as well was belt-and-braces that mostly got in
// the way: 250% is a perfectly reasonable thing to ask of the arm.
//
// The lower bound stays. 0% silently freezes an axis and a negative
// percentage would invert its direction; neither is a speed setting.
const float AXIS_PCT_MIN   = 1.0f;
const float AXIS_PCT_MAX   = 1.0e6f;   // effectively none; the backstops rule

// Derived engineering speeds — recomputed by applyMotionParams(), never
// written by hand. Kept as variables so every report reads the same
// numbers the motors were actually given.
float rotVelDegS = 0, rotAccDegS2 = 0;
float armVelDegS = 0, armAccDegS2 = 0;
float zVelMmS    = 0, zAccMmS2    = 0;

// HARD engineering ceilings, ~4x the nominal trajectory. These are the
// backstop: whatever masterRpm and the percentages work out to, no axis
// is ever handed more than this, and the clamp is reported rather than
// applied silently.
//
// RM and ZM keep theirs, because for those two the gearing is KNOWN
// (28.4375:1 and 20 mm/rev), so a deg/s or mm/s ceiling is a statement
// about something real.
//
// THE ARM NO LONGER HAS ONE. Its old 100 deg/s ceiling was computed from
// ARM_GEAR_RATIO = 1.0, a placeholder that turned out to be wrong, so the
// ceiling was not protecting the arm from anything — it was throttling it
// to roughly 17 motor RPM while claiming that was 100 deg/s. A limit
// derived from an unmeasured constant is worse than no limit: it is a
// number that looks like a safety margin and isn't one.
//
// What replaces it is a bound in the unit that IS known and does mean
// something on this axis: motor RPM. Past a few hundred RPM an open-loop
// step/dir stepper stops following and starts skipping steps, silently,
// with no encoder to notice — which on this board corrupts the position
// reference. That, not an invented deg/s figure, is the real hazard.
const float ROT_VEL_MAX = 120.0f, ROT_ACC_MAX = 400.0f;
const float Z_VEL_MAX   = 140.0f, Z_ACC_MAX   = 400.0f;
const float ARM_RPM_MAX = 400.0f, ARM_ACC_RPM_MAX = 2000.0f;
const float MOTION_MIN  = 0.05f;

// Set while applyMotionParams() is clamping, so the report can say so.
bool speedClampedRot = false, speedClampedArm = false, speedClampedZ = false;

// What the arm motors were actually asked for. Reported instead of a
// deg/s figure, because until ARM_GEAR_RATIO is measured this is the only
// arm speed number on the board that is true.
float armMotorRpmActual = 0.0f;

// ---- PID — ONE PRESET, from the Stepper MATLAB report, Table 2 ----
// Plant identified in that report (ClearCore -> TB6600 -> stepper -> 1:50):
//      G(s) = 12.5 / (s * (s + 12.5))     [rad per STEP pulse]
// Pole placement for POT < 5% (zeta = 0.7071), ts in [0.4, 0.8] s gives
// the PID row, which is the only one this controller uses:
//      Kp = 24.97   Ki = 120.00   Kd = 1.33   N = 50     ts 0.57 s
//
// N is the derivative filter coefficient (report: 50..100, to reject the
// encoder's quantisation noise without altering the designed dynamics).
//
// v9.1 REMOVED the P / PI / PD alternatives and the PARALLEL / I-PD /
// PREFILTER form selector. Only the PID row was ever used, and the form
// field configured a controller structure that does not exist on this
// board — it was a menu that changed nothing but could still be got
// wrong. PID_OFF replaces it: one switch that says whether these gains
// are in play at all.
//
// *** THIS BOARD DOES NOT CLOSE THE LOOP ***  The drive is open-loop
// step/dir; the encoder is a monitoring device only, exactly as the
// report states. These values are stored and echoed so the controller
// configuration matches the documented Simulink model — they are not
// running a PID here.
const float PID_PRESET_KP = 24.97f, PID_PRESET_KI = 120.00f;
const float PID_PRESET_KD = 1.33f,  PID_PRESET_N  = 50.0f;

float currentKp = PID_PRESET_KP, currentKi = PID_PRESET_KI;
float currentKd = PID_PRESET_KD, currentN  = PID_PRESET_N;
bool  pidEnabled = true;

String pidSummary() {
  if (!pidEnabled) return "DISABLED (gains held but not in use)";
  return "kp=" + String(currentKp, 3) + " ki=" + String(currentKi, 3)
       + " kd=" + String(currentKd, 3) + " N=" + String(currentN, 1);
}

int32_t rotVelPulses = 0, rotAccelPulses = 0;
int32_t armVelPulses = 0, armAccelPulses = 0;
int32_t zVelPulses   = 0, zAccelPulses   = 0;

const float ESTOP_DECEL_MULTIPLIER = 3.0;

float boostMultiplier = 1.0;
const float BOOST_MAX = 3.0;

// ══════════════════════════════════════════════════════════════
// PLC LINK — MELSEC MC PROTOCOL 3E, ASCII FRAMES, TCP 192.168.3.101:1025
//
// The PLC, not this board, owns the reference position and the physical
// limit switches. ClearCore is a READ-ONLY MC-protocol client: it
// batch-reads one word of bit devices and writes nothing at all. The one
// thing it asks the PLC to do — HOME — goes out on a wire from IO-0 into
// X0, not over this socket.
//
// WHY MC PROTOCOL AND NOT A CUSTOM TEXT PROTOCOL
// ----------------------------------------------
// v9.0 sent the line "M2\n" and waited for the substring "DONE". That
// only works if somebody writes a SOCOPEN/RECV/SEND ladder on the PLC to
// parse it. Port 1025 with a device comment list is the stock MELSEC
// Ethernet configuration, which already answers device read/write
// requests with no ladder code at all — so the handshake is bits in the
// PLC's own device memory, visible in GX Works while it runs. Nothing to
// write on the PLC side except the home sequence itself.
//
// ASCII rather than binary frames on purpose: an ASCII frame is
// legible in a packet capture and in this board's own log, so a wrong
// device number is a five-second diagnosis instead of a hex dump. It
// costs twice the bytes of binary, which at ~30 bytes per poll is
// irrelevant. Set PLC_MC_ASCII to 0 only if the PLC's Ethernet module is
// already fixed to binary; the frame builders below would then need
// rewriting, so this is deliberately NOT a switch that half-works.
//
// >>> THE PLC's Ethernet module must be configured for MC Protocol on
// >>> port 1025, ASCII, TCP. That is the screen the IP/port came from.
// ══════════════════════════════════════════════════════════════
#define PLC_LINK_PLACEHOLDER 0    // no PLC wired — simulate, never sets isHomed
#define PLC_LINK_ETHERNET    1    // MC protocol 3E over TCP  <-- the real machine
#define PLC_LINK_DIGITAL_IO  2    // request on an output pin, done on an input pin

#define PLC_LINK_MODE PLC_LINK_ETHERNET

// ---- PLC network endpoint (from the PLC configuration screen) ----
#define PLC_IP_0 192
#define PLC_IP_1 168
#define PLC_IP_2 3
#define PLC_IP_3 101
const uint16_t PLC_PORT = 1025;

// ClearCore's own address. MUST be on the PLC's 192.168.3.x subnet and
// must not collide with the PLC (.101) or anything else on the network.
#define CC_IP_0 192
#define CC_IP_1 168
#define CC_IP_2 3
#define CC_IP_3 200

// ══════════════════════════════════════════════════════════════
// PLC DEVICE MAP — transcribed from the GX Works comment list.
//
// Only the devices this board actually touches are wired up; the rest
// are recorded so the next person does not have to go and find the
// screenshot again. READ means ClearCore polls it, WRITE means ClearCore
// sets it.
//
//   X0   HOME request     WIRE   driven by ClearCore's IO-0 terminal, NOT
//                                over Ethernet — see the block below
//   M0   RUN                     (PLC-side, not used here)
//   M1   DONE             READ   homing finished, machine is on reference
//   M2   rHOME                   (PLC-side)
//   M3   STOP                    (PLC-side)
//   M4   rJOG                    (PLC-side)
//   M5   MinZ             READ   ZM  physical boundary reached
//   M6   OutR             READ   RM  physical boundary reached
//   M7   OutR1            READ   A1M physical boundary reached
//   M8   OutR2            READ   A2M physical boundary reached
//   M10  Run ZM           READ   PLC is driving ZM
//   M11  Run RM           READ   PLC is driving RM
//   M12  Run A1M          READ   PLC is driving A1M
//   M13  Run A2M          READ   PLC is driving A2M
//   M15/16/17  rLY/rLR/rLG      (lamps, PLC-side)
//   M20  AUTO / M21 HOME / M22 fLED / M23 sHOME   (PLC-side)
//   M25/26/27  ILY/ILR/ILG      (lamps, PLC-side)
//
// M1 through M13 all live inside the single word M0..M15, so ONE batch
// read per poll fetches the whole handshake. Three separate reads would
// be three round trips for the same information.
// ══════════════════════════════════════════════════════════════
const int PLC_M_DONE     = 1;    // M1
// M5..M8 are the per-axis optical HOME sensors. They say "this axis has
// reached its reference", nothing more. They are NOT limit switches and
// must never be used to set a working boundary — see the block below.
const int PLC_M_HOME_Z   = 5;    // M5  MinZ
const int PLC_M_HOME_ROT = 6;    // M6  OutR
const int PLC_M_HOME_A1  = 7;    // M7  OutR1
const int PLC_M_HOME_A2  = 8;    // M8  OutR2
const int PLC_M_RUN_Z    = 10;   // M10 Run ZM
const int PLC_M_RUN_ROT  = 11;   // M11 Run RM
const int PLC_M_RUN_A1   = 12;   // M12 Run A1M
const int PLC_M_RUN_A2   = 13;   // M13 Run A2M

// The word this board polls: M0, one word = M0..M15.
#define PLC_POLL_DEVICE_CODE  "M*"
const long     PLC_POLL_DEVICE_NUM = 0;
const uint16_t PLC_POLL_WORDS      = 1;

// ---- The HOME request: A WIRE, not a packet ----
//
// ClearCore's IO-0 terminal is wired directly to the PLC's X0 input, and
// that is the whole HOME request. **Ethernet is READ-ONLY.** Nothing in
// this sketch writes a PLC device any more; the socket exists only to
// batch-read M0..M15.
//
// This is the right way round, and it fixes a real defect rather than
// being a matter of taste. X devices are refreshed from the physical
// input terminals at the top of every PLC scan, so anything MC protocol
// wrote into X0 was overwritten within one scan (< 10 ms) the moment X0
// had a wire on it. The old code worked ONLY while X0 was unwired, and
// the failure — HOME never starting, M10..M13 never coming on — looked
// like a ClearCore fault. A physical output into a physical input is
// exactly what the PLC's scan expects, so there is nothing left to race.
//
// It is also faster and fails better: no socket, no round trip, no
// timeout to sit through, and the request survives an Ethernet dropout
// that would have stranded a written bit ON at the PLC.
//
// The PLC ladder needs no SOCOPEN/RECV and no change at all — X0 is the
// same HOME input it always was, in parallel with the panel pushbutton.
//
// IO-0 is a general-purpose digital I/O on ClearCore and is used here as
// an OUTPUT. Do not also configure it as an input elsewhere.
#define PLC_HOME_REQ_PIN_NAME  "IO-0"

// Level-style handshake: hold the request asserted until DONE comes back.
const bool          PLC_HOME_ACTIVE_HIGH = true;
const unsigned long PLC_HOME_TIMEOUT_MS  = 30000;   // then fail, do not hang

// ---- Poll cadence: slow when idle, fast while homing ----
//
// TWO rates, and the second one is not an optimisation — it is what keeps
// the HOME handshake correct.
//
// Idle, there is nothing urgent in the status word: the home sensors and
// the run bits are only interesting during a home, so 5 s is plenty and
// leaves the Mitsubishi's Ethernet module almost entirely alone.
//
// During a home it MUST be fast. plcHomeDoneAsserted() completes a cycle
// only after it has SEEN M10..M13 come on and then go off again — that
// gate is what stops a stale latched M1 from ending the next home before
// the machine has moved. At 5 s, a home sequence shorter than one poll
// interval finishes entirely between two polls: the run bits are never
// observed, plcSawRunDuringHome stays false, and a home that physically
// succeeded fails on the 30 s timeout. Intermittently, depending on where
// the poll happened to land, which is the worst kind of fault to chase.
//
// 200 ms is short enough to catch any real PLC home sequence, and it only
// applies for the few seconds a home is actually running.
// 5 s idle was chosen deliberately — it leaves the Mitsubishi alone when
// nothing in the status word is urgent. It is NOT the reason a sensor reads
// stale: a working link at 5 s shows a covered sensor within 5 s, whereas a
// dead link never shows it at all.
//
// It is now the DEFAULT rather than a constant, because 5 s is slow when you
// are standing at the machine waving a hand over a sensor to test it.
// SET_PLC_POLL:<ms> speeds it up with no re-flash; the value is not
// persisted, so a power cycle returns to the deliberate default.
const unsigned long PLC_POLL_IDLE_DEF_MS = 5000;
unsigned long plcPollIdleMs = PLC_POLL_IDLE_DEF_MS;
const unsigned long PLC_POLL_IDLE_MS   = PLC_POLL_IDLE_DEF_MS;  // the default
const unsigned long PLC_POLL_HOMING_MS = 200;
// Kept as the name the rest of the file and the tests use.
const unsigned long PLC_POLL_MS        = PLC_POLL_IDLE_MS;
// The socket timeout must be LONGER than the CPU monitoring timer below,
// or this board abandons a request the PLC is still going to answer. The
// late reply would then arrive while the next request is outstanding and
// be read as ITS answer — a boundary bit pattern applied one cycle late,
// which is the kind of fault that shows up once a week and never
// reproduces. Monitoring timer 500 ms, socket 800 ms.
const unsigned long PLC_TXN_TIMEOUT_MS     = 800;
const unsigned long PLC_CONNECT_TIMEOUT_MS = 2000;
// Reconnect attempts are rate-limited because EthernetClient::connect()
// BLOCKS. Trying on every loop pass would stall motion servicing for
// seconds at a time whenever the cable is out.
const unsigned long PLC_RECONNECT_MS = 3000;

// MC protocol 3E header fields. "03FF" + station 00 addresses the CPU the
// Ethernet module is mounted on, which is the normal single-CPU case.
#define PLC_MC_SUBHEADER_REQ  "5000"
#define PLC_MC_SUBHEADER_RES  "D000"
#define PLC_MC_NETWORK        "00"
#define PLC_MC_PC             "FF"
#define PLC_MC_DEST_IO        "03FF"
#define PLC_MC_DEST_STATION   "00"
// CPU monitoring timer, in units of 250 ms. 0002h = 2 = 500 ms.
// NOT 0000, which means "wait forever": a busy CPU would then hold the
// reply past this board's own socket timeout every time.
#define PLC_MC_MONITOR_TIMER  "0002"
#define PLC_MC_CMD_READ       "0401"
#define PLC_MC_SUB_WORD       "0000"
// No CMD_WRITE and no SUB_BIT: the link is read-only and there is nothing
// left that could assemble a write frame. See plcFrameReadWords() below.
// Everything before the response data length field: subheader(4) +
// network(2) + pc(2) + io(4) + station(2).
const int PLC_MC_RES_HEADER_CHARS = 14;

// ══════════════════════════════════════════════════════════════
// M5..M8 ARE HOME SENSORS, NOT LIMIT SWITCHES
//
// They are read to know when each axis has reached its reference, and for
// nothing else. In particular they do NOT:
//   - stop a jog or a run
//   - register the position they trip at as a working boundary
//
// An earlier revision did both. It is wrong twice over: a home sensor sits
// AT the reference, so stopping on it would make it impossible to jog off
// home, and writing its trip point into a limit would overwrite a taught
// boundary with a position that is not a boundary at all.
//
// Working boundaries come from the operator only — typed, or taught with
// SET HERE. Physical protection is the PLC's own ladder.
//
// WHAT THEY DO DO
// ---------------
// All four are WIRED and working, and they sit at OPPOSITE ENDS:
//
//   M5 MinZ   ZM  at the BOTTOM of the stroke   -> the DOWN end
//   M6 OutR   RM  at the CCW stop                -> the 0 deg end
//   M7 OutR1  A1M fully EXTENDED                 -> the far end
//   M8 OutR2  A2M fully EXTENDED                 -> the far end
//
// So M5/M6 mark the minimum of their axis and M7/M8 the maximum of theirs.
// HOME STATE is therefore M5 AND M6 covered while M7 and M8 are CLEAR: at
// home the lift is down, the turntable is at 0, and both arms are pulled
// IN, which is the opposite end from where M7/M8 sit.
//
// WHERE THEY ARE ENFORCED — P2P YES, JOG NO
// -----------------------------------------
// A covered sensor REFUSES a point-to-point leg that would drive that axis
// further into it. A program runs unattended and the operator is not
// watching the axis, so the move must not start.
//
// In JOG the same condition only WARNS. Jog is a dead-man control: it moves
// only while the button or key is held, the operator is looking at the
// machine, and jogging is how you get OFF a tripped sensor in the first
// place. Blocking it there also risks pinning the machine on its own switch.
// Physical protection while jogging is the PLC's own ladder.
// ══════════════════════════════════════════════════════════════
// Which direction each sensor sits at. -1 = the axis minimum, +1 = maximum.
const int PLC_SENSOR_END_Z   = -1;   // M5 MinZ  -> the DOWN end
const int PLC_SENSOR_END_ROT = -1;   // M6 OutR  -> the CCW end (0 deg)
const int PLC_SENSOR_END_A1  = +1;   // M7 OutR1 -> arm 1 fully extended
const int PLC_SENSOR_END_A2  = +1;   // M8 OutR2 -> arm 2 fully extended

// The HOME request line is the same terminal in EVERY link mode: it is a
// wire to the PLC's X0 input, and the Ethernet socket has nothing to do
// with it. Defining it once, outside the #if, is what stops the two modes
// drifting on to different terminals — they did, IO5 against a comment
// that said IO-0, and only one of them was on the machine.
#define PLC_HOME_REQ_PIN  IO0     // output: HOME request -> PLC X0

#if PLC_LINK_MODE == PLC_LINK_DIGITAL_IO
  // DONE comes back on a wire too in this mode. Under Ethernet it is read
  // from M1 instead, so this pin is unused there.
  #define PLC_HOME_DONE_PIN DI6     // input:  DONE back from PLC
#endif

#if PLC_LINK_MODE == PLC_LINK_ETHERNET
  #include <Ethernet.h>
  // ClearCore's Arduino wrapper provides the standard Arduino Ethernet
  // API (EthernetClient / Ethernet.begin), so this is written in that
  // style rather than the bare-metal EthernetMgr / EthernetTcpClient one.
  byte          plcMac[]  = {0x24, 0x15, 0x10, 0xB0, 0x00, 0x01};  // any unique MAC
  IPAddress     plcLocalIp(CC_IP_0, CC_IP_1, CC_IP_2, CC_IP_3);
  IPAddress     plcTargetIp(PLC_IP_0, PLC_IP_1, PLC_IP_2, PLC_IP_3);
  EthernetClient plcClient;
  bool          plcReportedError = false;
  unsigned long plcLastConnectTry = 0;
  unsigned long plcLastConnectLog = 0;
#endif

#if PLC_LINK_MODE == PLC_LINK_PLACEHOLDER
  // How long the stub waits before pretending the PLC answered. Set to 0
  // to make HOME always time out, which is the honest behaviour when no
  // PLC exists. It is deliberately NOT allowed to set isHomed.
  const unsigned long PLC_SIM_DONE_MS = 1200;
#endif

const unsigned long ALIVE_INTERVAL_MS       = 2000;

// ---- Jog dead-man watchdog ----
// The host holds a jog by sending one start command, so if that host
// crashes, unplugs, or simply fails to send the stop, the axis would run
// until ESTOP. (Exactly that happened: a GUI bug swallowed the release
// event and the arm kept moving.) The host must now refresh the jog with
// JOG_HB; if nothing arrives within the timeout the board stops itself.
// Set to 0 to disable when driving the board by hand from a terminal.
#define ENABLE_JOG_WATCHDOG 1
const unsigned long JOG_WATCHDOG_MS = 700;
const unsigned long RUN_REPORT_INTERVAL_MS  = 150;
const unsigned long JOG_REPORT_INTERVAL_MS  = 50;
const unsigned long HOME_REPORT_INTERVAL_MS = 100;
const unsigned long LED_FLASH_RX_MS = 12;
const unsigned long LED_FLASH_TX_MS = 55;

// ================= STATE =================
bool isConnected = false;

bool  hasLoadedProgram = false;
bool  loadedProgramIsDual = false;
float loadedD1A = 0, loadedRotA = 0, loadedA1A = 0, loadedA2A = 0;
float loadedD1B = 0, loadedRotB = 0, loadedA1B = 0, loadedA2B = 0;
float loadedDualD1 = 0, loadedDualRot = 0, loadedDualA1 = 0, loadedDualA2 = 0;

bool isMoving = false;
RunPhase runPhase = PHASE_NONE;   // enum declared in the TYPES block above
float runStartD1 = 0, runStartRot = 0, runStartA1 = 0, runStartA2 = 0;
float runTargetD1 = 0, runTargetRot = 0, runTargetA1 = 0, runTargetA2 = 0;
unsigned long lastRunReportTime = 0;

// AM1 and AM2 are separate motors on separate frog-leg linkages, so each
// elbow has its own jog direction. v8 kept a single armDir and drove both
// motors from it, which made independent arm control impossible and — worse
// — made the soft limit check stop BOTH arms based on AM1's position alone.
int rotDir = 0, a1Dir = 0, a2Dir = 0, jzDir = 0;
unsigned long lastJogReportTime = 0;

bool isHoming = false;
unsigned long lastHomeReportTime = 0;
unsigned long homeRequestedAt = 0;
HomeState homeState = HOME_IDLE;
bool isHomed = false;            // set only by a real PLC done signal
unsigned long lastAliveTime = 0;

// Jog dead-man watchdog. Refreshed by any jog command or JOG_HB.
unsigned long lastJogKeepAlive = 0;

bool          ledOn    = false;
unsigned long ledOffAt = 0;


// ══════════════════════════════════════════════════════════════
// LED (non-blocking — never call delay() here)
// ══════════════════════════════════════════════════════════════
void ledPulse(unsigned long durationMs) {
  digitalWrite(LED_PIN, HIGH);
  ledOn = true;
  ledOffAt = millis() + durationMs;
}

void serviceLed() {
  if (ledOn && (long)(millis() - ledOffAt) >= 0) {
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
  }
}

void sendFeedback(const String &line) {
  Serial.println(line);
  ledPulse(LED_FLASH_TX_MS);
}


// ══════════════════════════════════════════════════════════════
// INVERSE / FORWARD KINEMATICS
//
// Direct port of solve_ik_frogleg() from mophong_init.m. The MATLAB
// version silently CLAMPS an out-of-range target; this version reports
// the violation instead, because a machine must never quietly move
// somewhere other than where it was told.
// ══════════════════════════════════════════════════════════════
// (struct IkResult is declared near the top of this file — see the
//  "TYPES" block there for why it cannot live down here.)

double zOffsetForArm(int arm) {
  return (arm == 2) ? Z_OFFSET_ARM2_MM : Z_OFFSET_ARM1_MM;
}

IkResult solveIkFrogleg(int arm, double X, double Y, double Z) {
  IkResult r;
  r.ok = false; r.d1 = 0; r.th2 = 0; r.th3 = FOLD_ANGLE_HOME_DEG; r.R = 0;

  if (arm != 1 && arm != 2) {
    r.error = "[ERROR] arm must be 1 (A1M) or 2 (A2M), got " + String(arm);
    return r;
  }

  // --- d1 = Z - Z_offset ---
  // Checked against the OPERATOR's limits, not the factory envelope: a
  // target the operator has excluded must be refused here, at the point
  // it is commanded, rather than half-executed and then stopped by the
  // jog limit part-way through the move.
  double d1 = Z - zOffsetForArm(arm);
  if (axisEnforced("Z") && (d1 < limD1Min - 1e-6 || d1 > limD1Max + 1e-6)) {
    // Quoted in the frame the operator commanded in: d1 IS Z-from-HOME.
    // The old message led with the absolute figure, which nobody types.
    r.error = "[ERROR] Z=" + String(d1, 2) + " from HOME is out of ZM travel "
              "(allowed " + String(limD1Min, 1) + ".." + String(limD1Max, 1)
            + " mm above HOME; Z is never negative). That would put arm "
            + String(arm) + "'s deck at an absolute " + String(Z, 2) + " mm";
    return r;
  }

  // --- th2 = atan2(Y, X) ---
  double R   = sqrt(X * X + Y * Y);
  // atan2 gives (-180, 180]; RM counts 0..340 from its CCW stop, so a
  // bearing just clockwise of home must read 355, not -5, or it would be
  // refused as "below the CCW limit" while being perfectly reachable.
  double th2 = 0.0;
  if (R >= 1e-6) {
    th2 = atan2(Y, X) * RAD_TO_DEG;
    if (th2 < 0.0) th2 += 360.0;
  }

  if (axisEnforced("ROT") && (th2 < limRotMin - 1e-6 || th2 > limRotMax + 1e-6)) {
    r.error = "[ERROR] ROT=" + String(th2, 2) + " deg outside allowed ["
            + String(limRotMin, 1) + ", " + String(limRotMax, 1) + "]";
    return r;
  }

  // --- radial reach -> elbow angle ---
  // Each arm has its own elbow limits, so the reachable radius band is
  // per-arm too. Using arm 1's band for arm 2 is the v8 mistake.
  // The taught band is in MOTOR degrees; the reach curve is a function of
  // the FROG-LEG angle. Converting here is what keeps a re-calibration of
  // armGearRatio from silently moving the reachable envelope.
  // Arithmetic first: outside a3+a6 +/- (a4+a5) there is NO elbow angle at
  // all, on any machine, and foldAngleFromReach() would clamp and hand
  // back a pose that is not the one asked for. This refusal is not an
  // opinion about the envelope and is not switchable.
  if (R < ARM_RADIAL_OFFSET_MM - ARM_LINK_SUM_MM - 1e-6
      || R > ARM_RADIAL_OFFSET_MM + ARM_LINK_SUM_MM + 1e-6) {
    r.error = "[ERROR] R=" + String(R, 2) + " mm has no solution: the frog-leg "
              "spans a3+a6 +/- (a4+a5) = " + String(ARM_RADIAL_OFFSET_MM, 1)
            + " +/- " + String(ARM_LINK_SUM_MM, 1) + " mm";
    return r;
  }

  // Then the OPERATOR's envelope, and only if this axis is enforced.
  // There is no structural floor of 133.2 mm any more: that was
  // R(fold = 0) and it assumed the elbow's zero really is the folded home
  // pose, measured through an armGearRatio nobody has verified. Rejecting
  // a radius the arm is physically standing at is worse than having no
  // floor — see the same decision for the elbow boundaries themselves.
  const char *axisTok = (arm == 2) ? "A2" : "A1";
  if (axisEnforced(axisTok)) {
    double thMin, thMax;
    armBand(arm, thMin, thMax);
    double rMin, rMax;
    reachBandFor(armFoldFromMotor(thMin) + FOLD_ANGLE_HOME_DEG,
                 armFoldFromMotor(thMax) + FOLD_ANGLE_HOME_DEG, rMin, rMax);
    if (R < rMin - 1e-6 || R > rMax + 1e-6) {
      r.error = "[ERROR] R=" + String(R, 2) + " mm outside the band YOU taught for "
              + String(axisTok) + " [" + String(rMin, 1) + ", " + String(rMax, 1)
              + "] mm. Re-teach that boundary, or switch its enforcement off";
      return r;
    }
  }

  r.ok  = true;
  r.d1  = d1;
  r.th2 = th2;
  r.th3 = foldAngleFromReach(R);
  r.R   = R;
  return r;
}

void reportSingularityIfNear(double th3, const char *label) {
  if (th3 >= FOLD_SINGULARITY_WARN_DEG) {
    sendFeedback("[SINGULARITY] " + String(label) + " th3=" + String(th3, 2)
                 + " deg — frog-leg near straight, radial stiffness collapsing. Move slowly.");
  }
}

// Forward kinematics, for reporting only.
void forwardKinematics(double d1, double th2, double th3, int arm,
                       double &X, double &Y, double &Z) {
  double R = reachFromFoldAngle(th3);
  X = R * cos(th2 * DEG_TO_RAD);
  Y = R * sin(th2 * DEG_TO_RAD);
  Z = d1 + zOffsetForArm(arm);
}


// ══════════════════════════════════════════════════════════════
// MOTOR HELPERS
// ══════════════════════════════════════════════════════════════
// Motor RPM -> pulses per second. One place, so the microstep setting
// can never be applied to one axis and forgotten on another.
int32_t rpmToPulsesPerSec(float rpm) {
  return (int32_t)lround((double)rpm / 60.0 * PULSES_PER_MOTOR_REV);
}

// Clamps `v` into [0, hi] and records whether it had to.
float clampReport(float v, float hi, bool &flag) {
  if (v > hi) { flag = true; return hi; }
  return v;
}

// The universal RPM and the three percentages are the ONLY inputs. Every
// engineering figure below is derived, so the numbers the board reports
// are by construction the numbers the motors were given — they cannot
// drift apart the way two hand-maintained sets of fields do.
void applyMotionParams() {
  speedClampedRot = speedClampedArm = speedClampedZ = false;

  // Universal RPM -> this motor's RPM, through its calibration scale.
  float rotMotorRpm = masterRpm * (rotPct / 100.0f) * ROT_RPM_SCALE;
  float armMotorRpm = masterRpm * (armPct / 100.0f) * ARM_RPM_SCALE;
  float zMotorRpm   = masterRpm * (zPct   / 100.0f) * Z_RPM_SCALE;

  // Acceleration percentages are independent of the speed ones above.
  float rotMotorAcc = masterAccRpmS * (rotAccPct / 100.0f) * ROT_RPM_SCALE;
  float armMotorAcc = masterAccRpmS * (armAccPct / 100.0f) * ARM_RPM_SCALE;
  float zMotorAcc   = masterAccRpmS * (zAccPct   / 100.0f) * Z_RPM_SCALE;

  // The arm is bounded in MOTOR RPM, before any conversion, because motor
  // RPM is the only unit on this axis that currently means anything —
  // The arm's deg/s below is MOTOR deg/s, which needs no gear ratio;
  // from it is a guess. Stepper stall, not an invented angular speed, is
  // the thing worth guarding against here.
  armMotorRpm = clampReport(armMotorRpm, ARM_RPM_MAX,     speedClampedArm);
  armMotorAcc = clampReport(armMotorAcc, ARM_ACC_RPM_MAX, speedClampedArm);

  // Motor RPM -> the unit the axis actually moves in, so the RM and ZM
  // ceilings below are checked against something physically meaningful.
  rotVelDegS  = rotMotorRpm * 360.0f / (60.0f * (float)rotGearRatio);
  rotAccDegS2 = rotMotorAcc * 360.0f / (60.0f * (float)rotGearRatio);
  // MOTOR degrees per second. The frog-leg sweeps armVelDegS/armGearRatio,
  // which reportMotionProfile() quotes separately.
  armVelDegS  = armMotorRpm * 360.0f / 60.0f;
  armAccDegS2 = armMotorAcc * 360.0f / 60.0f;
  zVelMmS     = zMotorRpm   * (float)zMmPerRev / 60.0f;
  zAccMmS2    = zMotorAcc   * (float)zMmPerRev / 60.0f;

  // THE BACKSTOP for the two axes whose gearing is known. Without this, a
  // bigger universal number turns into a runaway on whichever axis
  // happens to be geared most favourably.
  rotVelDegS  = clampReport(rotVelDegS,  ROT_VEL_MAX, speedClampedRot);
  rotAccDegS2 = clampReport(rotAccDegS2, ROT_ACC_MAX, speedClampedRot);
  zVelMmS     = clampReport(zVelMmS,     Z_VEL_MAX,   speedClampedZ);
  zAccMmS2    = clampReport(zAccMmS2,    Z_ACC_MAX,   speedClampedZ);

  // Kept for reporting: what the arm motors were actually asked for.
  armMotorRpmActual = armMotorRpm;

  rotVelPulses   = (int32_t)lround(rotVelDegS  * pulsesPerDegRot());
  rotAccelPulses = (int32_t)lround(rotAccDegS2 * pulsesPerDegRot());
  armVelPulses   = (int32_t)lround(armVelDegS  * PULSES_PER_DEG_ARM_MOTOR);
  armAccelPulses = (int32_t)lround(armAccDegS2 * PULSES_PER_DEG_ARM_MOTOR);
  zVelPulses     = (int32_t)lround(zVelMmS     * pulsesPerMmZ());
  zAccelPulses   = (int32_t)lround(zAccMmS2    * pulsesPerMmZ());

  // A zero VelMax would silently freeze an axis; a zero AccelMax makes
  // ClearCore reject the move. Never let rounding produce either.
  if (rotVelPulses   < 1) rotVelPulses   = 1;
  if (rotAccelPulses < 1) rotAccelPulses = 1;
  if (armVelPulses   < 1) armVelPulses   = 1;
  if (armAccelPulses < 1) armAccelPulses = 1;
  if (zVelPulses     < 1) zVelPulses     = 1;
  if (zAccelPulses   < 1) zAccelPulses   = 1;

  MOTOR_ROT.VelMax(rotVelPulses);   MOTOR_ROT.AccelMax(rotAccelPulses);
  MOTOR_A1.VelMax(armVelPulses);    MOTOR_A1.AccelMax(armAccelPulses);
  MOTOR_A2.VelMax(armVelPulses);    MOTOR_A2.AccelMax(armAccelPulses);
  MOTOR_Z.VelMax(zVelPulses);       MOTOR_Z.AccelMax(zAccelPulses);
}

void reportMotionProfile() {
  sendFeedback("[SPEED] master " + String(masterRpm, 1) + " RPM, "
             + String(masterAccRpmS, 1) + " RPM/s | RM " + String(rotPct, 0)
             + "% | ARM " + String(armPct, 0) + "% | ZM " + String(zPct, 0) + "%"
             + " | RM acc " + String(rotAccPct, 0) + "% | ARM acc "
             + String(armAccPct, 0) + "% | ZM acc " + String(zAccPct, 0) + "%");
  // The arm leads with motor RPM and motor deg/s, both exact. The fold
  // deg/s that follows is only as good as armGearRatio, so it is quoted
  // with the ratio next to it rather than presented on its own.
  sendFeedback("[PROFILE] RM " + String(rotVelDegS, 2) + " deg/s, "
             + String(rotAccDegS2, 1) + " deg/s2"
             + String(speedClampedRot ? " (CLAMPED)" : "") + " | ARM "
             + String(armMotorRpmActual, 1) + " RPM"
             + String(speedClampedArm ? " (CLAMPED at " + String(ARM_RPM_MAX, 0)
                                        + " RPM)" : "")
             + " = " + String(armVelDegS, 1) + " motor deg/s = "
             + String(armFoldFromMotor(armVelDegS), 1) + " fold deg/s (ratio "
             + String(armGearRatio, 3) + ")" + " | ZM "
             + String(zVelMmS, 2) + " mm/s, "
             + String(zAccMmS2, 1) + " mm/s2"
             + String(speedClampedZ ? " (CLAMPED)" : ""));
  if (speedClampedRot || speedClampedZ) {
    sendFeedback("[WARN] RM and/or ZM hit their engineering ceiling. The master RPM is "
                 "higher than that axis's gearing can safely use — lower the master, "
                 "or lower that axis's percentage.");
  }
  if (speedClampedArm) {
    sendFeedback("[WARN] ARM clamped at " + String(ARM_RPM_MAX, 0) + " motor RPM. Past "
                 "this an open-loop stepper skips steps with nothing to detect it, "
                 "which corrupts the position reference silently.");
  }
}

// Returns true if `v` is inside [lo, hi]; otherwise reports why.
bool motionValueOk(double v, float lo, float hi, const char *what) {
  if (v >= lo && v <= hi) return true;
  sendFeedback("[ERROR] " + String(what) + "=" + String(v, 2) + " outside ["
             + String(lo, 1) + ", " + String(hi, 1) + "]");
  return false;
}

void motorsInit() {
  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_LOW);
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
  applyMotionParams();
  MOTOR_Z.EnableRequest(true);
  MOTOR_ROT.EnableRequest(true);
  MOTOR_A1.EnableRequest(true);
  MOTOR_A2.EnableRequest(true);
}

// Position readback in engineering units.
// NOTE: HOME is d1 = 0, rot = 0, and BOTH ELBOWS AT 0 — zero motor
// degrees, zero fold degrees. (th3_cad calls the same pose 60 deg; that
// label is not used or reported anywhere.) The elbow's
// pulse zero therefore corresponds to 60 deg, not 0 deg — that offset is
// applied here so every reported angle is a true th3_cad.
float currentD1()  { return (float)(MOTOR_Z.PositionRefCommanded()   / pulsesPerMmZ())  * (INVERT_Z    ? -1 : 1); }
float currentRot() { return (float)(MOTOR_ROT.PositionRefCommanded() / pulsesPerDegRot()) * (INVERT_ROT ? -1 : 1); }
// A1M/A2M report MOTOR degrees from the home reference — the raw count,
// the only elbow number this board actually knows. The frog-leg angle is
// currentA1Fold(), and the reach follows from that.
float currentA1()  { return (float)(MOTOR_A1.PositionRefCommanded()  / PULSES_PER_DEG_ARM_MOTOR) * (INVERT_ARM1 ? -1 : 1); }
float currentA2()  { return (float)(MOTOR_A2.PositionRefCommanded()  / PULSES_PER_DEG_ARM_MOTOR) * (INVERT_ARM2 ? -1 : 1); }
float currentA1Fold() { return (float)armFoldFromMotor(currentA1()) + FOLD_ANGLE_HOME_DEG; }
float currentA2Fold() { return (float)armFoldFromMotor(currentA2()) + FOLD_ANGLE_HOME_DEG; }

void moveJointsAbsolute(float d1, float rot, float a1, float a2) {
  int32_t zPulses   = (int32_t)lround(d1  * pulsesPerMmZ())  * (INVERT_Z    ? -1 : 1);
  int32_t rotPulses = (int32_t)lround(rot * pulsesPerDegRot()) * (INVERT_ROT ? -1 : 1);
  // a1/a2 arrive in MOTOR degrees, matching what currentA1()/currentA2()
  // report and what the taught limits are stored in.
  int32_t a1Pulses  = (int32_t)lround(a1 * PULSES_PER_DEG_ARM_MOTOR) * (INVERT_ARM1 ? -1 : 1);
  int32_t a2Pulses  = (int32_t)lround(a2 * PULSES_PER_DEG_ARM_MOTOR) * (INVERT_ARM2 ? -1 : 1);

  MOTOR_Z.Move(zPulses,     StepGenerator::MOVE_TARGET_ABSOLUTE);
  MOTOR_ROT.Move(rotPulses, StepGenerator::MOVE_TARGET_ABSOLUTE);
  MOTOR_A1.Move(a1Pulses,   StepGenerator::MOVE_TARGET_ABSOLUTE);
  MOTOR_A2.Move(a2Pulses,   StepGenerator::MOVE_TARGET_ABSOLUTE);
}

bool allMotorsSettled() {
  return MOTOR_Z.StepsComplete() && MOTOR_ROT.StepsComplete()
      && MOTOR_A1.StepsComplete() && MOTOR_A2.StepsComplete();
}

void decelStopAll(bool estop) {
  int32_t mult = estop ? (int32_t)ESTOP_DECEL_MULTIPLIER : 1;
  MOTOR_Z.MoveStopDecel(zAccelPulses     * mult);
  MOTOR_ROT.MoveStopDecel(rotAccelPulses * mult);
  MOTOR_A1.MoveStopDecel(armAccelPulses  * mult);
  MOTOR_A2.MoveStopDecel(armAccelPulses  * mult);
}


// ══════════════════════════════════════════════════════════════
// SOFT-LIMIT VALIDATION — defence in depth. The GUI already rejects
// bad targets, but the board must never assume the GUI is the only
// thing that can talk to it.
// ══════════════════════════════════════════════════════════════
// An axis whose enforcement the operator has switched OFF is skipped
// here as well as in the jog clamp. Validating against a boundary that
// will not be applied would refuse a target the machine is willing to
// drive to — the operator would be told the point is illegal by the same
// system that has been told to stop policing that axis.
bool jointTargetIsLegal(float d1, float rot, float a1, float a2, String &why) {
  if (axisEnforced("Z") && (d1 < limD1Min - 0.01 || d1 > limD1Max + 0.01)) {
    why = "d1=" + String(d1, 2) + " outside [" + String(limD1Min, 1) + ", "
        + String(limD1Max, 1) + "] mm"; return false;
  }
  if (axisEnforced("ROT") && (rot < limRotMin - 0.01 || rot > limRotMax + 0.01)) {
    why = "rot=" + String(rot, 2) + " outside [" + String(limRotMin, 1) + ", "
        + String(limRotMax, 1) + "] deg"; return false;
  }
  double b1Lo, b1Hi; armBand(1, b1Lo, b1Hi);
  if (axisEnforced("A1") && (a1 < b1Lo - 0.01 || a1 > b1Hi + 0.01)) {
    why = "A1M th3=" + String(a1, 2) + " outside [" + String(b1Lo, 1)
        + ", " + String(b1Hi, 1) + "] deg"; return false;
  }
  double b2Lo, b2Hi; armBand(2, b2Lo, b2Hi);
  if (axisEnforced("A2") && (a2 < b2Lo - 0.01 || a2 > b2Hi + 0.01)) {
    why = "A2M th3=" + String(a2, 2) + " outside [" + String(b2Lo, 1)
        + ", " + String(b2Hi, 1) + "] deg"; return false;
  }
  return true;
}


// ══════════════════════════════════════════════════════════════
// OPERATOR LIMIT EDITING
//
// Every write goes through applyLimit(), which is the single place that
// enforces the two rules a limit pair must always satisfy:
//   1. it stays inside the factory envelope (structure wins over opinion)
//   2. MIN stays below MAX by at least a usable span, so an axis can
//      never be boxed into a range it cannot be jogged out of
//
// The two TAUGHT elbow axes are exempt from both: they have no known
// envelope to check against, and their pair is unordered and simply
// sorted after each write. Rule 2's real guarantee — that the band is
// never empty — is still upheld, by refusing a write that would put both
// ends on the same position.
// ══════════════════════════════════════════════════════════════
// There is deliberately no "this boundary is locked" refusal any more.
// The old per-axis LOCK froze the value, and it was the only thing the
// per-axis button did — so the panel could tell you a boundary was
// UNLOCKED while saying nothing about whether it was switched on. The
// button now reports enforcement, and a switched-off axis is still fully
// editable: you want to be able to teach a boundary while it is not yet
// policing anything.
bool applyLimit(const String &axis, bool isMax, double value, String &why) {
  double *lo, *hi, floorV = 0, ceilV = 0, minSpan = 0;
  bool taught = false;
  String unit;

  if      (axis == "Z")   { lo=&limD1Min;  hi=&limD1Max;  floorV=D1_MIN_MM;
                            ceilV=D1_MAX_MM; minSpan=LIMIT_MIN_SPAN_MM;  unit=" mm"; }
  else if (axis == "ROT") { lo=&limRotMin; hi=&limRotMax; floorV=ROT_MIN_DEG;
                            ceilV=ROT_MAX_DEG; minSpan=LIMIT_MIN_SPAN_DEG; unit=" deg"; }
  // The elbows are TAUGHT: no envelope, no ordering — see
  // ARM_LIMITS_UNBOUNDED for why.
  else if (axis == "A1")  { lo=&limA1Min;  hi=&limA1Max;  taught=true; unit=" deg"; }
  else if (axis == "A2")  { lo=&limA2Min;  hi=&limA2Max;  taught=true; unit=" deg"; }
  else { why = "axis must be Z, ROT, A1 or A2 — got \"" + axis + "\""; return false; }

  if (taught) {
    // Store the captured number RAW, in the slot that was asked for.
    //
    // It is deliberately NOT sorted here. Sorting on write would fold the
    // first taught position against whatever stale value sat in the other
    // slot, and the operator's second SET HERE would then silently
    // overwrite their first. The pair is sorted where it is READ instead
    // — armBand() — so both taught numbers survive intact and order stops
    // being something anyone has to think about.
    //
    // The one arrangement that cannot work is both ends on the same
    // position: that pins the axis where it stands and no jog gets it
    // out again.
    double other = isMax ? *lo : *hi;
    if (fabs(value - other) < 1e-6) {
      why = "both elbow limits would be the same position ("
          + String(value, 2) + unit + "), leaving the axis no room to move — "
            "jog to the other end of the travel and SET HERE there";
      return false;
    }
    if (isMax) *hi = value; else *lo = value;
    return true;
  }

  if (value < floorV - 0.01 || value > ceilV + 0.01) {
    why = String(value, 2) + unit + " is outside the physical envelope ["
        + String(floorV, 1) + ", " + String(ceilV, 1) + "] — the structure "
          "cannot go there, so no setting can allow it";
    return false;
  }
  if (isMax && value < *lo + minSpan) {
    why = "upper limit " + String(value, 2) + unit + " must stay at least "
        + String(minSpan, 1) + unit + " above the lower limit ("
        + String(*lo, 2) + unit + ")";
    return false;
  }
  if (!isMax && value > *hi - minSpan) {
    why = "lower limit " + String(value, 2) + unit + " must stay at least "
        + String(minSpan, 1) + unit + " below the upper limit ("
        + String(*hi, 2) + unit + ")";
    return false;
  }

  if (isMax) *hi = value; else *lo = value;
  return true;
}

// Live position of an axis, in the unit its limits are expressed in.
bool currentValueForAxis(const String &axis, double &out) {
  if      (axis == "Z")   out = currentD1();
  else if (axis == "ROT") out = currentRot();
  else if (axis == "A1")  out = currentA1();
  else if (axis == "A2")  out = currentA2();
  else return false;
  return true;
}

void resetLimitsToFactory() {
  limD1Min  = D1_MIN_MM;          limD1Max  = D1_MAX_MM;
  limRotMin = ROT_MIN_DEG;        limRotMax = ROT_MAX_DEG;
  limA1Min  = armMotorFromFold(FOLD_ANGLE_MIN_DEG);
  limA1Max  = armMotorFromFold(FOLD_ANGLE_MAX_DEG);
  limA2Min  = armMotorFromFold(FOLD_ANGLE_MIN_DEG);
  limA2Max  = armMotorFromFold(FOLD_ANGLE_MAX_DEG);
}


// ══════════════════════════════════════════════════════════════
// REPORTING
// ══════════════════════════════════════════════════════════════
// A1M/A2M carry MOTOR degrees. The FOLD/R fields are the derived
// frog-leg angle and reach — appended rather than substituted so a host
// that only knows the old five fields still parses the line, and so
// somebody on a bare terminal can see both numbers at once.
void reportRunPosition(int percent) {
  sendFeedback("[CLEARCORE POS] D1: " + String(currentD1(), 2) + " mm | ROT: "
             + String(currentRot(), 2) + " deg | A1M: " + String(currentA1(), 2)
             + " deg | A2M: " + String(currentA2(), 2) + " deg (" + String(percent) + "%)"
             + " | FOLD1: " + String(currentA1Fold(), 2)
             + " deg | FOLD2: " + String(currentA2Fold(), 2)
             + " deg | R1: " + String(reachFromFoldAngle(currentA1Fold()), 1)
             + " mm | R2: " + String(reachFromFoldAngle(currentA2Fold()), 1) + " mm");
}

void reportJogPosition() {
  // v9 reports BOTH elbows. v8 sent a single "ARM:" field, which made the
  // second arm invisible to the host — the GUI still accepts that older
  // form, but only this one can show two independent arms.
  sendFeedback("[JOG POS] ROT: " + String(currentRot(), 2) + " deg | A1M: "
             + String(currentA1(), 2) + " deg | A2M: " + String(currentA2(), 2)
             + " deg | Z: " + String(currentD1(), 2) + " mm"
             + " | FOLD1: " + String(currentA1Fold(), 2)
             + " deg | FOLD2: " + String(currentA2Fold(), 2)
             + " deg | R1: " + String(reachFromFoldAngle(currentA1Fold()), 1)
             + " mm | R2: " + String(reachFromFoldAngle(currentA2Fold()), 1) + " mm");
}

// One machine-readable line per axis, so the GUI can mirror the board's
// idea of the limits instead of assuming its own copy is still correct.
void reportLimits() {
  sendFeedback("[LIMITS] Z " + String(limD1Min, 2) + ".." + String(limD1Max, 2)
             + " mm | ROT " + String(limRotMin, 2) + ".." + String(limRotMax, 2)
             + " deg | A1 " + String(min(limA1Min, limA1Max), 2) + ".."
             + String(max(limA1Min, limA1Max), 2)
             + " MOTOR deg | A2 " + String(min(limA2Min, limA2Max), 2) + ".."
             + String(max(limA2Min, limA2Max), 2) + " MOTOR deg");
  double r1Lo, r1Hi, r2Lo, r2Hi;
  double b1Lo, b1Hi, b2Lo, b2Hi;
  armBand(1, b1Lo, b1Hi); armBand(2, b2Lo, b2Hi);
  // armBand is MOTOR degrees; the reach curve is a function of the FOLD
  // angle, so the band is converted before it is asked for a radius.
  reachBandFor(armFoldFromMotor(b1Lo) + FOLD_ANGLE_HOME_DEG,
               armFoldFromMotor(b1Hi) + FOLD_ANGLE_HOME_DEG, r1Lo, r1Hi);
  reachBandFor(armFoldFromMotor(b2Lo) + FOLD_ANGLE_HOME_DEG,
               armFoldFromMotor(b2Hi) + FOLD_ANGLE_HOME_DEG, r2Lo, r2Hi);
  sendFeedback("[LIMITS_INFO] reach A1 " + String(r1Lo, 1) + ".."
             + String(r1Hi, 1) + " mm | reach A2 "
             + String(r2Lo, 1) + ".."
             + String(r2Hi, 1) + " mm | Zabs A1M "
             + String(Z_OFFSET_ARM1_MM + limD1Min, 1) + ".."
             + String(Z_OFFSET_ARM1_MM + limD1Max, 1) + " | A2M "
             + String(Z_OFFSET_ARM2_MM + limD1Min, 1) + ".."
             + String(Z_OFFSET_ARM2_MM + limD1Max, 1) + " mm | i_RM="
             + String(rotGearRatio, 4) + " | i_ARM=" + String(armGearRatio, 4)
             + " | enforced=" + String(!limitsEnabled ? "NO (DISABLED)"
                                       : isHomed ? "yes" : "no (unreferenced)"));
  // Per axis, 1 = enforced. The master switch is reported beside it and
  // not folded into the four digits: "all four are on but the master is
  // off" and "the master is on but this axis is off" are different
  // machine states and have to stay legible as different lines.
  sendFeedback(String("[LIMIT_ENFORCE] master=") + (limitsEnabled ? "yes" : "NO")
             + " | enforced: Z=" + String(limZEnforced ? 1 : 0)
             + " ROT=" + String(limRotEnforced ? 1 : 0)
             + " A1=" + String(limA1Enforced ? 1 : 0)
             + " A2=" + String(limA2Enforced ? 1 : 0));
}


// ══════════════════════════════════════════════════════════════
// MOTION CANCELLATION — one place, so an interlock can never be
// half-applied.
// ══════════════════════════════════════════════════════════════
void cancelJog() {
  rotDir = a1Dir = a2Dir = jzDir = 0;
  MOTOR_ROT.MoveVelocity(0);
  MOTOR_A1.MoveVelocity(0);
  MOTOR_A2.MoveVelocity(0);
  MOTOR_Z.MoveVelocity(0);
}

void cancelRun() {
  isMoving = false;
  runPhase = PHASE_NONE;
}

void cancelHoming() { isHoming = false; }


// ══════════════════════════════════════════════════════════════
// PROGRAM LOADING
// ══════════════════════════════════════════════════════════════
bool storeSequential(float d1a, float rota, float a1a, float a2a,
                     float d1b, float rotb, float a1b, float a2b) {
  String why;
  if (!jointTargetIsLegal(d1a, rota, a1a, a2a, why)) {
    sendFeedback("[ERROR] Point A rejected: " + why); return false;
  }
  if (!jointTargetIsLegal(d1b, rotb, a1b, a2b, why)) {
    sendFeedback("[ERROR] Point B rejected: " + why); return false;
  }
  loadedD1A = d1a; loadedRotA = rota; loadedA1A = a1a; loadedA2A = a2a;
  loadedD1B = d1b; loadedRotB = rotb; loadedA1B = a1b; loadedA2B = a2b;
  hasLoadedProgram = true;
  loadedProgramIsDual = false;
  sendFeedback("[LOADED] Point A/B stored.");
  reportSingularityIfNear(max(a1a, a2a), "A");
  reportSingularityIfNear(max(a1b, a2b), "B");
  return true;
}

bool storeDual(float d1, float rot, float a1, float a2) {
  String why;
  if (!jointTargetIsLegal(d1, rot, a1, a2, why)) {
    sendFeedback("[ERROR] Dual target rejected: " + why); return false;
  }
  loadedDualD1 = d1; loadedDualRot = rot; loadedDualA1 = a1; loadedDualA2 = a2;
  hasLoadedProgram = true;
  loadedProgramIsDual = true;
  sendFeedback("[LOADED] Simultaneous dual-arm target stored.");
  reportSingularityIfNear(a1, "A1M");
  reportSingularityIfNear(a2, "A2M");
  return true;
}


// ══════════════════════════════════════════════════════════════
// COMMAND PARSING HELPERS
// ══════════════════════════════════════════════════════════════
// Splits "a,b,c" into up to `maxOut` doubles. Returns the count parsed.
int parseCsv(const String &payload, double *out, int maxOut) {
  int count = 0, start = 0;
  while (count < maxOut) {
    int comma = payload.indexOf(',', start);
    String tok = (comma < 0) ? payload.substring(start) : payload.substring(start, comma);
    tok.trim();
    if (tok.length() == 0) break;
    out[count++] = tok.toDouble();
    if (comma < 0) break;
    start = comma + 1;
  }
  return count;
}


// ══════════════════════════════════════════════════════════════
// CARTESIAN COMMAND HANDLERS (new in v9)
//
// EVERY Z ON THE WIRE IS MEASURED FROM HOME.
//
// HOME is the P2P reference point — X 0, Y 0, Z 0. X and Y are measured
// from the turntable axis and are signed, because RM can put the arm
// behind the machine; Z is the lift's travel UP from HOME and cannot be
// negative, because HOME is the bottom of the stroke.
//
// solveIkFrogleg() still works in the ABSOLUTE frame (arm 1's deck at
// 514.3 mm with the lift down) and must keep doing so: that is the frame
// mophong_init.m is written in, and the parity sweep in firmware_check
// compares the two to machine precision. So the conversion happens at
// this edge, in ONE function, and is per arm — one carriage, two decks
// 9 mm apart, so the same Z-from-home is a different absolute height for
// arm 1 and arm 2.
// ══════════════════════════════════════════════════════════════
IkResult solveIkFromHome(int arm, double X, double Y, double zFromHome) {
  int a = (arm == 2) ? 2 : 1;
  return solveIkFrogleg(arm, X, Y, zFromHome + zOffsetForArm(a));
}

// Returns true and fills the joint quad when the IK succeeds; otherwise
// reports the reason and returns false. Z is FROM HOME.
bool ikToJoints(int arm, double X, double Y, double Z,
                float &d1, float &rot, float &a1, float &a2) {
  IkResult r = solveIkFromHome(arm, X, Y, Z);
  if (!r.ok) { sendFeedback(r.error); return false; }

  d1  = (float)r.d1;
  rot = (float)r.th2;
  // The idle arm HOLDS WHERE IT IS. It does not park at home.
  //
  // mophong_init.m pins AM2 at th3_home_cad for the whole trajectory, and
  // this function used to copy that by writing FOLD_ANGLE_HOME_DEG into
  // the idle slot. On a plot that is free; on the machine it is a move of
  // up to 120 deg that the operator never commanded, on an arm that may
  // well be carrying a substrate. The GUI already computed the idle angle
  // as "leave it alone" (solve_ik's idle_deg), so the board snapping it
  // home also meant LOAD and LOAD_XYZ produced different poses from the
  // same coordinates depending on which one you used.
  // r.th3 is a FROG-LEG angle; the motors are commanded in MOTOR degrees.
  float activeMotor = (float)armMotorFromFold(r.th3 - FOLD_ANGLE_HOME_DEG);
  a1 = (arm == 1) ? activeMotor : currentA1();
  a2 = (arm == 2) ? activeMotor : currentA2();

  sendFeedback("[IK] arm=" + String(arm) + " d1=" + String(r.d1, 3)
             + " rot=" + String(r.th2, 3) + " th3=" + String(r.th3, 3)
             + " R=" + String(r.R, 3));
  reportSingularityIfNear(r.th3, arm == 1 ? "A1M" : "A2M");
  return true;
}

void handleMoveXyz(const String &payload) {
  double v[4];
  if (parseCsv(payload, v, 4) != 4) {
    sendFeedback("[ERROR] MOVE_XYZ needs arm,X,Y,Z"); return;
  }
  float d1, rot, a1, a2;
  if (!ikToJoints((int)v[0], v[1], v[2], v[3], d1, rot, a1, a2)) return;

  cancelJog(); cancelHoming();
  runStartD1 = currentD1(); runStartRot = currentRot();
  runStartA1 = currentA1(); runStartA2 = currentA2();
  runTargetD1 = d1; runTargetRot = rot; runTargetA1 = a1; runTargetA2 = a2;
  moveJointsAbsolute(d1, rot, a1, a2);
  isMoving = true;
  runPhase = PHASE_DUAL;      // single-leg move, same completion path
  lastRunReportTime = millis();
  sendFeedback("[RUN] Cartesian move executing...");
}

void handleLoadXyz(const String &payload) {
  double v[7];
  if (parseCsv(payload, v, 7) != 7) {
    sendFeedback("[ERROR] LOAD_XYZ needs arm,Xa,Ya,Za,Xb,Yb,Zb"); return;
  }
  int arm = (int)v[0];
  float d1a, rota, a1a, a2a, d1b, rotb, a1b, a2b;
  if (!ikToJoints(arm, v[1], v[2], v[3], d1a, rota, a1a, a2a)) return;
  if (!ikToJoints(arm, v[4], v[5], v[6], d1b, rotb, a1b, a2b)) return;
  storeSequential(d1a, rota, a1a, a2a, d1b, rotb, a1b, a2b);
}

void handleLoadXyzBoth(const String &payload) {
  double v[6];
  if (parseCsv(payload, v, 6) != 6) {
    sendFeedback("[ERROR] LOAD_XYZ_BOTH needs Xa,Ya,Za,Xb,Yb,Zb"); return;
  }
  // One carriage lifts both decks, so in the FROM-HOME frame there is one
  // Z and the two commanded values must be EQUAL.
  //
  // This used to demand Za - Zb = 9 mm, and that was right while Z was an
  // absolute height: the two decks are 9 mm apart, so one carriage
  // position was two different absolute numbers. From HOME it is the
  // carriage's own travel, identical for both, and the 9 mm is applied
  // inside solveIkFromHome() instead. Keeping the old rule as well would
  // apply the drop twice.
  double dz = v[2] - v[5];
  if (fabs(dz) > 0.5) {
    sendFeedback("[ERROR] Both arms share one ZM carriage, and Z is measured from "
                 "HOME, so Za and Zb must be EQUAL. Got " + String(v[2], 2)
               + " and " + String(v[5], 2) + " (differ by " + String(dz, 2)
               + " mm). The 9 mm deck offset is applied by the board.");
    return;
  }
  IkResult r1 = solveIkFromHome(1, v[0], v[1], v[2]);
  if (!r1.ok) { sendFeedback(r1.error); return; }
  IkResult r2 = solveIkFromHome(2, v[3], v[4], v[5]);
  if (!r2.ok) { sendFeedback(r2.error); return; }

  if (fabs(r1.th2 - r2.th2) > 1.0) {
    sendFeedback("[ERROR] Both arms share RM: the two points must lie on the same "
                 "bearing, got " + String(r1.th2, 2) + " and " + String(r2.th2, 2) + " deg");
    return;
  }
  storeDual((float)r1.d1, (float)r1.th2, (float)r1.th3, (float)r2.th3);
}

void handleIkQuery(const String &payload) {
  double v[4];
  if (parseCsv(payload, v, 4) != 4) {
    sendFeedback("[ERROR] IK needs arm,X,Y,Z (Z measured from HOME)"); return;
  }
  IkResult r = solveIkFromHome((int)v[0], v[1], v[2], v[3]);
  if (!r.ok) { sendFeedback(r.error); return; }
  sendFeedback("[IK] arm=" + String((int)v[0]) + " d1=" + String(r.d1, 3)
             + " rot=" + String(r.th2, 3) + " th3=" + String(r.th3, 3)
             + " R=" + String(r.R, 3));
}

void handleFkQuery(const String &payload) {
  double v[5];
  if (parseCsv(payload, v, 5) != 5) {
    sendFeedback("[ERROR] FK needs d1,rot,a1,a2,arm"); return;
  }
  int arm = (int)v[4];
  double th3 = (arm == 2) ? v[3] : v[2];
  double X, Y, Z;
  forwardKinematics(v[0], v[1], th3, arm, X, Y, Z);
  // Z is reported FROM HOME, the same frame IK accepts, with the real
  // height alongside. FK answering in a different frame from the one IK
  // takes is how a round trip through the pair stops being a round trip.
  sendFeedback("[FK] arm=" + String(arm) + " X=" + String(X, 3)
             + " Y=" + String(Y, 3)
             + " Z=" + String(Z - zOffsetForArm(arm == 2 ? 2 : 1), 3)
             + " (from HOME) | Zabs=" + String(Z, 3));
}


// ══════════════════════════════════════════════════════════════
// RUN EXECUTION
// ══════════════════════════════════════════════════════════════
void beginRun() {
  if (!hasLoadedProgram) {
    sendFeedback("[WARN] RUN ignored — nothing loaded. Send LOAD/LOAD_XYZ first.");
    return;
  }
  cancelJog();
  cancelHoming();

  runStartD1 = currentD1(); runStartRot = currentRot();
  runStartA1 = currentA1(); runStartA2 = currentA2();

  if (loadedProgramIsDual) {
    runPhase = PHASE_DUAL;
    runTargetD1 = loadedDualD1; runTargetRot = loadedDualRot;
    runTargetA1 = loadedDualA1; runTargetA2 = loadedDualA2;
    sendFeedback("[RUN] Moving both arms simultaneously...");
  } else {
    // Leg 1 of 4: back to the reference pose before anything else.
    runPhase = PHASE_TO_HOME_FIRST;
    runTargetD1 = Z_HOME_MM_BOARD; runTargetRot = ROT_HOME_DEG_BOARD;
    runTargetA1 = ARM_HOME_MOTOR_DEG; runTargetA2 = ARM_HOME_MOTOR_DEG;
    sendFeedback("[RUN] Leg 1/4 — returning to HOME before Point A...");
  }
  String whySensor;
  if (runLegBlockedBySensor(runTargetD1, runTargetRot, runTargetA1, runTargetA2,
                            whySensor)) {
    runPhase = PHASE_NONE;
    sendFeedback("[ERROR] RUN refused — " + whySensor + ".");
    sendFeedback("[WARN] Jog that axis off its sensor, then RUN again.");
    return;
  }
  moveJointsAbsolute(runTargetD1, runTargetRot, runTargetA1, runTargetA2);
  isMoving = true;
  lastRunReportTime = millis();
}

// Starts one leg of a sequential program from wherever the machine is now.
// skipSensorBlock is for RESET_POSITION only: it drives toward the M5/M6
// end on purpose, so the ordinary "don't drive further into a covered
// sensor" check would refuse the one move whose job is to reach that end.
void beginRunLeg(RunPhase phase, float d1, float rot, float a1, float a2,
                 bool skipSensorBlock = false) {
  if (!skipSensorBlock) {
    String why;
    if (runLegBlockedBySensor(d1, rot, a1, a2, why)) {
      // Abandon the whole program, not just this leg: the remaining legs
      // were computed for a machine that could complete this one.
      isMoving = false;
      runPhase = PHASE_NONE;
      sendFeedback("[ERROR] RUN stopped — " + why + ".");
      sendFeedback("[WARN] Jog that axis off its sensor, then RUN again.");
      return;
    }
  }
  runPhase = phase;
  runStartD1 = currentD1(); runStartRot = currentRot();
  runStartA1 = currentA1(); runStartA2 = currentA2();
  runTargetD1 = d1; runTargetRot = rot; runTargetA1 = a1; runTargetA2 = a2;
  moveJointsAbsolute(runTargetD1, runTargetRot, runTargetA1, runTargetA2);
}

// RESET POSITION: drives to (0,0,0,0) with the board's own motor control
// -- the same run-leg machinery as RUN, not the PLC handshake HOME uses --
// and with the M5..M8 sensor block skipped (see beginRunLeg). Taught soft
// limits ARE still checked, once, up front.
void beginResetPosition() {
  if (isMoving || isHoming) {
    sendFeedback("[ERROR] RESET_POSITION refused — the machine is already moving.");
    return;
  }
  String why;
  if (!jointTargetIsLegal(Z_HOME_MM_BOARD, ROT_HOME_DEG_BOARD,
                          ARM_HOME_MOTOR_DEG, ARM_HOME_MOTOR_DEG, why)) {
    sendFeedback("[ERROR] RESET_POSITION refused — home is outside a taught boundary ("
               + why + "). Fix the boundary before resetting.");
    return;
  }
  cancelJog();
  sendFeedback("[RESET_POSITION] Moving to (0,0,0,0) under the board's own motor "
               "control -- no PLC handshake, M5..M8 sensor block skipped.");
  isMoving = true;
  lastRunReportTime = millis();
  beginRunLeg(PHASE_RESET_HOME, Z_HOME_MM_BOARD, ROT_HOME_DEG_BOARD,
              ARM_HOME_MOTOR_DEG, ARM_HOME_MOTOR_DEG, /*skipSensorBlock=*/true);
}

int runProgressPercent() {
  // Progress by the dominant axis, so a move that is mostly rotation
  // doesn't sit at 0% while the elbow barely changes.
  float span = max(max(fabs(runTargetD1 - runStartD1), fabs(runTargetRot - runStartRot)),
                   max(fabs(runTargetA1 - runStartA1), fabs(runTargetA2 - runStartA2)));
  if (span < 1e-3) return 100;
  float done = max(max(fabs(currentD1() - runStartD1), fabs(currentRot() - runStartRot)),
                   max(fabs(currentA1() - runStartA1), fabs(currentA2() - runStartA2)));
  int pct = (int)((done / span) * 100.0);
  return constrain(pct, 0, 100);
}

void serviceRun() {
  if (!isMoving) return;

  unsigned long now = millis();
  if (now - lastRunReportTime >= RUN_REPORT_INTERVAL_MS) {
    lastRunReportTime = now;
    reportRunPosition(runProgressPercent());
  }
  if (!allMotorsSettled()) return;

  // One helper for "start the next leg", so the four transitions cannot
  // drift apart on which start values they re-anchor.
  if (runPhase == PHASE_TO_HOME_FIRST) {
    sendFeedback("[RUN] HOME reached. Leg 2/4 — moving to Point A...");
    beginRunLeg(PHASE_TO_A, loadedD1A, loadedRotA, loadedA1A, loadedA2A);
    return;
  }
  if (runPhase == PHASE_TO_A) {
    sendFeedback("[RUN] Point A reached. Leg 3/4 — moving to Point B...");
    beginRunLeg(PHASE_TO_B, loadedD1B, loadedRotB, loadedA1B, loadedA2B);
    return;
  }
  if (runPhase == PHASE_TO_B) {
    sendFeedback("[RUN] Point B reached. Leg 4/4 — returning to HOME...");
    beginRunLeg(PHASE_TO_HOME_LAST, Z_HOME_MM_BOARD, ROT_HOME_DEG_BOARD,
                ARM_HOME_MOTOR_DEG, ARM_HOME_MOTOR_DEG);
    return;
  }

  reportRunPosition(100);
  isMoving = false;
  bool wasReset = (runPhase == PHASE_RESET_HOME);
  runPhase = PHASE_NONE;
  if (wasReset) {
    // Its own message, not [RUN] or [HOME]: no PLC handshake ran, and no
    // P2P program completed.
    sendFeedback("[RESET_POSITION] TARGET REACHED");
    return;
  }
  // English on the wire. The GUI still accepts the old Vietnamese string
  // so a board running v8 keeps working, but nothing emits it any more.
  sendFeedback("[RUN] TARGET REACHED");
}


// ══════════════════════════════════════════════════════════════
// JOG
// ══════════════════════════════════════════════════════════════
void applyJogVelocities() {
  int32_t rotV = (int32_t)(rotVelPulses * boostMultiplier);
  int32_t armV = (int32_t)(armVelPulses * boostMultiplier);
  int32_t zV   = (int32_t)(zVelPulses   * boostMultiplier);

  MOTOR_ROT.MoveVelocity(rotDir * rotV * (INVERT_ROT ? -1 : 1));
  // Extending means th3_cad INCREASES. Each elbow follows its own
  // direction now. INVERT_ARM1/INVERT_ARM2 were both confirmed false on
  // real hardware (same raw sign, not mirrored) — do not flip them
  // without re-testing on the bench.
  MOTOR_A1.MoveVelocity(a1Dir * armV * (INVERT_ARM1 ? -1 : 1));
  MOTOR_A2.MoveVelocity(a2Dir * armV * (INVERT_ARM2 ? -1 : 1));
  MOTOR_Z.MoveVelocity(jzDir * zV * (INVERT_Z ? -1 : 1));
}

bool anyJogActive() { return rotDir || a1Dir || a2Dir || jzDir; }

// ── Soft limits are only meaningful once the machine has a reference ──
//
// At power-on the step counters read 0, which this firmware maps to the
// HOME pose: elbow = 0 (the retracted stop) and d1 = 0 (the bottom
// of the lift). Those are exactly the MINIMUMS, so an EARLIER version
// that clamped absolutely refused to retract the arms or lower Z from
// the moment it booted, no matter where the machine physically was.
//
// The fix for that was to suspend the limits entirely until a reference
// existed. That was the wrong half of the problem to solve: it meant a
// boundary the operator had just taught at -300 let the axis run past
// -420, which is a machine with no protection at all.
//
// What is done now: the taught boundaries apply IMMEDIATELY, and every
// clamp is DIRECTIONAL — it stops `dir` only when that direction takes
// the axis further outside its band. An axis sitting outside a boundary
// at power-on can therefore always be jogged back in, and the boot-time
// deadlock cannot happen. A reference is still needed before any
// ABSOLUTE move, which is what RESET_COORD and HOME are for.
// Enforcement needs BOTH a reference to measure against and the master
// switch on. Without the reference the numbers are meaningless; without
// the switch the operator has deliberately asked for a free machine.
// A TAUGHT BOUNDARY APPLIES IMMEDIATELY — isHomed is NOT part of this.
//
// This used to be `isHomed && limitsEnabled`, and jog was free in both
// directions until the machine had a reference. The argument was that the
// counters mean nothing before then. It does not survive contact with how
// boundaries are actually set: the operator jogs to the stop and presses
// SET HERE, so the boundary is captured against the SAME counters it is
// later compared with. It is meaningful in exactly the frame it was
// taught in, reference or not. Waiting for a reference meant a limit
// somebody had just taught at -300 let the axis run to -427.
//
// Being outside the band at power-on cannot trap an axis, because every
// check below is DIRECTIONAL: it stops `dir` only when that direction
// takes the axis further out. Jogging back toward the band is always
// allowed.
//
// isHomed still matters, and still gates the things it should: absolute
// Cartesian moves and anything that claims to know where the machine is.
bool softLimitsActive() { return limitsEnabled; }

// The per-axis answer. Everything that clamps an axis asks THIS, not
// softLimitsActive(), or the per-axis switch would be decoration: an axis
// the operator turned off would still be stopped by the global one.
bool axisLimited(const String &axis) {
  return softLimitsActive() && axisEnforced(axis);
}

void warnUnreferencedOnce() {
  static bool warned = false;
  if (warned || isHomed) return;
  warned = true;
  // Soft limits ARE applied — they no longer wait for a reference. What is
  // missing without one is any claim that the reported POSITIONS mean
  // anything absolute, which is what P2P needs and jog does not.
  sendFeedback("[WARN] No reference yet. Your taught boundaries ARE being applied "
               "against the current counters, so jog is protected — but the "
               "reported positions are relative to wherever this board powered "
               "up. Run HOME, or RESET_COORD, before commanding absolute moves.");
}

// Per-elbow soft limit. Each arm is clamped against its OWN measured
// angle — reaching AM1's stop must not halt AM2.
//
// `whichArm` is 1 or 2 rather than a MotorDriver reference: the Arduino
// prototype generator copies a signature verbatim to the top of the
// sketch, and a namespace-qualified reference parameter there is a
// second easy way to break the build. Plain ints are always safe.
void serviceArmSoftLimit(int &dir, float angle, int whichArm) {
  if (!axisLimited(whichArm == 1 ? "A1" : "A2")) return;
  double loLim, hiLim; armBand(whichArm, loLim, hiLim);

  bool atMax = (dir > 0 && angle >= hiLim);
  bool atMin = (dir < 0 && angle <= loLim);
  if (!atMax && !atMin) return;

  dir = 0;
  if (whichArm == 1) MOTOR_A1.MoveVelocity(0);
  else               MOTOR_A2.MoveVelocity(0);

  String axis = String(whichArm == 1 ? "A1" : "A2") + (atMax ? "_FWD" : "_BACK");
  sendFeedback("[LIMIT] " + axis + " — th3 at "
             + String(atMax ? hiLim : loLim, 2)
             + (atMax ? " deg (upper limit)" : " deg (lower limit)"));
}

// Jog must respect the same soft limits as P2P. v8 let jog run past the
// modelled joint range entirely.
void serviceJogSoftLimits() {
  if (anyJogActive()) warnUnreferencedOnce();

  serviceArmSoftLimit(a1Dir, currentA1(), 1);
  serviceArmSoftLimit(a2Dir, currentA2(), 2);

  // Each axis is gated on its OWN switch. There is no early return on the
  // master here any more: axisLimited() already folds it in, and a shared
  // return would mean switching ZM's boundary off also stopped RM being
  // clamped.
  if (axisLimited("Z")) {
    if (jzDir > 0 && currentD1() >= limD1Max) {
      jzDir = 0; MOTOR_Z.MoveVelocity(0);
      sendFeedback("[LIMIT] Z_UP");
    }
    if (jzDir < 0 && currentD1() <= limD1Min) {
      jzDir = 0; MOTOR_Z.MoveVelocity(0);
      sendFeedback("[LIMIT] Z_DOWN");
    }
  }
  if (axisLimited("ROT")) {
    if (rotDir > 0 && currentRot() >= limRotMax) {
      rotDir = 0; MOTOR_ROT.MoveVelocity(0);
      sendFeedback("[LIMIT] ROT_CW");
    }
    if (rotDir < 0 && currentRot() <= limRotMin) {
      rotDir = 0; MOTOR_ROT.MoveVelocity(0);
      sendFeedback("[LIMIT] ROT_CCW");
    }
  }
}

void serviceJogReporting() {
  if (!anyJogActive()) return;
  unsigned long now = millis();
  if (now - lastJogReportTime >= JOG_REPORT_INTERVAL_MS) {
    lastJogReportTime = now;
    reportJogPosition();
  }
}

// Dead-man watchdog: if the host stops refreshing while an axis is
// held, stop. This is the last line of defence against a runaway — it
// does not care WHY the stop never arrived (host crash, cable pull, or
// a GUI bug swallowing the key-release event).
void serviceJogWatchdog() {
#if ENABLE_JOG_WATCHDOG
  if (!anyJogActive()) return;
  if (millis() - lastJogKeepAlive < JOG_WATCHDOG_MS) return;
  cancelJog();
  sendFeedback("[WATCHDOG] Jog stopped — no keep-alive from host for "
             + String((int)JOG_WATCHDOG_MS) + " ms.");
#endif
}

void startJog(int &axisDir, int dir) {
  if (isMoving)  { cancelRun();    sendFeedback("[WARN] RUN canceled by jog command."); }
  if (isHoming)  { cancelHoming(); plcClearHomeRequest();
                   sendFeedback("[WARN] Homing canceled by jog command."); }
  axisDir = dir;
  lastJogKeepAlive = millis();
  applyJogVelocities();
}

// Both elbows together — the linked "reach in / reach out" gesture the
// GUI's LINK toggle sends, and what v8's ARM_FWD/ARM_BACK meant.
void startArmJogLinked(int dir) {
  if (isMoving)  { cancelRun();    sendFeedback("[WARN] RUN canceled by jog command."); }
  if (isHoming)  { cancelHoming(); plcClearHomeRequest();
                   sendFeedback("[WARN] Homing canceled by jog command."); }
  a1Dir = dir;
  a2Dir = dir;
  lastJogKeepAlive = millis();
  applyJogVelocities();
}

void stopArmJog(bool arm1, bool arm2) {
  if (arm1) { a1Dir = 0; MOTOR_A1.MoveVelocity(0); }
  if (arm2) { a2Dir = 0; MOTOR_A2.MoveVelocity(0); }
}


// ══════════════════════════════════════════════════════════════
// HOMING
// ══════════════════════════════════════════════════════════════
// ══════════════════════════════════════════════════════════════
// PLC TRANSPORT — MC PROTOCOL 3E, ASCII
//
// FRAME LAYOUT (request), all fields ASCII hex unless noted:
//
//   5000        subheader
//   00          network no.
//   FF          PC no.
//   03FF        request destination module I/O no.
//   00          request destination multidrop station no.
//   NNNN        request data length: the character count of everything
//               from the monitoring timer to the end of this frame
//   0010        CPU monitoring timer, units of 250 ms
//   0401/1401   command: batch read / batch write
//   0000/0001   subcommand: word units / bit units
//   M*000000    device code (2 chars) + device number (6 chars)
//   0001        number of points
//   [data]      write only: one char per bit, or 4 chars per word
//
// Response:
//
//   D000 00 FF 03FF 00  NNNN  EEEE  [data]
//
// where NNNN is the character count of (end code + data) and EEEE is the
// end code — "0000" is success, anything else is a PLC error number that
// is worth logging verbatim because the manual indexes them.
//
// The request data length is COMPUTED from the assembled string rather
// than hand-counted. A wrong length makes the PLC either hang waiting for
// bytes that never arrive or answer with an error that looks like a
// device problem, and both cost an afternoon.
// ══════════════════════════════════════════════════════════════

// ---- Polled state, shared by every mode ----
uint16_t      plcStatusWord   = 0;       // last M0..M15 read
bool          plcStatusValid  = false;   // false until one poll has landed
unsigned long plcLastPollOk   = 0;
unsigned long plcLastPollSent = 0;
bool          plcLinkUp       = false;
// Rising-edge memory for the four home sensors, so a sensor that stays
// covered is reported once instead of on every 50 ms poll.
bool plcHomeSensorPrev[4] = {false, false, false, false};
// Home request currently asserted on the PLC, so STOP knows to clear it.
bool plcHomeRequested = false;
// Set once the four Run bits have been seen ON during a homing cycle.
// Without it, "all four Run bits are off" is also true in the instant
// before the PLC has started, and DONE from the PREVIOUS home would be
// accepted immediately.
bool plcSawRunDuringHome = false;

bool plcBit(int number) {
  if (number < 0 || number > 15) return false;
  return (plcStatusWord >> number) & 1;
}
bool plcAnyRunBit() {
  return plcBit(PLC_M_RUN_Z) || plcBit(PLC_M_RUN_ROT)
      || plcBit(PLC_M_RUN_A1) || plcBit(PLC_M_RUN_A2);
}

String plcStatusSummary() {
  // NOT just "no data": the host reads the bit fields out of this line, and
  // omitting them left every sensor lamp showing its initial CLEAR — which
  // on a safety display is indistinguishable from "sensor not covered".
  // Each field reports "unknown" instead.
  if (!plcStatusValid) {
    return String("NO DEVICE DATA | M1(DONE)=? home Z/R/A1/A2=???? "
                  "run Z/R/A1/A2=????");
  }
  String s = "M1(DONE)=" + String(plcBit(PLC_M_DONE) ? 1 : 0);
  s += " home Z/R/A1/A2=" + String(plcBit(PLC_M_HOME_Z) ? 1 : 0)
     + String(plcBit(PLC_M_HOME_ROT) ? 1 : 0)
     + String(plcBit(PLC_M_HOME_A1) ? 1 : 0)
     + String(plcBit(PLC_M_HOME_A2) ? 1 : 0);
  s += " run Z/R/A1/A2=" + String(plcBit(PLC_M_RUN_Z) ? 1 : 0)
     + String(plcBit(PLC_M_RUN_ROT) ? 1 : 0)
     + String(plcBit(PLC_M_RUN_A1) ? 1 : 0)
     + String(plcBit(PLC_M_RUN_A2) ? 1 : 0);
  return s;
}

// ---- ASCII hex helpers ----
// Uppercase, zero padded, fixed width. MC protocol is not tolerant about
// either: a lower-case 'a' and a missing leading zero are both rejected.
String plcHex(unsigned long value, int width) {
  static const char digits[] = "0123456789ABCDEF";
  // Built in a char buffer rather than by appending String(char): on some
  // cores String(char) resolves to the int constructor and writes the
  // character's decimal CODE instead of the character, which produces a
  // frame that looks almost right and is rejected by the PLC.
  char buf[12];
  if (width > 11) width = 11;
  for (int i = 0; i < width; i++) {
    int shift = (width - 1 - i) * 4;
    buf[i] = digits[(value >> shift) & 0xF];
  }
  buf[width] = '\0';
  return String(buf);
}
String plcDec(unsigned long value, int width) {
  String out = String((unsigned long)value);
  while (out.length() < width) out = String("0") + out;
  return out;
}
// Device number field: 6 characters, hexadecimal for X/Y/B/W and decimal
// for M/D/T. Passing the wrong base silently addresses another device.
String plcDeviceNum(long number, bool hexNumbering) {
  return hexNumbering ? plcHex((unsigned long)number, 6)
                      : plcDec((unsigned long)number, 6);
}
long plcParseHex(const String &s, int from, int count) {
  long v = 0;
  for (int i = from; i < from + count && i < s.length(); i++) {
    char c = s.charAt(i);
    int d = (c >= '0' && c <= '9') ? (c - '0')
          : (c >= 'A' && c <= 'F') ? (c - 'A' + 10)
          : (c >= 'a' && c <= 'f') ? (c - 'a' + 10) : -1;
    if (d < 0) return -1;
    v = (v << 4) | d;
  }
  return v;
}

// Assembles a complete 3E ASCII request. `body` is everything from the
// command field onwards; the monitoring timer and the length are added
// here so no caller can get the length wrong.
String plcBuildFrame(const String &body) {
  String payload = String(PLC_MC_MONITOR_TIMER) + body;
  return String(PLC_MC_SUBHEADER_REQ) + PLC_MC_NETWORK + PLC_MC_PC
       + PLC_MC_DEST_IO + PLC_MC_DEST_STATION
       + plcHex((unsigned long)payload.length(), 4) + payload;
}

// Batch read in WORD units. Bit devices read this way come back packed 16
// to a word, which is how one request covers M0..M15.
String plcFrameReadWords(const char *deviceCode, long deviceNum,
                         bool hexNumbering, uint16_t words) {
  return plcBuildFrame(String(PLC_MC_CMD_READ) + PLC_MC_SUB_WORD
                     + deviceCode + plcDeviceNum(deviceNum, hexNumbering)
                     + plcHex(words, 4));
}

// *** THERE IS NO WRITE FRAME BUILDER, ON PURPOSE ***
//
// The link is READ-ONLY. The only thing this board ever asked the PLC to
// do was HOME, and that is now a wire from IO-0 into X0. Leaving a
// plcFrameWriteBit() here "in case it is useful" would be an invitation to
// reintroduce exactly the defect the wire fixes — X devices are refreshed
// from their terminals every scan, so a written X0 is overwritten within
// one scan and the request silently never arrives.
//
// If a future feature really does need to set a PLC device, use an
// internal relay (M-device, decimal numbering), not an X, and add the
// builder back deliberately with that constraint written down.

#if PLC_LINK_MODE == PLC_LINK_ETHERNET
String        plcRxBuf;
bool          plcTxnActive   = false;   // a request is out, awaiting reply
unsigned long plcTxnSentAt   = 0;
unsigned long plcTxnTimeouts   = 0;
unsigned long plcGoodReads     = 0;   // replies parsed successfully
unsigned long plcSendAttempts  = 0;   // frames handed to the socket
unsigned long plcConnectTries  = 0;
unsigned long plcConnectFails  = 0;
// How many connects SUCCEEDED. Reported, because "we have never opened a
// socket" and "we open one repeatedly and get no answer" are different
// faults and want different words: the first is cable or address, the second
// is the protocol. Without this the host could only see the socket's current
// state, which cycles, so its lamp cycled with it.
unsigned long plcConnectsOk    = 0;
// Echoes every frame in and out. Off by default — it is a lot of traffic —
// but it is the only way to see whether the PLC is answering at all, and
// with what. PLC_DEBUG:1 turns it on.
bool plcDebug = false;

// ══════════════════════════════════════════════════════════════
// LINK STATE IS ABOUT DATA, NOT ABOUT THE SOCKET
//
// "Connected" used to mean "a TCP socket opened". That made the host's lamp
// FLAP green/red every few seconds whenever the PLC accepted a connection
// but did not answer device reads:
//
//   connect OK -> lamp green -> poll -> no reply -> 800 ms timeout ->
//   socket dropped for resync -> lamp red -> 3 s later reconnect -> green
//
// The socket really was going up and down, so the lamp was not lying about
// the socket — it was answering a question nobody asks. What the operator
// needs to know is whether device data is arriving, which is the thing HOME
// and the sensors depend on.
//
// plcDataState() reports that, and the flapping stops because a link that
// opens and never answers now sits steadily on NO_REPLY.
// ══════════════════════════════════════════════════════════════
const unsigned long PLC_DATA_STALE_MS = 4000;   // > 3 idle polls at 500 ms

const char *plcDataState() {
  if (plcGoodReads == 0) return "NONE";
  if (millis() - plcLastPollOk > PLC_DATA_STALE_MS) return "STALE";
  return "OK";
}

// Opens (or re-opens) the socket. Rate limited because connect() BLOCKS:
// retrying on every loop pass would stall motion servicing whenever the
// cable is out. Reports an unreachable PLC once per outage rather than on
// every attempt.
bool plcEnsureConnected() {
  if (plcClient.connected()) { plcLinkUp = true; return true; }

  plcLinkUp = false;
  unsigned long now = millis();
  if (plcLastConnectTry != 0 && (now - plcLastConnectTry) < PLC_RECONNECT_MS) {
    return false;
  }
  plcLastConnectTry = now;

  plcClient.stop();
  plcTxnActive = false;
  plcRxBuf = "";
  plcConnectTries++;
  if (plcClient.connect(plcTargetIp, PLC_PORT)) {
    plcReportedError = false;
    plcLinkUp = true;
    plcConnectsOk++;
    // Rate limited, and worded as what it is: a TCP socket, not a working
    // conversation. While the PLC accepts connections but answers nothing
    // this reconnects every PLC_RECONNECT_MS forever, and one line per
    // attempt buried the log.
    if (plcConnectTries == 1 || millis() - plcLastConnectLog > 30000) {
      plcLastConnectLog = millis();
      sendFeedback("[PLC] TCP socket open to " + String(PLC_IP_0) + "."
                 + String(PLC_IP_1) + "." + String(PLC_IP_2) + "."
                 + String(PLC_IP_3) + ":" + String((int)PLC_PORT)
                 + " (attempt " + String((unsigned long)plcConnectTries)
                 + "). This does NOT mean device reads work — watch data= in "
                   "[PLC_STATE].");
    }
    return true;
  }
  plcConnectFails++;
  if (!plcReportedError) {
    plcReportedError = true;
    sendFeedback("[ERROR] PLC unreachable at " + String(PLC_IP_0) + "." + String(PLC_IP_1)
               + "." + String(PLC_IP_2) + "." + String(PLC_IP_3) + ":"
               + String((int)PLC_PORT) + " — check the cable, that ClearCore is on "
               "192.168.3.x, and that the PLC's Ethernet module has MC protocol "
               "open on this port.");
  }
  return false;
}

// Fire and forget: the reply is collected by plcServiceRx() on later loop
// passes. Nothing here waits, so a slow or absent PLC cannot stall the
// motion loop — that is the whole reason this is a state machine and not
// a blocking request/response call.
// No `isRead` parameter: every transaction on this socket is a read now,
// so a flag saying which kind it is could only ever be wrong.
bool plcSend(const String &frame) {
  if (!plcEnsureConnected()) return false;
  plcSendAttempts++;
  if (plcDebug) sendFeedback("[PLC_TX] " + frame);
  plcClient.print(frame);
  plcClient.flush();
  plcRxBuf = "";
  plcTxnActive = true;
  plcTxnSentAt = millis();
  return true;
}

// Parses one complete response. Returns true when the frame was consumed
// (successfully or not) so the transaction can be closed.
bool plcConsumeResponse() {
  // Need the fixed header plus the 4-char length field before the total
  // frame size is even knowable.
  if (plcRxBuf.length() < PLC_MC_RES_HEADER_CHARS + 4) return false;
  long dataLen = plcParseHex(plcRxBuf, PLC_MC_RES_HEADER_CHARS, 4);
  if (dataLen < 0) {
    sendFeedback("[ERROR] PLC sent a malformed response length — dropping the "
                 "socket and resynchronising.");
    plcClient.stop();
    return true;
  }
  int total = PLC_MC_RES_HEADER_CHARS + 4 + (int)dataLen;
  if (plcRxBuf.length() < total) return false;      // still arriving

  String frame = plcRxBuf.substring(0, total);
  plcRxBuf = plcRxBuf.substring(total);

  if (!frame.startsWith(String(PLC_MC_SUBHEADER_RES))) {
    sendFeedback("[ERROR] PLC response subheader was \"" + frame.substring(0, 4)
               + "\", expected " PLC_MC_SUBHEADER_RES " — the port is probably not "
                 "speaking MC protocol 3E ASCII.");
    return true;
  }

  long endCode = plcParseHex(frame, PLC_MC_RES_HEADER_CHARS + 4, 4);
  if (endCode != 0) {
    sendFeedback("[ERROR] PLC end code " + plcHex((unsigned long)endCode, 4)
               + " — the read was refused. Check that M0 exists and that MC "
                 "protocol is enabled on the port.");
    return true;
  }

  // Every reply is a word read. One word of packed bits, 4 ASCII hex
  // chars, LSB = lowest device.
  long w = plcParseHex(frame, PLC_MC_RES_HEADER_CHARS + 8, 4);
  if (w < 0) {
    sendFeedback("[ERROR] PLC returned unreadable device data.");
    return true;
  }
  bool first = !plcStatusValid;
  uint16_t previous = plcStatusWord;
  plcStatusWord  = (uint16_t)w;
  plcStatusValid = true;
  plcLastPollOk  = millis();
  plcGoodReads++;
  // PUSH the word whenever it changes rather than waiting to be asked. The
  // host polls PLC_STATUS every few seconds; on its own that made a sensor
  // change take up to one host poll PLUS one PLC poll to appear. Only on a
  // change, so it cannot become a log flood.
  if (first || plcStatusWord != previous) {
    sendFeedback("[PLC_STATE] link=UP socket=OPEN data=" + String(plcDataState())
               + " conn=" + String((unsigned long)plcConnectsOk) + "/"
               + String((unsigned long)plcConnectTries)
               + " word=" + plcHex(plcStatusWord, 4)
               + " timeouts=" + String((unsigned long)plcTxnTimeouts)
               + " | " + plcStatusSummary());
  }
  return true;
}

void plcServiceRx() {
  bool got = false;
  while (plcClient.available() > 0) {
    char c = (char)plcClient.read();
    if (plcRxBuf.length() < 200) plcRxBuf += c;   // never grow unbounded
    got = true;
  }
  if (got && plcDebug) sendFeedback("[PLC_RX] " + plcRxBuf);
  if (!plcTxnActive) { plcRxBuf = ""; return; }

  if (plcConsumeResponse()) { plcTxnActive = false; return; }

  if (millis() - plcTxnSentAt >= PLC_TXN_TIMEOUT_MS) {
    plcTxnActive = false;
    plcRxBuf = "";
    plcTxnTimeouts++;
    // The stream is now out of step: if that reply is merely late it will
    // arrive while the NEXT request is outstanding and be parsed as its
    // answer. Dropping the socket is the only way to resynchronise for
    // certain, and it costs one reconnect on a link that is already
    // misbehaving.
    plcClient.stop();
    // One line per outage, not one per timeout: at a 50 ms poll a dead
    // PLC would otherwise produce 20 log lines a second and bury
    // everything else.
    if (plcTxnTimeouts == 1 || plcTxnTimeouts % 100 == 0) {
      sendFeedback("[PLC] No reply within " + String((int)PLC_TXN_TIMEOUT_MS)
                 + " ms (" + String((unsigned long)plcTxnTimeouts)
                 + " so far). The socket is open but the PLC is not answering "
                   "device reads.");
      if (plcGoodReads == 0) {
        sendFeedback("[PLC] The socket opening and closing every few seconds IS this "
                     "fault: TCP connects, the read times out, the socket is dropped "
                     "to resynchronise, and it reconnects. Cable and address are fine "
                     "— MC protocol is not answering on port "
                   + String((int)PLC_PORT) + ". Run PLC_TEST.");
      }
    }
  }
}
#endif  // PLC_LINK_ETHERNET

// Reacts to the four boundary bits. Called after every successful poll.
//
// Two jobs, and the order matters: STOP the axis first, then register the
// position. Registering first would spend a few hundred microseconds in
// String building while the axis is still driving into its stop.
// All four optical home sensors covered?
bool plcAllHomeSensors() {
  return plcBit(PLC_M_HOME_Z) && plcBit(PLC_M_HOME_ROT)
      && plcBit(PLC_M_HOME_A1) && plcBit(PLC_M_HOME_A2);
}

// Reports home-sensor transitions. Read-only: it stops nothing and writes
// no limits (see the header block on M5..M8).
// JOG: warn only, never stop. See the header block — jog is a dead-man
// control and is also how the operator gets off a tripped sensor.
//
// Rate-limited to once per axis per entry into the condition, or a held jog
// against a covered sensor would emit a line every 50 ms poll.
bool plcJogWarned[4] = {false, false, false, false};

void plcServiceSensorJogWarning() {
  const int  bits[4] = {PLC_M_HOME_Z, PLC_M_HOME_ROT, PLC_M_HOME_A1, PLC_M_HOME_A2};
  const int  ends[4] = {PLC_SENSOR_END_Z, PLC_SENSOR_END_ROT,
                        PLC_SENSOR_END_A1, PLC_SENSOR_END_A2};
  int       *dirs[4] = {&jzDir, &rotDir, &a1Dir, &a2Dir};
  const char *names[4] = {"ZM", "RM", "A1M", "A2M"};
  const char *sensor[4] = {"M5 MinZ", "M6 OutR", "M7 OutR1", "M8 OutR2"};

  for (int i = 0; i < 4; i++) {
    bool into = plcBit(bits[i]) && *dirs[i] == ends[i];
    if (into && !plcJogWarned[i]) {
      plcJogWarned[i] = true;
      sendFeedback("[WARN] " + String(names[i]) + " is jogging INTO "
                 + String(sensor[i]) + ", which is covered. Jog is not blocked "
                   "— watch the machine.");
    } else if (!into) {
      plcJogWarned[i] = false;
    }
  }
}

// P2P: refuse a leg that drives any axis further into a covered sensor.
bool runLegBlockedBySensor(float d1, float rot, float a1, float a2, String &why) {
  const int  bits[4] = {PLC_M_HOME_Z, PLC_M_HOME_ROT, PLC_M_HOME_A1, PLC_M_HOME_A2};
  const int  ends[4] = {PLC_SENSOR_END_Z, PLC_SENSOR_END_ROT,
                        PLC_SENSOR_END_A1, PLC_SENSOR_END_A2};
  const char *names[4] = {"ZM", "RM", "A1M", "A2M"};
  const char *sensor[4] = {"M5 MinZ", "M6 OutR", "M7 OutR1", "M8 OutR2"};
  float now[4]  = {currentD1(), currentRot(), currentA1(), currentA2()};
  float want[4] = {d1, rot, a1, a2};

  for (int i = 0; i < 4; i++) {
    if (!plcBit(bits[i])) continue;
    float delta = want[i] - now[i];
    if (fabs(delta) < 1e-3) continue;              // not moving that axis
    int dir = (delta > 0) ? 1 : -1;
    if (dir != ends[i]) continue;                  // moving AWAY, allowed
    why = String(names[i]) + " is on " + String(sensor[i])
        + " and the leg would drive it further in (" + String(now[i], 2)
        + " -> " + String(want[i], 2) + ")";
    return true;
  }
  return false;
}

// HOME STATE: M5 and M6 covered, M7 and M8 clear.
//
// This is the machine physically sitting on its reference, so it is the
// one condition that may zero the counters without anybody asking. It
// fires on the RISING EDGE only — while the machine sits at home the
// condition stays true, and re-zeroing every 50 ms poll would silently
// swallow any real motion away from home.
bool plcHomeStateActive() {
  return plcBit(PLC_M_HOME_Z) && plcBit(PLC_M_HOME_ROT)
      && !plcBit(PLC_M_HOME_A1) && !plcBit(PLC_M_HOME_A2);
}

bool plcHomeStatePrev = false;

void plcServiceHomeState() {
  bool now = plcHomeStateActive();
  if (now == plcHomeStatePrev) return;
  plcHomeStatePrev = now;
  if (!now) return;

  // Never while something is moving: the counters would be zeroed against
  // a position the axis has already left.
  if (isMoving || anyJogActive()) {
    sendFeedback("[PLC_HOME] HOME state reached but the machine is still moving — "
                 "coordinates NOT reset. Stop, then it will latch on the next entry.");
    plcHomeStatePrev = false;      // let it fire again once stopped
    return;
  }

  MOTOR_Z.PositionRefSet(0);
  MOTOR_ROT.PositionRefSet(0);
  MOTOR_A1.PositionRefSet(0);
  MOTOR_A2.PositionRefSet(0);
  isHomed = true;
  homeState = HOME_COMPLETE;
  if (isHoming) { isHoming = false; plcClearHomeRequest(); }
  sendFeedback("[PLC_HOME] HOME STATE — M5 and M6 covered, M7/M8 clear.");
  sendFeedback("[COORD_RESET] Coordinates reset to the standard home pose: "
               "d1=0.00 mm, ROT=0.00 deg, A1M=0.00 motor deg, A2M=0.00 motor deg.");
  reportJogPosition();
}

void plcServiceHomeSensors() {
  const int  bitNums[4]   = {PLC_M_HOME_Z, PLC_M_HOME_ROT,
                             PLC_M_HOME_A1, PLC_M_HOME_A2};
  const char *axisName[4] = {"ZM", "RM", "A1M", "A2M"};
  const char *bitName[4]  = {"M5 MinZ", "M6 OutR", "M7 OutR1", "M8 OutR2"};

  for (int i = 0; i < 4; i++) {
    bool on = plcBit(bitNums[i]);
    if (on != plcHomeSensorPrev[i]) {
      plcHomeSensorPrev[i] = on;
      const int ends[4] = {PLC_SENSOR_END_Z, PLC_SENSOR_END_ROT,
                           PLC_SENSOR_END_A1, PLC_SENSOR_END_A2};
      sendFeedback("[PLC_HOME] " + String(axisName[i]) + " sensor "
                 + String(bitName[i]) + (on ? " REACHED" : " left")
                 + (ends[i] > 0 ? " (far end)" : " (home end)"));
    }
  }
}

void plcServicePoll() {
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
  plcServiceRx();
  if (plcTxnActive) return;                 // one request at a time
  unsigned long now = millis();
  // isHoming, not plcHomeRequested: the fast rate has to cover the whole
  // cycle, including the window after DONE while finishHoming() is still
  // deciding, not just the interval where the request line is up.
  unsigned long interval = isHoming ? PLC_POLL_HOMING_MS : plcPollIdleMs;
  if (plcLastPollSent != 0 && (now - plcLastPollSent) < interval) return;
  plcLastPollSent = now;
  plcSend(plcFrameReadWords(PLC_POLL_DEVICE_CODE, PLC_POLL_DEVICE_NUM,
                            false, PLC_POLL_WORDS));
#endif
}

// Called from loop(). Poll, then act on what came back.
void servicePlc() {
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
  static unsigned long lastActedOn = 0;
  plcServicePoll();
  if (plcStatusValid && plcLastPollOk != lastActedOn) {
    lastActedOn = plcLastPollOk;
    plcServiceHomeSensors();
    plcServiceSensorJogWarning();
    plcServiceHomeState();
  }
#endif
}

// The request is a level on a wire. It goes out the same way in every
// link mode, because it is not a network operation at all.
void plcAssertHomeRequest() {
  plcHomeRequested = true;
  plcSawRunDuringHome = false;

  digitalWrite(PLC_HOME_REQ_PIN, PLC_HOME_ACTIVE_HIGH ? HIGH : LOW);

#if PLC_LINK_MODE == PLC_LINK_ETHERNET
  // Force the next poll to be fresh: a status word read BEFORE the line
  // went high could still be carrying the previous cycle's DONE.
  plcStatusValid = false;
  sendFeedback("[PLC] HOME request asserted on " PLC_HOME_REQ_PIN_NAME
               " -> X0 (hard-wired) — held until M1 (DONE).");
#elif PLC_LINK_MODE == PLC_LINK_DIGITAL_IO
  sendFeedback("[PLC] HOME request asserted on " PLC_HOME_REQ_PIN_NAME " -> X0.");
#else
  sendFeedback("[PLC] PLACEHOLDER: HOME request raised — no PLC wired.");
#endif
}

// True when the PLC says homing has finished.
//
// DONE (M1) alone is not enough. The PLC leaves M1 latched after a home,
// so a second HOME would be answered instantly by the previous cycle's
// bit before the machine had moved at all. The gate is therefore: the run
// bits M10..M13 must have been seen ON at least once during this cycle,
// they must now all be OFF, and M1 must be set.
bool plcHomeDoneAsserted() {
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
  if (!plcStatusValid) return false;
  if (plcAnyRunBit()) { plcSawRunDuringHome = true; return false; }
  if (!plcSawRunDuringHome) return false;    // PLC has not started yet
  if (!plcBit(PLC_M_DONE)) return false;
  // DONE with a home sensor still uncovered means the PLC and the sensors
  // disagree. Warn rather than refuse: refusing would hang the machine on
  // a miswired sensor, and staying silent would hide it.
  if (!plcAllHomeSensors()) {
    sendFeedback("[WARN] PLC returned DONE but not all home sensors are covered "
                 "(M5..M8). Check the sensor wiring — the reference may be wrong.");
  }
  return true;
#elif PLC_LINK_MODE == PLC_LINK_DIGITAL_IO
  int lvl = digitalRead(PLC_HOME_DONE_PIN);
  return PLC_HOME_ACTIVE_HIGH ? (lvl == HIGH) : (lvl == LOW);
#else
  return PLC_SIM_DONE_MS > 0 &&
         (millis() - homeRequestedAt) >= PLC_SIM_DONE_MS;
#endif
}

// Dropping the line CANNOT fail, which is the other reason this belongs on
// a wire. The old Ethernet write could be refused by a dead socket and
// leave the request latched at the PLC, so the machine re-homed itself the
// moment the link came back. A digitalWrite always lands.
void plcClearHomeRequest() {
  if (!plcHomeRequested) return;
  plcHomeRequested = false;
  digitalWrite(PLC_HOME_REQ_PIN, PLC_HOME_ACTIVE_HIGH ? LOW : HIGH);
  sendFeedback("[PLC] HOME request cleared (" PLC_HOME_REQ_PIN_NAME " off).");
}

void plcNetworkInit() {
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
  // Static addressing: the PLC is at a fixed .101, so DHCP would only add
  // a failure mode. Change plcLocalIp if .200 is already taken.
  Ethernet.begin(plcMac, plcLocalIp);
  sendFeedback("[PLC] ClearCore " + String(CC_IP_0) + "." + String(CC_IP_1) + "."
             + String(CC_IP_2) + "." + String(CC_IP_3)
             + " -> PLC " + String(PLC_IP_0) + "." + String(PLC_IP_1) + "."
             + String(PLC_IP_2) + "." + String(PLC_IP_3) + ":" + String((int)PLC_PORT)
             + " (MC protocol 3E, ASCII, READ-ONLY, polling M0..M15 every "
             + String((int)(PLC_POLL_IDLE_MS / 1000)) + " s idle / "
             + String((int)PLC_POLL_HOMING_MS) + " ms while homing)");
  if (Ethernet.linkStatus() == LinkOFF) {
    sendFeedback("[WARN] No Ethernet link detected — HOME will time out and the "
                 "PLC boundary switches will not be seen until the cable is in.");
  }
#endif
}

// HOME hands control to the PLC, which drives the machine to the
// reference position it owns. This board only stops its own motion,
// raises the request, and waits for DONE. It must NOT command a move of
// its own here — that would fight the PLC for the same axes.
void beginHoming() {
  cancelJog();
  cancelRun();
  decelStopAll(false);          // release the axes before the PLC takes over

  isHoming = true;
  homeState = HOME_REQUESTED;
  isHomed = false;              // stale until the PLC confirms
  homeRequestedAt = millis();
  lastHomeReportTime = homeRequestedAt;

  // HOME completion is gated on the PLC's run bits and DONE, all of which
  // arrive by MC-protocol device read. Without a single successful read the
  // cycle cannot finish however well the PLC homes the machine, so say that
  // NOW rather than after a 30 s timeout that looks like a PLC fault.
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
  if (plcGoodReads == 0) {
    sendFeedback("[WARN] No PLC device read has succeeded yet, so DONE and the run "
                 "bits cannot be seen and this HOME will time out. The request line "
                 "still goes out. Run PLC_TEST to find out why the read fails.");
  }
#endif

  plcAssertHomeRequest();
  sendFeedback("[HOME] Homing started. HOME request asserted on "
               PLC_HOME_REQ_PIN_NAME " -> X0, waiting for the run bits "
               "M10..M13 to finish and M1 (DONE) — timeout "
             + String((int)(PLC_HOME_TIMEOUT_MS / 1000)) + "s.");
}

void finishHoming(bool ok, const String &reason) {
  plcClearHomeRequest();
  isHoming = false;
  homeState = ok ? HOME_COMPLETE : HOME_FAILED;

  if (ok) {
#if PLC_LINK_MODE == PLC_LINK_PLACEHOLDER
    // The stub did not really home anything, so the step counters still
    // do not correspond to the machine. Refusing to zero them here is
    // what stops a fake HOME from silently corrupting the reference.
    sendFeedback("[WARN] PLACEHOLDER HOME — position reference NOT zeroed and "
                 "isHomed stays false. Wire the PLC before trusting this.");
    sendFeedback("[HOME] Homing complete (simulated).");
#else
    // The PLC has put the machine on its reference position, so this
    // board's step counters must be re-anchored to the home pose or
    // every later absolute move would be offset by the drift.
    // RESET THE COORDINATE SYSTEM TO THE STANDARD HOME POSE.
    //
    // The PLC has just put the machine on its reference position, so this
    // is the one moment the step counters can be anchored to a known pose.
    // All four go to zero, which by definition IS the standard home:
    //   d1  = 0 mm      lift at the bottom of its stroke
    //   ROT = 0 deg     turntable centred
    //   A1M = A2M = 0   motor degrees from home, i.e. fold 0 deg, R 133.2 mm
    // Skipping this leaves every later absolute move offset by however far
    // the machine had drifted from where the board thought it was.
    MOTOR_Z.PositionRefSet(0);
    MOTOR_ROT.PositionRefSet(0);
    MOTOR_A1.PositionRefSet(0);
    MOTOR_A2.PositionRefSet(0);
    isHomed = true;
    sendFeedback("[COORD_RESET] Coordinates reset to the standard home pose: "
                 "d1=0.00 mm, ROT=0.00 deg, A1M=0.00 motor deg, A2M=0.00 motor deg "
                 "(fold " + String(FOLD_ANGLE_HOME_DEG, 2) + " deg, R "
               + String(reachFromFoldAngle(FOLD_ANGLE_HOME_DEG), 1) + " mm).");
    sendFeedback("[HOME] Homing complete. Coordinates reset to standard home.");
    reportJogPosition();
#endif
  } else {
    sendFeedback("[HOME] FAILED — " + reason);
    sendFeedback("[ERROR] HOME timeout: PLC did not return DONE.");
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
    // Name the likely cause instead of leaving "no DONE" to cover three very
    // different faults.
    if (plcGoodReads == 0) {
      sendFeedback("[ERROR] Root cause: this board has never read a device from the "
                   "PLC, so it could not have seen DONE. Fix the MC-protocol link "
                   "first — PLC_TEST.");
    } else if (!plcSawRunDuringHome) {
      sendFeedback("[ERROR] Device reads work, but M10..M13 never came ON — the PLC "
                   "never started homing. Check the IO-0 -> X0 wire, its 24 V "
                   "return, and the PLC's own home sequence.");
    } else {
      sendFeedback("[ERROR] The PLC ran the axes but never set M1 (DONE). Check the "
                   "end of its home sequence.");
    }
#endif
  }
}

void serviceHoming() {
  if (!isHoming) return;
  unsigned long now = millis();

  if (now - lastHomeReportTime >= HOME_REPORT_INTERVAL_MS) {
    lastHomeReportTime = now;
    reportJogPosition();
  }

  if (plcHomeDoneAsserted()) {
    finishHoming(true, "");
    return;
  }
  if (now - homeRequestedAt >= PLC_HOME_TIMEOUT_MS) {
    finishHoming(false, "no DONE from PLC within "
               + String((int)(PLC_HOME_TIMEOUT_MS / 1000)) + "s");
  }
}


// ══════════════════════════════════════════════════════════════
// OPTIONAL LIMIT SENSORS
// ══════════════════════════════════════════════════════════════
#if ENABLE_ROT_Z_LIMIT_SENSORS
void serviceLimitSensors() {
  if (rotDir > 0 && digitalRead(ROT_LIMIT_CW_PIN) == LIMIT_ACTIVE_STATE) {
    rotDir = 0; MOTOR_ROT.MoveVelocity(0); sendFeedback("[LIMIT] ROT_CW");
  }
  if (rotDir < 0 && digitalRead(ROT_LIMIT_CCW_PIN) == LIMIT_ACTIVE_STATE) {
    rotDir = 0; MOTOR_ROT.MoveVelocity(0); sendFeedback("[LIMIT] ROT_CCW");
  }
  if (jzDir > 0 && digitalRead(Z_LIMIT_UP_PIN) == LIMIT_ACTIVE_STATE) {
    jzDir = 0; MOTOR_Z.MoveVelocity(0); sendFeedback("[LIMIT] Z_UP");
  }
  if (jzDir < 0 && digitalRead(Z_LIMIT_DOWN_PIN) == LIMIT_ACTIVE_STATE) {
    jzDir = 0; MOTOR_Z.MoveVelocity(0); sendFeedback("[LIMIT] Z_DOWN");
  }
}
#endif


// ══════════════════════════════════════════════════════════════
// COMMAND DISPATCH
// ══════════════════════════════════════════════════════════════
void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  ledPulse(LED_FLASH_RX_MS);

  String upper = cmd;
  upper.toUpperCase();

  // ---- link management ----
  if (upper == "PING") { sendFeedback("PONG"); if (!isConnected) { isConnected = true; sendFeedback("[CONNECTED] Python GUI handshake success."); } return; }
  if (upper == "BYE")  { cancelJog(); cancelRun(); cancelHoming(); decelStopAll(false); isConnected = false; return; }
  if (upper == "LIMITS") { reportLimits(); return; }

  // RESET COORDINATE — declares "the machine is physically at its
  // reference right now" and zeroes every counter here. Use it on the
  // bench while PLC homing is unavailable: jog the arms fully retracted
  // and the lift to the bottom BY EYE, then send this. Soft limits
  // become active from that point.
  //
  // Getting this wrong offsets every later absolute move, so it is a
  // deliberate operator action, never automatic, and refused while
  // anything is moving — zeroing a counter mid-move would record a
  // position the machine has already left.
  // RESET_COORD           zero all four axes  (unchanged)
  // RESET_COORD:Z|ROT|A1|A2  zero ONE axis
  //
  // One axis at a time matters because the four axes are referenced by
  // different means and rarely at the same moment: ZM and RM have optical
  // stops, the elbows are taught by eye. Being forced to re-declare all
  // four to fix one of them meant either lying about three axes or leaving
  // the wrong one wrong.
  //
  // A SINGLE-axis reset does NOT set isHomed. The reference is only
  // complete when every axis has one, and claiming it after zeroing one
  // axis would switch the soft limits on against three counters that are
  // still meaningless — the exact failure that made limits unusable at
  // boot. Only the all-axis form, and a real PLC home, may claim it.
  if (upper == "RESET_COORD" || upper == "SET_REF"
      || upper.startsWith("RESET_COORD:")) {
    if (isMoving || isHoming || anyJogActive()) {
      sendFeedback("[ERROR] RESET_COORD refused while moving. Stop first."); return;
    }
    String axis = "";
    if (upper.startsWith("RESET_COORD:")) {
      axis = upper.substring(12);
      axis.trim();
      if (axis == "ALL") axis = "";
    }

    if (axis.length() == 0) {
      MOTOR_Z.PositionRefSet(0);
      MOTOR_ROT.PositionRefSet(0);
      MOTOR_A1.PositionRefSet(0);
      MOTOR_A2.PositionRefSet(0);
      isHomed = true;
      homeState = HOME_COMPLETE;
      sendFeedback("[COORD_RESET] All four axis counters zeroed at the current "
                   "position: d1=0.00 mm, ROT=0.00 deg, A1M=0.00 motor deg, "
                   "A2M=0.00 motor deg (fold "
                 + String(FOLD_ANGLE_HOME_DEG, 2) + " deg).");
      sendFeedback("[HOME] Reference set manually. Soft limits now ACTIVE.");
      sendFeedback("[WARN] RESET_COORD trusts your eye, not a sensor. If the machine was "
                   "not actually at its reference, every absolute move is now offset.");
    } else if (axis == "Z" || axis == "ROT" || axis == "A1" || axis == "A2") {
      if      (axis == "Z")   MOTOR_Z.PositionRefSet(0);
      else if (axis == "ROT") MOTOR_ROT.PositionRefSet(0);
      else if (axis == "A1")  MOTOR_A1.PositionRefSet(0);
      else                    MOTOR_A2.PositionRefSet(0);
      sendFeedback("[COORD_RESET] " + axis + " counter zeroed at the current position. "
                   "The other three are untouched.");
      if (!isHomed) {
        sendFeedback("[WARN] Still UNREFERENCED overall — a single-axis reset does not "
                     "enable soft limits. Zero the remaining axes, send RESET_COORD "
                     "with no axis, or run HOME.");
      }
    } else {
      sendFeedback("[ERROR] RESET_COORD axis must be Z, ROT, A1, A2 or ALL — got \""
                 + axis + "\"");
      return;
    }
    reportJogPosition();
    return;
  }

  // SET_LIMIT_ENFORCE:<axis>,<0|1>   switch ONE axis's boundary on or off
  if (upper.startsWith("SET_LIMIT_ENFORCE:")) {
    String payload = cmd.substring(18);
    int comma = payload.indexOf(',');
    if (comma < 0) {
      sendFeedback("[ERROR] SET_LIMIT_ENFORCE needs axis,0|1 (axis = Z, ROT, A1, A2)");
      return;
    }
    String axis = payload.substring(0, comma); axis.trim(); axis.toUpperCase();
    String state = payload.substring(comma + 1); state.trim();
    bool *on = limEnforceFor(axis);
    if (on == NULL) {
      sendFeedback("[ERROR] SET_LIMIT_ENFORCE axis must be Z, ROT, A1 or A2 — got \""
                 + axis + "\"");
      return;
    }
    bool want = (state.toInt() != 0);
    // Loud on the way OFF, quiet on the way ON — same rule as the master
    // switch. Making the machine less safe is the event worth a warning.
    if (!want && *on) {
      sendFeedback("[WARN] " + axis + " SOFT LIMIT DISABLED. Its taught boundary is "
                   "kept but nothing will stop that axis at it. Re-enable with "
                   "SET_LIMIT_ENFORCE:" + axis + ",1.");
    }
    *on = want;
    sendFeedback("[LIMIT_ENFORCE] " + axis + " = " + String(*on ? "1 — enforced"
                                                                : "0 — NOT ENFORCED"));
    return;
  }

  // SET_LIMIT_LOCK was the per-axis VALUE freeze and is gone. It is
  // refused rather than aliased: "locked" and "enforced" are different
  // states, so quietly mapping lock=1 onto enforce=1 would let an old GUI
  // believe it had frozen a value it had in fact only armed.
  if (upper.startsWith("SET_LIMIT_LOCK:")) {
    sendFeedback("[ERROR] SET_LIMIT_LOCK no longer exists. The per-axis control is "
                 "now enforcement, not a value lock — use "
                 "SET_LIMIT_ENFORCE:<axis>,<0|1>. Update the GUI to match this board.");
    return;
  }

  // SET_LIMITS_ENABLED:<0|1>   suspend/resume enforcement on EVERY axis
  if (upper.startsWith("SET_LIMITS_ENABLED:")) {
    bool want = cmd.substring(19).toInt() != 0;
    if (!want && limitsEnabled) {
      // Loud, because this is the one command that makes the machine less
      // safe, and it must not be possible to do it without noticing.
      sendFeedback("[WARN] SOFT LIMITS DISABLED. Nothing will stop an axis at its "
                   "taught boundary — the PLC's physical switches (M5..M8) are now "
                   "the only protection. Re-enable with SET_LIMITS_ENABLED:1.");
    }
    limitsEnabled = want;
    sendFeedback(String("[LIMITS_ENABLED] ") + (limitsEnabled ? "1 — enforced"
                                                             : "0 — SUSPENDED"));
    reportLimits();
    return;
  }

  if (upper == "CLEAR_REF") {
    isHomed = false;
    homeState = HOME_IDLE;
    sendFeedback("[HOME] Reference cleared. Positions are relative again until HOME or RESET_COORD — your taught boundaries are still applied.");
    return;
  }

  // ---- Operator-defined travel limits ----
  //   SET_LIMIT:axis,end,value      axis = Z|ROT|A1|A2   end = MIN|MAX
  if (upper.startsWith("SET_LIMIT:")) {
    String payload = cmd.substring(10);
    int c1 = payload.indexOf(','), c2 = payload.indexOf(',', c1 + 1);
    if (c1 < 0 || c2 < 0) {
      sendFeedback("[ERROR] SET_LIMIT needs axis,MIN|MAX,value"); return;
    }
    String axis = payload.substring(0, c1);        axis.trim(); axis.toUpperCase();
    String end  = payload.substring(c1 + 1, c2);   end.trim();  end.toUpperCase();
    double value = payload.substring(c2 + 1).toDouble();
    if (end != "MIN" && end != "MAX") {
      sendFeedback("[ERROR] SET_LIMIT end must be MIN or MAX, got \"" + end + "\""); return;
    }
    String why;
    if (!applyLimit(axis, end == "MAX", value, why)) {
      sendFeedback("[ERROR] SET_LIMIT " + axis + " " + end + " rejected: " + why); return;
    }
    sendFeedback("[LIMIT_SET] " + axis + " " + end + " = " + String(value, 2));
    reportLimits();
    return;
  }

  // "Set the current position as this limit" — the button on the GUI.
  // This is the form an operator can actually use: jog to the lowest
  // point the machine may go, press SET AS LOWER, done. No measuring, no
  // arithmetic, no chance of typing the number for the wrong axis.
  if (upper.startsWith("SET_LIMIT_HERE:")) {
    String payload = cmd.substring(15);
    int c1 = payload.indexOf(',');
    if (c1 < 0) { sendFeedback("[ERROR] SET_LIMIT_HERE needs axis,MIN|MAX"); return; }
    String axis = payload.substring(0, c1);   axis.trim(); axis.toUpperCase();
    String end  = payload.substring(c1 + 1);  end.trim();  end.toUpperCase();
    if (end != "MIN" && end != "MAX") {
      sendFeedback("[ERROR] SET_LIMIT_HERE end must be MIN or MAX, got \"" + end + "\""); return;
    }
    if (isMoving || isHoming || anyJogActive()) {
      sendFeedback("[ERROR] SET_LIMIT_HERE refused while moving — the position would "
                   "already be stale by the time it was stored. Stop first.");
      return;
    }
    double here;
    if (!currentValueForAxis(axis, here)) {
      sendFeedback("[ERROR] axis must be Z, ROT, A1 or A2 — got \"" + axis + "\""); return;
    }
    String why;
    if (!applyLimit(axis, end == "MAX", here, why)) {
      sendFeedback("[ERROR] SET_LIMIT_HERE " + axis + " " + end + " rejected: " + why); return;
    }
    sendFeedback("[LIMIT_SET] " + axis + " " + end + " = " + String(here, 2)
               + " (captured from the current position)");
    reportLimits();
    return;
  }

  if (upper == "RESET_LIMITS") {
    resetLimitsToFactory();
    sendFeedback("[LIMIT_SET] All limits restored to the factory envelope.");
    reportLimits();
    return;
  }

  // Jog keep-alive. Cheap and silent — the host sends this every ~150 ms
  // for as long as any axis is held down.
  if (upper == "JOG_HB") { lastJogKeepAlive = millis(); return; }

  if (upper == "STATUS") {
    sendFeedback("[STATUS] fw=v9.1 indep-arms=yes watchdog="
               + String(ENABLE_JOG_WATCHDOG ? "on" : "off")
               + " plcLink=" + String(PLC_LINK_MODE)
               + " homed=" + String(isHomed ? "yes" : "no")
               + " homing=" + String(isHoming ? "yes" : "no")
               + " moving=" + String(isMoving ? "yes" : "no")
               + " jog[rot=" + String(rotDir) + " a1=" + String(a1Dir)
               + " a2=" + String(a2Dir) + " z=" + String(jzDir) + "]");
    sendFeedback("[PID] " + pidSummary() + " (stored only — this board runs OPEN LOOP)");
    reportMotionProfile();
    reportLimits();
    return;
  }

  // ---- emergency / stop first, so they can never be starved ----
  if (upper == "ESTOP") {
    cancelJog(); cancelRun(); cancelHoming(); plcClearHomeRequest();
    decelStopAll(true);
    sendFeedback("[ESTOP] EMERGENCY STOP");
    return;
  }
  if (upper == "STOP") {
    cancelJog(); cancelRun(); cancelHoming(); plcClearHomeRequest();
    decelStopAll(false);
    sendFeedback("[ESTOP] EMERGENCY STOP");
    return;
  }

  // ---- parameters ----
  // ---- PID gains ----  SET_PID:kp,ki,kd[,N]
  // One preset, no form selector. See the PID block near the top for why
  // both were removed rather than kept "just in case".
  if (upper.startsWith("SET_PID:")) {
    double v[4];
    int got = parseCsv(cmd.substring(8), v, 4);
    if (got < 3) { sendFeedback("[ERROR] SET_PID needs kp,ki,kd[,N]"); return; }
    if (v[0] < 0 || v[1] < 0 || v[2] < 0) {
      sendFeedback("[ERROR] PID gains must not be negative"); return;
    }
    if (got >= 4) {
      if (v[3] < 1.0 || v[3] > 200.0) {
        sendFeedback("[ERROR] N must be 1..200 (report recommends 50..100)"); return;
      }
      currentN = (float)v[3];
    }
    currentKp = v[0]; currentKi = v[1]; currentKd = v[2];
    sendFeedback("[PARAMS_OK] " + pidSummary());
    if (!pidEnabled) {
      sendFeedback("[WARN] PID is currently DISABLED — the gains were stored but are "
                   "not in use. Send PID_ON to enable them.");
    }
    return;
  }

  // The disable switch that replaced the form selector. Because this
  // board is open loop the gains do nothing mechanically either way —
  // what PID_OFF really does is stop them being reported as active, so
  // nobody tunes against a controller that is not running.
  if (upper == "PID_OFF") {
    pidEnabled = false;
    sendFeedback("[PARAMS_OK] PID DISABLED. Gains retained: kp=" + String(currentKp, 3)
               + " ki=" + String(currentKi, 3) + " kd=" + String(currentKd, 3)
               + " N=" + String(currentN, 1));
    return;
  }
  if (upper == "PID_ON") {
    pidEnabled = true;
    sendFeedback("[PARAMS_OK] PID ENABLED. " + pidSummary());
    return;
  }
  if (upper == "PID_RESET") {
    currentKp = PID_PRESET_KP; currentKi = PID_PRESET_KI;
    currentKd = PID_PRESET_KD; currentN  = PID_PRESET_N;
    pidEnabled = true;
    sendFeedback("[PARAMS_OK] PID restored to the report preset. " + pidSummary());
    return;
  }

  // ---- Speed: ONE universal RPM + one percentage per motor, PLUS an ----
  // ---- independent acceleration percentage per motor ----
  //   SET_SPEED:masterRpm,masterAccRpmS,rotPct,armPct,zPct,
  //             rotAccPct,armAccPct,zAccPct
  // The 3 accel fields were appended, never inserted -- a stale 5-field
  // sender fails the count check below and is refused loudly, rather than
  // having its 5th field silently misread as something else.
  // Every field is range-checked BEFORE any is applied, so one bad value
  // cannot leave the axes half-configured.
  if (upper.startsWith("SET_SPEED:")) {
    double v[8];
    if (parseCsv(cmd.substring(10), v, 8) != 8) {
      sendFeedback("[ERROR] SET_SPEED needs masterRpm,masterAccRpmS,rotPct,armPct,zPct,"
                   "rotAccPct,armAccPct,zAccPct");
      return;
    }
    if (!motionValueOk(v[0], MASTER_RPM_MIN, MASTER_RPM_MAX, "master RPM")   ||
        !motionValueOk(v[1], MASTER_ACC_MIN, MASTER_ACC_MAX, "master accel") ||
        !motionValueOk(v[2], AXIS_PCT_MIN,   AXIS_PCT_MAX,   "RM %")         ||
        !motionValueOk(v[3], AXIS_PCT_MIN,   AXIS_PCT_MAX,   "ARM %")        ||
        !motionValueOk(v[4], AXIS_PCT_MIN,   AXIS_PCT_MAX,   "ZM %")         ||
        !motionValueOk(v[5], AXIS_PCT_MIN,   AXIS_PCT_MAX,   "RM accel %")   ||
        !motionValueOk(v[6], AXIS_PCT_MIN,   AXIS_PCT_MAX,   "ARM accel %")  ||
        !motionValueOk(v[7], AXIS_PCT_MIN,   AXIS_PCT_MAX,   "ZM accel %")) return;

    masterRpm     = (float)v[0];
    masterAccRpmS = (float)v[1];
    rotPct        = (float)v[2];
    armPct        = (float)v[3];
    zPct          = (float)v[4];
    rotAccPct     = (float)v[5];
    armAccPct     = (float)v[6];
    zAccPct       = (float)v[7];
    applyMotionParams();
    if (anyJogActive()) applyJogVelocities();   // a held jog adopts it immediately
    sendFeedback("[MOTION_OK]");
    reportMotionProfile();
    return;
  }

  // ---- Legacy engineering-unit form, converted into the new model ----
  // Kept so an older GUI still configures the machine instead of silently
  // doing nothing. Each axis's requested speed is turned back into the
  // percentage that produces it at the CURRENT master RPM.
  if (upper.startsWith("SET_MOTION:")) {
    double v[6];
    if (parseCsv(cmd.substring(11), v, 6) != 6) {
      sendFeedback("[ERROR] SET_MOTION needs rotVel,rotAcc,armVel,armAcc,zVel,zAcc "
                   "(deg/s, deg/s2, mm/s, mm/s2)");
      return;
    }
    // The arm's bound is expressed in motor RPM now, so its deg/s field is
    // range-checked against whatever ARM_RPM_MAX works out to at the
    // current gear ratio rather than a fixed angular figure.
    // Motor deg/s, so no gear ratio is involved any more.
    float armVelCeil = ARM_RPM_MAX * 360.0f / 60.0f;
    if (!motionValueOk(v[0], MOTION_MIN, ROT_VEL_MAX, "RM vel")    ||
        !motionValueOk(v[1], MOTION_MIN, ROT_ACC_MAX, "RM accel")  ||
        !motionValueOk(v[2], MOTION_MIN, armVelCeil,  "ARM vel")   ||
        !motionValueOk(v[4], MOTION_MIN, Z_VEL_MAX,   "ZM vel")    ||
        !motionValueOk(v[5], MOTION_MIN, Z_ACC_MAX,   "ZM accel")) return;

    // deg/s -> motor RPM -> percentage of (master * this axis's scale).
    float rotRpm = (float)v[0] * (float)rotGearRatio * 60.0f / 360.0f;
    float armRpm = (float)v[2] * 60.0f / 360.0f;   // motor deg/s -> motor RPM
    float zRpm   = (float)v[4] * 60.0f / (float)zMmPerRev;

    rotPct = constrain(rotRpm / (masterRpm * ROT_RPM_SCALE) * 100.0f,
                       AXIS_PCT_MIN, AXIS_PCT_MAX);
    armPct = constrain(armRpm / (masterRpm * ARM_RPM_SCALE) * 100.0f,
                       AXIS_PCT_MIN, AXIS_PCT_MAX);
    zPct   = constrain(zRpm   / (masterRpm * Z_RPM_SCALE)   * 100.0f,
                       AXIS_PCT_MIN, AXIS_PCT_MAX);
    applyMotionParams();
    sendFeedback("[WARN] SET_MOTION is superseded by SET_SPEED. The per-axis speeds "
                 "were converted into percentages of the current master RPM; "
                 "acceleration fields were ignored -- set RM/ARM/ZM acceleration % "
                 "directly with SET_SPEED.");
    sendFeedback("[MOTION_OK]");
    reportMotionProfile();
    return;
  }

  // ---- Legacy v8 command, kept so an old host still configures PID ----
  // Its 4th/5th fields were ONE motor RPM applied to all four axes with
  // no per-axis scaling — the bug that made the elbow violent while the
  // turntable crawled. Those fields are ignored rather than reinterpreted.
  if (upper.startsWith("SET_PARAMS:")) {
    double v[7];
    int got = parseCsv(cmd.substring(11), v, 7);
    if (got < 3) { sendFeedback("[ERROR] SET_PARAMS needs at least kp,ki,kd"); return; }
    if (v[0] < 0 || v[1] < 0 || v[2] < 0) {
      sendFeedback("[ERROR] PID gains must not be negative"); return;
    }
    currentKp = v[0]; currentKi = v[1]; currentKd = v[2];
    if (got >= 6 && v[5] >= 1.0 && v[5] <= 200.0) currentN = (float)v[5];
    sendFeedback("[PARAMS_OK] " + pidSummary());
    if (got >= 5) {
      sendFeedback("[WARN] SET_PARAMS speed/accel fields ignored — they were a single "
                   "unscaled motor RPM for all axes. Use SET_SPEED.");
    }
    if (got >= 7) {
      sendFeedback("[WARN] SET_PARAMS PID form field ignored — v9.1 has one PID "
                   "preset and no form selector.");
    }
    return;
  }
  if (upper == "PROFILE") { reportMotionProfile(); return; }
  if (upper.startsWith("SET_BOOST:")) {
    float b = cmd.substring(10).toFloat();
    boostMultiplier = constrain(b, 0.1f, BOOST_MAX);
    if (anyJogActive()) applyJogVelocities();
    return;
  }

  // ---- Cartesian (v9) ----
  if (upper.startsWith("MOVE_XYZ:"))      { handleMoveXyz(cmd.substring(9));       return; }
  if (upper.startsWith("LOAD_XYZ_BOTH:")) { handleLoadXyzBoth(cmd.substring(14));  return; }
  if (upper.startsWith("LOAD_XYZ:"))      { handleLoadXyz(cmd.substring(9));       return; }
  if (upper.startsWith("IK:"))            { handleIkQuery(cmd.substring(3));       return; }
  if (upper.startsWith("FK:"))            { handleFkQuery(cmd.substring(3));       return; }

  // ---- joint space (v8-compatible) ----
  if (upper.startsWith("LOAD_BOTH:")) {
    double v[4];
    if (parseCsv(cmd.substring(10), v, 4) != 4) { sendFeedback("[ERROR] LOAD_BOTH needs d1,rot,a1,a2"); return; }
    storeDual(v[0], v[1], v[2], v[3]);
    return;
  }
  if (upper.startsWith("LOAD:")) {
    double v[8];
    if (parseCsv(cmd.substring(5), v, 8) != 8) { sendFeedback("[ERROR] LOAD needs 8 values"); return; }
    storeSequential(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
    return;
  }
  if (upper == "RUN")  { beginRun();     return; }
  if (upper == "HOME") { beginHoming();  return; }
  // Drives to (0,0,0,0) under the board's own motor control -- no PLC
  // handshake, M5..M8 sensor block skipped. See beginResetPosition().
  if (upper == "RESET_POSITION") { beginResetPosition(); return; }

  // PLC diagnostics. Worth having on the wire rather than only in the GUI:
  // when HOME does not work the question is always "what does the PLC
  // actually say", and this answers it from a plain serial terminal.
  // Calibrating the elbow drivetrain WITHOUT a re-flash. This is the one
  // number that turns the motor rotation the board counts into a real
  // frog-leg angle, so it has to be settable from the bench.
  if (upper.startsWith("SET_ARM_RATIO:")) {
    double r = cmd.substring(14).toDouble();
    if (r < ARM_GEAR_RATIO_MIN || r > ARM_GEAR_RATIO_MAX) {
      sendFeedback("[ERROR] arm gear ratio must be between "
                 + String(ARM_GEAR_RATIO_MIN, 2) + " and "
                 + String(ARM_GEAR_RATIO_MAX, 0) + ", got " + String(r, 4));
      return;
    }
    double before = armGearRatio;
    armGearRatio = r;
    // The TAUGHT LIMITS ARE NOT TOUCHED. They are motor degrees, which is
    // exactly why they are stored that way: a re-calibration changes what
    // frog-leg angle each boundary corresponds to, but not where the
    // boundary physically is, so nothing has to be re-taught.
    applyMotionParams();
    sendFeedback("[ARM_RATIO] " + String(before, 4) + " -> " + String(armGearRatio, 4)
               + " motor deg per fold deg. Taught limits are motor degrees and were "
                 "left alone; reported fold angles and reach figures have moved.");
    reportMotionProfile();
    reportLimits();
    return;
  }
  // Calibrating RM without a re-flash. Unlike the arm, lim_rot_min/max are
  // stored in RM DEGREES (the ratio-scaled output angle), so — like
  // SET_Z_LEAD, not SET_ARM_RATIO — a re-calibration moves where those
  // boundaries physically sit and must be re-checked.
  if (upper.startsWith("SET_ROT_RATIO:")) {
    double r = cmd.substring(14).toDouble();
    if (r < ROT_GEAR_RATIO_MIN || r > ROT_GEAR_RATIO_MAX) {
      sendFeedback("[ERROR] RM gear ratio must be between "
                 + String(ROT_GEAR_RATIO_MIN, 2) + " and "
                 + String(ROT_GEAR_RATIO_MAX, 0) + ", got " + String(r, 4));
      return;
    }
    double before = rotGearRatio, beforePos = currentRot();
    rotGearRatio = r;
    applyMotionParams();
    sendFeedback("[ROT_RATIO] " + String(before, 4) + " -> " + String(rotGearRatio, 4)
               + " motor deg per RM deg. RM now reads " + String(currentRot(), 2)
               + " deg at the SAME physical position (was " + String(beforePos, 2)
               + " deg) — the turntable has NOT moved.");
    sendFeedback("[WARN] lim_rot_min / lim_rot_max are stored in RM degrees, which just "
                 "changed meaning for the same physical place. Re-check both against "
                 "the machine before trusting them.");
    reportMotionProfile();
    reportLimits();
    return;
  }
  if (upper == "ROT_RATIO") {
    sendFeedback("[ROT_RATIO] " + String(rotGearRatio, 4)
               + " motor deg per RM deg (default " + String(ROT_GEAR_RATIO_DEF, 4)
               + " = 4.375 * 6.5, Simscape/machine agree — confirm on the bench if RM "
                 "turns more or less than commanded)");
    return;
  }

  // Calibrating the ZM drivetrain without a re-flash, for the same reason
  // SET_ARM_RATIO exists: this is the number that turns pulses into real
  // millimetres, and it was assumed rather than measured.
  if (upper.startsWith("SET_Z_LEAD:")) {
    double v = cmd.substring(11).toDouble();
    if (v < Z_MM_PER_REV_MIN || v > Z_MM_PER_REV_MAX) {
      sendFeedback("[ERROR] ZM lead must be between " + String(Z_MM_PER_REV_MIN, 2)
                 + " and " + String(Z_MM_PER_REV_MAX, 0) + " mm/rev, got " + String(v, 4));
      return;
    }
    // The counter is in PULSES, so re-scaling changes what the current
    // position READS without the carriage moving. Report both figures so
    // the change is visible rather than surprising.
    double before = currentD1();
    double oldLead = zMmPerRev;
    zMmPerRev = v;
    applyMotionParams();
    sendFeedback("[Z_LEAD] " + String(oldLead, 3) + " -> " + String(zMmPerRev, 3)
               + " mm/rev. The current position now reads " + String(currentD1(), 2)
               + " mm (was " + String(before, 2) + " mm) — the carriage has NOT moved.");
    sendFeedback("[WARN] Every ZM soft limit is in millimetres, so re-check "
                 "lim_z_min / lim_z_max against the machine after changing this.");
    reportMotionProfile();
    reportLimits();
    return;
  }
  if (upper == "Z_LEAD") {
    sendFeedback("[Z_LEAD] " + String(zMmPerRev, 4) + " mm per motor revolution ("
               + String(pulsesPerMmZ(), 2) + " pulses/mm, default "
               + String(Z_MM_PER_REV_DEF, 1) + " — MEASURE IT)");
    return;
  }

  if (upper == "ARM_RATIO") {
    sendFeedback("[ARM_RATIO] " + String(armGearRatio, 4)
               + " motor deg per fold deg (default " + String(ARM_GEAR_RATIO_DEF, 2)
               + ", derived from the Simscape -2 knee gain — confirm on the bench)");
    return;
  }

  if (upper == "PLC_STATUS") {
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
    sendFeedback("[PLC_COUNTS] connects " + String((unsigned long)plcConnectTries)
               + " (failed " + String((unsigned long)plcConnectFails) + ") | frames sent "
               + String((unsigned long)plcSendAttempts) + " | good reads "
               + String((unsigned long)plcGoodReads) + " | timeouts "
               + String((unsigned long)plcTxnTimeouts) + " | rx buffer \""
               + plcRxBuf + "\" | poll " + String((unsigned long)plcPollIdleMs)
               + " ms idle");
    if (plcGoodReads == 0) {
      sendFeedback("[PLC] NO device read has EVER succeeded. HOME cannot complete "
                   "without it (DONE is gated on the run bits), and every sensor "
                   "lamp will read unknown. Run PLC_TEST for the reason.");
    }
    sendFeedback("[PLC_STATE] link=" + String(plcLinkUp ? "UP" : "DOWN")
               + " socket=" + String(plcClient.connected() ? "OPEN" : "CLOSED")
               + " data=" + String(plcDataState())
               + " conn=" + String((unsigned long)plcConnectsOk) + "/"
               + String((unsigned long)plcConnectTries)
               + " word=" + (plcStatusValid ? plcHex(plcStatusWord, 4) : String("----"))
               + " timeouts=" + String((unsigned long)plcTxnTimeouts)
               + " | " + plcStatusSummary());
    if (!plcStatusValid) {
      sendFeedback("[PLC] No device data yet. Either the socket is not open, or the "
                   "Ethernet module is not configured for MC protocol on port "
                 + String((int)PLC_PORT) + " in ASCII.");
    }
#else
    sendFeedback("[PLC_STATE] link mode " + String(PLC_LINK_MODE)
               + " — no Ethernet client compiled in.");
#endif
    return;
  }

  if (upper.startsWith("PLC_DEBUG:")) {
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
    plcDebug = cmd.substring(10).toInt() != 0;
    sendFeedback(String("[PLC] Frame echo ") + (plcDebug ? "ON — every [PLC_TX] and "
                 "[PLC_RX] is logged verbatim." : "off."));
#else
    sendFeedback("[PLC] No Ethernet client compiled in.");
#endif
    return;
  }

  if (upper.startsWith("SET_PLC_POLL:")) {
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
    long ms = cmd.substring(13).toInt();
    if (ms < 20 || ms > 60000) {
      sendFeedback("[ERROR] PLC poll interval must be 20..60000 ms, got " + String(ms));
      return;
    }
    plcPollIdleMs = (unsigned long)ms;
    sendFeedback("[PLC] Idle poll interval " + String((unsigned long)plcPollIdleMs)
               + " ms (default " + String((unsigned long)PLC_POLL_IDLE_DEF_MS)
               + " ms, not persisted). Homing always polls at "
               + String((int)PLC_POLL_HOMING_MS) + " ms.");
#else
    sendFeedback("[PLC] No Ethernet client compiled in.");
#endif
    return;
  }

  // One read, reported in full. This is the command to run when the sensors
  // read stale: it says whether a frame went out, whether anything came
  // back, and exactly what — which separates "no link", "link but no MC
  // protocol", and "MC protocol but the wrong device" from each other.
  if (upper == "PLC_TEST") {
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
    sendFeedback("[PLC_TEST] target " + String(PLC_IP_0) + "." + String(PLC_IP_1)
               + "." + String(PLC_IP_2) + "." + String(PLC_IP_3) + ":"
               + String((int)PLC_PORT) + "  local " + String(CC_IP_0) + "."
               + String(CC_IP_1) + "." + String(CC_IP_2) + "." + String(CC_IP_3));
    sendFeedback("[PLC_TEST] Ethernet link: "
               + String(Ethernet.linkStatus() == LinkOFF ? "DOWN (no cable/no PHY)"
                                                         : "up"));
    plcClient.stop();
    plcLastConnectTry = 0;
    bool connected = plcEnsureConnected();
    sendFeedback(String("[PLC_TEST] TCP connect: ") + (connected ? "OK" : "FAILED"));
    if (!connected) {
      sendFeedback("[PLC_TEST] Nothing was sent. Check the cable, that ClearCore is "
                   "on the PLC's subnet, and that the PLC has a socket OPEN on port "
                 + String((int)PLC_PORT) + ".");
      return;
    }
    String frame = plcFrameReadWords(PLC_POLL_DEVICE_CODE, PLC_POLL_DEVICE_NUM,
                                     false, PLC_POLL_WORDS);
    sendFeedback("[PLC_TEST] TX " + frame);
    plcRxBuf = "";
    plcClient.print(frame);
    plcClient.flush();
    // Deliberately BLOCKING, unlike the poll: this is a one-shot diagnostic
    // run from a terminal with the machine stopped, and a definite answer is
    // worth more here than keeping the loop responsive.
    unsigned long t0 = millis();
    while (millis() - t0 < PLC_TXN_TIMEOUT_MS * 2) {
      while (plcClient.available() > 0) {
        char c = (char)plcClient.read();
        if (plcRxBuf.length() < 200) plcRxBuf += c;
      }
      if (plcRxBuf.length() >= PLC_MC_RES_HEADER_CHARS + 8) break;
    }
    if (plcRxBuf.length() == 0) {
      sendFeedback("[PLC_TEST] RX nothing. The socket is open but the PLC did not "
                   "answer a device read — this is almost always MC protocol not "
                   "enabled on that port, or the port set to BINARY while this "
                   "board speaks ASCII.");
      return;
    }
    sendFeedback("[PLC_TEST] RX " + plcRxBuf);
    if (!plcRxBuf.startsWith(String(PLC_MC_SUBHEADER_RES))) {
      sendFeedback("[PLC_TEST] Subheader is not " PLC_MC_SUBHEADER_RES
                   " — the port is answering, but not with MC protocol 3E ASCII.");
      return;
    }
    long endCode = plcParseHex(plcRxBuf, PLC_MC_RES_HEADER_CHARS + 4, 4);
    if (endCode != 0) {
      sendFeedback("[PLC_TEST] End code " + plcHex((unsigned long)endCode, 4)
                 + " — the PLC refused the read. Check that M0..M15 exist and that "
                   "the module permits reads.");
      return;
    }
    long w = plcParseHex(plcRxBuf, PLC_MC_RES_HEADER_CHARS + 8, 4);
    plcStatusWord = (uint16_t)w;
    plcStatusValid = true;
    plcLastPollOk = millis();
    sendFeedback("[PLC_TEST] OK — M0..M15 = " + plcHex((unsigned long)w, 4)
               + " | " + plcStatusSummary());
#else
    sendFeedback("[PLC_TEST] No Ethernet client compiled in (link mode "
               + String(PLC_LINK_MODE) + ").");
#endif
    return;
  }

  if (upper == "PLC_RECONNECT") {
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
    plcClient.stop();
    plcLastConnectTry = 0;      // skip the rate limit for a manual request
    plcReportedError  = false;
    plcStatusValid    = false;
    sendFeedback("[PLC] Socket dropped, reconnecting on the next service pass.");
#else
    sendFeedback("[PLC] Nothing to reconnect in link mode " + String(PLC_LINK_MODE) + ".");
#endif
    return;
  }


  // ---- jog ----
  if (upper == "ROT_CW")   { startJog(rotDir,  1); return; }
  if (upper == "ROT_CCW")  { startJog(rotDir, -1); return; }
  if (upper == "ROT_STOP") { rotDir = 0; MOTOR_ROT.MoveVelocity(0); return; }

  // --- independent per-arm jog (new in v9) ---
  if (upper == "A1_FWD")  { startJog(a1Dir,  1); return; }
  if (upper == "A1_BACK") { startJog(a1Dir, -1); return; }
  if (upper == "A1_STOP") { stopArmJog(true, false); return; }
  if (upper == "A2_FWD")  { startJog(a2Dir,  1); return; }
  if (upper == "A2_BACK") { startJog(a2Dir, -1); return; }
  if (upper == "A2_STOP") { stopArmJog(false, true); return; }

  // --- both arms together (v8-compatible, and what LINK sends) ---
  if (upper == "ARM_FWD")  { startArmJogLinked( 1); return; }
  if (upper == "ARM_BACK") { startArmJogLinked(-1); return; }
  if (upper == "ARM_STOP") { stopArmJog(true, true); return; }

  if (upper == "Z_UP")     { startJog(jzDir,  1); return; }
  if (upper == "Z_DOWN")   { startJog(jzDir, -1); return; }
  if (upper == "Z_STOP")   { jzDir = 0; MOTOR_Z.MoveVelocity(0); return; }

  // --- single-elbow absolute moves, no IK, no other axis touched ---
  if (upper.startsWith("MOVE_A1:") || upper.startsWith("MOVE_A2:")) {
    // The target is in MOTOR degrees — the same unit A1M_POS reports and
    // the taught limits are stored in, so the operator can type back the
    // number they just read off the screen.
    bool isA1 = upper.startsWith("MOVE_A1:");
    double target = cmd.substring(8).toDouble();
    double loLim, hiLim; armBand(isA1 ? 1 : 2, loLim, hiLim);
    if (target < loLim - 0.01 || target > hiLim + 0.01) {
      sendFeedback("[ERROR] motor=" + String(target, 2) + " deg outside ["
                 + String(loLim, 2) + ", " + String(hiLim, 2) + "] motor deg");
      return;
    }
    cancelJog(); cancelHoming();
    int32_t pulses = (int32_t)lround(target * PULSES_PER_DEG_ARM_MOTOR);
    if (isA1) {
      MOTOR_A1.Move(pulses * (INVERT_ARM1 ? -1 : 1), StepGenerator::MOVE_TARGET_ABSOLUTE);
    } else {
      MOTOR_A2.Move(pulses * (INVERT_ARM2 ? -1 : 1), StepGenerator::MOVE_TARGET_ABSOLUTE);
    }
    double targetFold = armFoldFromMotor(target) + FOLD_ANGLE_HOME_DEG;
    reportSingularityIfNear(targetFold, isA1 ? "A1M" : "A2M");
    sendFeedback("[RUN] " + String(isA1 ? "A1M" : "A2M") + " -> motor="
               + String(target, 2) + " deg (fold=" + String(targetFold, 2)
               + " deg, R=" + String(reachFromFoldAngle(targetFold), 1) + " mm)");
    return;
  }
  if (upper.startsWith("MOVE_R1:") || upper.startsWith("MOVE_R2:")) {
    // Same thing but commanded in millimetres of radial reach, which is
    // what an operator actually measures against a cassette.
    bool isA1 = upper.startsWith("MOVE_R1:");
    double rTarget = cmd.substring(8).toDouble();
    double rMin, rMax;
    // armBand() is in MOTOR degrees; reachBandFor() wants FOLD degrees.
    double bLo, bHi; armBand(isA1 ? 1 : 2, bLo, bHi);
    reachBandFor(armFoldFromMotor(bLo) + FOLD_ANGLE_HOME_DEG,
                 armFoldFromMotor(bHi) + FOLD_ANGLE_HOME_DEG,
                 rMin, rMax);
    if (rTarget < rMin - 0.01 || rTarget > rMax + 0.01) {
      sendFeedback("[ERROR] R=" + String(rTarget, 2) + " outside ["
                 + String(rMin, 1) + ", " + String(rMax, 1) + "] mm");
      return;
    }
    double th3 = foldAngleFromReach(rTarget);
    cancelJog(); cancelHoming();
    int32_t pulses = (int32_t)lround(
        armMotorFromFold(th3 - FOLD_ANGLE_HOME_DEG) * PULSES_PER_DEG_ARM_MOTOR);
    if (isA1) {
      MOTOR_A1.Move(pulses * (INVERT_ARM1 ? -1 : 1), StepGenerator::MOVE_TARGET_ABSOLUTE);
    } else {
      MOTOR_A2.Move(pulses * (INVERT_ARM2 ? -1 : 1), StepGenerator::MOVE_TARGET_ABSOLUTE);
    }
    reportSingularityIfNear(th3, isA1 ? "A1M" : "A2M");
    sendFeedback("[RUN] " + String(isA1 ? "A1M" : "A2M") + " -> R=" + String(rTarget, 2)
               + " mm (th3=" + String(th3, 2) + " deg)");
    return;
  }

  sendFeedback("[ERROR] Unknown command: " + cmd);
}


// ══════════════════════════════════════════════════════════════
// ARDUINO ENTRY POINTS
// ══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) { /* wait, but never forever */ }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

#if ENABLE_ROT_Z_LIMIT_SENSORS
  pinMode(ROT_LIMIT_CW_PIN,  INPUT);
  pinMode(ROT_LIMIT_CCW_PIN, INPUT);
  pinMode(Z_LIMIT_UP_PIN,    INPUT);
  pinMode(Z_LIMIT_DOWN_PIN,  INPUT);
#endif
  // The HOME request line is configured in EVERY link mode — it is a wire
  // to X0, not a feature of the Ethernet path. It is also driven to its
  // inactive level before anything else runs: a floating or latched-high
  // terminal at power-on would ask the PLC to home a machine nobody has
  // looked at yet.
  pinMode(PLC_HOME_REQ_PIN, OUTPUT);
  plcHomeRequested = true;          // so the clear below actually writes
  plcClearHomeRequest();
#if PLC_LINK_MODE == PLC_LINK_DIGITAL_IO
  pinMode(PLC_HOME_DONE_PIN, INPUT);
#endif

  motorsInit();
  plcNetworkInit();
  lastAliveTime = millis();
  lastJogKeepAlive = millis();

  // Loud, unambiguous banner: if you do not see "v9" here, the board is
  // still running old firmware and the per-arm commands will not exist.
  sendFeedback("[BOOT] ==========================================");
  sendFeedback("[BOOT] STCR4000S controller v9.1 — on-board frog-leg IK");
  sendFeedback("[BOOT] Independent arms: A1_FWD/A1_BACK, A2_FWD/A2_BACK");
  sendFeedback("[BOOT] Speed: universal RPM + per-motor % (SET_SPEED)");
  sendFeedback("[BOOT] Limits: SET_LIMIT / SET_LIMIT_HERE, reference: RESET_COORD");
  sendFeedback("[BOOT] Limits live in RAM only — the host must re-send them on connect.");
  sendFeedback("[BOOT] Jog watchdog: " + String(ENABLE_JOG_WATCHDOG ? "ON" : "OFF")
             + " (" + String((int)JOG_WATCHDOG_MS) + " ms) — host must send JOG_HB");
  sendFeedback("[BOOT] PLC: MC protocol 3E ASCII -> "
             + String(PLC_IP_0) + "." + String(PLC_IP_1) + "." + String(PLC_IP_2) + "."
             + String(PLC_IP_3) + ":" + String((int)PLC_PORT)
             + " | link mode " + String(PLC_LINK_MODE)
             + " | HOME req " PLC_HOME_REQ_PIN_NAME " -> X0 (WIRE)"
             + " | DONE M" + String(PLC_M_DONE)
             + " | home sensors M" + String(PLC_M_HOME_Z) + "..M" + String(PLC_M_HOME_A2)
             + " | run M" + String(PLC_M_RUN_Z) + "..M" + String(PLC_M_RUN_A2)
             + " | timeout " + String((int)(PLC_HOME_TIMEOUT_MS / 1000)) + "s");
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
  sendFeedback("[BOOT] PLC Ethernet is READ-ONLY (batch read M0..M15). The HOME "
               "request is a wire: " PLC_HOME_REQ_PIN_NAME " -> X0. If HOME never "
               "starts, check that wire and the 24 V return first — the network "
               "cannot start a home and never could.");
#endif
#if PLC_LINK_MODE == PLC_LINK_PLACEHOLDER
  sendFeedback("[BOOT] PLC link is in PLACEHOLDER mode — set PLC_LINK_MODE to "
               "PLC_LINK_ETHERNET once the PLC program is ready.");
#endif
  reportLimits();
  reportMotionProfile();
  sendFeedback("[PID] " + pidSummary() + " (stored only — this board runs OPEN LOOP)");
  sendFeedback("[BOOT] Elbow convention: A1M/A2M report MOTOR degrees from home. "
               "HOME IS 0 — 0 motor deg = 0 fold deg = fully retracted, R = "
             + String(reachFromFoldAngle(FOLD_ANGLE_HOME_DEG), 1) + " mm. "
             + String(armMotorFromFold(FOLD_ANGLE_MAX_DEG), 0)
             + " motor deg = fold " + String(FOLD_ANGLE_MAX_DEG, 0)
             + " deg = straight, R = "
             + String(reachFromFoldAngle(FOLD_ANGLE_MAX_DEG), 1) + " mm.");
  sendFeedback("[BOOT] The 60 deg you may remember was th3_cad, the CAD frame's label "
               "for the SAME retracted pose. It is not reported anywhere any more. "
               "RE-HOME after flashing from v8.");
  sendFeedback("[BOOT] ==========================================");
}

void loop() {
  while (Serial.available() > 0) {
    handleCommand(Serial.readStringUntil('\n'));
  }

  serviceLed();
#if ENABLE_ROT_Z_LIMIT_SENSORS
  serviceLimitSensors();
#endif
  serviceJogWatchdog();
  serviceJogSoftLimits();
  serviceJogReporting();
  serviceRun();
  // Polled BEFORE serviceHoming so a DONE that arrives this pass is acted
  // on in the same pass, and so a boundary bit stops an axis before the
  // run service issues its next segment.
  servicePlc();
  serviceHoming();

  unsigned long now = millis();
  if (isConnected && (now - lastAliveTime >= ALIVE_INTERVAL_MS)) {
    lastAliveTime = now;
    if (!isMoving && !isHoming && !anyJogActive()) {
      sendFeedback("[ALIVE] uptime: " + String(now / 1000) + "s");
    }
  }
}
