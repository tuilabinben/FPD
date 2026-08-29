#include "ClearCore.h"
#include <math.h>

// ---- Motor connector mapping ----
#define MOTOR_Z   ConnectorM0
#define MOTOR_ROT ConnectorM1
#define MOTOR_A1  ConnectorM2
#define MOTOR_A2  ConnectorM3

#define LED_PIN LED_BUILTIN


// TYPES — MUST STAY ABOVE THE FIRST FUNCTION DEFINITION.  [notes §1]
struct IkResult {
  bool   ok;
  double d1;
  double th2;
  double th3;
  double R;
  String error;
};

enum RunPhase { PHASE_NONE, PHASE_TO_HOME_FIRST, PHASE_TO_A, PHASE_TO_B,
                PHASE_TO_HOME_LAST, PHASE_DUAL, PHASE_RESET_HOME };

enum HomeState { HOME_IDLE, HOME_REQUESTED, HOME_COMPLETE, HOME_FAILED };

void plcNetworkInit();
void servicePlc();
double armFoldFromMotor(double motorDeg);
double armMotorFromFold(double foldDeg);
double pulsesPerMmZ();
float currentA1Fold();
float currentA2Fold();
String plcStatusSummary();
bool plcBit(int number);
bool plcHomeStateActive();
extern bool plcLimitSensorEnabled[3];   // order Z/ROT/A2 — defined near plcLimitBitFor()
int plcLimitEndFor(int i);              // which end that switch refuses RIGHT NOW
bool runLegBlockedByLimit(float d1, float rot, float a2, String &why);
void applyMotionParams();
void applyJogVelocities();
void reportMotionProfile();
void reachBandFor(double thMin, double thMax, double &rMin, double &rMax);

void reportLimits();
void resetLimitsToFactory();
void cancelJog();
void cancelRun();
void cancelHoming();
void cancelScan(const String &why);
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
// ══════════════════════════════════════════════════════════════
const double A3_MM = 45.0;
const double A4_MM = 91.25;
const double A5_MM = 91.25;
const double A6_MM = 377.5;

const double D_BASE_MM  = 388.0;
const double D3_ARM1_MM = 50.0;
const double D3_ARM2_MM = 41.0;
const double D4_MM = 46.5;
const double D5_MM = 24.8;
const double D6_MM = 5.0;

const double Z_OFFSET_ARM1_MM = D_BASE_MM + D3_ARM1_MM + D4_MM + D5_MM + D6_MM;
const double Z_OFFSET_ARM2_MM = D_BASE_MM + D3_ARM2_MM + D4_MM + D5_MM + D6_MM;
const double ARM2_Z_DROP_MM   = D3_ARM1_MM - D3_ARM2_MM;

const double ARM_LINK_SUM_MM      = A4_MM + A5_MM;
const double ARM_RADIAL_OFFSET_MM = A3_MM + A6_MM;

const double I_RM_TOTAL = 1 * 6.5;

const double ARM_ZERO_CAD_DEG = 0.0;

const double FOLD_ANGLE_HOME_DEG     = 0.0;
const double FOLD_ANGLE_SPEC_MAX_DEG = 146.68;
const double FOLD_ANGLE_MIN_DEG      = FOLD_ANGLE_HOME_DEG;
const double FOLD_ANGLE_MAX_DEG      = 180.0;
const double FOLD_SINGULARITY_WARN_DEG = 170.0;

// MOTOR degrees per FROG-LEG degree. MEASURED on the machine: at full
// extension the arm reaches 575 mm, where the earlier 10.0 put the same
// motor position at 498 mm, so 10.0 * fold(498)/fold(575) = 7.80. A reach
// measurement rather than an angle one, because reach is what a tape
// measure can read on this machine.
//
// The Simscape model says 2 -- shoulder x-1, knee x-2 -- and the model is
// wrong here: it describes the LINKAGE, not the gearbox in front of it.
// Do not restore 2 from the .m.
const double ARM_GEAR_RATIO_DEF = 7.80;
const double ARM_GEAR_RATIO_MIN = 0.01, ARM_GEAR_RATIO_MAX = 1000.0;
double armGearRatio = ARM_GEAR_RATIO_DEF;

double armFoldFromMotor(double motorDeg) {
  return (armGearRatio == 0.0) ? motorDeg : motorDeg / armGearRatio;
}
double armMotorFromFold(double foldDeg) { return foldDeg * armGearRatio; }

const float Z_HOME_MM_BOARD      = 0.0f;
const float ROT_HOME_DEG_BOARD   = 0.0f;
const float ARM_HOME_MOTOR_DEG   = 0.0f;

const double D1_MIN_MM = 0.0, D1_MAX_MM = 285.0;
// ── RM ZERO IS THE CCW STOP, NOT MID-TRAVEL ──────────────────────  [notes §5]
const double ROT_MIN_DEG = 0.0, ROT_MAX_DEG = 340.0;

// OPERATOR-DEFINED WORKING LIMITS  (NEW in v9.1)  [notes §6]
double limD1Min  = D1_MIN_MM,          limD1Max  = D1_MAX_MM;
double limRotMin = ROT_MIN_DEG,        limRotMax = ROT_MAX_DEG;
double limA1Min  = FOLD_ANGLE_MIN_DEG * ARM_GEAR_RATIO_DEF,
       limA1Max  = FOLD_ANGLE_MAX_DEG * ARM_GEAR_RATIO_DEF;
double limA2Min  = FOLD_ANGLE_MIN_DEG * ARM_GEAR_RATIO_DEF,
       limA2Max  = FOLD_ANGLE_MAX_DEG * ARM_GEAR_RATIO_DEF;
// PER-AXIS ENFORCEMENT, AND THE MASTER ENABLE  [notes §8]
bool limZEnforced = true, limRotEnforced = true;
bool limA1Enforced = true, limA2Enforced = true;

bool limitsEnabled = true;

bool *limEnforceFor(const String &axis) {
  if (axis == "Z")   return &limZEnforced;
  if (axis == "ROT") return &limRotEnforced;
  if (axis == "A1")  return &limA1Enforced;
  if (axis == "A2")  return &limA2Enforced;
  return NULL;
}

bool axisEnforced(const String &axis) {
  bool *on = limEnforceFor(axis);
  return on == NULL ? true : *on;
}

void armBand(int arm, double &lo, double &hi) {
  double a = (arm == 2) ? limA2Min : limA1Min;
  double b = (arm == 2) ? limA2Max : limA1Max;
  lo = (a < b) ? a : b;
  hi = (a < b) ? b : a;
}

const double LIMIT_MIN_SPAN_DEG = 1.0;
const double LIMIT_MIN_SPAN_MM  = 1.0;

const bool ARM_LIMITS_UNBOUNDED = true;

double reachFromFoldAngle(double foldDeg) {
  return ARM_RADIAL_OFFSET_MM
       - ARM_LINK_SUM_MM * cos((foldDeg + ARM_ZERO_CAD_DEG) * DEG_TO_RAD);
}
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
// ══════════════════════════════════════════════════════════════
const double MOTOR_STEPS_PER_REV  = 200.0;
const double MICROSTEPS_PER_STEP  = 16.0;
const double PULSES_PER_MOTOR_REV = MOTOR_STEPS_PER_REV * MICROSTEPS_PER_STEP;

const double ROT_GEAR_RATIO_DEF = I_RM_TOTAL;
const double ROT_GEAR_RATIO_MIN = 0.01, ROT_GEAR_RATIO_MAX = 1000.0;
double rotGearRatio = ROT_GEAR_RATIO_DEF;
double pulsesPerDegRot() { return (PULSES_PER_MOTOR_REV * rotGearRatio) / 360.0; }

// AM1/AM2 elbow gearing. This is PULSES PER MOTOR DEGREE and is exact --
// the driver's own step count, nothing derived. The gear train between
// the motor and the frog-leg link is armGearRatio, measured at 7.80.
//   [notes §13]
const double PULSES_PER_DEG_ARM_MOTOR = PULSES_PER_MOTOR_REV / 360.0;

const double Z_MICROSTEPS_PER_STEP  = 4.0;
const double PULSES_PER_MOTOR_REV_Z = MOTOR_STEPS_PER_REV * Z_MICROSTEPS_PER_STEP;

const double Z_MM_PER_REV_DEF = 20.0;
const double Z_MM_PER_REV_MIN = 0.1, Z_MM_PER_REV_MAX = 500.0;
double zMmPerRev = Z_MM_PER_REV_DEF;

double pulsesPerMmZ() { return PULSES_PER_MOTOR_REV_Z / zMmPerRev; }

const double Z_MM_PER_MOTOR_REV = Z_MM_PER_REV_DEF;

const bool INVERT_Z    = false;
const bool INVERT_ROT  = true;   // RM ran backwards on the machine: D gave CCW
const bool INVERT_ARM1 = false;
const bool INVERT_ARM2 = false;

// ══════════════════════════════════════════════════════════════
// LIMIT SENSORS — opt-in, see the header note.
// ══════════════════════════════════════════════════════════════
#define ENABLE_ROT_Z_LIMIT_SENSORS 0

#if ENABLE_ROT_Z_LIMIT_SENSORS
  #define ROT_LIMIT_CW_PIN   IO1
  #define ROT_LIMIT_CCW_PIN  IO2
  #define Z_LIMIT_UP_PIN     IO3
  #define Z_LIMIT_DOWN_PIN   IO4
  const int LIMIT_ACTIVE_STATE = HIGH;
#endif

// MOTION PROFILE — ONE UNIVERSAL RPM, ONE PERCENTAGE PER MOTOR  [notes §14]
const float MASTER_RPM_NOMINAL = 140.0f;
const float ROT_RPM_SCALE = 140.0f   / MASTER_RPM_NOMINAL;
const float Z_RPM_SCALE   = 105.0f   / MASTER_RPM_NOMINAL;

const float ARM_RPM_SCALE = 1.0f;

const float MASTER_RPM_DEF     = 150.0f;
const float MASTER_ACC_DEF     = 375.0f;
// SET ON THE MACHINE. This combination is the one that ran stably; a bench
// result, not a calculation, so do not re-derive it from anything.
const float ARM_PCT_DEF        = 62.5f;
const float ROT_PCT_DEF        = 50.0f;
const float Z_PCT_DEF          = 200.0f;

// ACCELERATION HAS ITS OWN DEFAULTS, and they are NOT the speed ones.
//
// Acceleration decides how far an axis carries on after the operator lets
// go -- coast = v^2 / 2a -- so an axis tuned for speed alone overshoots.
// The arm proved it: sharing a 125% speed figure it ramped for 0.40 s and
// coasted 225 MOTOR degrees, most of its taught band, on every release.
//
// Off the machine, with the speeds above. Keep them in step with
// DEFAULT_*_ACC_PCT in robot_sim/config.py -- python_check.py reads this
// file and fails if they drift -- and do not collapse them back onto the
// speed percentages.
const float ROT_ACC_PCT_DEF    = 100.0f;
const float ARM_ACC_PCT_DEF    = 70.0f;
const float Z_ACC_PCT_DEF      = 200.0f;

float masterRpm     = MASTER_RPM_DEF;
float masterAccRpmS = MASTER_ACC_DEF;
float rotPct        = ROT_PCT_DEF;
float armPct        = ARM_PCT_DEF;
float zPct          = Z_PCT_DEF;

float rotAccPct     = ROT_ACC_PCT_DEF;
float armAccPct     = ARM_ACC_PCT_DEF;
float zAccPct       = Z_ACC_PCT_DEF;

const float MASTER_RPM_MIN = 1.0f,   MASTER_RPM_MAX = 400.0f;
const float MASTER_ACC_MIN = 1.0f,   MASTER_ACC_MAX = 2000.0f;

const float AXIS_PCT_MIN   = 1.0f;
const float AXIS_PCT_MAX   = 1.0e6f;

float rotVelDegS = 0, rotAccDegS2 = 0;
float armVelDegS = 0, armAccDegS2 = 0;
float zVelMmS    = 0, zAccMmS2    = 0;

const float ROT_VEL_MAX = 120.0f, ROT_ACC_MAX = 400.0f;
const float Z_VEL_MAX   = 140.0f, Z_ACC_MAX   = 400.0f;
const float ARM_RPM_MAX = 400.0f, ARM_ACC_RPM_MAX = 2000.0f;
const float MOTION_MIN  = 0.05f;

bool speedClampedRot = false, speedClampedArm = false, speedClampedZ = false;

float armMotorRpmActual = 0.0f;

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

// PLC LINK — MELSEC MC PROTOCOL 3E, TCP 192.168.3.101:1025  [notes §20]
#define PLC_LINK_PLACEHOLDER 0
#define PLC_LINK_ETHERNET    1
#define PLC_LINK_DIGITAL_IO  2

#define PLC_LINK_MODE PLC_LINK_ETHERNET

// ---- PLC network endpoint (from the PLC configuration screen) ----
#define PLC_IP_0 192
#define PLC_IP_1 168
#define PLC_IP_2 3
#define PLC_IP_3 101
const uint16_t PLC_PORT = 1025;

#define CC_IP_0 192
#define CC_IP_1 168
#define CC_IP_2 3
#define CC_IP_3 200

// M30..M32 ARE THE ONLY DEVICES THIS BOARD READS.
// M1 (DONE), M5..M8 (home sensors) and M10..M13 (run) were all polled and
// reported once. They gated nothing - HOME completes on M30..M32 and the
// home-state latch is the same three bits - so reporting them only invited
// the operator to read a lamp that decides nothing. The panel showed M5..M8
// while M30 was the bit actually refusing a jog.

// TRAVEL LIMIT SWITCHES. Unlike the old M5..M8 these DO stop an axis.
// A1M has no switch fitted, so there is deliberately no PLC_M_LIMIT_A1.
//
// ZM AND A2M ARE SWAPPED FROM THE OBVIOUS ORDER, measured on the machine:
// M32 is the device that tracks ZM (covered at the bottom of the stroke,
// clearing as Z rises), and M30 is A2M's. The board originally assumed the
// tidy M30=ZM / M32=A2M order, so ZM watched M30 — which sits at 1 — and
// with its switch at the minimum end every Z_DOWN was refused wherever the
// carriage actually was. That is the jog fault chased through soft limits,
// gear ratios and poll rates for a whole session; none of those could have
// fixed it, because the bit being read was never ZM's.
const int PLC_M_LIMIT_Z   = 32;
const int PLC_M_LIMIT_ROT = 31;
const int PLC_M_LIMIT_A2  = 30;

#define PLC_POLL_DEVICE_CODE  "M*"
const long     PLC_POLL_DEVICE_NUM = 0;
const uint16_t PLC_POLL_WORDS      = 3;   // M0..M47, so M30..M32 are covered

const unsigned long PLC_HOME_TIMEOUT_MS  = 30000;

const unsigned long PLC_POLL_IDLE_DEF_MS = 20;
unsigned long plcPollIdleMs = PLC_POLL_IDLE_DEF_MS;
const unsigned long PLC_POLL_IDLE_MS   = PLC_POLL_IDLE_DEF_MS;
const unsigned long PLC_POLL_HOMING_MS = 10;
const unsigned long PLC_POLL_MS        = PLC_POLL_IDLE_MS;
const unsigned long PLC_TXN_TIMEOUT_MS     = 800;
const unsigned long PLC_CONNECT_TIMEOUT_MS = 2000;
const unsigned long PLC_RECONNECT_MS = 3000;

// COMMUNICATION DATA CODE MUST MATCH THE PLC'S OWN SETTING, EXACTLY.  [notes §25]
#define PLC_MC_ASCII 0

#if PLC_MC_ASCII
#define PLC_MC_SUBHEADER_REQ  "5000"
#define PLC_MC_SUBHEADER_RES  "D000"
#define PLC_MC_NETWORK        "00"
#define PLC_MC_PC             "FF"
#define PLC_MC_DEST_IO        "03FF"
#define PLC_MC_DEST_STATION   "00"
#define PLC_MC_MONITOR_TIMER  "0002"
#define PLC_MC_CMD_READ       "0401"
#define PLC_MC_SUB_WORD       "0000"
const int PLC_MC_RES_HEADER_UNITS = 14;
#else
const uint8_t PLC_MC_SUBHEADER_REQ_B0 = 0x50, PLC_MC_SUBHEADER_REQ_B1 = 0x00;
const uint8_t PLC_MC_SUBHEADER_RES_B0 = 0xD0, PLC_MC_SUBHEADER_RES_B1 = 0x00;
const uint8_t  PLC_MC_NETWORK_B        = 0x00;
const uint8_t  PLC_MC_PC_B             = 0xFF;
const uint8_t  PLC_MC_DEST_IO_LO       = 0xFF, PLC_MC_DEST_IO_HI = 0x03;
const uint8_t  PLC_MC_DEST_STATION_B   = 0x00;
const uint16_t PLC_MC_MONITOR_TIMER_B  = 0x0002;
const uint16_t PLC_MC_CMD_READ_B       = 0x0401;
const uint16_t PLC_MC_SUB_WORD_B       = 0x0000;
const uint8_t  PLC_MC_DEVICE_CODE_M_B  = 0x90;
const int PLC_MC_RES_HEADER_UNITS = 7;
#endif

// M30..M32 limit lamps
#define PLC_LIMIT_LED_Z_PIN    IO3
#define PLC_LIMIT_LED_ROT_PIN  IO4
#define PLC_LIMIT_LED_A2_PIN   IO5
#define PLC_LIMIT_LED_PIN_NAMES "IO-3/IO-4/IO-5"
const unsigned long PLC_LIMIT_LED_BLINK_MS = 250;

// ==============================================================
// 340 DEGREE SCAN  --  distance sensor on the arm, one sweep per layer
// ==============================================================
// A separate application (Scan/) drives this. It is in THIS firmware and
// not a second sketch on purpose: the scan sweeps RM with the same jog
// primitives an operator's key press uses, so every soft limit, every PLC
// travel switch and the E-STOP path apply to it unchanged. A second sketch
// would have had to reimplement all of that, and would have got it wrong.
//
// PINS. IO-3/4/5 are the PLC limit lamps and IO-1/IO-2 belong to the opt-in
// rotary limit sensors, so the scan takes IO-0 and, for the echo, IO-1 --
// which is free while ENABLE_ROT_Z_LIMIT_SENSORS is 0. Check that flag
// before wiring.
#define SCAN_TRIG_PIN    IO0    // ultrasonic trigger out
#define SCAN_ECHO_PIN    IO1    // ultrasonic echo in
#define SCAN_ANALOG_PIN  A9     // analog laser / IR distance in

// TWO SENSOR TYPES, chosen at runtime with SET_SCAN_SENSOR. The rig has not
// settled on ultrasonic vs laser, and re-flashing to try the other one is a
// bad way to find out which reads better on a shiny wafer edge.
enum ScanSensorKind { SCAN_SENSOR_ULTRASONIC = 0, SCAN_SENSOR_ANALOG = 1 };
int scanSensorKind = SCAN_SENSOR_ULTRASONIC;

// Ultrasonic: HC-SR04 family. 10 us trigger, echo pulse width is the round
// trip, so the distance is half of it. 30 ms of echo is about 5 m, past
// which there is nothing this machine can see and waiting costs sweep
// accuracy -- a blocking read smears the angle the sample is stamped with.
const unsigned long SCAN_TRIG_US       = 10;
const unsigned long SCAN_ECHO_TIMEOUT_US = 30000;
const double SCAN_MM_PER_US = 0.1715;        // 343 m/s, halved for the return trip

// Analog: raw counts -> mm, straight line. BOTH ZERO BY DEFAULT, so an
// uncalibrated analog sensor reads 0 mm rather than a plausible-looking
// number nobody has any reason to trust. SET_SCAN_CAL supplies them.
double scanAnalogMmPerCount = 0.0;
double scanAnalogOffsetMm   = 0.0;

// Scanning sweeps slowly. At full RM speed a 30 ms ultrasonic read happens
// over 3 degrees of travel, which is the width of the feature you are
// trying to find. A quarter of that is the point of this scale.
const float SCAN_SPEED_SCALE = 0.20f;

const double SCAN_SWEEP_DEG_DEF  = 340.0;   // the turntable's whole travel
// A SHORTER sweep is allowed -- scanning one wall is a real job, and 340 is
// the travel, not a requirement. Longer is not: the turntable cannot reach
// past its own stop, so the extra degrees would be spent grinding into the
// RM soft limit and the layer would abort mid-sweep. Refused up front
// instead, where the number can still be corrected.
const double SCAN_SWEEP_DEG_MIN  = 1.0;
const double SCAN_DEG_STEP_MIN   = 0.10;
const double SCAN_DEG_STEP_MAX   = 90.0;   // a quarter turn, the coarsest that still means anything
const double SCAN_Z_STEP_MIN_MM  = 0.10;
const int    SCAN_LAYERS_MAX     = 500;

// Positions are integer step counts divided by a pulses-per-unit figure, so
// an axis commanded to exactly 340 deg reads 339.998. Every arrival test
// here needs a tolerance or the phase never advances -- the sweep would run
// into the RM soft limit waiting for a number it cannot land on.
const double SCAN_ANGLE_EPS_DEG  = 0.05;
const double SCAN_Z_EPS_MM       = 0.02;

// SEEK finds the RM travel switch, which is the scan's reference. Every
// layer starts from it, and every other layer ends back on it.
enum ScanPhase { SCAN_OFF, SCAN_SEEK, SCAN_SWEEP, SCAN_LIFT };
ScanPhase scanPhase = SCAN_OFF;

// Far more than a turn: the switch should be found inside 360 deg, and
// anything past that means it is not going to be. Without this a miswired
// switch turns into an axis grinding against its soft limit until somebody
// notices.
const double SCAN_SEEK_MAX_DEG = 400.0;

int    scanLayer = 0;             // 1-based once running
int    scanLayers = 0;
double scanZStepMm = 0.0;
double scanDegStep = 1.0;
double scanSweepDeg = SCAN_SWEEP_DEG_DEF;
double scanStartRot = 0.0;
double scanStartZ = 0.0;
double scanNextDeg = 0.0;         // absolute RM angle the next sample is due at
double scanLayerTargetZ = 0.0;
long   scanPointsSent = 0;
int    scanSweepDir = -1;         // +1 or -1; alternates layer to layer
double scanSweepFrom = 0.0;       // the angle THIS layer started at

// -1 = axis minimum, +1 = axis maximum
// ZM sits at the BOTTOM of the stroke, so it stops Z_DOWN and Z_UP comes
// off it. RM is mounted inverted: its switch stops ROT_CW, not ROT_CCW.
const int PLC_LIMIT_END_Z   = -1;
const int PLC_LIMIT_END_ROT = +1;
const int PLC_LIMIT_END_A2  = -1;

// A2M's switch is wired at BOTH ends of its travel; the other two are not.
// One PLC device, two physical switches in parallel, so the bit alone
// cannot say which end tripped it. The DIRECTION THE AXIS WAS TRAVELLING
// when the bit went on does say, and that is what plcServiceLimitLatch()
// records: covered while driving forward is the FORWARD limit, covered
// while driving back is the BACK limit. Only that direction is refused;
// the other stays available, or the axis would be pinned on its own
// switch with no way off.
//
// PLC_LIMIT_END_* above stays the HOME-side end. It is what the latch
// falls back to when the bit comes on with the axis stopped (which is
// what sitting at HOME looks like), it is the end HOME drives toward, and
// it is the only end that may count toward the home state -- a bit that
// is on because the arm is fully EXTENDED must never be read as "at home"
// and zero the counters.
//
// KNOWN GAP, deliberate: a board that boots with the bit ALREADY on has
// no edge to latch from, so it assumes the home end. Powering up with the
// arm parked at home is the normal case and that assumption is right; if
// the machine was powered down sitting on the FAR switch it will read as
// the back end until the switch clears once. The alternative -- refusing
// to guess -- either pins the axis (both directions refused) or blocks
// HOME forever, since HOME is what would produce the edge in the first
// place.
const bool PLC_LIMIT_BOTH_ENDS_Z   = false;
const bool PLC_LIMIT_BOTH_ENDS_ROT = false;
const bool PLC_LIMIT_BOTH_ENDS_A2  = true;

#if PLC_LINK_MODE == PLC_LINK_DIGITAL_IO
  #define PLC_HOME_DONE_PIN DI6
#endif

#if PLC_LINK_MODE == PLC_LINK_ETHERNET
  #include <Ethernet.h>
  byte          plcMac[]  = {0x24, 0x15, 0x10, 0xB0, 0x00, 0x01};
  IPAddress     plcLocalIp(CC_IP_0, CC_IP_1, CC_IP_2, CC_IP_3);
  IPAddress     plcTargetIp(PLC_IP_0, PLC_IP_1, PLC_IP_2, PLC_IP_3);
  EthernetClient plcClient;
  bool          plcReportedError = false;
  unsigned long plcLastConnectTry = 0;
  unsigned long plcLastConnectLog = 0;
#endif

#if PLC_LINK_MODE == PLC_LINK_PLACEHOLDER
  const unsigned long PLC_SIM_DONE_MS = 1200;
#endif

const unsigned long ALIVE_INTERVAL_MS       = 2000;

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
RunPhase runPhase = PHASE_NONE;
float runStartD1 = 0, runStartRot = 0, runStartA1 = 0, runStartA2 = 0;
float runTargetD1 = 0, runTargetRot = 0, runTargetA1 = 0, runTargetA2 = 0;
unsigned long lastRunReportTime = 0;

int rotDir = 0, a1Dir = 0, a2Dir = 0, jzDir = 0;
unsigned long lastJogReportTime = 0;

bool isHoming = false;
// Which axes HOME is still driving. Declared here, not beside
// beginHoming(), because cancelHoming() above needs it too.
bool homeAxisActive[3] = {false, false, false};
// Set when HOME starts with a both-ends switch already tripped at its FAR
// end. That axis has to drive OFF the far switch before the bit means
// "arrived" again, or HOME finishes instantly at the wrong end.
bool homeWaitForClear[3] = {false, false, false};
unsigned long lastHomeReportTime = 0;
unsigned long homeRequestedAt = 0;
HomeState homeState = HOME_IDLE;
bool isHomed = false;
unsigned long lastAliveTime = 0;

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


// INVERSE / FORWARD KINEMATICS  [notes §29]

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

  double d1 = Z - zOffsetForArm(arm);
  if (axisEnforced("Z") && (d1 < limD1Min - 1e-6 || d1 > limD1Max + 1e-6)) {
    r.error = "[ERROR] Z=" + String(d1, 2) + " from HOME is out of ZM travel "
              "(allowed " + String(limD1Min, 1) + ".." + String(limD1Max, 1)
            + " mm above HOME; Z is never negative). That would put arm "
            + String(arm) + "'s deck at an absolute " + String(Z, 2) + " mm";
    return r;
  }

  double R   = sqrt(X * X + Y * Y);
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

  if (R < ARM_RADIAL_OFFSET_MM - ARM_LINK_SUM_MM - 1e-6
      || R > ARM_RADIAL_OFFSET_MM + ARM_LINK_SUM_MM + 1e-6) {
    r.error = "[ERROR] R=" + String(R, 2) + " mm has no solution: the frog-leg "
              "spans a3+a6 +/- (a4+a5) = " + String(ARM_RADIAL_OFFSET_MM, 1)
            + " +/- " + String(ARM_LINK_SUM_MM, 1) + " mm";
    return r;
  }

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
// Motor RPM -> pulses per second.
int32_t rpmToPulsesPerSec(float rpm) {
  return (int32_t)lround((double)rpm / 60.0 * PULSES_PER_MOTOR_REV);
}

float clampReport(float v, float hi, bool &flag) {
  if (v > hi) { flag = true; return hi; }
  return v;
}

void applyMotionParams() {
  speedClampedRot = speedClampedArm = speedClampedZ = false;

  float rotMotorRpm = masterRpm * (rotPct / 100.0f) * ROT_RPM_SCALE;
  float armMotorRpm = masterRpm * (armPct / 100.0f) * ARM_RPM_SCALE;
  float zMotorRpm   = masterRpm * (zPct   / 100.0f) * Z_RPM_SCALE;

  float rotMotorAcc = masterAccRpmS * (rotAccPct / 100.0f) * ROT_RPM_SCALE;
  float armMotorAcc = masterAccRpmS * (armAccPct / 100.0f) * ARM_RPM_SCALE;
  float zMotorAcc   = masterAccRpmS * (zAccPct   / 100.0f) * Z_RPM_SCALE;

  armMotorRpm = clampReport(armMotorRpm, ARM_RPM_MAX,     speedClampedArm);
  armMotorAcc = clampReport(armMotorAcc, ARM_ACC_RPM_MAX, speedClampedArm);

  rotVelDegS  = rotMotorRpm * 360.0f / (60.0f * (float)rotGearRatio);
  rotAccDegS2 = rotMotorAcc * 360.0f / (60.0f * (float)rotGearRatio);
  armVelDegS  = armMotorRpm * 360.0f / 60.0f;
  armAccDegS2 = armMotorAcc * 360.0f / 60.0f;
  zVelMmS     = zMotorRpm   * (float)zMmPerRev / 60.0f;
  zAccMmS2    = zMotorAcc   * (float)zMmPerRev / 60.0f;

  rotVelDegS  = clampReport(rotVelDegS,  ROT_VEL_MAX, speedClampedRot);
  rotAccDegS2 = clampReport(rotAccDegS2, ROT_ACC_MAX, speedClampedRot);
  zVelMmS     = clampReport(zVelMmS,     Z_VEL_MAX,   speedClampedZ);
  zAccMmS2    = clampReport(zAccMmS2,    Z_ACC_MAX,   speedClampedZ);

  armMotorRpmActual = armMotorRpm;

  rotVelPulses   = (int32_t)lround(rotVelDegS  * pulsesPerDegRot());
  rotAccelPulses = (int32_t)lround(rotAccDegS2 * pulsesPerDegRot());
  armVelPulses   = (int32_t)lround(armVelDegS  * PULSES_PER_DEG_ARM_MOTOR);
  armAccelPulses = (int32_t)lround(armAccDegS2 * PULSES_PER_DEG_ARM_MOTOR);
  zVelPulses     = (int32_t)lround(zVelMmS     * pulsesPerMmZ());
  zAccelPulses   = (int32_t)lround(zAccMmS2    * pulsesPerMmZ());

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

float currentD1()  { return (float)(MOTOR_Z.PositionRefCommanded()   / pulsesPerMmZ())  * (INVERT_Z    ? -1 : 1); }
float currentRot() { return (float)(MOTOR_ROT.PositionRefCommanded() / pulsesPerDegRot()) * (INVERT_ROT ? -1 : 1); }
float currentA1()  { return (float)(MOTOR_A1.PositionRefCommanded()  / PULSES_PER_DEG_ARM_MOTOR) * (INVERT_ARM1 ? -1 : 1); }
float currentA2()  { return (float)(MOTOR_A2.PositionRefCommanded()  / PULSES_PER_DEG_ARM_MOTOR) * (INVERT_ARM2 ? -1 : 1); }
float currentA1Fold() { return (float)armFoldFromMotor(currentA1()) + FOLD_ANGLE_HOME_DEG; }
float currentA2Fold() { return (float)armFoldFromMotor(currentA2()) + FOLD_ANGLE_HOME_DEG; }

void moveJointsAbsolute(float d1, float rot, float a1, float a2) {
  int32_t zPulses   = (int32_t)lround(d1  * pulsesPerMmZ())  * (INVERT_Z    ? -1 : 1);
  int32_t rotPulses = (int32_t)lround(rot * pulsesPerDegRot()) * (INVERT_ROT ? -1 : 1);
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


// OPERATOR LIMIT EDITING  [notes §36]
bool applyLimit(const String &axis, bool isMax, double value, String &why) {
  double *lo, *hi, floorV = 0, ceilV = 0, minSpan = 0;
  bool taught = false;
  String unit;

  if      (axis == "Z")   { lo=&limD1Min;  hi=&limD1Max;  floorV=D1_MIN_MM;
                            ceilV=D1_MAX_MM; minSpan=LIMIT_MIN_SPAN_MM;  unit=" mm"; }
  else if (axis == "ROT") { lo=&limRotMin; hi=&limRotMax; floorV=ROT_MIN_DEG;
                            ceilV=ROT_MAX_DEG; minSpan=LIMIT_MIN_SPAN_DEG; unit=" deg"; }
  // ARM_LIMITS_UNBOUNDED for why.
  else if (axis == "A1")  { lo=&limA1Min;  hi=&limA1Max;  taught=true; unit=" deg"; }
  else if (axis == "A2")  { lo=&limA2Min;  hi=&limA2Max;  taught=true; unit=" deg"; }
  else { why = "axis must be Z, ROT, A1 or A2 — got \"" + axis + "\""; return false; }

  if (taught) {
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


// REPORTING  [notes §38]
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
  sendFeedback("[JOG POS] ROT: " + String(currentRot(), 2) + " deg | A1M: "
             + String(currentA1(), 2) + " deg | A2M: " + String(currentA2(), 2)
             + " deg | Z: " + String(currentD1(), 2) + " mm"
             + " | FOLD1: " + String(currentA1Fold(), 2)
             + " deg | FOLD2: " + String(currentA2Fold(), 2)
             + " deg | R1: " + String(reachFromFoldAngle(currentA1Fold()), 1)
             + " mm | R2: " + String(reachFromFoldAngle(currentA2Fold()), 1) + " mm");
}

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
  sendFeedback(String("[LIMIT_ENFORCE] master=") + (limitsEnabled ? "yes" : "NO")
             + " | enforced: Z=" + String(limZEnforced ? 1 : 0)
             + " ROT=" + String(limRotEnforced ? 1 : 0)
             + " A1=" + String(limA1Enforced ? 1 : 0)
             + " A2=" + String(limA2Enforced ? 1 : 0));
}


// ══════════════════════════════════════════════════════════════
// MOTION CANCELLATION
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

// HOME drives the motors itself now, so cancelling has to STOP them. When
// it only cleared the flag, an interrupted home left the axes running at
// half speed toward their switches with nothing watching for arrival.
void cancelHoming() {
  isHoming = false;
  for (int i = 0; i < 3; i++) { homeAxisActive[i] = false; homeWaitForClear[i] = false; }
  jzDir = rotDir = a2Dir = 0;
  applyJogVelocities();
}


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


// EVERY Z ON THE WIRE IS MEASURED FROM HOME.  [notes §39]
IkResult solveIkFromHome(int arm, double X, double Y, double zFromHome) {
  int a = (arm == 2) ? 2 : 1;
  return solveIkFrogleg(arm, X, Y, zFromHome + zOffsetForArm(a));
}

bool ikToJoints(int arm, double X, double Y, double Z,
                float &d1, float &rot, float &a1, float &a2) {
  IkResult r = solveIkFromHome(arm, X, Y, Z);
  if (!r.ok) { sendFeedback(r.error); return false; }

  d1  = (float)r.d1;
  rot = (float)r.th2;
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
  runPhase = PHASE_DUAL;
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
    runPhase = PHASE_TO_HOME_FIRST;
    runTargetD1 = Z_HOME_MM_BOARD; runTargetRot = ROT_HOME_DEG_BOARD;
    runTargetA1 = ARM_HOME_MOTOR_DEG; runTargetA2 = ARM_HOME_MOTOR_DEG;
    sendFeedback("[RUN] Leg 1/4 — returning to HOME before Point A...");
  }
  String whyLimit;
  if (runLegBlockedByLimit(runTargetD1, runTargetRot, runTargetA2, whyLimit)) {
    runPhase = PHASE_NONE;
    sendFeedback("[ERROR] RUN refused — " + whyLimit + ".");
    sendFeedback("[WARN] Jog that axis off its limit, then RUN again.");
    return;
  }
  moveJointsAbsolute(runTargetD1, runTargetRot, runTargetA1, runTargetA2);
  isMoving = true;
  lastRunReportTime = millis();
}

void beginRunLeg(RunPhase phase, float d1, float rot, float a1, float a2,
                 bool skipSensorBlock = false) {
  if (!skipSensorBlock) {
    String whyLimit;
    if (runLegBlockedByLimit(d1, rot, a2, whyLimit)) {
      isMoving = false;
      runPhase = PHASE_NONE;
      sendFeedback("[ERROR] RUN stopped — " + whyLimit + ".");
      sendFeedback("[WARN] Jog that axis off its limit, then RUN again.");
      return;
    }
  }
  runPhase = phase;
  runStartD1 = currentD1(); runStartRot = currentRot();
  runStartA1 = currentA1(); runStartA2 = currentA2();
  runTargetD1 = d1; runTargetRot = rot; runTargetA1 = a1; runTargetA2 = a2;
  moveJointsAbsolute(runTargetD1, runTargetRot, runTargetA1, runTargetA2);
}

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
              ARM_HOME_MOTOR_DEG, ARM_HOME_MOTOR_DEG, true);
}

int runProgressPercent() {
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
    sendFeedback("[RESET_POSITION] TARGET REACHED");
    return;
  }
  sendFeedback("[RUN] TARGET REACHED");
}


// ══════════════════════════════════════════════════════════════
// JOG
// ══════════════════════════════════════════════════════════════
// Homing drives the axes onto their own switches at a QUARTER jog speed.
// The switch state arrives over Ethernet, polled every PLC_POLL_HOMING_MS,
// so the axis keeps moving for up to one poll after the switch closes —
// at speed, momentum carries it past the switch before MoveVelocity(0)
// actually brakes it. Slow speed shrinks both the poll-latency overshoot
// and the stopping distance itself; it is not a comfort setting.
const float HOME_SPEED_SCALE = 0.25f;

// WHICH WAY HOME DRIVES, per axis. Deliberately NOT PLC_LIMIT_END_*.
//
// Those two look like one fact and are not: PLC_LIMIT_END_* says which
// direction a covered switch REFUSES, this says which direction HOME
// travels to go and find it. They agreed until the device mapping turned
// out to be wrong, and sharing one constant meant a wrong end sent HOME the
// wrong way with no separate place to correct it. Both must still point at
// the SAME physical switch, so HOME_DIR_* == PLC_LIMIT_END_* here, axis by
// axis: ZM down and A2M retract are negative, but RM is mounted inverted
// (see PLC_LIMIT_END_ROT) and its switch sits at the +1/CW end, not -1.
const int HOME_DIR_Z   = -1;
const int HOME_DIR_ROT = +1;
const int HOME_DIR_A2  = -1;

int homeDirFor(int i) {
  const int dirs[3] = {HOME_DIR_Z, HOME_DIR_ROT, HOME_DIR_A2};
  return dirs[i];
}

void applyJogVelocities() {
  float scale = isHoming ? HOME_SPEED_SCALE
              : (scanPhase != SCAN_OFF ? SCAN_SPEED_SCALE : 1.0f);
  int32_t rotV = (int32_t)(rotVelPulses * boostMultiplier * scale);
  int32_t armV = (int32_t)(armVelPulses * boostMultiplier * scale);
  int32_t zV   = (int32_t)(zVelPulses   * boostMultiplier * scale);

  MOTOR_ROT.MoveVelocity(rotDir * rotV * (INVERT_ROT ? -1 : 1));
  MOTOR_A1.MoveVelocity(a1Dir * armV * (INVERT_ARM1 ? -1 : 1));
  MOTOR_A2.MoveVelocity(a2Dir * armV * (INVERT_ARM2 ? -1 : 1));
  MOTOR_Z.MoveVelocity(jzDir * zV * (INVERT_Z ? -1 : 1));
}

bool anyJogActive() { return rotDir || a1Dir || a2Dir || jzDir; }

// ── Soft limits are only meaningful once the machine has a reference ──  [notes §42]
bool softLimitsActive() { return limitsEnabled; }

bool axisLimited(const String &axis) {
  return softLimitsActive() && axisEnforced(axis);
}

// HOME is the minimum of every axis, so a FACTORY-DEFAULT floor sits at 0.
// Without a reference the counter reads 0 wherever the board powered up,
// not at the bottom of travel, so the axis sits exactly ON that floor and
// every inward jog is refused - pinned, with nothing to escape from. The
// far end cannot collide with the counter origin, so it still applies.
//
// Only an UNTOUCHED default is relaxed. A taught floor applies with or
// without a reference: it was captured against the same counters it is
// compared with. Mirrors _axis_bounds() in the GUI; the two must agree.
bool axisFloorIsDefault(const String &axis) {
  if (axis == "Z")   return limD1Min  == D1_MIN_MM;
  if (axis == "ROT") return limRotMin == ROT_MIN_DEG;
  if (axis == "A1")  return limA1Min  == armMotorFromFold(FOLD_ANGLE_MIN_DEG);
  if (axis == "A2")  return limA2Min  == armMotorFromFold(FOLD_ANGLE_MIN_DEG);
  return false;
}

bool axisLowerLimited(const String &axis) {
  if (!axisLimited(axis)) return false;
  return isHomed || !axisFloorIsDefault(axis);
}

void warnUnreferencedOnce() {
  static bool warned = false;
  if (warned || isHomed) return;
  warned = true;
  sendFeedback("[WARN] No reference yet. Your taught boundaries ARE being applied "
               "against the current counters, so jog is protected — but the "
               "reported positions are relative to wherever this board powered "
               "up. Run HOME, or RESET_COORD, before commanding absolute moves.");
}

void serviceArmSoftLimit(int &dir, float angle, int whichArm) {
  if (!axisLimited(whichArm == 1 ? "A1" : "A2")) return;
  double loLim, hiLim; armBand(whichArm, loLim, hiLim);

  bool atMax = (dir > 0 && angle >= hiLim);
  bool atMin = (dir < 0 && angle <= loLim
                && axisLowerLimited(whichArm == 1 ? "A1" : "A2"));
  if (!atMax && !atMin) return;

  dir = 0;
  if (whichArm == 1) MOTOR_A1.MoveVelocity(0);
  else               MOTOR_A2.MoveVelocity(0);

  String axis = String(whichArm == 1 ? "A1" : "A2") + (atMax ? "_FWD" : "_BACK");
  sendFeedback("[LIMIT] " + axis + " — th3 at "
             + String(atMax ? hiLim : loLim, 2)
             + (atMax ? " deg (upper limit)" : " deg (lower limit)"));
}

void serviceJogSoftLimits() {
  if (anyJogActive()) warnUnreferencedOnce();

  serviceArmSoftLimit(a1Dir, currentA1(), 1);
  serviceArmSoftLimit(a2Dir, currentA2(), 2);

  if (axisLimited("Z")) {
    if (jzDir > 0 && currentD1() >= limD1Max) {
      jzDir = 0; MOTOR_Z.MoveVelocity(0);
      sendFeedback("[LIMIT] Z_UP");
    }
    if (jzDir < 0 && currentD1() <= limD1Min && axisLowerLimited("Z")) {
      jzDir = 0; MOTOR_Z.MoveVelocity(0);
      sendFeedback("[LIMIT] Z_DOWN");
    }
  }
  if (axisLimited("ROT")) {
    if (rotDir > 0 && currentRot() >= limRotMax) {
      rotDir = 0; MOTOR_ROT.MoveVelocity(0);
      sendFeedback("[LIMIT] ROT_CW");
    }
    if (rotDir < 0 && currentRot() <= limRotMin && axisLowerLimited("ROT")) {
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

void serviceJogWatchdog() {
#if ENABLE_JOG_WATCHDOG
  if (!anyJogActive()) return;
  // HOME drives the same direction variables a jog does, but it is NOT a
  // jog: no host is holding a button, so no keep-alive arrives and this
  // watchdog cancelled the move 700 ms in. That is why HOME looked like it
  // did nothing at all. HOME has its own timeout and its own stop
  // condition (each axis's switch), so leave it alone.
  if (isHoming) return;
  // SCAN drives rotDir the same way, but no host jog client is holding a
  // key -- it stops itself, on the switch or the sweep count or SCAN_STOP.
  // Un-exempted, every scan died here at 700 ms and looked like a PLC
  // switch fault.
  if (scanPhase != SCAN_OFF) return;
  if (millis() - lastJogKeepAlive < JOG_WATCHDOG_MS) return;
  cancelJog();
  sendFeedback("[WATCHDOG] Jog stopped — no keep-alive from host for "
             + String((int)JOG_WATCHDOG_MS) + " ms.");
#endif
}

void startJog(int &axisDir, int dir) {
  if (isMoving)  { cancelRun();    sendFeedback("[WARN] RUN canceled by jog command."); }
  if (isHoming)  { cancelHoming();
                   sendFeedback("[WARN] Homing canceled by jog command."); }
  axisDir = dir;
  lastJogKeepAlive = millis();
  applyJogVelocities();
}

void startArmJogLinked(int dir) {
  if (isMoving)  { cancelRun();    sendFeedback("[WARN] RUN canceled by jog command."); }
  if (isHoming)  { cancelHoming();
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


// PLC TRANSPORT — MC PROTOCOL 3E  [notes §44]

// ---- Polled state, shared by every mode ----
// Three words, M0..M47. One word was enough while every device lived in
// M0..M15; the limit bits are M30..M32, which straddle the second and
// third word, so the poll has to cover all three or they are invisible.
const int PLC_STATUS_WORDS = 3;
uint16_t      plcStatusWords[PLC_STATUS_WORDS] = {0, 0, 0};
uint16_t      plcStatusWord   = 0;   // M0..M15, kept for the existing readouts
bool          plcStatusValid  = false;
unsigned long plcLastPollOk   = 0;
unsigned long plcLastPollSent = 0;
bool          plcLinkUp       = false;
bool          plcLinkEnabled  = true;
bool plcBit(int number) {
  if (number < 0 || number >= PLC_STATUS_WORDS * 16) return false;
  return (plcStatusWords[number / 16] >> (number % 16)) & 1;
}

String plcStatusSummary() {
  if (!plcStatusValid) {
    return String("NO DEVICE DATA | limit Z/R/A2=??? end Z/R/A2=???");
  }
  // M30..M32, the only bits read, and the ones that stop an axis.
  String s = "limit Z/R/A2=" + String(plcBit(PLC_M_LIMIT_Z) ? 1 : 0)
     + String(plcBit(PLC_M_LIMIT_ROT) ? 1 : 0)
     + String(plcBit(PLC_M_LIMIT_A2) ? 1 : 0);
  // Per-sensor boundary switch — SET_PLC_SENSOR_ENFORCE. A 0 here means
  // that bit above is cosmetic: it no longer stops the axis or counts
  // toward HOME.
  // Which end each switch is refusing right now. Fixed for ZM and RM;
  // A2M's follows the latch, and the GUI cannot work it out on its own.
  s += " end Z/R/A2=";
  for (int i = 0; i < 3; i++) s += (plcLimitEndFor(i) > 0) ? "+" : "-";
  s += " enforce Z/R/A2=" + String(plcLimitSensorEnabled[0] ? 1 : 0)
     + String(plcLimitSensorEnabled[1] ? 1 : 0)
     + String(plcLimitSensorEnabled[2] ? 1 : 0);
  return s;
}

String plcHex(unsigned long value, int width) {
  static const char digits[] = "0123456789ABCDEF";
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

#if PLC_MC_ASCII
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

String plcBuildFrame(const String &body) {
  String payload = String(PLC_MC_MONITOR_TIMER) + body;
  return String(PLC_MC_SUBHEADER_REQ) + PLC_MC_NETWORK + PLC_MC_PC
       + PLC_MC_DEST_IO + PLC_MC_DEST_STATION
       + plcHex((unsigned long)payload.length(), 4) + payload;
}

String plcFrameReadWords(const char *deviceCode, long deviceNum,
                         bool hexNumbering, uint16_t words) {
  return plcBuildFrame(String(PLC_MC_CMD_READ) + PLC_MC_SUB_WORD
                     + deviceCode + plcDeviceNum(deviceNum, hexNumbering)
                     + plcHex(words, 4));
}
#else
uint8_t plcByteAt(const String &s, int i) { return (uint8_t)s.charAt(i); }
uint16_t plcU16At(const String &s, int i) {
  return (uint16_t)plcByteAt(s, i) | ((uint16_t)plcByteAt(s, i + 1) << 8);
}
uint16_t plcU16AtBytes(const uint8_t *b, int i) {
  return (uint16_t)b[i] | ((uint16_t)b[i + 1] << 8);
}
String plcHexDumpBytes(const uint8_t *buf, int len) {
  String out;
  for (int i = 0; i < len; i++) {
    if (i) out += ' ';
    out += plcHex(buf[i], 2);
  }
  return out;
}

// Frames are built in a raw byte buffer, never an Arduino String: the
// request is half NUL bytes (subcommand, device number) and String is
// NUL-terminated, so a String-built frame can be truncated on the wire.
void plcBuildReadFrameBin(uint8_t *buf, int &len, uint8_t deviceCode,
                          uint32_t deviceNum, uint16_t numWords) {
  const uint16_t dataLen = 12;
  len = 0;
  buf[len++] = PLC_MC_SUBHEADER_REQ_B0;
  buf[len++] = PLC_MC_SUBHEADER_REQ_B1;
  buf[len++] = PLC_MC_NETWORK_B;
  buf[len++] = PLC_MC_PC_B;
  buf[len++] = PLC_MC_DEST_IO_LO;
  buf[len++] = PLC_MC_DEST_IO_HI;
  buf[len++] = PLC_MC_DEST_STATION_B;
  buf[len++] = (uint8_t)(dataLen & 0xFF);
  buf[len++] = (uint8_t)((dataLen >> 8) & 0xFF);
  buf[len++] = (uint8_t)(PLC_MC_MONITOR_TIMER_B & 0xFF);
  buf[len++] = (uint8_t)((PLC_MC_MONITOR_TIMER_B >> 8) & 0xFF);
  buf[len++] = (uint8_t)(PLC_MC_CMD_READ_B & 0xFF);
  buf[len++] = (uint8_t)((PLC_MC_CMD_READ_B >> 8) & 0xFF);
  buf[len++] = (uint8_t)(PLC_MC_SUB_WORD_B & 0xFF);
  buf[len++] = (uint8_t)((PLC_MC_SUB_WORD_B >> 8) & 0xFF);
  buf[len++] = (uint8_t)(deviceNum & 0xFF);
  buf[len++] = (uint8_t)((deviceNum >> 8) & 0xFF);
  buf[len++] = (uint8_t)((deviceNum >> 16) & 0xFF);
  buf[len++] = deviceCode;
  buf[len++] = (uint8_t)(numWords & 0xFF);
  buf[len++] = (uint8_t)((numWords >> 8) & 0xFF);
}
#endif

#if PLC_MC_ASCII
String plcBuildPollFrame() {
  return plcFrameReadWords(PLC_POLL_DEVICE_CODE, PLC_POLL_DEVICE_NUM,
                           false, PLC_POLL_WORDS);
}
#else
uint8_t plcTxBytes[64];
int     plcTxCount = 0;
void plcBuildPollFrame() {
  plcBuildReadFrameBin(plcTxBytes, plcTxCount, PLC_MC_DEVICE_CODE_M_B,
                       (uint32_t)PLC_POLL_DEVICE_NUM, PLC_POLL_WORDS);
}
#endif

// *** THERE IS NO WRITE FRAME BUILDER, ON PURPOSE ***  [notes §48]

#if PLC_LINK_MODE == PLC_LINK_ETHERNET
String        plcRxBuf;
const int     PLC_RX_CAP = 256;
uint8_t       plcRxBytes[PLC_RX_CAP];
int           plcRxCount = 0;
bool          plcTxnActive   = false;
unsigned long plcTxnSentAt   = 0;
unsigned long plcTxnTimeouts   = 0;
unsigned long plcGoodReads     = 0;
unsigned long plcSendAttempts  = 0;
unsigned long plcConnectTries  = 0;
unsigned long plcConnectFails  = 0;
unsigned long plcConnectsOk    = 0;
bool plcDebug = false;

// LINK STATE IS ABOUT DATA, NOT ABOUT THE SOCKET  [notes §50]
unsigned long plcDataStaleMs() {
  unsigned long interval = isHoming ? PLC_POLL_HOMING_MS : plcPollIdleMs;
  return interval * 3 + 1000;
}

const char *plcDataState() {
  if (plcGoodReads == 0) return "NONE";
  if (millis() - plcLastPollOk > plcDataStaleMs()) return "STALE";
  return "OK";
}

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
  plcRxCount = 0;
  plcConnectTries++;
  if (plcClient.connect(plcTargetIp, PLC_PORT)) {
    plcReportedError = false;
    plcLinkUp = true;
    plcConnectsOk++;
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

// Send the frame as RAW BYTES with an explicit length. Never print().
void plcWriteFrame(const String &frame) {
  plcClient.write((const uint8_t *)frame.c_str(), frame.length());
}

bool plcSendPoll() {
  if (!plcEnsureConnected()) return false;
  plcSendAttempts++;
#if PLC_MC_ASCII
  String frame = plcBuildPollFrame();
  if (plcDebug) sendFeedback("[PLC_TX] " + frame);
  plcWriteFrame(frame);
#else
  plcBuildPollFrame();
  if (plcDebug) sendFeedback("[PLC_TX] " + plcHexDumpBytes(plcTxBytes, plcTxCount));
  plcClient.write(plcTxBytes, plcTxCount);
#endif
  plcClient.flush();
  plcRxBuf = "";
  plcRxCount = 0;
  plcTxnActive = true;
  plcTxnSentAt = millis();
  return true;
}

void plcOnGoodRead(const uint16_t *words, int count) {
  bool first = !plcStatusValid;
  uint16_t previous[PLC_STATUS_WORDS];
  for (int i = 0; i < PLC_STATUS_WORDS; i++) previous[i] = plcStatusWords[i];

  bool changed = false;
  for (int i = 0; i < PLC_STATUS_WORDS; i++) {
    uint16_t v = (i < count) ? words[i] : 0;
    if (plcStatusWords[i] != v) changed = true;
    plcStatusWords[i] = v;
  }
  plcStatusWord  = plcStatusWords[0];
  plcStatusValid = true;
  plcLastPollOk  = millis();
  plcGoodReads++;
  (void)previous;

  if (first || changed) {
    sendFeedback("[PLC_STATE] link=UP socket=OPEN data=" + String(plcDataState())
               + " conn=" + String((unsigned long)plcConnectsOk) + "/"
               + String((unsigned long)plcConnectTries)
               + " word=" + plcHex(plcStatusWords[0], 4)
               + " w1=" + plcHex(plcStatusWords[1], 4)
               + " w2=" + plcHex(plcStatusWords[2], 4)
               + " timeouts=" + String((unsigned long)plcTxnTimeouts)
               + " | " + plcStatusSummary());
  }
}

#if PLC_MC_ASCII
bool plcConsumeResponse() {
  if (plcRxBuf.length() < PLC_MC_RES_HEADER_UNITS + 4) return false;
  long dataLen = plcParseHex(plcRxBuf, PLC_MC_RES_HEADER_UNITS, 4);
  if (dataLen < 0) {
    sendFeedback("[ERROR] PLC sent a malformed response length — dropping the "
                 "socket and resynchronising.");
    plcClient.stop();
    return true;
  }
  int total = PLC_MC_RES_HEADER_UNITS + 4 + (int)dataLen;
  if (plcRxBuf.length() < total) return false;

  String frame = plcRxBuf.substring(0, total);
  plcRxBuf = plcRxBuf.substring(total);

  if (!frame.startsWith(String(PLC_MC_SUBHEADER_RES))) {
    sendFeedback("[ERROR] PLC response subheader was \"" + frame.substring(0, 4)
               + "\", expected " PLC_MC_SUBHEADER_RES " — the port is probably not "
                 "speaking MC protocol 3E ASCII.");
    return true;
  }

  long endCode = plcParseHex(frame, PLC_MC_RES_HEADER_UNITS + 4, 4);
  if (endCode != 0) {
    sendFeedback("[ERROR] PLC end code " + plcHex((unsigned long)endCode, 4)
               + " — the read was refused. Check that M0 exists and that MC "
                 "protocol is enabled on the port.");
    return true;
  }

  int avail = ((int)dataLen - 4) / 4;          // words after the end code
  if (avail > PLC_STATUS_WORDS) avail = PLC_STATUS_WORDS;
  uint16_t words[PLC_STATUS_WORDS] = {0, 0, 0};
  for (int i = 0; i < avail; i++) {
    long w = plcParseHex(frame, PLC_MC_RES_HEADER_UNITS + 8 + i * 4, 4);
    if (w < 0) {
      sendFeedback("[ERROR] PLC returned unreadable device data.");
      return true;
    }
    words[i] = (uint16_t)w;
  }
  plcOnGoodRead(words, avail);
  return true;
}
#else
bool plcConsumeResponse() {
  if (plcRxCount < PLC_MC_RES_HEADER_UNITS + 4) return false;
  int dataLen = (int)plcU16AtBytes(plcRxBytes, PLC_MC_RES_HEADER_UNITS);
  int total = PLC_MC_RES_HEADER_UNITS + 2 + dataLen;
  if (plcRxCount < total) return false;

  bool bad = (plcRxBytes[0] != PLC_MC_SUBHEADER_RES_B0
           || plcRxBytes[1] != PLC_MC_SUBHEADER_RES_B1);
  uint16_t endCode = plcU16AtBytes(plcRxBytes, PLC_MC_RES_HEADER_UNITS + 2);

  int avail = (dataLen - 2) / 2;               // words after the end code
  if (avail > PLC_STATUS_WORDS) avail = PLC_STATUS_WORDS;
  uint16_t words[PLC_STATUS_WORDS] = {0, 0, 0};
  for (int i = 0; i < avail; i++) {
    words[i] = plcU16AtBytes(plcRxBytes, PLC_MC_RES_HEADER_UNITS + 4 + i * 2);
  }

  int leftover = plcRxCount - total;
  for (int i = 0; i < leftover; i++) plcRxBytes[i] = plcRxBytes[total + i];
  plcRxCount = leftover;

  if (bad) {
    sendFeedback("[ERROR] PLC response subheader was not D0 00 — the port is "
                 "probably not speaking MC protocol 3E BINARY.");
    return true;
  }
  if (endCode != 0) {
    sendFeedback("[ERROR] PLC end code " + plcHex((unsigned long)endCode, 4)
               + " — the read was refused. Check that M0 exists and that MC "
                 "protocol is enabled on the port.");
    return true;
  }
  if (avail <= 0) {
    sendFeedback("[ERROR] PLC returned no device words (data length "
               + String(dataLen) + ").");
    return true;
  }
  plcOnGoodRead(words, avail);
  return true;
}
#endif

void plcServiceRx() {
  bool got = false;
  while (plcClient.available() > 0) {
    uint8_t v = (uint8_t)plcClient.read();
#if PLC_MC_ASCII
    if (plcRxBuf.length() < 200) plcRxBuf += (char)v;
#else
    if (plcRxCount < PLC_RX_CAP) plcRxBytes[plcRxCount++] = v;
#endif
    got = true;
  }
  if (got && plcDebug) {
#if PLC_MC_ASCII
    sendFeedback("[PLC_RX] " + plcRxBuf);
#else
    sendFeedback("[PLC_RX] " + plcHexDumpBytes(plcRxBytes, plcRxCount));
#endif
  }
  if (!plcTxnActive) { plcRxBuf = ""; plcRxCount = 0; return; }

  if (plcConsumeResponse()) { plcTxnActive = false; return; }

  if (millis() - plcTxnSentAt >= PLC_TXN_TIMEOUT_MS) {
    plcTxnActive = false;
    plcRxBuf = "";
    plcRxCount = 0;
    plcTxnTimeouts++;
    plcClient.stop();
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

#if PLC_MC_ASCII
void plcTestReport(const String &raw) {
  if (raw.length() == 0) {
    sendFeedback("[PLC_TEST] RX nothing. The socket is open but the PLC did not "
                 "answer a device read — this is almost always MC protocol not "
                 "enabled on that port, or the Communication Data Code set to "
                 "BINARY while this board speaks ASCII.");
    return;
  }
  sendFeedback("[PLC_TEST] RX " + raw);
  if (!raw.startsWith(String(PLC_MC_SUBHEADER_RES))) {
    sendFeedback("[PLC_TEST] Subheader is not " PLC_MC_SUBHEADER_RES
                 " — the port is answering, but not with MC protocol 3E ASCII.");
    return;
  }
  long endCode = plcParseHex(raw, PLC_MC_RES_HEADER_UNITS + 4, 4);
  if (endCode != 0) {
    sendFeedback("[PLC_TEST] End code " + plcHex((unsigned long)endCode, 4)
               + " — the PLC refused the read. Check that M0..M15 exist and that "
                 "the module permits reads.");
    return;
  }
  long w = plcParseHex(raw, PLC_MC_RES_HEADER_UNITS + 8, 4);
  plcStatusWord = (uint16_t)w;
  plcStatusWords[0] = plcStatusWord;
  plcStatusValid = true;
  plcLastPollOk = millis();
  sendFeedback("[PLC_TEST] OK — M0..M15 = " + plcHex((unsigned long)plcStatusWord, 4)
             + " | " + plcStatusSummary());
}
#else
void plcTestReport(const uint8_t *raw, int rawLen) {
  if (rawLen == 0) {
    sendFeedback("[PLC_TEST] RX nothing. The socket is open but the PLC did not "
                 "answer a device read — this is almost always MC protocol not "
                 "enabled on that port, or the Communication Data Code set to "
                 "ASCII while this board speaks BINARY.");
    return;
  }
  sendFeedback("[PLC_TEST] RX " + plcHexDumpBytes(raw, rawLen));
  if (rawLen < PLC_MC_RES_HEADER_UNITS + 4
   || raw[0] != PLC_MC_SUBHEADER_RES_B0
   || raw[1] != PLC_MC_SUBHEADER_RES_B1) {
    sendFeedback("[PLC_TEST] Subheader is not D0 00 — the port is answering, but "
                 "not with MC protocol 3E BINARY.");
    return;
  }
  uint16_t endCode = plcU16AtBytes(raw, PLC_MC_RES_HEADER_UNITS + 2);
  if (endCode != 0) {
    sendFeedback("[PLC_TEST] End code " + plcHex((unsigned long)endCode, 4)
               + " — the PLC refused the read. Check that M0..M15 exist and that "
                 "the module permits reads.");
    return;
  }
  int dataLen = (int)plcU16AtBytes(raw, PLC_MC_RES_HEADER_UNITS);
  int avail = (dataLen - 2) / 2;
  if (avail > PLC_STATUS_WORDS) avail = PLC_STATUS_WORDS;
  for (int i = 0; i < PLC_STATUS_WORDS; i++) {
    plcStatusWords[i] = (i < avail)
        ? plcU16AtBytes(raw, PLC_MC_RES_HEADER_UNITS + 4 + i * 2) : 0;
  }
  plcStatusWord = plcStatusWords[0];
  plcStatusValid = true;
  plcLastPollOk = millis();
  sendFeedback("[PLC_TEST] OK — words " + plcHex(plcStatusWords[0], 4) + " "
             + plcHex(plcStatusWords[1], 4) + " " + plcHex(plcStatusWords[2], 4)
             + " | " + plcStatusSummary());
}
#endif
#endif

// M30..M32 travel limits — stop the axis
bool plcLimitLedBlink = false;
unsigned long plcLimitLedLastBlink = 0;
bool plcLimitWarned[3] = {false, false, false};

// Per-sensor boundary switch, index order Z/ROT/A2 throughout this file.
// For ONE broken switch: a stuck or noisy sensor should not have to take
// HOME and the other two axes' protection down with it. Separate from
// plcLinkEnabled, which stops the whole PLC socket. Disabled -> the axis
// is never stopped by that switch, AND it stops counting toward the
// M30+M31+M32 HOME condition in plcHomeStateActive() below — otherwise a
// broken switch blocks HOME forever with no way to say "trust the rest".
bool plcLimitSensorEnabled[3] = {true, true, true};

// Rising-edge detection and the end a both-ends switch caught. 0 = nothing
// latched, so plcLimitEndFor() falls back to the home end.
bool plcLimitPrevBit[3] = {false, false, false};
int  plcLimitLatchedEnd[3] = {0, 0, 0};

int plcLimitBitFor(int i) {
  const int bits[3] = {PLC_M_LIMIT_Z, PLC_M_LIMIT_ROT, PLC_M_LIMIT_A2};
  return bits[i];
}
// The HOME-side end of each switch. Fixed, and the fallback for a
// both-ends switch that has nothing latched.
int plcLimitHomeEndFor(int i) {
  const int ends[3] = {PLC_LIMIT_END_Z, PLC_LIMIT_END_ROT, PLC_LIMIT_END_A2};
  return ends[i];
}
bool plcLimitBothEndsFor(int i) {
  const bool both[3] = {PLC_LIMIT_BOTH_ENDS_Z, PLC_LIMIT_BOTH_ENDS_ROT,
                        PLC_LIMIT_BOTH_ENDS_A2};
  return both[i];
}
// Which end is CURRENTLY refusing. Same as the home end for a
// single-ended switch; for A2M it is whichever end the latch caught.
int plcLimitEndFor(int i) {
  if (!plcLimitBothEndsFor(i)) return plcLimitHomeEndFor(i);
  return plcLimitLatchedEnd[i] ? plcLimitLatchedEnd[i] : plcLimitHomeEndFor(i);
}

// WHICH WAY THE AXIS WAS GOING, REMEMBERED.
//
// The latch needs the travel direction at the instant the switch closed,
// but it only finds out the switch closed when a poll lands — up to one
// poll interval later. Anything that stops the axis in between erases the
// only evidence of which end was hit, and the latch then falls back to the
// home end and reports the WRONG one.
//
// That is not a rare race. The taught SOFT limit for a fully extended arm
// sits at essentially the same place as the physical far switch, and
// serviceJogSoftLimits() runs every loop pass while the PLC bit arrives
// every 20 ms — so the soft limit zeroes a2Dir first, every single time,
// and a far-end trip was reported as COVERED MIN. A far end mislabelled as
// the home end is not cosmetic: plcLimitSensorSatisfied() then accepts a
// fully EXTENDED arm as the home reference and zeroes the counters at the
// wrong end of the travel.
//
// So the direction is sampled every loop pass and kept for a short while
// after the axis stops. Bounded, because a direction from minutes ago is
// not evidence about anything: past PLC_TRAVEL_DIR_MEMORY_MS the latch goes
// back to assuming the home end, which is the old behaviour.
const unsigned long PLC_TRAVEL_DIR_MEMORY_MS = 1000;
int           plcLastTravelDir[3] = {0, 0, 0};
unsigned long plcLastTravelAt[3]  = {0, 0, 0};

// The LIVE direction: the jog direction if it is being jogged, otherwise
// the sign of the remaining distance on the run leg. 0 when it is not
// moving at all.
int plcAxisTravelDirNow(int i) {
  const int jog[3] = {jzDir, rotDir, a2Dir};
  if (jog[i] > 0) return +1;
  if (jog[i] < 0) return -1;
  if (runPhase != PHASE_NONE) {
    const float now[3]  = {currentD1(), currentRot(), currentA2()};
    const float want[3] = {runTargetD1, runTargetRot, runTargetA2};
    float delta = want[i] - now[i];
    if (fabs(delta) > 1e-3) return (delta > 0) ? +1 : -1;
  }
  return 0;
}

// Called every loop pass, BEFORE anything that can zero a direction.
void plcRememberTravelDir() {
  for (int i = 0; i < 3; i++) {
    int dir = plcAxisTravelDirNow(i);
    if (dir) { plcLastTravelDir[i] = dir; plcLastTravelAt[i] = millis(); }
  }
}

// What the latch asks. Live direction first; failing that, the one this
// axis was travelling in a moment ago. 0 only when the axis has genuinely
// been still, which is the case the latch has to guess its way out of.
int plcAxisTravelDir(int i) {
  int dir = plcAxisTravelDirNow(i);
  if (dir) return dir;
  if (plcLastTravelDir[i]
      && millis() - plcLastTravelAt[i] <= PLC_TRAVEL_DIR_MEMORY_MS) {
    return plcLastTravelDir[i];
  }
  return 0;
}
// -1 if the axis token isn't one of the three PLC-sensored axes.
int plcLimitSensorIndexFor(const String &axis) {
  if (axis == "Z")   return 0;
  if (axis == "ROT") return 1;
  if (axis == "A2")  return 2;
  return -1;
}
// True when this axis's PLC switch either agrees (tripped) or has been
// told not to matter. Used ONLY for HOME — jog/run stops still need the
// real bit, since "satisfied" here is not "safe to drive into".
bool plcLimitSensorSatisfied(int i) {
  if (!plcLimitSensorEnabled[i]) return true;
  if (!plcBit(plcLimitBitFor(i))) return false;
  // A both-ends switch caught at the FAR end is not the reference. Without
  // this, an arm parked fully extended satisfies the home state and the
  // counters are zeroed at the wrong end of the travel.
  return plcLimitEndFor(i) == plcLimitHomeEndFor(i);
}

void plcServiceLimitLeds() {
  if (millis() - plcLimitLedLastBlink >= PLC_LIMIT_LED_BLINK_MS) {
    plcLimitLedLastBlink = millis();
    plcLimitLedBlink = !plcLimitLedBlink;
  }
  bool z   = plcStatusValid && plcBit(PLC_M_LIMIT_Z);
  bool rot = plcStatusValid && plcBit(PLC_M_LIMIT_ROT);
  bool a2  = plcStatusValid && plcBit(PLC_M_LIMIT_A2);
  digitalWrite(PLC_LIMIT_LED_Z_PIN,   z   ? HIGH : (plcLimitLedBlink ? HIGH : LOW));
  digitalWrite(PLC_LIMIT_LED_ROT_PIN, rot ? HIGH : (plcLimitLedBlink ? HIGH : LOW));
  digitalWrite(PLC_LIMIT_LED_A2_PIN,  a2  ? HIGH : (plcLimitLedBlink ? HIGH : LOW));
}

// Works out which end a both-ends switch just caught, and forgets it again
// when the switch clears. Must run BEFORE plcServiceLimitStops(), which is
// what zeroes the direction this reads.
void plcServiceLimitLatch() {
  if (!plcStatusValid) return;   // stale data: keep whatever was latched
  const char *names[3] = {"ZM", "RM", "A2M"};
  const char *devs[3]  = {"M32", "M31", "M30"};
  for (int i = 0; i < 3; i++) {
    bool on = plcBit(plcLimitBitFor(i));
    if (plcLimitBothEndsFor(i)) {
      if (on && !plcLimitPrevBit[i]) {
        int dir = plcAxisTravelDir(i);
        bool live = plcAxisTravelDirNow(i) != 0;
        plcLimitLatchedEnd[i] = dir ? dir : plcLimitHomeEndFor(i);
        sendFeedback("[PLC_LIMIT] " + String(names[i]) + " tripped "
                   + String(devs[i]) + " at its "
                   + String(plcLimitLatchedEnd[i] > 0 ? "FORWARD" : "BACK")
                   + " end"
                   + String(dir ? (live ? "" : " (from the direction it was"
                                             " travelling just before it stopped)")
                                : " (nothing was moving, assumed)") + ".");
      } else if (!on) {
        plcLimitLatchedEnd[i] = 0;
      }
    }
    plcLimitPrevBit[i] = on;
  }
}

void plcServiceLimitStops() {
  int *dirs[3] = {&jzDir, &rotDir, &a2Dir};
  const char *names[3] = {"ZM", "RM", "A2M"};
  const char *devs[3]  = {"M32", "M31", "M30"};   // order Z/ROT/A2 — see the swap note above

  for (int i = 0; i < 3; i++) {
    bool tripped = plcLimitSensorEnabled[i] && plcStatusValid && plcBit(plcLimitBitFor(i));
    if (!tripped) { plcLimitWarned[i] = false; continue; }
    if (*dirs[i] == plcLimitEndFor(i)) {
      *dirs[i] = 0;
      if (i == 0)      MOTOR_Z.MoveVelocity(0);
      else if (i == 1) MOTOR_ROT.MoveVelocity(0);
      else             MOTOR_A2.MoveVelocity(0);
      if (!plcLimitWarned[i]) {
        plcLimitWarned[i] = true;
        sendFeedback("[PLC_LIMIT] " + String(names[i]) + " stopped — "
                   + String(devs[i]) + " is ON at its "
                   + String(plcLimitEndFor(i) > 0 ? "FORWARD" : "BACK")
                   + " end. Jog the other way to come off it.");
      }
    }
  }
}

// True when a run leg would drive an axis further into a tripped limit.
bool runLegBlockedByLimit(float d1, float rot, float a2, String &why) {
  const char *names[3] = {"ZM", "RM", "A2M"};
  const char *devs[3]  = {"M32", "M31", "M30"};   // order Z/ROT/A2 — see the swap note above
  float now[3]  = {currentD1(), currentRot(), currentA2()};
  float want[3] = {d1, rot, a2};

  for (int i = 0; i < 3; i++) {
    if (!(plcLimitSensorEnabled[i] && plcStatusValid && plcBit(plcLimitBitFor(i)))) continue;
    float delta = want[i] - now[i];
    if (fabs(delta) < 1e-3) continue;
    int dir = (delta > 0) ? 1 : -1;
    if (dir != plcLimitEndFor(i)) continue;
    why = String(names[i]) + " is on its travel limit (" + String(devs[i])
        + ") and the leg would drive it further in (" + String(now[i], 2)
        + " -> " + String(want[i], 2) + ")";
    return true;
  }
  return false;
}

// HOME state = M30 && M31 && M32, EXCEPT a switch whose boundary has been
// disabled (SET_PLC_SENSOR_ENFORCE:<axis>,0) counts as already satisfied —
// a broken switch must not be able to block HOME forever.
bool plcHomeStateActive() {
  if (!plcStatusValid) return false;
  return plcLimitSensorSatisfied(0) && plcLimitSensorSatisfied(1)
      && plcLimitSensorSatisfied(2);
}

bool plcHomeStatePrev = false;

void plcServiceHomeState() {
  bool now = plcHomeStateActive();
  if (now == plcHomeStatePrev) return;
  plcHomeStatePrev = now;
  if (!now) return;

  if (isMoving || anyJogActive()) {
    sendFeedback("[PLC_HOME] HOME state reached but the machine is still moving — "
                 "coordinates NOT reset. Stop, then it will latch on the next entry.");
    plcHomeStatePrev = false;
    return;
  }

  MOTOR_Z.PositionRefSet(0);
  MOTOR_ROT.PositionRefSet(0);
  MOTOR_A1.PositionRefSet(0);
  MOTOR_A2.PositionRefSet(0);
  isHomed = true;
  homeState = HOME_COMPLETE;
  if (isHoming) { isHoming = false; }
  sendFeedback("[PLC_HOME] HOME STATE — M30, M31 and M32 all true.");
  sendFeedback("[COORD_RESET] Coordinates reset to the standard home pose: "
               "d1=0.00 mm, ROT=0.00 deg, A1M=0.00 motor deg, A2M=0.00 motor deg.");
  reportJogPosition();
}

void plcServicePoll() {
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
  plcServiceRx();
  if (plcTxnActive) return;
  unsigned long now = millis();
  unsigned long interval = isHoming ? PLC_POLL_HOMING_MS : plcPollIdleMs;
  if (plcLastPollSent != 0 && (now - plcLastPollSent) < interval) return;
  plcLastPollSent = now;
  plcSendPoll();
#endif
}

void servicePlc() {
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
  if (!plcLinkEnabled) {
    if (plcClient.connected()) {
      plcClient.stop();
      plcLinkUp = false;
      plcStatusValid = false;
    }
    return;
  }

  static unsigned long lastActedOn = 0;
  plcServicePoll();
  // Lamps and limit stops run EVERY pass, not only on a fresh poll: the
  // blink has to keep ticking between polls, and a jog started after the
  // last reply must still be stopped by an already-tripped switch.
  plcServiceLimitLeds();
  plcServiceLimitLatch();
  plcServiceLimitStops();
  if (plcStatusValid && plcLastPollOk != lastActedOn) {
    lastActedOn = plcLastPollOk;
    plcServiceHomeState();
  }
#endif
}


void plcNetworkInit() {
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
  Ethernet.begin(plcMac, plcLocalIp);
  sendFeedback("[PLC] ClearCore " + String(CC_IP_0) + "." + String(CC_IP_1) + "."
             + String(CC_IP_2) + "." + String(CC_IP_3)
             + " -> PLC " + String(PLC_IP_0) + "." + String(PLC_IP_1) + "."
             + String(PLC_IP_2) + "." + String(PLC_IP_3) + ":" + String((int)PLC_PORT)
             + " (MC protocol 3E, " + String(PLC_MC_ASCII ? "ASCII" : "BINARY")
             + ", READ-ONLY, polling M0..M47 every "
             + String((int)(PLC_POLL_IDLE_MS / 1000)) + " s idle / "
             + String((int)PLC_POLL_HOMING_MS) + " ms while homing)");
  if (Ethernet.linkStatus() == LinkOFF) {
    sendFeedback("[WARN] No Ethernet link detected — HOME will time out and the "
                 "PLC boundary switches will not be seen until the cable is in.");
  }
#endif
}

// HOME IS DRIVEN BY THIS BOARD, NOT REQUESTED FROM THE PLC.  [notes §61]
//
// It used to assert IO-0 into the PLC's X0 and wait for the PLC's own home
// sequence to finish. Nothing on the PLC side ever ran, so HOME sat there
// and timed out. The board already knows where every switch is (M30..M32)
// and already owns the motors, so it drives each axis onto its own switch
// itself. No device is written and no request line is raised.
//
// All three move AT ONCE, each stopping independently the moment its own
// bit reads covered. A1M has no switch fitted and is therefore never moved
// by HOME — if it is extended, it stays extended while RM turns.
void beginHoming() {
  cancelJog();
  cancelRun();
  decelStopAll(false);

  isHoming = true;
  homeState = HOME_REQUESTED;
  isHomed = false;
  homeRequestedAt = millis();
  lastHomeReportTime = homeRequestedAt;

#if PLC_LINK_MODE == PLC_LINK_ETHERNET
  if (!plcStatusValid) {
    isHoming = false;
    homeState = HOME_FAILED;
    sendFeedback("[HOME] FAILED — no PLC device data, so the switches cannot be "
                 "seen. HOME drives the axes onto M30..M32 and would have no way "
                 "to know when to stop. Fix the link first — PLC_TEST.");
    sendFeedback("[ERROR] HOME refused: the switch states are unknown.");
    return;
  }
#endif

  int  *dirs[3]        = {&jzDir, &rotDir, &a2Dir};
  const char *names[3] = {"ZM", "RM", "A2M"};
  String moving, already;
  for (int i = 0; i < 3; i++) {
    // Already sitting on its switch: nothing to do, and driving further in
    // is the one direction that must never be commanded. A DISABLED switch
    // (SET_PLC_SENSOR_ENFORCE:<axis>,0 — broken sensor) is treated the
    // same way: never driven, since there would be nothing to stop it
    // arriving at its mechanical end blind.
    // Reads the LATCH directly rather than the effective-end helper: the
    // question here is "do we KNOW this is the far switch", and a bit with
    // nothing latched must answer no, or HOME would drive an axis that is
    // already sitting on its reference.
    bool tripped = plcBit(plcLimitBitFor(i));
    bool atFarEnd = tripped && plcLimitBothEndsFor(i)
                 && plcLimitLatchedEnd[i] != 0
                 && plcLimitLatchedEnd[i] != plcLimitHomeEndFor(i);
    if (!plcLimitSensorEnabled[i] || (tripped && !atFarEnd)) {
      homeAxisActive[i] = false;
      homeWaitForClear[i] = false;
      *dirs[i] = 0;
      already += String(already.length() ? ", " : "") + names[i];
      continue;
    }
    homeAxisActive[i] = true;
    // Tripped, but at the FAR end: the bit is already on and means the
    // opposite of arrival. Ignore it until the axis drives clear of it.
    homeWaitForClear[i] = atFarEnd;
    if (atFarEnd) {
      sendFeedback("[HOME] " + String(names[i]) + " is on its FAR switch -- "
                   "driving off it before homing.");
    }
    *dirs[i] = homeDirFor(i);            // backward until this axis's switch trips
    moving += String(moving.length() ? ", " : "") + names[i];
  }
  applyJogVelocities();

  sendFeedback("[HOME] Homing started — this board drives the axes, the PLC is "
               "not asked. Moving: " + String(moving.length() ? moving : "nothing")
             + (already.length() ? " | already on switch: " + already : "")
             + " | at " + String((int)(HOME_SPEED_SCALE * 100)) + "% speed, timeout "
             + String((int)(PLC_HOME_TIMEOUT_MS / 1000)) + "s.");
  if (!moving.length()) {
    sendFeedback("[HOME] Every switch is already covered.");
  }
  sendFeedback("[HOME] A1M has no switch and is NOT moved by HOME.");
}

void finishHoming(bool ok, const String &reason) {
  isHoming = false;
  homeState = ok ? HOME_COMPLETE : HOME_FAILED;

  if (ok) {
#if PLC_LINK_MODE == PLC_LINK_PLACEHOLDER
    sendFeedback("[WARN] PLACEHOLDER HOME — position reference NOT zeroed and "
                 "isHomed stays false. Wire the PLC before trusting this.");
    sendFeedback("[HOME] Homing complete (simulated).");
#else
    // RESET THE COORDINATE SYSTEM TO THE STANDARD HOME POSE.  [notes §60]
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
    sendFeedback("[ERROR] HOME timeout: this board drives the axes itself and never "
                 "saw one or more of M30..M32 come ON.");
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
    if (plcGoodReads == 0) {
      sendFeedback("[ERROR] Root cause: this board has never read a device from the "
                   "PLC, so it could not have seen the switches at all. Fix the "
                   "MC-protocol link first — PLC_TEST.");
    } else {
      sendFeedback("[ERROR] Device reads work, so a switch is stuck, broken, or the "
                   "axis is mechanically obstructed. If a switch is known broken, "
                   "SET_PLC_SENSOR_ENFORCE:<axis>,0 excludes it from HOME.");
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

  // Stop each axis the instant ITS OWN switch reads covered. They finish
  // independently — one axis arriving must not halt the other two.
  int  *dirs[3]        = {&jzDir, &rotDir, &a2Dir};
  const char *names[3] = {"ZM", "RM", "A2M"};
  bool changed = false;
  for (int i = 0; i < 3; i++) {
    if (!homeAxisActive[i]) continue;
    bool on = plcBit(plcLimitBitFor(i));
    if (homeWaitForClear[i]) {
      if (on) continue;                 // still on the far switch
      homeWaitForClear[i] = false;      // clear of it; the next ON is arrival
      continue;
    }
    if (!on) continue;
    homeAxisActive[i] = false;
    *dirs[i] = 0;
    changed = true;
    sendFeedback("[HOME] " + String(names[i]) + " reached its switch.");
  }
  if (changed) applyJogVelocities();

  if (plcHomeStateActive()) {
    finishHoming(true, "");
    return;
  }
  if (now - homeRequestedAt >= PLC_HOME_TIMEOUT_MS) {
    String stuck;
    for (int i = 0; i < 3; i++) {
      if (homeAxisActive[i]) stuck += String(stuck.length() ? ", " : "") + names[i];
    }
    jzDir = rotDir = a2Dir = 0;
    applyJogVelocities();
    finishHoming(false, "never reached: " + (stuck.length() ? stuck : String("?"))
               + " within " + String((int)(PLC_HOME_TIMEOUT_MS / 1000)) + "s");
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
// ══════════════════════════════════════════════════════════════
// 340 DEGREE SCAN
// ══════════════════════════════════════════════════════════════

// One distance reading, in mm. Returns a NEGATIVE number when the sensor
// did not answer -- never 0, because 0 mm is a legitimate reading and
// "nothing came back" is not. The scan reports the miss as a hole in the
// layer rather than dropping the point, so a dead sensor looks like a dead
// sensor and not like a small object.
double scanReadDistanceMm() {
  if (scanSensorKind == SCAN_SENSOR_ANALOG) {
    if (scanAnalogMmPerCount == 0.0) return -1.0;   // never calibrated
    int raw = analogRead(SCAN_ANALOG_PIN);
    return raw * scanAnalogMmPerCount + scanAnalogOffsetMm;
  }
  digitalWrite(SCAN_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SCAN_TRIG_PIN, HIGH);
  delayMicroseconds(SCAN_TRIG_US);
  digitalWrite(SCAN_TRIG_PIN, LOW);
  unsigned long us = pulseIn(SCAN_ECHO_PIN, HIGH, SCAN_ECHO_TIMEOUT_US);
  if (us == 0) return -1.0;                          // echo timed out
  return (double)us * SCAN_MM_PER_US;
}

const char *scanSensorName() {
  return scanSensorKind == SCAN_SENSOR_ANALOG ? "ANALOG" : "ULTRASONIC";
}

// Is RM sitting on its travel switch right now? The scan's whole frame
// hangs off this one bit: it is the only position on the turntable the
// board can identify without a reference, so it is where every layer
// starts and where every other layer ends.
bool scanRotSwitchOn() {
  return plcStatusValid && plcLimitSensorEnabled[1] && plcBit(PLC_M_LIMIT_ROT);
}

void cancelScan(const String &why) {
  if (scanPhase == SCAN_OFF) return;
  scanPhase = SCAN_OFF;
  rotDir = jzDir = 0;
  applyJogVelocities();
  sendFeedback("[SCAN_ABORT] " + why);
}

// Samples are stamped with the angle read just BEFORE the sensor fires, not
// after. An ultrasonic read blocks for the whole 30 ms timeout when nothing
// comes back, and stamping afterwards would file every miss at an angle the
// arm had already left.
void scanEmitPoint() {
  double deg = currentRot();
  // INVERT_ROT makes currentRot() hand back NEGATIVE zero at the reference,
  // so a sample there would be stamped "-0.00". Harmless arithmetically,
  // ugly in a CSV, and it makes the first column look signed when it is
  // not. -0.0 == 0.0 is true, so this replaces it.
  if (deg == 0.0) deg = 0.0;
  double mm  = scanReadDistanceMm();
  scanPointsSent++;
  sendFeedback("[SCAN_PT] " + String(scanLayer) + "," + String(deg, 2)
             + "," + String(mm, 2));
}

// Layers ALTERNATE direction: the first sweeps away from the switch, the
// next comes back to it, and so on.
//
// The earlier version rewound between layers so that every layer was
// sampled travelling the same way, which keeps the drivetrain backlash on
// one side. This is the operator's call and it buys two things that are
// worth more here: the return leg collects a layer instead of being dead
// travel, so a scan takes half as long, and every layer that ends on the
// switch RE-REFERENCES the turntable, so angle error cannot accumulate
// over a tall scan. What it costs is a fixed backlash offset between odd
// and even layers -- a constant, and one that can be measured and removed
// afterwards, unlike drift.
void scanBeginLayer(int dir) {
  scanSweepDir = dir;
  scanSweepFrom = currentRot();
  scanNextDeg = scanSweepFrom;
  scanPhase = SCAN_SWEEP;
  sendFeedback("[SCAN_LAYER] " + String(scanLayer) + "/" + String(scanLayers)
             + " z=" + String(currentD1(), 2) + " mm"
             + " dir=" + String(dir > 0 ? "+" : "-")
             + " from=" + String(scanSweepFrom, 2));
  rotDir = dir;
  jzDir = 0;
  applyJogVelocities();
}

// True once the axis has reached the next angle a sample is due at. The
// test has to follow the sweep direction: going backwards, "arrived" means
// the angle has fallen TO it, not risen to it.
bool scanReachedNext() {
  if (scanSweepDir > 0) return currentRot() >= scanNextDeg - SCAN_ANGLE_EPS_DEG;
  return currentRot() <= scanNextDeg + SCAN_ANGLE_EPS_DEG;
}

double scanTravelled() {
  double d = currentRot() - scanSweepFrom;
  return d < 0 ? -d : d;
}

void serviceScan() {
  if (scanPhase == SCAN_OFF) return;

  // ---- finding the reference --------------------------------------
  if (scanPhase == SCAN_SEEK) {
    if (scanRotSwitchOn()) {
      // plcServiceLimitStops() has already stopped the axis -- driving
      // into a covered switch is the one direction it refuses, which is
      // exactly the behaviour being used here rather than worked around.
      rotDir = 0;
      applyJogVelocities();
      scanStartRot = currentRot();
      sendFeedback("[SCAN_REF] RM on its switch at " + String(scanStartRot, 2)
                 + " deg - sweeping from here");
      scanLayer = 1;
      scanBeginLayer(-PLC_LIMIT_END_ROT);   // away from the switch
      // Deliberately NO return: falling through into the sweep below takes
      // the sample at the reference angle now, rather than a service tick
      // later when the axis has already moved off it. That first point is
      // the one every other layer is aligned against.
    } else if (rotDir == 0) {
      cancelScan("RM stopped before reaching its switch - a soft limit is in "
                 "the way, or the switch is not wired");
      return;
    } else if (scanTravelled() > SCAN_SEEK_MAX_DEG) {
      cancelScan("RM turned " + String(SCAN_SEEK_MAX_DEG, 0)
               + " deg without finding its switch");
      return;
    } else {
      return;                               // still turning, nothing to do
    }
  }

  // The jog soft-limit and PLC-switch services can zero rotDir/jzDir under
  // us -- that is the whole reason the scan drives through them rather than
  // commanding the motors itself. Being stopped mid-sweep means the layer
  // is short, and saying so beats reporting it as complete.
  //
  // A sweep heading BACK toward the switch is the exception: being stopped
  // there is arrival, not a fault, and it is handled below.
  if (scanPhase == SCAN_SWEEP && rotDir == 0 && !scanRotSwitchOn()) {
    cancelScan("RM was stopped mid-sweep by a soft limit or a PLC switch");
    return;
  }
  if (scanPhase == SCAN_LIFT && jzDir == 0) {
    cancelScan("ZM was stopped by a soft limit or a PLC switch before the next layer");
    return;
  }

  // ---- sweeping ----------------------------------------------------
  if (scanPhase == SCAN_SWEEP) {
    while (scanReachedNext() && scanTravelled() <= scanSweepDeg + SCAN_ANGLE_EPS_DEG) {
      scanEmitPoint();
      scanNextDeg += scanSweepDir * scanDegStep;
    }

    bool backAtSwitch = (scanSweepDir == PLC_LIMIT_END_ROT) && scanRotSwitchOn();
    if (!backAtSwitch && scanTravelled() < scanSweepDeg - SCAN_ANGLE_EPS_DEG) return;

    rotDir = 0;
    applyJogVelocities();
    if (backAtSwitch) {
      // Every arrival at the switch is a fresh reference. Without this the
      // start angle drifts by whatever the last sweep overshot, layer after
      // layer, and a tall scan ends up rotated against its own base.
      scanStartRot = currentRot();
      sendFeedback("[SCAN_REF] RM back on its switch at "
                 + String(scanStartRot, 2) + " deg");
    }
    if (scanLayer >= scanLayers) {
      scanPhase = SCAN_OFF;
      sendFeedback("[SCAN_DONE] " + String(scanLayers) + " layers, "
                 + String(scanPointsSent) + " points");
      return;
    }
    scanLayerTargetZ = scanStartZ + scanZStepMm * (double)scanLayer;
    scanPhase = SCAN_LIFT;
    jzDir = 1;
    applyJogVelocities();
    return;
  }

  // ---- lifting between layers --------------------------------------
  if (scanPhase == SCAN_LIFT) {
    if (currentD1() < scanLayerTargetZ - SCAN_Z_EPS_MM) return;
    jzDir = 0;
    applyJogVelocities();
    scanLayer++;
    scanBeginLayer(-scanSweepDir);       // back the way it came
  }
}

void handleScanStart(const String &payload) {
  if (isMoving || isHoming || anyJogActive()) {
    sendFeedback("[ERROR] SCAN refused - the machine is already moving.");
    return;
  }
  if (scanPhase != SCAN_OFF) {
    sendFeedback("[ERROR] SCAN refused - a scan is already running.");
    return;
  }
  int c1 = payload.indexOf(',');
  int c2 = payload.indexOf(',', c1 + 1);
  int c3 = payload.indexOf(',', c2 + 1);
  if (c1 < 0 || c2 < 0) {
    sendFeedback("[ERROR] SCAN_START needs zStepMm,degStep,layers[,sweepDeg]");
    return;
  }
  double zStep = payload.substring(0, c1).toFloat();
  double dStep = payload.substring(c1 + 1, c2).toFloat();
  int layers = (c3 < 0 ? payload.substring(c2 + 1)
                       : payload.substring(c2 + 1, c3)).toInt();
  double sweep = (c3 < 0) ? SCAN_SWEEP_DEG_DEF : payload.substring(c3 + 1).toFloat();

  if (zStep < SCAN_Z_STEP_MIN_MM) {
    sendFeedback("[ERROR] Z step must be at least " + String(SCAN_Z_STEP_MIN_MM, 2)
               + " mm, got " + String(zStep, 3));
    return;
  }
  if (dStep < SCAN_DEG_STEP_MIN || dStep > SCAN_DEG_STEP_MAX) {
    sendFeedback("[ERROR] angular step must be between " + String(SCAN_DEG_STEP_MIN, 2)
               + " and " + String(SCAN_DEG_STEP_MAX, 0) + " deg, got " + String(dStep, 3));
    return;
  }
  if (layers < 1 || layers > SCAN_LAYERS_MAX) {
    sendFeedback("[ERROR] layers must be between 1 and " + String(SCAN_LAYERS_MAX)
               + ", got " + String(layers));
    return;
  }
  if (sweep < SCAN_SWEEP_DEG_MIN || sweep > SCAN_SWEEP_DEG_DEF) {
    sendFeedback("[ERROR] sweep must be between " + String(SCAN_SWEEP_DEG_MIN, 0)
               + " and " + String(SCAN_SWEEP_DEG_DEF, 0)
               + " deg - the turntable's whole travel - got " + String(sweep, 2));
    return;
  }
  // A sweep shorter than one step collects a single point per layer and
  // still costs the full seek and lift. Almost certainly a typo, and
  // silently producing a one-point "scan" is the unhelpful answer.
  if (sweep < dStep) {
    sendFeedback("[ERROR] a " + String(sweep, 2) + " deg sweep is shorter than the "
               + String(dStep, 2) + " deg step, so a layer would hold one point.");
    return;
  }
  // The LAST layer is the one that has to fit. The lift only moves between
  // layers, so the top of the scan is startZ + zStep * (layers - 1) -- using
  // layers there would refuse scans that actually fit.
  double topZ = currentD1() + zStep * (double)(layers - 1);
  if (topZ > D1_MAX_MM) {
    sendFeedback("[ERROR] SCAN would need Z = " + String(topZ, 1)
               + " mm, past the " + String(D1_MAX_MM, 0) + " mm stroke. "
                 "Lower the start height, the step, or the layer count.");
    return;
  }
  // The scan is referenced to the RM switch, so without it there is no
  // frame to sweep in. Refused rather than started from wherever the
  // turntable happens to be sitting: two scans taken on different days
  // would then have angle columns that mean different things.
  if (!plcStatusValid) {
    sendFeedback("[ERROR] SCAN refused - no PLC device data, so the RM switch "
                 "cannot be seen. Check the link with PLC_TEST.");
    return;
  }
  if (!plcLimitSensorEnabled[1]) {
    sendFeedback("[ERROR] SCAN refused - RM's switch is disabled "
                 "(SET_PLC_SENSOR_ENFORCE:ROT,1 to put it back). It is the "
                 "reference every layer starts from.");
    return;
  }
  if (scanSensorKind == SCAN_SENSOR_ANALOG && scanAnalogMmPerCount == 0.0) {
    sendFeedback("[WARN] the analog sensor has no calibration, so every reading "
                 "will come back -1. Send SET_SCAN_CAL first.");
  }

  scanZStepMm = zStep;
  scanDegStep = dStep;
  scanLayers  = layers;
  scanSweepDeg = sweep;
  scanStartZ   = currentD1();
  scanSweepFrom = currentRot();
  scanPointsSent = 0;
  scanLayer = 0;
  sendFeedback("[SCAN_BEGIN] sensor=" + String(scanSensorName())
             + " layers=" + String(layers)
             + " zStep=" + String(zStep, 2)
             + " degStep=" + String(dStep, 2)
             + " sweep=" + String(sweep, 1)
             + " fromZ=" + String(scanStartZ, 2));

  if (scanRotSwitchOn()) {
    // Already there. Nothing to seek, and driving into a covered switch is
    // refused anyway, so this would otherwise abort on the spot.
    scanPhase = SCAN_SEEK;
    serviceScan();
    return;
  }
  sendFeedback("[SCAN_SEEK] turning RM to its switch to reference the sweep...");
  scanPhase = SCAN_SEEK;
  rotDir = PLC_LIMIT_END_ROT;
  applyJogVelocities();
}

void sendScanStatus() {
  const char *phase = scanPhase == SCAN_OFF ? "IDLE"
                    : scanPhase == SCAN_SEEK ? "SEEK"
                    : scanPhase == SCAN_SWEEP ? "SWEEP" : "LIFT";
  sendFeedback(String("[SCAN_STATUS] phase=") + phase
             + " sensor=" + String(scanSensorName())
             + " layer=" + String(scanLayer) + "/" + String(scanLayers)
             + " dir=" + String(scanSweepDir > 0 ? "+" : "-")
             + " points=" + String(scanPointsSent)
             + " cal=" + String(scanAnalogMmPerCount, 5)
             + "," + String(scanAnalogOffsetMm, 2));
}

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

  // RESET_COORD:Z|ROT|A1|A2  zero ONE axis
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

  if (upper.startsWith("SET_LIMIT_LOCK:")) {
    sendFeedback("[ERROR] SET_LIMIT_LOCK no longer exists. The per-axis control is "
                 "now enforcement, not a value lock — use "
                 "SET_LIMIT_ENFORCE:<axis>,<0|1>. Update the GUI to match this board.");
    return;
  }

  if (upper.startsWith("SET_LIMITS_ENABLED:")) {
    bool want = cmd.substring(19).toInt() != 0;
    if (!want && limitsEnabled) {
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

  if (upper.startsWith("SET_PLC_LINK:")) {
    bool want = cmd.substring(13).toInt() != 0;
    if (!want && plcLinkEnabled) {
      sendFeedback("[WARN] PLC LINK DISABLED. The ClearCore will disconnect from the PLC and stop polling.");
    }
    plcLinkEnabled = want;
    sendFeedback(String("[PLC_LINK] ") + (plcLinkEnabled ? "1 — ENABLED" : "0 — DISABLED"));
    if (!plcLinkEnabled) {
      // Instant lamp update — otherwise the GUI shows whatever state it
      // last had (often CONNECTED) for up to one heartbeat, which reads
      // as a fault rather than "off on purpose".
      sendFeedback("[PLC_STATE] link=DISABLED socket=CLOSED data=NONE conn=0/0 "
                   "word=---- timeouts=0 | LINK DISABLED — SET_PLC_LINK:1 to "
                   "re-enable | limit Z/R/A2=???");
    }
    return;
  }

  if (upper.startsWith("SET_PLC_SENSOR_ENFORCE:")) {
    String payload = cmd.substring(23);
    int comma = payload.indexOf(',');
    if (comma < 0) {
      sendFeedback("[ERROR] SET_PLC_SENSOR_ENFORCE needs axis,0|1 (axis = Z, ROT, A2)");
      return;
    }
    String axis = payload.substring(0, comma); axis.trim(); axis.toUpperCase();
    int i = plcLimitSensorIndexFor(axis);
    if (i < 0) {
      sendFeedback("[ERROR] SET_PLC_SENSOR_ENFORCE axis must be Z, ROT or A2 — got \""
                 + axis + "\"");
      return;
    }
    bool want = payload.substring(comma + 1).toInt() != 0;
    if (!want && plcLimitSensorEnabled[i]) {
      sendFeedback("[WARN] " + axis + "'s PLC travel-limit switch DISABLED. It will "
                   "not stop the axis, and HOME will complete without waiting for it.");
    }
    plcLimitSensorEnabled[i] = want;
    sendFeedback(String("[PLC_SENSOR_ENFORCE] ") + axis
               + (plcLimitSensorEnabled[i] ? " 1 — ENFORCED" : " 0 — NOT ENFORCED"));
    return;
  }

  if (upper == "CLEAR_REF") {
    isHomed = false;
    homeState = HOME_IDLE;
    sendFeedback("[HOME] Reference cleared. Positions are relative again until HOME or RESET_COORD — your taught boundaries are still applied.");
    return;
  }

  // ---- Operator-defined travel limits ----
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
    cancelScan("emergency stop");
    cancelJog(); cancelRun(); cancelHoming();
    decelStopAll(true);
    sendFeedback("[ESTOP] EMERGENCY STOP");
    return;
  }
  if (upper == "STOP") {
    cancelScan("STOP");
    cancelJog(); cancelRun(); cancelHoming();
    decelStopAll(false);
    sendFeedback("[ESTOP] EMERGENCY STOP");
    return;
  }

  // ---- parameters ----
  // ---- PID gains ----  SET_PID:kp,ki,kd[,N]
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
    if (anyJogActive()) applyJogVelocities();
    sendFeedback("[MOTION_OK]");
    reportMotionProfile();
    return;
  }

  // ---- Legacy engineering-unit form, converted into the new model ----
  if (upper.startsWith("SET_MOTION:")) {
    double v[6];
    if (parseCsv(cmd.substring(11), v, 6) != 6) {
      sendFeedback("[ERROR] SET_MOTION needs rotVel,rotAcc,armVel,armAcc,zVel,zAcc "
                   "(deg/s, deg/s2, mm/s, mm/s2)");
      return;
    }
    float armVelCeil = ARM_RPM_MAX * 360.0f / 60.0f;
    if (!motionValueOk(v[0], MOTION_MIN, ROT_VEL_MAX, "RM vel")    ||
        !motionValueOk(v[1], MOTION_MIN, ROT_ACC_MAX, "RM accel")  ||
        !motionValueOk(v[2], MOTION_MIN, armVelCeil,  "ARM vel")   ||
        !motionValueOk(v[4], MOTION_MIN, Z_VEL_MAX,   "ZM vel")    ||
        !motionValueOk(v[5], MOTION_MIN, Z_ACC_MAX,   "ZM accel")) return;

    float rotRpm = (float)v[0] * (float)rotGearRatio * 60.0f / 360.0f;
    float armRpm = (float)v[2] * 60.0f / 360.0f;
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
  if (upper == "RESET_POSITION") { beginResetPosition(); return; }

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
    applyMotionParams();
    sendFeedback("[ARM_RATIO] " + String(before, 4) + " -> " + String(armGearRatio, 4)
               + " motor deg per fold deg. Taught limits are motor degrees and were "
                 "left alone; reported fold angles and reach figures have moved.");
    reportMotionProfile();
    reportLimits();
    return;
  }
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

  if (upper.startsWith("SET_Z_LEAD:")) {
    double v = cmd.substring(11).toDouble();
    if (v < Z_MM_PER_REV_MIN || v > Z_MM_PER_REV_MAX) {
      sendFeedback("[ERROR] ZM lead must be between " + String(Z_MM_PER_REV_MIN, 2)
                 + " and " + String(Z_MM_PER_REV_MAX, 0) + " mm/rev, got " + String(v, 4));
      return;
    }
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
               + String(Z_MM_PER_REV_DEF, 1) + " — confirmed on the machine)");
    return;
  }

  if (upper == "ARM_RATIO") {
    sendFeedback("[ARM_RATIO] " + String(armGearRatio, 4)
               + " motor deg per fold deg (default " + String(ARM_GEAR_RATIO_DEF, 2)
               + ", MEASURED on the machine from the 575 mm full-extension reach)");
    return;
  }

  // ---- 340 degree scan -------------------------------------------
  if (upper.startsWith("SCAN_START:")) {
    handleScanStart(cmd.substring(11));
    return;
  }
  if (upper == "SCAN_STOP") {
    if (scanPhase == SCAN_OFF) sendFeedback("[SCAN_STATUS] phase=IDLE - nothing to stop");
    else cancelScan("stopped by the operator");
    return;
  }
  if (upper == "SCAN_STATUS") { sendScanStatus(); return; }
  if (upper == "SCAN_READ") {
    // One shot, for aiming the sensor and checking the calibration without
    // committing to a sweep.
    sendFeedback("[SCAN_READ] " + String(scanReadDistanceMm(), 2) + " mm ("
               + String(scanSensorName()) + ")");
    return;
  }
  if (upper.startsWith("SET_SCAN_SENSOR:")) {
    String kind = upper.substring(16);
    kind.trim();
    if (kind == "ULTRASONIC")   scanSensorKind = SCAN_SENSOR_ULTRASONIC;
    else if (kind == "ANALOG")  scanSensorKind = SCAN_SENSOR_ANALOG;
    else {
      sendFeedback("[ERROR] SET_SCAN_SENSOR takes ULTRASONIC or ANALOG, got " + kind);
      return;
    }
    sendFeedback(String("[SCAN_SENSOR] ") + scanSensorName());
    return;
  }
  if (upper.startsWith("SET_SCAN_CAL:")) {
    String payload = cmd.substring(13);
    int comma = payload.indexOf(',');
    if (comma < 0) {
      sendFeedback("[ERROR] SET_SCAN_CAL needs mmPerCount,offsetMm");
      return;
    }
    double perCount = payload.substring(0, comma).toFloat();
    double offset = payload.substring(comma + 1).toFloat();
    if (perCount <= 0.0) {
      sendFeedback("[ERROR] mmPerCount must be positive, got " + String(perCount, 5));
      return;
    }
    scanAnalogMmPerCount = perCount;
    scanAnalogOffsetMm = offset;
    sendFeedback("[SCAN_CAL] " + String(perCount, 5) + " mm/count, offset "
               + String(offset, 2) + " mm");
    return;
  }

  if (upper == "PLC_STATUS") {
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
    if (!plcLinkEnabled) {
      // A raw socket/data dump here would show the leftover state from
      // before SET_PLC_LINK:0, which reads as a fault (NO REPLY,
      // UNREACHABLE) rather than "off on purpose". "limit Z/R/A2=???"
      // reuses the existing unknown-sensor path on the GUI side — no
      // separate handling needed there to mark the sensors unknown too.
      sendFeedback("[PLC_STATE] link=DISABLED socket=CLOSED data=NONE conn=0/0 "
                   "word=---- timeouts=0 | LINK DISABLED — SET_PLC_LINK:1 to "
                   "re-enable | limit Z/R/A2=???");
      return;
    }
    sendFeedback("[PLC_COUNTS] connects " + String((unsigned long)plcConnectTries)
               + " (failed " + String((unsigned long)plcConnectFails) + ") | frames sent "
               + String((unsigned long)plcSendAttempts) + " | good reads "
               + String((unsigned long)plcGoodReads) + " | timeouts "
               + String((unsigned long)plcTxnTimeouts) + " | rx buffer \""
#if PLC_MC_ASCII
               + plcRxBuf
#else
               + plcHexDumpBytes(plcRxBytes, plcRxCount)
#endif
               + "\" | poll " + String((unsigned long)plcPollIdleMs)
               + " ms idle");
    if (plcGoodReads == 0) {
      sendFeedback("[PLC] NO device read has EVER succeeded. HOME cannot complete "
                   "without it (HOME completes on M30..M32), and every sensor "
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
    // Name the layer that is actually failing. A socket that has NEVER
    // opened cannot be an MC-protocol problem: no frame has left the board,
    // so the encoding and the port's protocol setting have not been tested
    // at all. Saying "or MC protocol is misconfigured" here sent someone
    // to the GX Works3 screens while the fault was below TCP.
    if (!plcStatusValid) {
      if (plcConnectsOk == 0) {
        sendFeedback("[PLC] TCP connect has NEVER succeeded ("
                   + String((unsigned long)plcConnectFails) + " failed), so no "
                     "frame has been sent and MC protocol is NOT the suspect yet. "
                     "This is cable, addressing, or the PLC not listening on port "
                   + String((int)PLC_PORT) + ". Run PLC_TEST — it reports the PHY "
                     "link separately.");
      } else if (plcSendAttempts == 0) {
        sendFeedback("[PLC] The socket has opened before, but no frame has been "
                     "sent yet. Nothing is wrong with the link — wait one poll.");
      } else {
        sendFeedback("[PLC] Frames are going out and the socket opens, but no reply "
                     "has ever landed. THAT is the MC-protocol case: check the "
                     "Ethernet module has MC protocol on port "
                   + String((int)PLC_PORT) + " with Communication Data Code = "
                   + String(PLC_MC_ASCII ? "ASCII" : "BINARY") + ".");
      }
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
    if (ms < 1 || ms > 60000) {
      sendFeedback("[ERROR] PLC poll interval must be 1..60000 ms, got " + String(ms));
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
#if PLC_MC_ASCII
    String frame = plcBuildPollFrame();
    sendFeedback("[PLC_TEST] TX " + frame);
    plcRxBuf = "";
    plcWriteFrame(frame);
    plcClient.flush();
    unsigned long t0 = millis();
    while (millis() - t0 < PLC_TXN_TIMEOUT_MS * 2) {
      while (plcClient.available() > 0) {
        char c = (char)plcClient.read();
        if (plcRxBuf.length() < 200) plcRxBuf += c;
      }
      if ((int)plcRxBuf.length() >= PLC_MC_RES_HEADER_UNITS + 4) break;
    }
    plcTestReport(plcRxBuf);
#else
    plcBuildPollFrame();
    sendFeedback("[PLC_TEST] TX " + plcHexDumpBytes(plcTxBytes, plcTxCount));
    plcRxCount = 0;
    plcClient.write(plcTxBytes, plcTxCount);
    plcClient.flush();
    unsigned long t0 = millis();
    while (millis() - t0 < PLC_TXN_TIMEOUT_MS * 2) {
      while (plcClient.available() > 0) {
        if (plcRxCount < PLC_RX_CAP) plcRxBytes[plcRxCount++] = (uint8_t)plcClient.read();
      }
      if (plcRxCount >= PLC_MC_RES_HEADER_UNITS + 4) break;
    }
    plcTestReport(plcRxBytes, plcRxCount);
#endif
#else
    sendFeedback("[PLC_TEST] No Ethernet client compiled in (link mode "
               + String(PLC_LINK_MODE) + ").");
#endif
    return;
  }

  if (upper == "PLC_RECONNECT") {
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
    plcClient.stop();
    plcLastConnectTry = 0;
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

  if (upper == "A1_FWD")  { startJog(a1Dir,  1); return; }
  if (upper == "A1_BACK") { startJog(a1Dir, -1); return; }
  if (upper == "A1_STOP") { stopArmJog(true, false); return; }
  if (upper == "A2_FWD")  { startJog(a2Dir,  1); return; }
  if (upper == "A2_BACK") { startJog(a2Dir, -1); return; }
  if (upper == "A2_STOP") { stopArmJog(false, true); return; }

  if (upper == "ARM_FWD")  { startArmJogLinked( 1); return; }
  if (upper == "ARM_BACK") { startArmJogLinked(-1); return; }
  if (upper == "ARM_STOP") { stopArmJog(true, true); return; }

  if (upper == "Z_UP")     { startJog(jzDir,  1); return; }
  if (upper == "Z_DOWN")   { startJog(jzDir, -1); return; }
  if (upper == "Z_STOP")   { jzDir = 0; MOTOR_Z.MoveVelocity(0); return; }

  if (upper.startsWith("MOVE_A1:") || upper.startsWith("MOVE_A2:")) {
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
    bool isA1 = upper.startsWith("MOVE_R1:");
    double rTarget = cmd.substring(8).toDouble();
    double rMin, rMax;
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
  while (!Serial && (millis() - t0) < 3000) {  }

  pinMode(SCAN_TRIG_PIN, OUTPUT);
  digitalWrite(SCAN_TRIG_PIN, LOW);
  pinMode(SCAN_ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

#if ENABLE_ROT_Z_LIMIT_SENSORS
  pinMode(ROT_LIMIT_CW_PIN,  INPUT);
  pinMode(ROT_LIMIT_CCW_PIN, INPUT);
  pinMode(Z_LIMIT_UP_PIN,    INPUT);
  pinMode(Z_LIMIT_DOWN_PIN,  INPUT);
#endif
  pinMode(PLC_LIMIT_LED_Z_PIN,   OUTPUT);
  pinMode(PLC_LIMIT_LED_ROT_PIN, OUTPUT);
  pinMode(PLC_LIMIT_LED_A2_PIN,  OUTPUT);
#if PLC_LINK_MODE == PLC_LINK_DIGITAL_IO
  pinMode(PLC_HOME_DONE_PIN, INPUT);
#endif

  motorsInit();
  plcNetworkInit();
  lastAliveTime = millis();
  lastJogKeepAlive = millis();

  sendFeedback("[BOOT] ==========================================");
  sendFeedback("[BOOT] STCR4000S controller v9.1 — on-board frog-leg IK");
  sendFeedback("[BOOT] Independent arms: A1_FWD/A1_BACK, A2_FWD/A2_BACK");
  sendFeedback("[BOOT] Speed: universal RPM + per-motor % (SET_SPEED)");
  sendFeedback("[BOOT] Limits: SET_LIMIT / SET_LIMIT_HERE, reference: RESET_COORD");
  sendFeedback("[BOOT] Limits live in RAM only — the host must re-send them on connect.");
  sendFeedback("[BOOT] Jog watchdog: " + String(ENABLE_JOG_WATCHDOG ? "ON" : "OFF")
             + " (" + String((int)JOG_WATCHDOG_MS) + " ms) — host must send JOG_HB");
  sendFeedback("[BOOT] PLC: MC protocol 3E " + String(PLC_MC_ASCII ? "ASCII" : "BINARY") + " -> "
             + String(PLC_IP_0) + "." + String(PLC_IP_1) + "." + String(PLC_IP_2) + "."
             + String(PLC_IP_3) + ":" + String((int)PLC_PORT)
             + " | link mode " + String(PLC_LINK_MODE)
             + " | HOME drives axes onto M" + String(PLC_M_LIMIT_Z) + "/M"
             + String(PLC_M_LIMIT_ROT) + "/M" + String(PLC_M_LIMIT_A2) + " itself"
             + " | timeout " + String((int)(PLC_HOME_TIMEOUT_MS / 1000)) + "s");
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
  sendFeedback("[BOOT] PLC Ethernet is READ-ONLY (batch read M0..M47). Nothing is "
               "ever written to the PLC — HOME reads M30..M32 and drives the axes "
               "itself. If HOME never starts, check the Ethernet link — PLC_TEST.");
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
  // FIRST, before anything that can zero a direction -- the soft limits and
  // the watchdog both do, and the PLC latch needs to know which way the
  // axis was going when its switch closed. See plcRememberTravelDir().
  plcRememberTravelDir();
#if ENABLE_ROT_Z_LIMIT_SENSORS
  serviceLimitSensors();
#endif
  serviceJogWatchdog();
  serviceJogSoftLimits();
  serviceJogReporting();
  serviceRun();
  servicePlc();
  serviceHoming();
  serviceScan();

  unsigned long now = millis();
  if (isConnected && (now - lastAliveTime >= ALIVE_INTERVAL_MS)) {
    lastAliveTime = now;
    if (!isMoving && !isHoming && !anyJogActive()) {
      sendFeedback("[ALIVE] uptime: " + String(now / 1000) + "s");
    }
  }
}
