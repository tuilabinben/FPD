#include "ClearCore.h"
#include <math.h>

// ---- Motor connector mapping ----
#define MOTOR_Z   ConnectorM0
#define MOTOR_ROT ConnectorM1
#define MOTOR_A1  ConnectorM2
#define MOTOR_A2  ConnectorM3

#define LED_PIN LED_BUILTIN


// TYPES — MUST STAY ABOVE THE FIRST FUNCTION DEFINITION.
struct IkResult {
  bool   ok;
  double d1;
  double th2;
  double th3;
  double R;
  String error;
};

enum RunPhase { PHASE_NONE, PHASE_TO_HOME_FIRST, PHASE_TO_A, PHASE_TO_B,
                PHASE_TO_HOME_LAST, PHASE_DUAL, PHASE_RESET_HOME,
                PHASE_TEST_OUT, PHASE_TEST_BACK };

enum MotionProfileKind {
  PROFILE_NONE = 0,
  PROFILE_TRAPEZOID = 1,
  PROFILE_SCURVE = 2,
  PROFILE_PURE_SCURVE = 3,
};

// A planned profile, evaluated at one instant per pass.
struct ProfilePlan {
  bool   sCurve;      // false = trapezoid, and the three fields below apply
  double T;           // total time, seconds
  double Theta;       // total displacement, in u (always 1.0 here)
  double vp, ta, tv, alphaMax;                 // trapezoid
  double dur[7], jrk[7], s0[7], v0[7], a0[7], tStart[7];   // s-curve phases
};

// One axis's jog ease.
struct JogRamp {
  bool   active    = false;   // easing up toward vp
  bool   releasing = false;   // easing down toward 0
  unsigned long t0 = 0;
  double tJ = 0, tA = 0, J = 0, alphaMax = 0, vp = 0;
  int    dir = 0;             // direction locked in when the ramp started
};

enum JogAxisId { JOG_AXIS_ROT, JOG_AXIS_A1, JOG_AXIS_A2, JOG_AXIS_Z };

// One scan leg's planned velocity shape -- see scanPlanMove().
struct ScanMove {
  bool   active   = false;    // following the plan
  bool   creeping = false;    // plan spent, closing the last of the gap
  int    dir      = 0;
  double creepV   = 0.0;      // pulses/s
  unsigned long t0 = 0;
  ProfilePlan plan;
};

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
void plcServiceLimitLatch();            // decides that end; plcOnGoodRead() calls it
bool runLegBlockedByLimit(float d1, float rot, float a2, String &why);
void applyMotionParams();
void applyJogVelocities();
bool scanOwnsRot();      // a profiled scan leg owns this axis right now
bool scanOwnsZ();
void reportMotionProfile();
void reachBandFor(double thMin, double thMax, double &rMin, double &rMax);

void reportLimits();
void resetLimitsToFactory();
void cancelJog();
void cancelRun();
void commandRunLeg();
void moveJointsAbsolute(float d1, float rot, float a1, float a2);
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


// ---- GEOMETRY -- mirrors robot_sim/config.py ----
// HOME is th3_cad 60 (base -30), R 133.2 mm; straight is th3_cad 180, R 613.2 mm.
// Only the two SUMS are used.
const double A3_MM = 45.0;
const double A4_MM = 160.0;
const double A5_MM = 160.0;
const double A6_MM = 248.2;

const double D_BASE_MM  = 388.0;
const double D3_ARM1_MM = 50.0;
const double D3_ARM2_MM = 41.0;
const double D4_MM = 46.5;
const double D5_MM = 24.8;
const double D6_MM = 5.0;

const double Z_OFFSET_ARM1_MM = D_BASE_MM + D3_ARM1_MM + D4_MM + D5_MM + D6_MM;
const double Z_OFFSET_ARM2_MM = D_BASE_MM + D3_ARM2_MM + D4_MM + D5_MM + D6_MM;

const double ARM_LINK_SUM_MM      = A4_MM + A5_MM;
const double ARM_RADIAL_OFFSET_MM = A3_MM + A6_MM;

const double I_RM_TOTAL = 1 * 6.5;

// motor deg = fold * armGearRatio, 0 at HOME -- the only figure stored anywhere
// fold deg  = th3_cad - ARM_ZERO_CAD_DEG, 0..120
// base deg  = fold - 30, what the operator reads: -30 HOME, +60 working max
const double ARM_ZERO_CAD_DEG = 60.0;

const double FOLD_ANGLE_HOME_DEG     = 0.0;
// base +60, R 570.3 mm. A NOTE, not a limit -- nothing here refuses a target past it.
const double FOLD_ANGLE_SPEC_MAX_DEG = 90.0;
const double FOLD_ANGLE_MIN_DEG      = FOLD_ANGLE_HOME_DEG;
const double FOLD_ANGLE_MAX_DEG      = 120.0;      // straight, base +90, R 613.2 mm
const double FOLD_SINGULARITY_WARN_DEG = 110.0;    // 10 deg short of straight

// MOTOR degrees per FROG-LEG degree. Measured on the machine -- not the model's 2.
// SET_ARM_RATIO changes it with no re-flash.
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
// RM zero is the CCW stop, not mid-travel.
const double ROT_MIN_DEG = 0.0, ROT_MAX_DEG = 340.0;

// OPERATOR-DEFINED WORKING LIMITS 
double limD1Min  = D1_MIN_MM,          limD1Max  = D1_MAX_MM;
double limRotMin = ROT_MIN_DEG,        limRotMax = ROT_MAX_DEG;
double limA1Min  = FOLD_ANGLE_MIN_DEG * ARM_GEAR_RATIO_DEF,
       limA1Max  = FOLD_ANGLE_MAX_DEG * ARM_GEAR_RATIO_DEF;
double limA2Min  = FOLD_ANGLE_MIN_DEG * ARM_GEAR_RATIO_DEF,
       limA2Max  = FOLD_ANGLE_MAX_DEG * ARM_GEAR_RATIO_DEF;
// PER-AXIS ENFORCEMENT, AND THE MASTER ENABLE
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

// ---- MOTOR CALIBRATION ----
const double MOTOR_STEPS_PER_REV  = 200.0;
const double MICROSTEPS_PER_STEP  = 16.0;
const double PULSES_PER_MOTOR_REV = MOTOR_STEPS_PER_REV * MICROSTEPS_PER_STEP;

const double ROT_GEAR_RATIO_DEF = I_RM_TOTAL;
const double ROT_GEAR_RATIO_MIN = 0.01, ROT_GEAR_RATIO_MAX = 1000.0;
double rotGearRatio = ROT_GEAR_RATIO_DEF;
double pulsesPerDegRot() { return (PULSES_PER_MOTOR_REV * rotGearRatio) / 360.0; }

// Pulses per MOTOR degree. Motor-to-link gearing is armGearRatio.
const double PULSES_PER_DEG_ARM_MOTOR = PULSES_PER_MOTOR_REV / 360.0;

const double Z_MICROSTEPS_PER_STEP  = 4.0;
const double PULSES_PER_MOTOR_REV_Z = MOTOR_STEPS_PER_REV * Z_MICROSTEPS_PER_STEP;

const double Z_MM_PER_REV_DEF = 20.0;
const double Z_MM_PER_REV_MIN = 0.1, Z_MM_PER_REV_MAX = 500.0;
double zMmPerRev = Z_MM_PER_REV_DEF;

double pulsesPerMmZ() { return PULSES_PER_MOTOR_REV_Z / zMmPerRev; }

const bool INVERT_Z    = false;
// RM's count must RISE moving away from M31: HOME and the scan find the switch by
// driving the count down.
const bool INVERT_ROT  = false;
const bool INVERT_ARM1 = false;
const bool INVERT_ARM2 = false;


// MOTION PROFILE — ONE UNIVERSAL RPM, ONE PERCENTAGE PER MOTOR
const float MASTER_RPM_NOMINAL = 140.0f;
const float ROT_RPM_SCALE = 140.0f   / MASTER_RPM_NOMINAL;
const float Z_RPM_SCALE   = 105.0f   / MASTER_RPM_NOMINAL;

const float ARM_RPM_SCALE = 1.0f;

const float MASTER_RPM_DEF     = 150.0f;
const float MASTER_ACC_DEF     = 300.0f;
// Bench results. Keep in step with robot_sim/config.py -- python_check asserts it.
const float ARM_PCT_DEF        = 62.5f;
const float ROT_PCT_DEF        = 50.0f;
const float Z_PCT_DEF          = 200.0f;

// Acceleration has its own defaults, not the speed ones.
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

// PLC LINK -- MELSEC MC PROTOCOL 3E, TCP 192.168.3.101:1025
#define PLC_IP_0 192
#define PLC_IP_1 168
#define PLC_IP_2 3
#define PLC_IP_3 101
const uint16_t PLC_PORT = 1025;

#define CC_IP_0 192
#define CC_IP_1 168
#define CC_IP_2 3
#define CC_IP_3 200

// M30..M32 are the only devices this board reads.

// Travel limit switches. A1M has none fitted.
// M32 is ZM's and M30 is A2M's -- swapped from the tidy order, measured on the machine.
const int PLC_M_LIMIT_Z   = 32;
const int PLC_M_LIMIT_ROT = 31;
const int PLC_M_LIMIT_A2  = 30;

const long     PLC_POLL_DEVICE_NUM = 0;
const uint16_t PLC_POLL_WORDS      = 3;   // M0..M47, so M30..M32 are covered

const unsigned long PLC_HOME_TIMEOUT_MS  = 30000;

const unsigned long PLC_POLL_IDLE_DEF_MS = 20;
unsigned long plcPollIdleMs = PLC_POLL_IDLE_DEF_MS;
const unsigned long PLC_POLL_HOMING_MS = 10;
const unsigned long PLC_TXN_TIMEOUT_MS     = 800;
const unsigned long PLC_RECONNECT_MS = 3000;

// COMMUNICATION DATA CODE MUST MATCH THE PLC'S OWN SETTING, EXACTLY: BINARY.

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

// M30..M32 limit lamps
#define PLC_LIMIT_LED_Z_PIN    IO3
#define PLC_LIMIT_LED_ROT_PIN  IO4
#define PLC_LIMIT_LED_A2_PIN   IO5
#define PLC_LIMIT_LED_PIN_NAMES "IO-3/IO-4/IO-5"
const unsigned long PLC_LIMIT_LED_BLINK_MS = 250;

// ---- 340 DEGREE SCAN -- distance sensor on the arm, one sweep per layer ----
// IO-3/4/5 are the PLC limit lamps, so the scan takes IO-0 and IO-1.
#define SCAN_TRIG_PIN    IO0    // ultrasonic trigger out
#define SCAN_ECHO_PIN    IO1    // ultrasonic echo in
#define SCAN_ANALOG_PIN  A9     // analog laser / IR distance in

// Two sensor types, chosen at runtime with SET_SCAN_SENSOR.
enum ScanSensorKind { SCAN_SENSOR_ULTRASONIC = 0, SCAN_SENSOR_ANALOG = 1 };
int scanSensorKind = SCAN_SENSOR_ULTRASONIC;

// Ultrasonic (HC-SR04 family): 10 us trigger, echo width is the round trip.
// A 30 ms echo is about 5 m.
const unsigned long SCAN_TRIG_US       = 10;
const unsigned long SCAN_ECHO_TIMEOUT_US = 30000;
const double SCAN_MM_PER_US = 0.1715;        // 343 m/s, halved for the return trip

// Analog: raw counts -> mm, a straight line. Both zero until SET_SCAN_CAL, so an
// uncalibrated sensor reads 0 mm.
double scanAnalogMmPerCount = 0.0;
double scanAnalogOffsetMm   = 0.0;

// Fallback sweep speed scale, used when SCAN_START carries no deg/s.
const float SCAN_SPEED_SCALE = 0.20f;

// Requested sweep speed, deg/s at the output; 0 = none given. Clamped to what RM can do.
double scanRotDegS = 0.0;
const double SCAN_ROT_DEG_S_MIN = 0.01;

const double SCAN_SWEEP_DEG_DEF  = 340.0;   // the turntable's whole travel
// A shorter sweep is allowed; one longer than the travel is refused.
const double SCAN_SWEEP_DEG_MIN  = 1.0;
const double SCAN_DEG_STEP_MIN   = 0.10;
const double SCAN_DEG_STEP_MAX   = 90.0;   // a quarter turn, the coarsest that still means anything
const double SCAN_Z_STEP_MIN_MM  = 0.10;
const int    SCAN_LAYERS_MAX     = 500;

// Arrival tolerance: positions are step counts, so 340 deg reads 339.998.
const double SCAN_ANGLE_EPS_DEG  = 0.05;
const double SCAN_Z_EPS_MM       = 0.02;

// SEEK finds the RM switch, the scan's reference. RETURN goes back to the start pose
// once every layer is in.
enum ScanPhase { SCAN_OFF, SCAN_SEEK, SCAN_SWEEP, SCAN_LIFT, SCAN_RETURN };
ScanPhase scanPhase = SCAN_OFF;

// Give up seeking after this much travel -- a miswired switch.
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

// -1 = axis minimum, +1 = axis maximum. All three switches sit at the HOME end.
const int PLC_LIMIT_END_Z   = -1;
const int PLC_LIMIT_END_ROT = -1;
const int PLC_LIMIT_END_A2  = -1;

// A2M's switch is wired at BOTH ends. PLC_LIMIT_END_A2 is its HOME-side end; which end
// actually tripped is worked out by plcLimitEndFor().
const bool PLC_LIMIT_BOTH_ENDS_Z   = false;
const bool PLC_LIMIT_BOTH_ENDS_ROT = false;
const bool PLC_LIMIT_BOTH_ENDS_A2  = true;


  #include <Ethernet.h>
  byte          plcMac[]  = {0x24, 0x15, 0x10, 0xB0, 0x00, 0x01};
  IPAddress     plcLocalIp(CC_IP_0, CC_IP_1, CC_IP_2, CC_IP_3);
  IPAddress     plcTargetIp(PLC_IP_0, PLC_IP_1, PLC_IP_2, PLC_IP_3);
  EthernetClient plcClient;
  bool          plcReportedError = false;
  unsigned long plcLastConnectTry = 0;
  unsigned long plcLastConnectLog = 0;


const unsigned long ALIVE_INTERVAL_MS       = 2000;

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

// ---- MOTION PROFILE -- the shape of a run leg's ramp ----
// Ported from Compare_Angular_Motion_Profiles.m. The step generator only does a
// trapezoid, so a profiled leg is INTERPOLATED: every pass commands each axis to
// start + u*(target-start), with one u shared by all four axes.
MotionProfileKind motionProfile = PROFILE_NONE;

// 0.5 eases half the ramp; 1 is the pure S-curve.
const double SCURVE_RATIO = 0.5;

ProfilePlan runPlan;
bool          runProfileActive = false;
unsigned long runProfileT0 = 0;

// TEST_MOVE: one motor, out and back, with the test's own profile.
bool   testActive = false;
int    testAxis = 0;                       // 0 ZM, 1 RM, 2 A1M, 3 A2M
double testVel = 0.0;                      // that axis's own units per second
double testAcc = 0.0;                      // ...and per second squared
MotionProfileKind testProfile = PROFILE_NONE;
unsigned long testLegT0 = 0, testOutMs = 0;

MotionProfileKind legProfile() { return testActive ? testProfile : motionProfile; }

// generateTrapezoidalAngular(), reduced to its timing.
void planTrapezoid(ProfilePlan &p, double Theta, double omegaMax, double alphaMax) {
  p.sCurve = false;
  p.Theta = Theta;
  p.alphaMax = alphaMax;
  double thetaMin = omegaMax * omegaMax / alphaMax;
  if (Theta >= thetaMin) {
    p.vp = omegaMax;
    p.ta = p.vp / alphaMax;
    p.tv = Theta / p.vp - p.ta;
  } else {
    // Too short to reach omegaMax: the trapezium becomes a triangle.
    p.vp = sqrt(Theta * alphaMax);
    p.ta = p.vp / alphaMax;
    p.tv = 0.0;
  }
  p.T = 2.0 * p.ta + p.tv;
}

// generateSCurveAngular(). r = 1 gives the pure S-curve.
void planSCurve(ProfilePlan &p, double Theta, double omegaMax,
                double alphaMax, double r) {
  p.sCurve = true;
  p.Theta = Theta;
  p.alphaMax = alphaMax;

  double taMax = (1.0 + r) * omegaMax / alphaMax;
  double thetaMin = omegaMax * taMax;
  double vp = (Theta >= thetaMin) ? omegaMax
                                  : sqrt(Theta * alphaMax / (1.0 + r));
  double tJ = r * vp / alphaMax;
  double tA = (1.0 - r) * vp / alphaMax;
  double tV = Theta / vp - (2.0 * tJ + tA);
  if (tV < 1e-12) tV = 0.0;
  double J = (tJ > 0.0) ? alphaMax / tJ : 0.0;

  const double durs[7] = {tJ, tA, tJ, tV, tJ, tA, tJ};
  const double jerks[7] = {J, 0.0, -J, 0.0, -J, 0.0, J};
  p.T = 0.0;
  for (int i = 0; i < 7; i++) {
    p.dur[i] = durs[i];
    p.jrk[i] = jerks[i];
    p.T += durs[i];
  }

  // State at the START of each phase, integrated forward once.
  p.s0[0] = p.v0[0] = p.a0[0] = 0.0;
  p.tStart[0] = 0.0;
  for (int k = 1; k < 7; k++) {
    double h = p.dur[k - 1], j = p.jrk[k - 1];
    p.a0[k] = p.a0[k - 1] + j * h;
    p.v0[k] = p.v0[k - 1] + p.a0[k - 1] * h + 0.5 * j * h * h;
    p.s0[k] = p.s0[k - 1] + p.v0[k - 1] * h + 0.5 * p.a0[k - 1] * h * h
            + j * h * h * h / 6.0;
    p.tStart[k] = p.tStart[k - 1] + p.dur[k - 1];
  }
}

// Displacement at time t; pinned to Theta past the end.
double profileAt(const ProfilePlan &p, double t) {
  if (t <= 0.0) return 0.0;
  if (t >= p.T) return p.Theta;

  if (!p.sCurve) {
    if (t <= p.ta) return 0.5 * p.alphaMax * t * t;
    double thetaA = 0.5 * p.alphaMax * p.ta * p.ta;
    if (t <= p.ta + p.tv) return thetaA + p.vp * (t - p.ta);
    double tau = t - (p.ta + p.tv);
    return thetaA + p.vp * p.tv + p.vp * tau - 0.5 * p.alphaMax * tau * tau;
  }

  // First phase that contains t AND has a length -- at r = 1 two phases are zero length.
  int k = 6;
  double tEnd = 0.0;
  for (int i = 0; i < 7; i++) {
    tEnd += p.dur[i];
    if (t <= tEnd + 1e-12 && p.dur[i] > 1e-14) { k = i; break; }
  }
  double tau = t - p.tStart[k];
  if (tau < 0.0) tau = 0.0;
  if (tau > p.dur[k]) tau = p.dur[k];
  double j = p.jrk[k];
  return p.s0[k] + p.v0[k] * tau + 0.5 * p.a0[k] * tau * tau
       + j * tau * tau * tau / 6.0;
}

// Plans the current leg on u = 0..1 from runStart*/runTarget*, at the tightest limit of
// the four axes. False = nothing to plan; the caller falls back to plain Move().
bool planRunProfile() {
  const double dz  = fabs((double)runTargetD1  - runStartD1);
  const double dr  = fabs((double)runTargetRot - runStartRot);
  const double da1 = fabs((double)runTargetA1  - runStartA1);
  const double da2 = fabs((double)runTargetA2  - runStartA2);
  if (dz < 1e-6 && dr < 1e-6 && da1 < 1e-6 && da2 < 1e-6) return false;

  const double deltas[4] = {dz, dr, da1, da2};
  double vmax[4] = {zVelMmS, rotVelDegS, armVelDegS, armVelDegS};
  if (testActive) vmax[testAxis] = testVel;     // the typed RPM, not the setting
  double amax[4] = {zAccMmS2, rotAccDegS2, armAccDegS2, armAccDegS2};
  // The test axis runs at the TYPED accel; handleTestMove() set AccelMax to it.
  if (testActive) amax[testAxis] = testAcc;

  double omegaU = 0.0, alphaU = 0.0;
  for (int i = 0; i < 4; i++) {
    if (deltas[i] < 1e-9) continue;          // this axis is not moving
    if (vmax[i] <= 0.0 || amax[i] <= 0.0) return false;
    const double vu = vmax[i] / deltas[i];
    const double au = amax[i] / deltas[i];
    if (omegaU <= 0.0 || vu < omegaU) omegaU = vu;
    if (alphaU <= 0.0 || au < alphaU) alphaU = au;
  }
  if (omegaU <= 0.0 || alphaU <= 0.0) return false;

  switch (legProfile()) {
    case PROFILE_TRAPEZOID:   planTrapezoid(runPlan, 1.0, omegaU, alphaU); break;
    case PROFILE_SCURVE:      planSCurve(runPlan, 1.0, omegaU, alphaU, SCURVE_RATIO); break;
    case PROFILE_PURE_SCURVE: planSCurve(runPlan, 1.0, omegaU, alphaU, 1.0); break;
    default: return false;
  }
  return runPlan.T > 1e-6;
}

// Every run leg starts here. With no profile: one absolute Move per axis.
void commandRunLeg() {
  runProfileActive = false;
  if (legProfile() != PROFILE_NONE && planRunProfile()) {
    runProfileT0 = millis();
    runProfileActive = true;
    // Nothing commanded yet: at u = 0 the setpoint is where the axes already are.
    return;
  }
  moveJointsAbsolute(runTargetD1, runTargetRot, runTargetA1, runTargetA2);
}

const char *profileNameOf(MotionProfileKind k) {
  switch (k) {
    case PROFILE_TRAPEZOID:   return "TRAPEZOIDAL";
    case PROFILE_SCURVE:      return "SCURVE";
    case PROFILE_PURE_SCURVE: return "PURE_SCURVE";
    default:                  return "NONE";
  }
}
const char *motionProfileName() { return profileNameOf(motionProfile); }

bool parseProfileToken(const String &kind, MotionProfileKind &out) {
  if      (kind == "NONE")        out = PROFILE_NONE;
  else if (kind == "TRAPEZOIDAL") out = PROFILE_TRAPEZOID;
  else if (kind == "SCURVE")      out = PROFILE_SCURVE;
  else if (kind == "PURE_SCURVE") out = PROFILE_PURE_SCURVE;
  else return false;
  return true;
}
unsigned long lastRunReportTime = 0;

int rotDir = 0, a1Dir = 0, a2Dir = 0, jzDir = 0;

// ---- XYZ JOG: the tool point along Cartesian X / Y / Z ----
// XJOG:<arm>,<sx>,<sy>,<sz>,<mm/s>   signs -1/0/+1; all zero (or XJOG_STOP) eases out.
// A Cartesian setpoint is walked every XJOG_TICK_MS through IK and commanded with
// Move(ABSOLUTE). Every check runs BEFORE the step: a re-commanded setpoint cannot be
// stopped by zeroing a direction afterwards.
const unsigned long XJOG_TICK_MS       = 5;
const double XJOG_ACCEL_MM_S2          = 300.0;
// No speed ceiling: a release stops inside this many seconds whatever the speed.
const double XJOG_EASE_MAX_S           = 0.5;
const double XJOG_JOINT_HEADROOM       = 0.8;
const double XJOG_MAX_DT_S             = 0.05;   // a stalled pass must not become a jump
// A shortened step still asking a joint for this many times its speed is an IK branch
// jump: refused.
const double XJOG_JUMP_RATIO           = 1.5;
// RM resting this far past an end of its travel (a jog coasts) still starts.
const double XJOG_START_SNAP_DEG       = 0.5;

bool   xjogActive = false;
int    xjogArm = 1;
int    xjogDir[3] = {0, 0, 0};          // held sign per X / Y / Z
double xjogVel[3] = {0.0, 0.0, 0.0};    // mm/s, ramping toward dir * speed
double xjogSpeed  = 0.0;                // mm/s
double xjogEase[3] = {0.0, 0.0, 0.0};   // mm/s^2 an axis is slowing at; 0 = not slowing
double xjogPos[3] = {0.0, 0.0, 0.0};    // the setpoint: X, Y, Z-from-HOME
double xjogRot = 0.0, xjogMotor = 0.0;  // joint targets last commanded (deg, motor deg)
int    xjogJointDir[3] = {0, 0, 0};     // Z / ROT / A2 -- the PLC helpers' order
unsigned long xjogLastMs = 0;

bool xjogHeld() { return xjogDir[0] || xjogDir[1] || xjogDir[2]; }

// State only -- the caller stops the motors.
void xjogClear() {
  xjogActive = false;
  for (int i = 0; i < 3; i++) {
    xjogDir[i] = 0; xjogVel[i] = 0.0; xjogJointDir[i] = 0; xjogEase[i] = 0.0;
  }
}

// ---- JOG RAMP: ease up on a held key, ease down on a voluntary release ----
// S-curve profiles only. Never used by a safety stop: those call MoveVelocity(0).
JogRamp jogRampRot, jogRampA1, jogRampA2, jogRampZ;


bool jogRampWanted() {
  return motionProfile == PROFILE_SCURVE || motionProfile == PROFILE_PURE_SCURVE;
}

// Velocity at time t into an ease from rest to vp.
double jogRampV(const JogRamp &r, double t) {
  double T = 2.0 * r.tJ + r.tA;
  if (t <= 0.0)  return 0.0;
  if (t >= T)    return r.vp;
  if (t <= r.tJ) return 0.5 * r.J * t * t;
  double v1 = 0.5 * r.alphaMax * r.tJ;
  if (t <= r.tJ + r.tA) return v1 + r.alphaMax * (t - r.tJ);
  double tau = t - (r.tJ + r.tA);
  double v2 = v1 + r.alphaMax * r.tA;
  return v2 + r.alphaMax * tau - 0.5 * r.J * tau * tau;
}

// Arms a fresh ramp-up at this axis's own jog speed and accel (pulses/s, pulses/s^2).
void armJogRamp(JogRamp &r, int dir, double vp, double alphaMax) {
  r.dir      = dir;
  r.vp       = fabs(vp);
  r.alphaMax = fabs(alphaMax);
  r.active = r.releasing = false;
  if (!jogRampWanted() || r.vp <= 1e-9 || r.alphaMax <= 1e-9) return;
  double rS = (motionProfile == PROFILE_PURE_SCURVE) ? 1.0 : SCURVE_RATIO;
  r.tJ = rS * r.vp / r.alphaMax;
  r.tA = (1.0 - rS) * r.vp / r.alphaMax;
  r.J  = (r.tJ > 0.0) ? r.alphaMax / r.tJ : 0.0;
  r.t0 = millis();
  r.active = true;
}

// Starts the release ease, only from a steady jog. False = the caller hard-stops.
bool releaseJogRamp(JogRamp &r) {
  bool wasSteady = jogRampWanted() && !r.active && r.vp > 1e-9;
  r.active = false;
  if (!wasSteady) { r.releasing = false; return false; }
  r.t0 = millis();
  r.releasing = true;
  return true;
}

// Velocity command for one mid-ease axis this pass. False = nothing to send.
bool jogRampTick(JogRamp &r, int liveDir, int32_t &pulsesOut) {
  if (!r.active && !r.releasing) return false;
  if (r.active && liveDir == 0) { r.active = false; return false; }  // safety stop won this tick

  double t = (millis() - r.t0) / 1000.0;
  double T = 2.0 * r.tJ + r.tA;
  double v = r.active ? jogRampV(r, t) : jogRampV(r, T - t);
  pulsesOut = (int32_t)lround(r.dir * v);

  if (r.active    && t >= T) r.active    = false;   // now steady at vp
  if (r.releasing && t >= T) r.releasing = false;    // now at rest
  return true;
}

// Every loop() pass: drives only the axes mid-ease.
void serviceJogRamps() {
  int32_t p;
  if (jogRampTick(jogRampRot, rotDir, p)) MOTOR_ROT.MoveVelocity(p * (INVERT_ROT  ? -1 : 1));
  if (jogRampTick(jogRampA1,  a1Dir,  p)) MOTOR_A1.MoveVelocity(p * (INVERT_ARM1 ? -1 : 1));
  if (jogRampTick(jogRampA2,  a2Dir,  p)) MOTOR_A2.MoveVelocity(p * (INVERT_ARM2 ? -1 : 1));
  if (jogRampTick(jogRampZ,   jzDir,  p)) MOTOR_Z.MoveVelocity(p * (INVERT_Z    ? -1 : 1));
}

unsigned long lastJogReportTime = 0;

bool isHoming = false;
// Which axes HOME is still driving. Up here because cancelHoming() needs it too.
bool homeAxisActive[3] = {false, false, false};
// HOME started with a both-ends switch tripped at its FAR end: that axis drives off it
// before the bit means arrival again.
bool homeWaitForClear[3] = {false, false, false};
unsigned long lastHomeReportTime = 0;
unsigned long homeRequestedAt = 0;
bool isHomed = false;
unsigned long lastAliveTime = 0;

unsigned long lastJogKeepAlive = 0;

bool          ledOn    = false;
unsigned long ledOffAt = 0;


// ---- LED (non-blocking -- never call delay() here) ----
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


// INVERSE / FORWARD KINEMATICS

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

  // Physical stroke only: taught boundaries do not gate a P2P solve.
  double d1 = Z - zOffsetForArm(arm);
  if (d1 < D1_MIN_MM - 1e-6 || d1 > D1_MAX_MM + 1e-6) {
    r.error = "[ERROR] Z=" + String(d1, 2) + " from HOME is out of ZM travel "
              "(the stroke is " + String(D1_MIN_MM, 1) + ".." + String(D1_MAX_MM, 1)
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

  // Physical travel: 340..360 is the wedge RM cannot reach from either side.
  if (th2 < ROT_MIN_DEG - 1e-6 || th2 > ROT_MAX_DEG + 1e-6) {
    r.error = "[ERROR] ROT=" + String(th2, 2) + " deg is outside RM's travel ["
            + String(ROT_MIN_DEG, 1) + ", " + String(ROT_MAX_DEG, 1) + "] - the "
              "turntable cannot reach that bearing from either side";
    return r;
  }

  if (R < ARM_RADIAL_OFFSET_MM - ARM_LINK_SUM_MM - 1e-6
      || R > ARM_RADIAL_OFFSET_MM + ARM_LINK_SUM_MM + 1e-6) {
    r.error = "[ERROR] R=" + String(R, 2) + " mm has no solution: the frog-leg "
              "spans a3+a6 +/- (a4+a5) = " + String(ARM_RADIAL_OFFSET_MM, 1)
            + " +/- " + String(ARM_LINK_SUM_MM, 1) + " mm";
    return r;
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


// ---- MOTOR HELPERS ----
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


// Physical travel only (ZM's stroke, RM's travel). Taught boundaries do not gate a P2P
// target; the elbows are bounded by the frog-leg arithmetic in solveIkFrogleg().
bool jointTargetIsLegal(float d1, float rot, float a1, float a2, String &why) {
  (void)a1; (void)a2;
  if (d1 < D1_MIN_MM - 0.01 || d1 > D1_MAX_MM + 0.01) {
    why = "d1=" + String(d1, 2) + " outside ZM's travel [" + String(D1_MIN_MM, 1)
        + ", " + String(D1_MAX_MM, 1) + "] mm"; return false;
  }
  if (rot < ROT_MIN_DEG - 0.01 || rot > ROT_MAX_DEG + 0.01) {
    why = "rot=" + String(rot, 2) + " outside RM's travel [" + String(ROT_MIN_DEG, 1)
        + ", " + String(ROT_MAX_DEG, 1) + "] deg"; return false;
  }
  return true;
}


// OPERATOR LIMIT EDITING
bool applyLimit(const String &axis, bool isMax, double value, String &why) {
  double *lo, *hi, floorV = 0, ceilV = 0, minSpan = 0;
  bool taught = false;
  String unit;

  if      (axis == "Z")   { lo=&limD1Min;  hi=&limD1Max;  floorV=D1_MIN_MM;
                            ceilV=D1_MAX_MM; minSpan=LIMIT_MIN_SPAN_MM;  unit=" mm"; }
  else if (axis == "ROT") { lo=&limRotMin; hi=&limRotMax; floorV=ROT_MIN_DEG;
                            ceilV=ROT_MAX_DEG; minSpan=LIMIT_MIN_SPAN_DEG; unit=" deg"; }
  // Elbows: no envelope, unordered.
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


// REPORTING
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
                                       : isHomed ? "yes" : "yes (unreferenced)"));
  sendFeedback(String("[LIMIT_ENFORCE] master=") + (limitsEnabled ? "yes" : "NO")
             + " | enforced: Z=" + String(limZEnforced ? 1 : 0)
             + " ROT=" + String(limRotEnforced ? 1 : 0)
             + " A1=" + String(limA1Enforced ? 1 : 0)
             + " A2=" + String(limA2Enforced ? 1 : 0));
}


// ---- MOTION CANCELLATION ----
void cancelJog() {
  cancelScan("another motion command took over");
  rotDir = a1Dir = a2Dir = jzDir = 0;
  // Before the MoveVelocity(0)s: left armed, the next XYZ tick re-commands its setpoint.
  xjogClear();
  // Clear the ramps too, or serviceJogRamps() re-issues a velocity over the stop.
  jogRampRot.active = jogRampRot.releasing = false;
  jogRampA1.active  = jogRampA1.releasing  = false;
  jogRampA2.active  = jogRampA2.releasing  = false;
  jogRampZ.active   = jogRampZ.releasing   = false;
  MOTOR_ROT.MoveVelocity(0);
  MOTOR_A1.MoveVelocity(0);
  MOTOR_A2.MoveVelocity(0);
  MOTOR_Z.MoveVelocity(0);
}

void cancelRun() {
  isMoving = false;
  runPhase = PHASE_NONE;
  // The interpolator stops with it.
  runProfileActive = false;
  // A test raised one motor's VelMax to the typed RPM; put it back.
  if (testActive) { testActive = false; applyMotionParams(); }
}

// HOME drives the motors itself, so cancelling has to stop them.
void cancelHoming() {
  isHoming = false;
  for (int i = 0; i < 3; i++) { homeAxisActive[i] = false; homeWaitForClear[i] = false; }
  jzDir = rotDir = a2Dir = 0;
  applyJogVelocities();
}


// ---- PROGRAM LOADING ----
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
  reportSingularityIfNear(armFoldFromMotor(max(a1a, a2a)), "A");
  reportSingularityIfNear(armFoldFromMotor(max(a1b, a2b)), "B");
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
  reportSingularityIfNear(armFoldFromMotor(a1), "A1M");
  reportSingularityIfNear(armFoldFromMotor(a2), "A2M");
  return true;
}


// ---- COMMAND PARSING HELPERS ----
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


// EVERY Z ON THE WIRE IS MEASURED FROM HOME.
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
  commandRunLeg();
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
  storeDual((float)r1.d1, (float)r1.th2,
            (float)armMotorFromFold(r1.th3 - FOLD_ANGLE_HOME_DEG),
            (float)armMotorFromFold(r2.th3 - FOLD_ANGLE_HOME_DEG));
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


// ---- RUN EXECUTION ----
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
  commandRunLeg();
  isMoving = true;
  lastRunReportTime = millis();
}

void beginRunLeg(RunPhase phase, float d1, float rot, float a1, float a2,
                 bool skipSensorBlock = false) {
  if (!skipSensorBlock) {
    String whyLimit;
    if (runLegBlockedByLimit(d1, rot, a2, whyLimit)) {
      cancelRun();
      sendFeedback("[ERROR] RUN stopped — " + whyLimit + ".");
      sendFeedback("[WARN] Jog that axis off its limit, then RUN again.");
      return;
    }
  }
  runPhase = phase;
  runStartD1 = currentD1(); runStartRot = currentRot();
  runStartA1 = currentA1(); runStartA2 = currentA2();
  runTargetD1 = d1; runTargetRot = rot; runTargetA1 = a1; runTargetA2 = a2;
  commandRunLeg();
}

// TEST_MOVE:<Z|ROT|A1|A2>,<target>,<rpm>,<NONE|TRAPEZOIDAL|SCURVE|PURE_SCURVE>[,<acc>]
// <target> is ABSOLUTE in the axis's own units (mm, turntable deg, arm MOTOR deg); the
// test goes there and back. <rpm> and <acc> (RPM/s) are the motor's, with no upper
// limit; <acc> left out is the axis's applied accel.
void handleTestMove(const String &payload) {
  if (isMoving || isHoming || scanPhase != SCAN_OFF || anyJogActive()) {
    sendFeedback("[ERROR] TEST refused - the machine is already moving.");
    return;
  }
  int c1 = payload.indexOf(',');
  int c2 = payload.indexOf(',', c1 + 1);
  int c3 = payload.indexOf(',', c2 + 1);
  if (c1 < 0 || c2 < 0 || c3 < 0) {
    sendFeedback("[ERROR] TEST_MOVE needs motor,target,rpm,profile");
    return;
  }
  String axis = payload.substring(0, c1); axis.trim(); axis.toUpperCase();
  double want = payload.substring(c1 + 1, c2).toDouble();
  double rpm = payload.substring(c2 + 1, c3).toDouble();
  int c4 = payload.indexOf(',', c3 + 1);
  String kind = c4 < 0 ? payload.substring(c3 + 1) : payload.substring(c3 + 1, c4);
  kind.trim(); kind.toUpperCase();
  const bool accGiven = c4 >= 0;
  double accRpmS = accGiven ? payload.substring(c4 + 1).toDouble() : 1.0;
  MotionProfileKind prof;
  if (!parseProfileToken(kind, prof)) {
    sendFeedback("[ERROR] TEST_MOVE profile must be NONE, TRAPEZOIDAL, SCURVE or "
                 "PURE_SCURVE, got " + kind);
    return;
  }
  int i = axis == "Z" ? 0 : axis == "ROT" ? 1 : axis == "A1" ? 2 : axis == "A2" ? 3 : -1;
  if (i < 0) {
    sendFeedback("[ERROR] TEST_MOVE motor must be Z, ROT, A1 or A2, got \"" + axis + "\"");
    return;
  }
  if (!(rpm > 0.0)) {
    sendFeedback("[ERROR] TEST_MOVE RPM must be above 0");
    return;
  }
  if (!(accRpmS > 0.0)) {
    sendFeedback("[ERROR] TEST_MOVE accel must be above 0 RPM/s");
    return;
  }

  // One motor degree in this axis's own units: mm, turntable deg, motor deg.
  const double perMotorDeg = (i == 0) ? zMmPerRev / 360.0
                           : (i == 1) ? 1.0 / rotGearRatio : 1.0;
  const char *units[4] = {" mm", " deg", " motor deg", " motor deg"};
  float target[4] = {currentD1(), currentRot(), currentA1(), currentA2()};
  target[i] = (float)want;

  // Only the target is new: physical travel always, the taught band when enforced.
  String why;
  if (!jointTargetIsLegal(target[0], target[1], target[2], target[3], why)) {
    sendFeedback("[ERROR] TEST refused - " + why);
    return;
  }
  const char *names[4] = {"Z", "ROT", "A1", "A2"};
  if (axisLimited(names[i])) {
    double lo, hi;
    if (i == 0)      { lo = limD1Min;  hi = limD1Max; }
    else if (i == 1) { lo = limRotMin; hi = limRotMax; }
    else             armBand(i == 2 ? 1 : 2, lo, hi);
    if (target[i] > hi + 0.01 || target[i] < lo - 0.01) {
      sendFeedback("[ERROR] TEST refused - " + String(target[i], 2) + String(units[i])
                 + " is outside " + String(names[i]) + "'s taught band "
                 + String(lo, 2) + ".." + String(hi, 2) + String(units[i]) + ".");
      return;
    }
  }

  testAxis = i;
  testProfile = prof;
  testVel = rpm * 6.0 * perMotorDeg;         // rpm * 360 / 60 motor deg/s
  const double appliedAcc[4] = {zAccMmS2, rotAccDegS2, armAccDegS2, armAccDegS2};
  const int32_t appliedAccPulses[4] = {zAccelPulses, rotAccelPulses,
                                       armAccelPulses, armAccelPulses};
  testAcc = accGiven ? accRpmS * 6.0 * perMotorDeg : appliedAcc[i];
  // Lift the motor's own VelMax / AccelMax to the typed values for the test.
  const double ppr = (i == 0) ? PULSES_PER_MOTOR_REV_Z : PULSES_PER_MOTOR_REV;
  int32_t velPulses = (int32_t)lround(rpm / 60.0 * ppr);
  int32_t accPulses = accGiven ? (int32_t)lround(accRpmS / 60.0 * ppr)
                               : appliedAccPulses[i];
  if (velPulses < 1) velPulses = 1;
  if (accPulses < 1) accPulses = 1;
  if (i == 0)      { MOTOR_Z.VelMax(velPulses);   MOTOR_Z.AccelMax(accPulses); }
  else if (i == 1) { MOTOR_ROT.VelMax(velPulses); MOTOR_ROT.AccelMax(accPulses); }
  else if (i == 2) { MOTOR_A1.VelMax(velPulses);  MOTOR_A1.AccelMax(accPulses); }
  else             { MOTOR_A2.VelMax(velPulses);  MOTOR_A2.AccelMax(accPulses); }
  testActive = true;

  sendFeedback("[TEST] " + String(names[i]) + " to " + String(want, 2)
             + String(units[i]) + " and back, at " + String(rpm, 1) + " motor RPM, "
             + (accGiven ? String(accRpmS, 1) + " RPM/s" : String("applied accel"))
             + ", " + String(profileNameOf(prof)) + ".");
  isMoving = true;
  lastRunReportTime = millis();
  testLegT0 = millis();
  beginRunLeg(PHASE_TEST_OUT, target[0], target[1], target[2], target[3]);
}

void beginResetPosition() {
  if (isMoving || isHoming) {
    sendFeedback("[ERROR] RESET_POSITION refused — the machine is already moving.");
    return;
  }
  String why;
  if (!jointTargetIsLegal(Z_HOME_MM_BOARD, ROT_HOME_DEG_BOARD,
                          ARM_HOME_MOTOR_DEG, ARM_HOME_MOTOR_DEG, why)) {
    sendFeedback("[ERROR] RESET_POSITION refused — home is outside the physical "
                 "travel (" + why + ").");
    return;
  }
  cancelJog();
  sendFeedback("[RESET_POSITION] Moving to (0,0,0,0) under the board's own motor "
               "control -- no PLC handshake, M30..M32 leg block skipped.");
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

  // Walk the profile BEFORE the settled test: between setpoints the axes ARE settled.
  if (runProfileActive) {
    double t = (now - runProfileT0) / 1000.0;
    if (t < runPlan.T) {
      double u = profileAt(runPlan, t) / runPlan.Theta;
      moveJointsAbsolute(
        runStartD1  + (float)(u * (runTargetD1  - runStartD1)),
        runStartRot + (float)(u * (runTargetRot - runStartRot)),
        runStartA1  + (float)(u * (runTargetA1  - runStartA1)),
        runStartA2  + (float)(u * (runTargetA2  - runStartA2)));
      return;
    }
    // Time is up: command the exact target once.
    runProfileActive = false;
    moveJointsAbsolute(runTargetD1, runTargetRot, runTargetA1, runTargetA2);
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

  if (runPhase == PHASE_TEST_OUT) {
    testOutMs = millis() - testLegT0;
    testLegT0 = millis();
    beginRunLeg(PHASE_TEST_BACK, runStartD1, runStartRot, runStartA1, runStartA2);
    return;
  }

  reportRunPosition(100);
  isMoving = false;
  bool wasReset = (runPhase == PHASE_RESET_HOME);
  bool wasTest  = (runPhase == PHASE_TEST_BACK);
  runPhase = PHASE_NONE;
  if (wasTest) {
    testActive = false;
    applyMotionParams();                  // the motor's own VelMax again
    sendFeedback("[TEST] DONE - out " + String(testOutMs / 1000.0, 2) + " s, back "
               + String((millis() - testLegT0) / 1000.0, 2) + " s.");
    return;
  }
  if (wasReset) {
    sendFeedback("[RESET_POSITION] TARGET REACHED");
    return;
  }
  sendFeedback("[RUN] TARGET REACHED");
}


// Homing speed as a fraction of jog speed. The switch state arrives a poll late, so
// slow means a short overrun.
const float HOME_SPEED_SCALE = 0.25f;

// Which way HOME drives, per axis. Separate from PLC_LIMIT_END_* on purpose, though
// the two agree axis by axis.
const int HOME_DIR_Z   = -1;
const int HOME_DIR_ROT = -1;
const int HOME_DIR_A2  = -1;

int homeDirFor(int i) {
  const int dirs[3] = {HOME_DIR_Z, HOME_DIR_ROT, HOME_DIR_A2};
  return dirs[i];
}

// RM only; ZM keeps SCAN_SPEED_SCALE.
float scanRotScale() {
  if (scanRotDegS < SCAN_ROT_DEG_S_MIN || rotVelDegS <= 0.0f) return SCAN_SPEED_SCALE;
  float s = (float)(scanRotDegS / (double)rotVelDegS);
  return s > 1.0f ? 1.0f : s;      // never faster than the axis is configured for
}

// An axis mid-ease is left alone here: serviceJogRamps() owns it.
void applyJogVelocities() {
  // An XYZ jog owns the motors through Move().
  if (xjogActive) return;
  float scale = isHoming ? HOME_SPEED_SCALE
              : (scanPhase != SCAN_OFF ? SCAN_SPEED_SCALE : 1.0f);
  float rotScale = (!isHoming && scanPhase != SCAN_OFF) ? scanRotScale() : scale;
  int32_t rotV = (int32_t)(rotVelPulses * boostMultiplier * rotScale);
  int32_t armV = (int32_t)(armVelPulses * boostMultiplier * scale);
  int32_t zV   = (int32_t)(zVelPulses   * boostMultiplier * scale);

  if (!jogRampRot.active && !jogRampRot.releasing && !scanOwnsRot())
    MOTOR_ROT.MoveVelocity(rotDir * rotV * (INVERT_ROT ? -1 : 1));
  if (!jogRampA1.active && !jogRampA1.releasing)
    MOTOR_A1.MoveVelocity(a1Dir * armV * (INVERT_ARM1 ? -1 : 1));
  if (!jogRampA2.active && !jogRampA2.releasing)
    MOTOR_A2.MoveVelocity(a2Dir * armV * (INVERT_ARM2 ? -1 : 1));
  if (!jogRampZ.active && !jogRampZ.releasing && !scanOwnsZ())
    MOTOR_Z.MoveVelocity(jzDir * zV * (INVERT_Z ? -1 : 1));
}

// Arms axisId's ramp for a fresh key-down. Call BEFORE applyJogVelocities().
void armJogAxisRamp(JogAxisId axisId, int dir) {
  float scale = isHoming ? HOME_SPEED_SCALE
              : (scanPhase != SCAN_OFF ? SCAN_SPEED_SCALE : 1.0f);
  float rotScale = (!isHoming && scanPhase != SCAN_OFF) ? scanRotScale() : scale;
  switch (axisId) {
    case JOG_AXIS_ROT:
      armJogRamp(jogRampRot, dir, rotVelPulses * boostMultiplier * rotScale, rotAccelPulses);
      break;
    case JOG_AXIS_A1:
      armJogRamp(jogRampA1, dir, armVelPulses * boostMultiplier * scale, armAccelPulses);
      break;
    case JOG_AXIS_A2:
      armJogRamp(jogRampA2, dir, armVelPulses * boostMultiplier * scale, armAccelPulses);
      break;
    case JOG_AXIS_Z:
      armJogRamp(jogRampZ, dir, zVelPulses * boostMultiplier * scale, zAccelPulses);
      break;
  }
}

bool jointJogActive() { return rotDir || a1Dir || a2Dir || jzDir; }
bool anyJogActive() { return jointJogActive() || xjogActive; }

// ── Soft limits: master switch AND per-axis switch ──
bool softLimitsActive() { return limitsEnabled; }

bool axisLimited(const String &axis) {
  return softLimitsActive() && axisEnforced(axis);
}

// A FACTORY-DEFAULT floor is relaxed while there is no reference -- the counter reads 0
// wherever the board woke. A taught floor always applies. Mirrors _axis_bounds() in
// the GUI.
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
  // An XYZ jog with no key held is already easing to a stop by itself.
  if (!jointJogActive() && !xjogHeld()) return;
  // HOME is not a jog: no keep-alive arrives. It has its own timeout.
  if (isHoming) return;
  // Nor is a scan.
  if (scanPhase != SCAN_OFF) return;
  if (millis() - lastJogKeepAlive < JOG_WATCHDOG_MS) return;
  cancelJog();
  sendFeedback("[WATCHDOG] Jog stopped — no keep-alive from host for "
             + String((int)JOG_WATCHDOG_MS) + " ms.");
}

void startJog(int &axisDir, int dir, JogAxisId axisId) {
  if (xjogActive) cancelJog();          // one kind of jog at a time
  cancelScan("a jog command took over");
  if (isMoving)  { cancelRun();    sendFeedback("[WARN] RUN canceled by jog command."); }
  if (isHoming)  { cancelHoming();
                   sendFeedback("[WARN] Homing canceled by jog command."); }
  axisDir = dir;
  lastJogKeepAlive = millis();
  armJogAxisRamp(axisId, dir);
  applyJogVelocities();
}

void startArmJogLinked(int dir) {
  if (xjogActive) cancelJog();
  cancelScan("a jog command took over");
  if (isMoving)  { cancelRun();    sendFeedback("[WARN] RUN canceled by jog command."); }
  if (isHoming)  { cancelHoming();
                   sendFeedback("[WARN] Homing canceled by jog command."); }
  a1Dir = dir;
  a2Dir = dir;
  lastJogKeepAlive = millis();
  armJogAxisRamp(JOG_AXIS_A1, dir);
  armJogAxisRamp(JOG_AXIS_A2, dir);
  applyJogVelocities();
}

// A voluntary release: eases down from a steady profiled jog, otherwise a hard stop.
void stopArmJog(bool arm1, bool arm2) {
  if (arm1) { a1Dir = 0; if (!releaseJogRamp(jogRampA1)) MOTOR_A1.MoveVelocity(0); }
  if (arm2) { a2Dir = 0; if (!releaseJogRamp(jogRampA2)) MOTOR_A2.MoveVelocity(0); }
}


// PLC TRANSPORT — MC PROTOCOL 3E

// Polled state: three words, M0..M47, so M30..M32 are covered.
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
  String s = "limit Z/R/A2=" + String(plcBit(PLC_M_LIMIT_Z) ? 1 : 0)
     + String(plcBit(PLC_M_LIMIT_ROT) ? 1 : 0)
     + String(plcBit(PLC_M_LIMIT_A2) ? 1 : 0);
  // Which end each switch refuses right now. A2M's is worked out on the board.
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

// Frames are raw bytes, never a String: the request is half NUL bytes.
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

uint8_t plcTxBytes[64];
int     plcTxCount = 0;
void plcBuildPollFrame() {
  plcBuildReadFrameBin(plcTxBytes, plcTxCount, PLC_MC_DEVICE_CODE_M_B,
                       (uint32_t)PLC_POLL_DEVICE_NUM, PLC_POLL_WORDS);
}

// *** THERE IS NO WRITE FRAME BUILDER, ON PURPOSE ***

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

// LINK STATE IS ABOUT DATA, NOT ABOUT THE SOCKET
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

bool plcSendPoll() {
  if (!plcEnsureConnected()) return false;
  plcSendAttempts++;
  plcBuildPollFrame();
  if (plcDebug) sendFeedback("[PLC_TX] " + plcHexDumpBytes(plcTxBytes, plcTxCount));
  plcClient.write(plcTxBytes, plcTxCount);
  plcClient.flush();
  plcRxCount = 0;
  plcTxnActive = true;
  plcTxnSentAt = millis();
  return true;
}

void plcOnGoodRead(const uint16_t *words, int count) {
  bool first = !plcStatusValid;
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

  // The latch runs BEFORE the status goes out: the line below carries the end it decides.
  plcServiceLimitLatch();

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

void plcServiceRx() {
  bool got = false;
  while (plcClient.available() > 0) {
    uint8_t v = (uint8_t)plcClient.read();
    if (plcRxCount < PLC_RX_CAP) plcRxBytes[plcRxCount++] = v;
    got = true;
  }
  if (got && plcDebug) {
    sendFeedback("[PLC_RX] " + plcHexDumpBytes(plcRxBytes, plcRxCount));
  }
  if (!plcTxnActive) { plcRxCount = 0; return; }

  if (plcConsumeResponse()) { plcTxnActive = false; return; }

  if (millis() - plcTxnSentAt >= PLC_TXN_TIMEOUT_MS) {
    plcTxnActive = false;
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
  int got = (rawLen - (PLC_MC_RES_HEADER_UNITS + 4)) / 2;   // words actually received
  if (avail > got) avail = got;
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

// M30..M32 travel limits — stop the axis
bool plcLimitLedBlink = false;
unsigned long plcLimitLedLastBlink = 0;
bool plcLimitWarned[3] = {false, false, false};

// Per-sensor boundary switch, index order Z/ROT/A2. Disabled: never stops the axis, and
// counts as satisfied for HOME.
bool plcLimitSensorEnabled[3] = {true, true, true};

// Rising-edge detection, and the end a both-ends switch caught (0 = nothing latched).
bool plcLimitPrevBit[3] = {false, false, false};
int  plcLimitLatchedEnd[3] = {0, 0, 0};

int plcLimitBitFor(int i) {
  const int bits[3] = {PLC_M_LIMIT_Z, PLC_M_LIMIT_ROT, PLC_M_LIMIT_A2};
  return bits[i];
}
// The HOME-side end of each switch.
int plcLimitHomeEndFor(int i) {
  const int ends[3] = {PLC_LIMIT_END_Z, PLC_LIMIT_END_ROT, PLC_LIMIT_END_A2};
  return ends[i];
}
bool plcLimitBothEndsFor(int i) {
  const bool both[3] = {PLC_LIMIT_BOTH_ENDS_Z, PLC_LIMIT_BOTH_ENDS_ROT,
                        PLC_LIMIT_BOTH_ENDS_A2};
  return both[i];
}
// With a reference, POSITION decides: M30 on at or above this base angle is the FAR
// switch. The direction latch below is for a machine with no reference.
const double PLC_A2_FAR_END_BASE_DEG = 30.0;

double armBaseFromMotor(double motorDeg) {
  return armFoldFromMotor(motorDeg) + ARM_ZERO_CAD_DEG - 90.0;
}

// Only A2M is wired at both ends, so only A2M has a position to ask.
int plcLimitEndByPosition(int i) {
  if (i != 2) return plcLimitHomeEndFor(i);
  return armBaseFromMotor(currentA2()) >= PLC_A2_FAR_END_BASE_DEG
             ? -plcLimitHomeEndFor(i) : plcLimitHomeEndFor(i);
}

// Which end is refusing now. A2M: by position when homed, else the latched direction.
int plcLimitEndFor(int i) {
  if (!plcLimitBothEndsFor(i)) return plcLimitHomeEndFor(i);
  if (isHomed) return plcLimitEndByPosition(i);
  return plcLimitLatchedEnd[i] ? plcLimitLatchedEnd[i] : plcLimitHomeEndFor(i);
}

// How long a travel direction is remembered after the axis stops, for the latch.
const unsigned long PLC_TRAVEL_DIR_MEMORY_MS = 1000;
int           plcLastTravelDir[3] = {0, 0, 0};
unsigned long plcLastTravelAt[3]  = {0, 0, 0};

// Live direction: the jog direction, else the sign of the run leg's remaining distance.
// 0 = not moving.
int plcAxisTravelDirNow(int i) {
  const int jog[3] = {jzDir, rotDir, a2Dir};
  if (jog[i] > 0) return +1;
  if (jog[i] < 0) return -1;
  if (xjogActive && xjogJointDir[i]) return xjogJointDir[i];
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

// What the latch asks: the live direction, else the remembered one.
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
// True when this switch is tripped at its home end, or switched off. HOME only.
bool plcLimitSensorSatisfied(int i) {
  if (!plcLimitSensorEnabled[i]) return true;
  if (!plcBit(plcLimitBitFor(i))) return false;
  // A both-ends switch caught at the FAR end is not the reference.
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

// Decides which end a both-ends switch caught. Must run BEFORE plcServiceLimitStops().
void plcServiceLimitLatch() {
  if (!plcStatusValid) return;   // stale data: keep whatever was latched
  const char *names[3] = {"ZM", "RM", "A2M"};
  const char *devs[3]  = {"M32", "M31", "M30"};
  for (int i = 0; i < 3; i++) {
    bool on = plcBit(plcLimitBitFor(i));
    if (plcLimitBothEndsFor(i)) {
      if (on && !plcLimitPrevBit[i]) {
        if (isHomed) {
          // Referenced: where the arm IS says which switch this is.
          plcLimitLatchedEnd[i] = plcLimitEndByPosition(i);
          sendFeedback("[PLC_LIMIT] " + String(names[i]) + " tripped "
                     + String(devs[i]) + " at its "
                     + String(plcLimitLatchedEnd[i] > 0 ? "FORWARD" : "BACK")
                     + " end (by position: base "
                     + String(armBaseFromMotor(currentA2()), 2) + " deg).");
        } else {
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
        }
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

// ---- XYZ JOG, the moving half ----

// A covered switch refuses axis i (0 ZM, 1 RM, 2 A2M) moving in `dir`.
bool xjogSwitchRefuses(int i, int dir) {
  return dir != 0 && plcLimitSensorEnabled[i] && plcStatusValid
      && plcBit(plcLimitBitFor(i)) && dir == plcLimitEndFor(i);
}

// The taught band with the escape rule: only a step going FURTHER out is refused.
bool xjogBandRefuses(const String &axis, double lo, double hi, double from, double to) {
  if (!axisLimited(axis)) return false;
  if (to > hi && to > from) return true;
  return to < lo && to < from && axisLowerLimited(axis);
}

// (x, y) -> RM and the selected elbow. False, with a reason, when the step must not be
// taken.
bool xjogSolveXY(double x, double y, double &rot, double &motor, String &why) {
  // Z is the lift's business; an out-of-stroke counter must not refuse X/Y.
  IkResult r = solveIkFromHome(xjogArm, x, y,
                               constrain(xjogPos[2], (double)D1_MIN_MM, (double)D1_MAX_MM));
  if (!r.ok) { why = r.error; why.replace("[ERROR] ", ""); return false; }
  rot   = r.th2;
  motor = armMotorFromFold(r.th3 - FOLD_ANGLE_HOME_DEG);

  if (xjogBandRefuses("ROT", limRotMin, limRotMax, xjogRot, rot)) {
    why = "RM would leave its taught band " + String(limRotMin, 2) + ".."
        + String(limRotMax, 2) + " deg"; return false;
  }
  double lo, hi; armBand(xjogArm, lo, hi);
  if (xjogBandRefuses(xjogArm == 1 ? "A1" : "A2", lo, hi, xjogMotor, motor)) {
    why = String(xjogArm == 1 ? "A1M" : "A2M") + " would leave its taught band (R "
        + String(reachFromFoldAngle(r.th3), 1) + " mm)"; return false;
  }
  const double dRot = rot - xjogRot, dArm = motor - xjogMotor;
  if (xjogSwitchRefuses(1, dRot > 1e-9 ? 1 : (dRot < -1e-9 ? -1 : 0))) {
    why = "RM is on its travel switch (M31)"; return false;
  }
  if (xjogArm == 2 && xjogSwitchRefuses(2, dArm > 1e-9 ? 1 : (dArm < -1e-9 ? -1 : 0))) {
    why = "A2M is on its travel switch (M30)"; return false;
  }
  return true;
}

// How many times its own speed the busier of the two joints is asked for.
double xjogJointOver(double rot, double motor, double dt) {
  return max(fabs(rot - xjogRot) / (rotVelDegS * XJOG_JOINT_HEADROOM * dt),
             fabs(motor - xjogMotor) / (armVelDegS * XJOG_JOINT_HEADROOM * dt));
}

void xjogStepXY(double dt) {
  double nx = xjogPos[0] + xjogVel[0] * dt, ny = xjogPos[1] + xjogVel[1] * dt;
  double rot = xjogRot, motor = xjogMotor;
  String why;
  bool ok = xjogSolveXY(nx, ny, rot, motor, why);
  if (ok) {
    // Shorten the step until neither joint is asked past its own speed.
    double over = xjogJointOver(rot, motor, dt);
    if (over > 1.0) {
      nx = xjogPos[0] + (nx - xjogPos[0]) / over;
      ny = xjogPos[1] + (ny - xjogPos[1]) / over;
      // The speed the tool HAS is the shortened one; a release eases down from it.
      xjogVel[0] /= over; xjogVel[1] /= over;
      ok = xjogSolveXY(nx, ny, rot, motor, why);
      if (ok && xjogJointOver(rot, motor, dt) > XJOG_JUMP_RATIO) {
        why = "a joint would have to jump to follow (the tool is at the turntable "
              "axis, or the pose is outside the frame)";
        ok = false;
      }
    }
  }
  if (!ok) {
    xjogDir[0] = xjogDir[1] = 0;
    xjogVel[0] = xjogVel[1] = 0.0;
    xjogJointDir[1] = 0;
    if (xjogArm == 2) xjogJointDir[2] = 0;
    sendFeedback("[XJOG] XY stopped - " + why + ".");
    return;
  }
  xjogJointDir[1] = rot > xjogRot ? 1 : (rot < xjogRot ? -1 : 0);
  if (xjogArm == 2) xjogJointDir[2] = motor > xjogMotor ? 1 : (motor < xjogMotor ? -1 : 0);
  xjogPos[0] = nx; xjogPos[1] = ny;
  xjogRot = rot; xjogMotor = motor;
  MOTOR_ROT.Move((int32_t)lround(rot * pulsesPerDegRot()) * (INVERT_ROT ? -1 : 1),
                 StepGenerator::MOVE_TARGET_ABSOLUTE);
  const int32_t armPulses = (int32_t)lround(motor * PULSES_PER_DEG_ARM_MOTOR);
  if (xjogArm == 1) MOTOR_A1.Move(armPulses * (INVERT_ARM1 ? -1 : 1),
                                  StepGenerator::MOVE_TARGET_ABSOLUTE);
  else              MOTOR_A2.Move(armPulses * (INVERT_ARM2 ? -1 : 1),
                                  StepGenerator::MOVE_TARGET_ABSOLUTE);
}

void xjogStepZ(double dt) {
  double step = xjogVel[2] * dt;
  const double cap = zVelMmS * XJOG_JOINT_HEADROOM * dt;
  if (step > cap) step = cap;
  if (step < -cap) step = -cap;
  xjogVel[2] = step / dt;                 // the speed it has, as in xjogStepXY()
  const double nz = xjogPos[2] + step;
  const int dir = step > 0 ? 1 : -1;
  String why;
  if ((nz < D1_MIN_MM - 1e-6 && dir < 0) || (nz > D1_MAX_MM + 1e-6 && dir > 0)) {
    why = "ZM's stroke is " + String(D1_MIN_MM, 1) + ".." + String(D1_MAX_MM, 1)
        + " mm above HOME";
  } else if (xjogBandRefuses("Z", limD1Min, limD1Max, xjogPos[2], nz)) {
    why = "ZM would leave its taught band " + String(limD1Min, 2) + ".."
        + String(limD1Max, 2) + " mm";
  } else if (xjogSwitchRefuses(0, dir)) {
    why = "ZM is on its travel switch (M32)";
  }
  if (why.length() > 0) {
    xjogDir[2] = 0; xjogVel[2] = 0.0; xjogJointDir[0] = 0;
    sendFeedback("[XJOG] Z stopped - " + why + ".");
    return;
  }
  xjogJointDir[0] = dir;
  xjogPos[2] = nz;
  MOTOR_Z.Move((int32_t)lround(nz * pulsesPerMmZ()) * (INVERT_Z ? -1 : 1),
               StepGenerator::MOVE_TARGET_ABSOLUTE);
}

void serviceXjog() {
  if (!xjogActive) return;
  const unsigned long now = millis();
  if (now - xjogLastMs < XJOG_TICK_MS) return;
  double dt = (now - xjogLastMs) / 1000.0;
  if (dt > XJOG_MAX_DT_S) dt = XJOG_MAX_DT_S;
  xjogLastMs = now;

  for (int i = 0; i < 3; i++) {
    const double want = xjogDir[i] * xjogSpeed;
    const double v = xjogVel[i];
    // Slowing is its own ramp: latched when it starts, hard enough to stop inside
    // XJOG_EASE_MAX_S.
    const bool slowing = (v > 0.0 && want < v) || (v < 0.0 && want > v);
    if (!slowing) {
      xjogEase[i] = 0.0;
      const double dv = XJOG_ACCEL_MM_S2 * dt;
      if (v < want)      xjogVel[i] = min(want, v + dv);
      else if (v > want) xjogVel[i] = max(want, v - dv);
      continue;
    }
    if (xjogEase[i] == 0.0) xjogEase[i] = max(XJOG_ACCEL_MM_S2, fabs(v) / XJOG_EASE_MAX_S);
    const double dv = xjogEase[i] * dt;
    // Never through zero in one go.
    const double floorV = (v > 0.0) ? max(want, 0.0) : min(want, 0.0);
    xjogVel[i] = (v > 0.0) ? max(floorV, v - dv) : min(floorV, v + dv);
  }

  if (xjogVel[0] != 0.0 || xjogVel[1] != 0.0) xjogStepXY(dt);
  else { xjogJointDir[1] = 0; if (xjogArm == 2) xjogJointDir[2] = 0; }
  if (xjogVel[2] != 0.0) xjogStepZ(dt);
  else xjogJointDir[0] = 0;

  // Released, eased out, and the axes have caught the last setpoint.
  if (!xjogHeld() && xjogVel[0] == 0.0 && xjogVel[1] == 0.0 && xjogVel[2] == 0.0
      && allMotorsSettled()) {
    xjogClear();
    reportJogPosition();
  }
}

void handleXjog(const String &payload) {
  double v[5];
  if (parseCsv(payload, v, 5) != 5) {
    sendFeedback("[ERROR] XJOG needs arm,sx,sy,sz,mm_per_s"); return;
  }
  const int arm = (int)v[0];
  if (arm != 1 && arm != 2) {
    sendFeedback("[ERROR] XJOG arm must be 1 (A1M) or 2 (A2M), got " + String(arm)); return;
  }
  if (!(v[4] > 0.0)) { sendFeedback("[ERROR] XJOG speed must be above 0 mm/s"); return; }
  int want[3];
  for (int i = 0; i < 3; i++) want[i] = v[1 + i] > 0.5 ? 1 : (v[1 + i] < -0.5 ? -1 : 0);
  // The last key coming up on a jog that already ended is not a start.
  if (!xjogActive && !want[0] && !want[1] && !want[2]) return;

  if (jointJogActive()) cancelJog();    // one kind of jog at a time
  cancelScan("a jog command took over");
  if (isMoving)  { cancelRun();    sendFeedback("[WARN] RUN canceled by jog command."); }
  if (isHoming)  { cancelHoming(); sendFeedback("[WARN] Homing canceled by jog command."); }

  if (!xjogActive || arm != xjogArm) {
    // Start from where the tool IS: the setpoint is the live pose's own FK.
    xjogClear();
    const double rotNow = currentRot();
    const double motorNow = (arm == 1) ? currentA1() : currentA2();
    double bearing = rotNow;
    if (bearing < ROT_MIN_DEG && bearing > ROT_MIN_DEG - XJOG_START_SNAP_DEG) bearing = ROT_MIN_DEG;
    if (bearing > ROT_MAX_DEG && bearing < ROT_MAX_DEG + XJOG_START_SNAP_DEG) bearing = ROT_MAX_DEG;
    const double R = reachFromFoldAngle(arm == 1 ? currentA1Fold() : currentA2Fold());
    const double x0 = R * cos(bearing * DEG_TO_RAD), y0 = R * sin(bearing * DEG_TO_RAD);
    // The start pose must round-trip through IK, or the first step jumps to another joint
    // pose.
    IkResult chk = solveIkFromHome(arm, x0, y0,
                                   constrain((double)currentD1(), (double)D1_MIN_MM,
                                             (double)D1_MAX_MM));
    if (!chk.ok || fabs(chk.th2 - bearing) > 0.01
        || fabs(armMotorFromFold(chk.th3 - FOLD_ANGLE_HOME_DEG) - motorNow) > 0.5) {
      sendFeedback("[XJOG] XY stopped - refused to start: RM " + String(rotNow, 2)
                 + " deg / " + String(arm == 1 ? "A1M " : "A2M ") + String(motorNow, 2)
                 + " motor deg is not a pose the XYZ frame can express. HOME or reset "
                   "the coordinates, or jog back inside the travel with the JOINT layout.");
      sendFeedback("[XJOG] Z stopped - not started.");
      return;
    }
    xjogArm = arm;
    xjogRot = rotNow;
    xjogMotor = motorNow;
    xjogPos[0] = x0;
    xjogPos[1] = y0;
    xjogPos[2] = currentD1();
    xjogLastMs = millis();
    xjogActive = true;
    sendFeedback("[XJOG] arm " + String(arm) + " from X " + String(xjogPos[0], 1)
               + " Y " + String(xjogPos[1], 1) + " Z " + String(xjogPos[2], 1) + " mm.");
  }
  for (int i = 0; i < 3; i++) xjogDir[i] = want[i];
  xjogSpeed = v[4];                       // no ceiling -- see XJOG_EASE_MAX_S
  lastJogKeepAlive = millis();
  warnUnreferencedOnce();
}

// HOME state = M30 && M31 && M32; a disabled switch counts as satisfied.
bool plcHomeStateActive() {
  if (!plcStatusValid) return false;
  return plcLimitSensorSatisfied(0) && plcLimitSensorSatisfied(1)
      && plcLimitSensorSatisfied(2);
}

bool plcHomeStatePrev = false;
bool plcHomeStateWarned = false;   // one warning per entry, not one per poll

void plcServiceHomeState() {
  bool now = plcHomeStateActive();
  if (now == plcHomeStatePrev) return;
  plcHomeStatePrev = now;
  if (!now) return;

  // Never zero the counters under a move or a scan.
  if (isMoving || anyJogActive() || scanPhase != SCAN_OFF) {
    if (!plcHomeStateWarned) {
      plcHomeStateWarned = true;
      sendFeedback("[PLC_HOME] HOME state reached but the machine is still moving — "
                   "coordinates NOT reset. They will be once it stops here.");
    }
    plcHomeStatePrev = false;
    return;
  }
  plcHomeStateWarned = false;

  MOTOR_Z.PositionRefSet(0);
  MOTOR_ROT.PositionRefSet(0);
  MOTOR_A1.PositionRefSet(0);
  MOTOR_A2.PositionRefSet(0);
  isHomed = true;
  if (isHoming) { isHoming = false; }
  sendFeedback("[PLC_HOME] HOME STATE — M30, M31 and M32 all true.");
  sendFeedback("[COORD_RESET] Coordinates reset to the standard home pose: "
               "d1=0.00 mm, ROT=0.00 deg, A1M=0.00 motor deg, A2M=0.00 motor deg.");
  reportJogPosition();
}

void plcServicePoll() {
  plcServiceRx();
  if (plcTxnActive) return;
  unsigned long now = millis();
  unsigned long interval = isHoming ? PLC_POLL_HOMING_MS : plcPollIdleMs;
  if (plcLastPollSent != 0 && (now - plcLastPollSent) < interval) return;
  plcLastPollSent = now;
  plcSendPoll();
}

void servicePlc() {
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
  // Lamps and limit stops run every pass, not only on a fresh poll.
  plcServiceLimitLeds();
  plcServiceLimitLatch();
  plcServiceLimitStops();
  if (plcStatusValid && plcLastPollOk != lastActedOn) {
    lastActedOn = plcLastPollOk;
    plcServiceHomeState();
  }
}


void plcNetworkInit() {
  Ethernet.begin(plcMac, plcLocalIp);
  sendFeedback("[PLC] ClearCore " + String(CC_IP_0) + "." + String(CC_IP_1) + "."
             + String(CC_IP_2) + "." + String(CC_IP_3)
             + " -> PLC " + String(PLC_IP_0) + "." + String(PLC_IP_1) + "."
             + String(PLC_IP_2) + "." + String(PLC_IP_3) + ":" + String((int)PLC_PORT)
             + " (MC protocol 3E,  BINARY, READ-ONLY, polling M0..M47 every "
             + String((unsigned long)plcPollIdleMs) + " ms idle / "
             + String((int)PLC_POLL_HOMING_MS) + " ms while homing)");
  if (Ethernet.linkStatus() == LinkOFF) {
    sendFeedback("[WARN] No Ethernet link detected — HOME will time out and the "
                 "PLC boundary switches will not be seen until the cable is in.");
  }
}

// HOME is driven by this board: each axis onto its own switch, all at once.
// A1M has no switch and does not move.
void beginHoming() {
  cancelJog();
  cancelRun();
  decelStopAll(false);

  isHoming = true;
  isHomed = false;
  homeRequestedAt = millis();
  lastHomeReportTime = homeRequestedAt;

  if (!plcStatusValid) {
    isHoming = false;
    sendFeedback("[HOME] FAILED — no PLC device data, so the switches cannot be "
                 "seen. HOME drives the axes onto M30..M32 and would have no way "
                 "to know when to stop. Fix the link first — PLC_TEST.");
    sendFeedback("[ERROR] HOME refused: the switch states are unknown.");
    return;
  }

  int  *dirs[3]        = {&jzDir, &rotDir, &a2Dir};
  const char *names[3] = {"ZM", "RM", "A2M"};
  String moving, already;
  for (int i = 0; i < 3; i++) {
    // Already on its switch, or its switch is disabled: not driven.
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
    // Tripped at the FAR end: ignore the bit until the axis drives clear of it.
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

  if (ok) {
    // RESET THE COORDINATE SYSTEM TO THE STANDARD HOME POSE.
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
  } else {
    sendFeedback("[HOME] FAILED — " + reason);
    sendFeedback("[ERROR] HOME timeout: this board drives the axes itself and never "
                 "saw one or more of M30..M32 come ON.");
    if (plcGoodReads == 0) {
      sendFeedback("[ERROR] Root cause: this board has never read a device from the "
                   "PLC, so it could not have seen the switches at all. Fix the "
                   "MC-protocol link first — PLC_TEST.");
    } else {
      sendFeedback("[ERROR] Device reads work, so a switch is stuck, broken, or the "
                   "axis is mechanically obstructed. If a switch is known broken, "
                   "SET_PLC_SENSOR_ENFORCE:<axis>,0 excludes it from HOME.");
    }
  }
}

void serviceHoming() {
  if (!isHoming) return;
  unsigned long now = millis();

  if (now - lastHomeReportTime >= HOME_REPORT_INTERVAL_MS) {
    lastHomeReportTime = now;
    reportJogPosition();
  }

  // Stop each axis the instant its own switch reads covered.
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


// ---- 340 DEGREE SCAN ----

// One distance reading in mm. NEGATIVE = the sensor did not answer; 0 is a real reading.
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

// Is RM on its travel switch -- the scan's reference.
bool scanRotSwitchOn() {
  return plcStatusValid && plcLimitSensorEnabled[1] && plcBit(PLC_M_LIMIT_ROT);
}

// ---- the scan's profiled moves ----
// A leg has a known length, so it gets the whole profile -- as a VELOCITY shape over
// rotDir / jzDir, never a position setpoint, so every limit and stop still works. The
// plan hands over to a slow creep at its tail; the leg ends on the angle or the switch.
const double SCAN_CREEP_FRACTION = 0.08;   // of the leg's own top speed

ScanMove scanRotMove, scanZMove;

bool scanOwnsRot() { return scanRotMove.active || scanRotMove.creeping; }
bool scanOwnsZ()   { return scanZMove.active   || scanZMove.creeping; }

// Velocity at time t, the derivative of profileAt().
double profileVelAt(const ProfilePlan &p, double t) {
  if (t <= 0.0 || t >= p.T) return 0.0;

  if (!p.sCurve) {
    if (t <= p.ta) return p.alphaMax * t;
    if (t <= p.ta + p.tv) return p.vp;
    return p.vp - p.alphaMax * (t - (p.ta + p.tv));
  }

  int k = 6;
  double tEnd = 0.0;
  for (int i = 0; i < 7; i++) {
    tEnd += p.dur[i];
    if (t <= tEnd + 1e-12 && p.dur[i] > 1e-14) { k = i; break; }
  }
  double tau = t - p.tStart[k];
  if (tau < 0.0) tau = 0.0;
  if (tau > p.dur[k]) tau = p.dur[k];
  return p.v0[k] + p.a0[k] * tau + 0.5 * p.jrk[k] * tau * tau;
}

// Plans one leg, in PULSES. False for PROFILE_NONE or a leg too short to shape.
bool scanPlanMove(ScanMove &m, int dir, double thetaPulses,
                  double vmaxPulses, double accelPulses) {
  m.active = m.creeping = false;
  m.dir = dir;
  m.creepV = vmaxPulses * SCAN_CREEP_FRACTION;
  if (m.creepV < 1.0) m.creepV = 1.0;

  if (motionProfile == PROFILE_NONE) return false;
  if (thetaPulses <= 1.0 || vmaxPulses <= 1e-9 || accelPulses <= 1e-9) return false;

  switch (motionProfile) {
    case PROFILE_TRAPEZOID:
      planTrapezoid(m.plan, thetaPulses, vmaxPulses, accelPulses);
      break;
    case PROFILE_SCURVE:
      planSCurve(m.plan, thetaPulses, vmaxPulses, accelPulses, SCURVE_RATIO);
      break;
    case PROFILE_PURE_SCURVE:
      planSCurve(m.plan, thetaPulses, vmaxPulses, accelPulses, 1.0);
      break;
    default:
      return false;
  }
  m.t0 = millis();
  m.active = true;
  return true;
}

bool scanMoveTick(ScanMove &m, int liveDir, int32_t &pulsesOut) {
  if (!m.active && !m.creeping) return false;
  // The direction was zeroed under us (limit, switch, cancel): give the axis up.
  if (liveDir == 0) { m.active = m.creeping = false; return false; }

  double v;
  if (m.active) {
    double t = (millis() - m.t0) / 1000.0;
    if (t >= m.plan.T) {
      m.active = false;
      m.creeping = true;
      v = m.creepV;
    } else {
      v = profileVelAt(m.plan, t);
      // Hand over to the creep at the end of the ramp-down -- second half only.
      if (v < m.creepV && t > 0.5 * m.plan.T) v = m.creepV;
    }
  } else {
    v = m.creepV;
  }

  pulsesOut = (int32_t)lround(m.dir * v);
  return true;
}

void serviceScanMoves() {
  int32_t p;
  if (scanMoveTick(scanRotMove, rotDir, p)) MOTOR_ROT.MoveVelocity(p * (INVERT_ROT ? -1 : 1));
  if (scanMoveTick(scanZMove,   jzDir,  p)) MOTOR_Z.MoveVelocity(p * (INVERT_Z   ? -1 : 1));
}

// SEEK: its length is not known, so ease-up only.
void scanSeekRotMove(int dir) {
  rotDir = dir;
  armJogAxisRamp(JOG_AXIS_ROT, dir);   // before apply, so apply skips it
  applyJogVelocities();
}

// A sweep: Theta is the sweep angle, so the whole profile applies.
void scanStartRotMove(int dir, double thetaDeg) {
  rotDir = dir;
  float rotScale = scanRotScale();
  if (!scanPlanMove(scanRotMove, dir, thetaDeg * pulsesPerDegRot(),
                    rotVelPulses * boostMultiplier * rotScale, rotAccelPulses))
    applyJogVelocities();              // PROFILE_NONE: flat, exactly as before
}

// A lift: Theta is the Z step.
void scanStartZMove(int dir, double thetaMm) {
  jzDir = dir;
  if (!scanPlanMove(scanZMove, dir, thetaMm * pulsesPerMmZ(),
                    zVelPulses * boostMultiplier * SCAN_SPEED_SCALE, zAccelPulses))
    applyJogVelocities();
}

// Hard and explicit: applyJogVelocities() skips an axis mid-ease, so the ramp is
// cleared here.
void scanStopRot() {
  rotDir = 0;
  jogRampRot.active = jogRampRot.releasing = false;
  scanRotMove.active = scanRotMove.creeping = false;
  MOTOR_ROT.MoveVelocity(0);
}

void scanStopZ() {
  jzDir = 0;
  jogRampZ.active = jogRampZ.releasing = false;
  scanZMove.active = scanZMove.creeping = false;
  MOTOR_Z.MoveVelocity(0);
}

void cancelScan(const String &why) {
  if (scanPhase == SCAN_OFF) return;
  const bool returning = (scanPhase == SCAN_RETURN);
  scanPhase = SCAN_OFF;
  // Clear, or a SCAN_START that asks for no speed inherits this one.
  scanRotDegS = 0.0;
  scanStopRot();
  scanStopZ();
  // Stopped on the way back with every layer in: a finished scan, not an aborted one.
  if (returning) {
    sendFeedback("[WARN] the return to the start was stopped (" + why + ") - RM at "
               + String(currentRot(), 2) + " deg, ZM at " + String(currentD1(), 2) + " mm.");
    sendFeedback("[SCAN_DONE] " + String(scanLayers) + " layers, "
               + String(scanPointsSent) + " points");
    return;
  }
  sendFeedback("[SCAN_ABORT] " + why);
}

// Every layer is in: back to where the first one started, both axes at once.
void scanBeginReturn() {
  scanPhase = SCAN_RETURN;
  const double dz   = currentD1() - scanStartZ;
  const double dRot = fabs(currentRot() - scanStartRot);
  sendFeedback("[SCAN_RETURN] " + String(scanLayers) + " layers in ("
             + String(scanPointsSent) + " points) - going back to the start: RM "
             + String(scanStartRot, 2) + " deg, ZM " + String(scanStartZ, 2) + " mm");
  if (dz > SCAN_Z_EPS_MM) scanStartZMove(-1, dz);
  if (!scanRotSwitchOn() && dRot > SCAN_ANGLE_EPS_DEG)
    scanStartRotMove(PLC_LIMIT_END_ROT, dRot);
}

// Each axis stops when it is back; the scan is done when both have.
void serviceScanReturn() {
  if (jzDir != 0 && currentD1() <= scanStartZ + SCAN_Z_EPS_MM) scanStopZ();
  if (rotDir != 0) {
    const bool back = (PLC_LIMIT_END_ROT < 0)
        ? currentRot() <= scanStartRot + SCAN_ANGLE_EPS_DEG
        : currentRot() >= scanStartRot - SCAN_ANGLE_EPS_DEG;
    if (back || scanRotSwitchOn()) scanStopRot();
  }
  if (rotDir != 0 || jzDir != 0) return;

  // Both stopped -- by arriving, or by a limit on the way. Say which.
  const bool rotHome = scanRotSwitchOn() || fabs(currentRot() - scanStartRot) < 1.0;
  const bool zHome   = fabs(currentD1() - scanStartZ) < 1.0;
  scanStopRot(); scanStopZ();
  scanPhase = SCAN_OFF;
  scanRotDegS = 0.0;          // same reason as cancelScan()
  if (!rotHome || !zHome) {
    sendFeedback("[WARN] the return to the start stopped short - RM at "
               + String(currentRot(), 2) + " deg, ZM at " + String(currentD1(), 2)
               + " mm (a soft limit or a PLC switch is in the way).");
  }
  sendFeedback("[SCAN_DONE] " + String(scanLayers) + " layers, "
             + String(scanPointsSent) + " points");
}

// Angle is read BEFORE the sensor fires: a miss blocks for the full timeout.
void scanEmitPoint() {
  double deg = currentRot();
  // No "-0.00" at the reference.
  if (deg == 0.0) deg = 0.0;
  double mm  = scanReadDistanceMm();
  scanPointsSent++;
  sendFeedback("[SCAN_PT] " + String(scanLayer) + "," + String(deg, 2)
             + "," + String(mm, 2));
}

// Layers alternate direction; every second one ends on the switch and re-references RM.
void scanBeginLayer(int dir) {
  scanSweepDir = dir;
  scanSweepFrom = currentRot();
  scanNextDeg = scanSweepFrom;
  scanPhase = SCAN_SWEEP;
  sendFeedback("[SCAN_LAYER] " + String(scanLayer) + "/" + String(scanLayers)
             + " z=" + String(currentD1(), 2) + " mm"
             + " dir=" + String(dir > 0 ? "+" : "-")
             + " from=" + String(scanSweepFrom, 2));
  scanStopZ();              // the lift has finished; make sure it is stopped
  scanStartRotMove(dir, scanSweepDeg);
}

// True once the axis reaches the next sample angle, in the sweep's own direction.
bool scanReachedNext() {
  if (scanSweepDir > 0) return currentRot() >= scanNextDeg - SCAN_ANGLE_EPS_DEG;
  return currentRot() <= scanNextDeg + SCAN_ANGLE_EPS_DEG;
}

double scanTravelled() {
  double d = currentRot() - scanSweepFrom;
  return d < 0 ? -d : d;
}

// Shortens the sweep to fit RM's far soft limit -- once, at the first reference.
void scanFitSweepToSoftLimit() {
  if (!axisLimited("ROT")) return;
  int away = -PLC_LIMIT_END_ROT;
  double room = (away > 0) ? limRotMax - currentRot() : currentRot() - limRotMin;
  room -= SCAN_ANGLE_EPS_DEG;
  if (room >= scanSweepDeg) return;
  if (room < scanDegStep) return;       // no room at all: let the limit abort it, loudly
  sendFeedback("[WARN] sweep shortened from " + String(scanSweepDeg, 2) + " to "
             + String(room, 2) + " deg to stay inside RM's soft limit ("
             + String(limRotMin, 2) + ".." + String(limRotMax, 2) + ").");
  scanSweepDeg = room;
}

void serviceScan() {
  if (scanPhase == SCAN_OFF) return;
  if (scanPhase == SCAN_RETURN) { serviceScanReturn(); return; }

  // ---- finding the reference --------------------------------------
  if (scanPhase == SCAN_SEEK) {
    if (scanRotSwitchOn()) {
      scanStopRot();
      scanStartRot = currentRot();
      sendFeedback("[SCAN_REF] RM on its switch at " + String(scanStartRot, 2)
                 + " deg - sweeping from here");
      scanLayer = 1;
      scanFitSweepToSoftLimit();
      scanBeginLayer(-PLC_LIMIT_END_ROT);   // away from the switch
      // No return: fall through and take the sample at the reference angle now.
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

  // A limit or a switch zeroed the direction mid-sweep: the layer is short, say so.
  // (Heading back to the switch, being stopped is arrival -- handled below.)
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

    scanStopRot();
    if (backAtSwitch) {
      // Every arrival at the switch is a fresh reference.
      scanStartRot = currentRot();
      sendFeedback("[SCAN_REF] RM back on its switch at "
                 + String(scanStartRot, 2) + " deg");
    }
    if (scanLayer >= scanLayers) {
      // [SCAN_DONE] waits for the return.
      scanBeginReturn();
      return;
    }
    scanLayerTargetZ = scanStartZ + scanZStepMm * (double)scanLayer;
    scanPhase = SCAN_LIFT;
    scanStartZMove(1, scanZStepMm);
    return;
  }

  // ---- lifting between layers --------------------------------------
  if (scanPhase == SCAN_LIFT) {
    if (currentD1() < scanLayerTargetZ - SCAN_Z_EPS_MM) return;
    scanStopZ();
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
  int c4 = (c3 < 0) ? -1 : payload.indexOf(',', c3 + 1);
  if (c1 < 0 || c2 < 0) {
    sendFeedback("[ERROR] SCAN_START needs zStepMm,degStep,layers[,sweepDeg[,rotDegS]]");
    return;
  }
  double zStep = payload.substring(0, c1).toFloat();
  double dStep = payload.substring(c1 + 1, c2).toFloat();
  int layers = (c3 < 0 ? payload.substring(c2 + 1)
                       : payload.substring(c2 + 1, c3)).toInt();
  double sweep = (c3 < 0) ? SCAN_SWEEP_DEG_DEF
               : (c4 < 0 ? payload.substring(c3 + 1)
                         : payload.substring(c3 + 1, c4)).toFloat();
  // Optional and LAST, so an older host's four fields still parse.
  double rotDegS = (c4 < 0) ? 0.0 : payload.substring(c4 + 1).toFloat();
  if (c4 >= 0 && rotDegS < 0.0) {
    sendFeedback("[ERROR] scan speed cannot be negative, got " + String(rotDegS, 3));
    return;
  }

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
  // A sweep shorter than one step is almost certainly a typo.
  if (sweep < dStep) {
    sendFeedback("[ERROR] a " + String(sweep, 2) + " deg sweep is shorter than the "
               + String(dStep, 2) + " deg step, so a layer would hold one point.");
    return;
  }
  // The LAST layer is the one that has to fit: startZ + zStep * (layers - 1).
  double topZ = currentD1() + zStep * (double)(layers - 1);
  if (topZ > D1_MAX_MM) {
    sendFeedback("[ERROR] SCAN would need Z = " + String(topZ, 1)
               + " mm, past the " + String(D1_MAX_MM, 0) + " mm stroke. "
                 "Lower the start height, the step, or the layer count.");
    return;
  }
  // No switch data, no reference to sweep from.
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

  // Warned, not refused: sampling is by position, so a clamp costs time, not data.
  if (rotDegS >= SCAN_ROT_DEG_S_MIN && rotDegS > (double)rotVelDegS) {
    sendFeedback("[WARN] scan asked for " + String(rotDegS, 1)
               + " deg/s, RM is configured for " + String(rotVelDegS, 1)
               + " deg/s - sweeping at the lower figure. Raise RM's speed "
                 "percentage, or ask for fewer points per layer.");
  }

  scanZStepMm = zStep;
  scanDegStep = dStep;
  scanLayers  = layers;
  scanSweepDeg = sweep;
  scanRotDegS  = rotDegS;
  scanStartZ   = currentD1();
  scanSweepFrom = currentRot();
  scanPointsSent = 0;
  scanLayer = 0;
  sendFeedback("[SCAN_BEGIN] sensor=" + String(scanSensorName())
             + " layers=" + String(layers)
             + " zStep=" + String(zStep, 2)
             + " degStep=" + String(dStep, 2)
             + " sweep=" + String(sweep, 1)
             + " fromZ=" + String(scanStartZ, 2)
             + " rotDegS=" + String((double)rotVelDegS * scanRotScale(), 2));

  if (scanRotSwitchOn()) {
    // Already on the switch: nothing to seek.
    scanPhase = SCAN_SEEK;
    serviceScan();
    return;
  }
  sendFeedback("[SCAN_SEEK] turning RM to its switch to reference the sweep...");
  scanPhase = SCAN_SEEK;
  scanSeekRotMove(PLC_LIMIT_END_ROT);
}

void sendScanStatus() {
  const char *phase = scanPhase == SCAN_OFF ? "IDLE"
                    : scanPhase == SCAN_SEEK ? "SEEK"
                    : scanPhase == SCAN_SWEEP ? "SWEEP"
                    : scanPhase == SCAN_RETURN ? "RETURN" : "LIFT";
  sendFeedback(String("[SCAN_STATUS] phase=") + phase
             + " sensor=" + String(scanSensorName())
             + " layer=" + String(scanLayer) + "/" + String(scanLayers)
             + " dir=" + String(scanSweepDir > 0 ? "+" : "-")
             + " points=" + String(scanPointsSent)
             + " rotDegS=" + String(scanPhase == SCAN_OFF ? 0.0
                                  : (double)rotVelDegS * scanRotScale(), 2)
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
                   "taught boundary — the PLC travel switches (M30..M32) are now "
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
      // Instant lamp update.
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

  if (upper.startsWith("SET_MOTION_PROFILE:")) {
    String kind = upper.substring(19);
    kind.trim();
    MotionProfileKind want;
    if (!parseProfileToken(kind, want)) {
      sendFeedback("[ERROR] SET_MOTION_PROFILE takes NONE, TRAPEZOIDAL, "
                   "SCURVE or PURE_SCURVE, got " + kind);
      return;
    }
    // Refused mid-move: swapping the plan under a running leg is a step in the setpoint.
    if (isMoving || isHoming || scanPhase != SCAN_OFF) {
      sendFeedback("[ERROR] SET_MOTION_PROFILE refused - the machine is moving.");
      return;
    }
    motionProfile = want;
    sendFeedback("[MOTION_PROFILE] " + String(motionProfileName())
               + (want == PROFILE_NONE
                  ? " - run legs use the step generator's own trapezoid."
                  : " - run legs are interpolated to this shape."));
    return;
  }

  if (upper == "MOTION_PROFILE") {
    sendFeedback("[MOTION_PROFILE] " + String(motionProfileName())
               + " (not persisted; the host re-sends it on connect)");
    return;
  }

  if (upper == "STATUS") {
    sendFeedback(String("[STATUS] fw=v10 indep-arms=yes watchdog=on")
               + " homed=" + String(isHomed ? "yes" : "no")
               + " homing=" + String(isHoming ? "yes" : "no")
               + " moving=" + String(isMoving ? "yes" : "no")
               + " jog[rot=" + String(rotDir) + " a1=" + String(a1Dir)
               + " a2=" + String(a2Dir) + " z=" + String(jzDir) + "]"
               + " xjog=" + String(xjogActive ? "yes" : "no"));
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
    if (jointJogActive()) applyJogVelocities();
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
      sendFeedback("[WARN] SET_PARAMS PID form field ignored — v10 has one PID "
                   "preset and no form selector.");
    }
    return;
  }
  if (upper == "PROFILE") { reportMotionProfile(); return; }
  if (upper.startsWith("SET_BOOST:")) {
    float b = cmd.substring(10).toFloat();
    boostMultiplier = constrain(b, 0.1f, BOOST_MAX);
    if (jointJogActive()) applyJogVelocities();
    return;
  }

  // ---- Cartesian ----
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
  if (upper.startsWith("TEST_MOVE:")) { handleTestMove(cmd.substring(10)); return; }
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
               + " — confirm on the bench if RM turns more or less than commanded)");
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
    // One shot, for aiming the sensor.
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
    if (!plcLinkEnabled) {
      // Report DISABLED, not leftover state.
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
               + plcHexDumpBytes(plcRxBytes, plcRxCount)
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
    // Name the layer that is actually failing.
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
                   + String((int)PLC_PORT) + " with Communication Data Code = BINARY.");
      }
    }
    return;
  }

  if (upper.startsWith("PLC_DEBUG:")) {
    plcDebug = cmd.substring(10).toInt() != 0;
    sendFeedback(String("[PLC] Frame echo ") + (plcDebug ? "ON — every [PLC_TX] and "
                 "[PLC_RX] is logged verbatim." : "off."));
    return;
  }

  if (upper.startsWith("SET_PLC_POLL:")) {
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
    return;
  }

  if (upper == "PLC_TEST") {
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
    return;
  }

  if (upper == "PLC_RECONNECT") {
    plcClient.stop();
    plcLastConnectTry = 0;
    plcReportedError  = false;
    plcStatusValid    = false;
    sendFeedback("[PLC] Socket dropped, reconnecting on the next service pass.");
    return;
  }


  // ---- XYZ jog: the tool along Cartesian axes ----
  if (upper.startsWith("XJOG:")) { handleXjog(cmd.substring(5)); return; }
  if (upper == "XJOG_STOP") {           // ease out; the watchdog no longer applies
    xjogDir[0] = xjogDir[1] = xjogDir[2] = 0;
    return;
  }

  // ---- jog ----
  if (upper == "ROT_CW")   { startJog(rotDir,  1, JOG_AXIS_ROT); return; }
  if (upper == "ROT_CCW")  { startJog(rotDir, -1, JOG_AXIS_ROT); return; }
  if (upper == "ROT_STOP") {
    rotDir = 0;
    if (!releaseJogRamp(jogRampRot)) MOTOR_ROT.MoveVelocity(0);
    return;
  }

  if (upper == "A1_FWD")  { startJog(a1Dir,  1, JOG_AXIS_A1); return; }
  if (upper == "A1_BACK") { startJog(a1Dir, -1, JOG_AXIS_A1); return; }
  if (upper == "A1_STOP") { stopArmJog(true, false); return; }
  if (upper == "A2_FWD")  { startJog(a2Dir,  1, JOG_AXIS_A2); return; }
  if (upper == "A2_BACK") { startJog(a2Dir, -1, JOG_AXIS_A2); return; }
  if (upper == "A2_STOP") { stopArmJog(false, true); return; }

  if (upper == "ARM_FWD")  { startArmJogLinked( 1); return; }
  if (upper == "ARM_BACK") { startArmJogLinked(-1); return; }
  if (upper == "ARM_STOP") { stopArmJog(true, true); return; }

  if (upper == "Z_UP")     { startJog(jzDir,  1, JOG_AXIS_Z); return; }
  if (upper == "Z_DOWN")   { startJog(jzDir, -1, JOG_AXIS_Z); return; }
  if (upper == "Z_STOP")   {
    jzDir = 0;
    if (!releaseJogRamp(jogRampZ)) MOTOR_Z.MoveVelocity(0);
    return;
  }

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


// ---- ARDUINO ENTRY POINTS ----
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) {  }

  pinMode(SCAN_TRIG_PIN, OUTPUT);
  digitalWrite(SCAN_TRIG_PIN, LOW);
  pinMode(SCAN_ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(PLC_LIMIT_LED_Z_PIN,   OUTPUT);
  pinMode(PLC_LIMIT_LED_ROT_PIN, OUTPUT);
  pinMode(PLC_LIMIT_LED_A2_PIN,  OUTPUT);

  motorsInit();
  plcNetworkInit();
  lastAliveTime = millis();
  lastJogKeepAlive = millis();

  sendFeedback("[BOOT] ==========================================");
  sendFeedback("[BOOT] STCR4000S controller v10 — on-board frog-leg IK");
  sendFeedback("[BOOT] Independent arms: A1_FWD/A1_BACK, A2_FWD/A2_BACK");
  sendFeedback("[BOOT] XYZ jog: XJOG:<arm>,<sx>,<sy>,<sz>,<mm/s> moves the tool point; XJOG_STOP eases out");
  sendFeedback("[BOOT] Speed: universal RPM + per-motor % (SET_SPEED)");
  sendFeedback("[BOOT] Limits: SET_LIMIT / SET_LIMIT_HERE, reference: RESET_COORD");
  sendFeedback("[BOOT] Limits live in RAM only — the host must re-send them on connect.");
  sendFeedback("[BOOT] Jog watchdog: ON (" + String((int)JOG_WATCHDOG_MS) + " ms) — host must send JOG_HB");
  sendFeedback("[BOOT] PLC: MC protocol 3E BINARY -> "
             + String(PLC_IP_0) + "." + String(PLC_IP_1) + "." + String(PLC_IP_2) + "."
             + String(PLC_IP_3) + ":" + String((int)PLC_PORT)
             + " | HOME drives axes onto M" + String(PLC_M_LIMIT_Z) + "/M"
             + String(PLC_M_LIMIT_ROT) + "/M" + String(PLC_M_LIMIT_A2) + " itself"
             + " | timeout " + String((int)(PLC_HOME_TIMEOUT_MS / 1000)) + "s");
  sendFeedback("[BOOT] PLC Ethernet is READ-ONLY (batch read M0..M47). Nothing is "
               "ever written to the PLC — HOME reads M30..M32 and drives the axes "
               "itself. If HOME never starts, check the Ethernet link — PLC_TEST.");
  reportLimits();
  reportMotionProfile();
  sendFeedback("[PID] " + pidSummary() + " (stored only — this board runs OPEN LOOP)");
  sendFeedback("[BOOT] Elbow convention: A1M/A2M report MOTOR degrees from home. "
               "HOME IS 0 — 0 motor deg = 0 fold deg = base -30 deg = fully retracted, R = "
             + String(reachFromFoldAngle(FOLD_ANGLE_HOME_DEG), 1) + " mm. "
             + String(armMotorFromFold(FOLD_ANGLE_MAX_DEG), 0)
             + " motor deg = fold " + String(FOLD_ANGLE_MAX_DEG, 0)
             + " deg = base +90 = straight, R = "
             + String(reachFromFoldAngle(FOLD_ANGLE_MAX_DEG), 1) + " mm.");
  sendFeedback("[BOOT] HOME = base -30 deg, R "
             + String(reachFromFoldAngle(FOLD_ANGLE_HOME_DEG), 1)
             + " mm | MAX = base +60 deg, R "
             + String(reachFromFoldAngle(FOLD_ANGLE_SPEC_MAX_DEG), 1)
             + " mm (" + String(armMotorFromFold(FOLD_ANGLE_SPEC_MAX_DEG), 0)
             + " motor deg). THE MAX IS A NOTE, NOT A LIMIT — nothing refuses a target "
               "past it; the taught elbow band is what stops the arm.");
  sendFeedback("[BOOT] The base angle is the GUI's display frame: base = fold - 30. "
               "RE-HOME after flashing from v8.");
  sendFeedback("[BOOT] ==========================================");
}

void loop() {
  while (Serial.available() > 0) {
    handleCommand(Serial.readStringUntil('\n'));
  }

  serviceLed();
  // FIRST, before anything that can zero a direction.
  plcRememberTravelDir();
  serviceJogWatchdog();
  serviceJogSoftLimits();
  serviceJogRamps();
  serviceXjog();
  serviceScanMoves();
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
