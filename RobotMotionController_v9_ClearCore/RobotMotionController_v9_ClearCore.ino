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
// ══════════════════════════════════════════════════════════════
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
const double ARM2_Z_DROP_MM   = D3_ARM1_MM - D3_ARM2_MM;

const double ARM_LINK_SUM_MM      = A4_MM + A5_MM;
const double ARM_RADIAL_OFFSET_MM = A3_MM + A6_MM;

const double I_RM_TOTAL = 1 * 6.5;

const double ARM_ZERO_CAD_DEG = 60.0;

const double FOLD_ANGLE_HOME_DEG     = 0.0;
const double FOLD_ANGLE_SPEC_MAX_DEG = 91.72;
const double FOLD_ANGLE_MIN_DEG      = FOLD_ANGLE_HOME_DEG;
const double FOLD_ANGLE_MAX_DEG      = 120.0;
const double FOLD_SINGULARITY_WARN_DEG = 110.0;

const double ARM_GEAR_RATIO_DEF = 20.0;
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

// AM1/AM2 elbow gearing — *** STILL A PLACEHOLDER. MEASURE IT. ***  [notes §13]
const double PULSES_PER_DEG_ARM_MOTOR = PULSES_PER_MOTOR_REV / 360.0;

// ══════════════════════════════════════════════════════════════
// ZM LEAD — MEASURE THIS.
// ══════════════════════════════════════════════════════════════
const double Z_MM_PER_REV_DEF = 5.0;
const double Z_MM_PER_REV_MIN = 0.1, Z_MM_PER_REV_MAX = 500.0;
double zMmPerRev = Z_MM_PER_REV_DEF;

double pulsesPerMmZ() { return PULSES_PER_MOTOR_REV / zMmPerRev; }

const double Z_MM_PER_MOTOR_REV = Z_MM_PER_REV_DEF;

const bool INVERT_Z    = false;
const bool INVERT_ROT  = false;
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
const float ARM_PCT_DEF        = 125.0f;
const float ROT_PCT_DEF        = 75.0f;
const float Z_PCT_DEF          = 50.0f;

float masterRpm     = MASTER_RPM_DEF;
float masterAccRpmS = MASTER_ACC_DEF;
float rotPct        = ROT_PCT_DEF;
float armPct        = ARM_PCT_DEF;
float zPct          = Z_PCT_DEF;

float rotAccPct     = ROT_PCT_DEF;
float armAccPct     = ARM_PCT_DEF;
float zAccPct       = Z_PCT_DEF;

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

// PLC LINK — MELSEC MC PROTOCOL 3E, ASCII FRAMES, TCP 192.168.3.101:1025  [notes §20]
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

const int PLC_M_DONE     = 1;
const int PLC_M_HOME_Z   = 5;
const int PLC_M_HOME_ROT = 6;
const int PLC_M_HOME_A1  = 7;
const int PLC_M_HOME_A2  = 8;
const int PLC_M_RUN_Z    = 10;
const int PLC_M_RUN_ROT  = 11;
const int PLC_M_RUN_A1   = 12;
const int PLC_M_RUN_A2   = 13;

#define PLC_POLL_DEVICE_CODE  "M*"
const long     PLC_POLL_DEVICE_NUM = 0;
const uint16_t PLC_POLL_WORDS      = 1;

#define PLC_HOME_REQ_PIN_NAME  "IO-0"

const bool          PLC_HOME_ACTIVE_HIGH = true;
const unsigned long PLC_HOME_TIMEOUT_MS  = 30000;

const unsigned long PLC_POLL_IDLE_DEF_MS = 5000;
unsigned long plcPollIdleMs = PLC_POLL_IDLE_DEF_MS;
const unsigned long PLC_POLL_IDLE_MS   = PLC_POLL_IDLE_DEF_MS;
const unsigned long PLC_POLL_HOMING_MS = 200;
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

// M5..M8 ARE HOME SENSORS, NOT LIMIT SWITCHES  [notes §26]
const int PLC_SENSOR_END_Z   = -1;
const int PLC_SENSOR_END_ROT = -1;
const int PLC_SENSOR_END_A1  = +1;
const int PLC_SENSOR_END_A2  = +1;

#define PLC_HOME_REQ_PIN  IO0

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

void beginRunLeg(RunPhase phase, float d1, float rot, float a1, float a2,
                 bool skipSensorBlock = false) {
  if (!skipSensorBlock) {
    String why;
    if (runLegBlockedBySensor(d1, rot, a1, a2, why)) {
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
void applyJogVelocities() {
  int32_t rotV = (int32_t)(rotVelPulses * boostMultiplier);
  int32_t armV = (int32_t)(armVelPulses * boostMultiplier);
  int32_t zV   = (int32_t)(zVelPulses   * boostMultiplier);

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

void serviceJogSoftLimits() {
  if (anyJogActive()) warnUnreferencedOnce();

  serviceArmSoftLimit(a1Dir, currentA1(), 1);
  serviceArmSoftLimit(a2Dir, currentA2(), 2);

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


// PLC TRANSPORT — MC PROTOCOL 3E, ASCII  [notes §44]

// ---- Polled state, shared by every mode ----
uint16_t      plcStatusWord   = 0;
bool          plcStatusValid  = false;
unsigned long plcLastPollOk   = 0;
unsigned long plcLastPollSent = 0;
bool          plcLinkUp       = false;
bool          plcLinkEnabled  = true;
bool plcHomeSensorPrev[4] = {false, false, false, false};
bool plcHomeRequested = false;
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
void plcAppendByte(String &s, uint8_t b) { s += (char)b; }
void plcAppendU16LE(String &s, uint16_t v) {
  plcAppendByte(s, (uint8_t)(v & 0xFF));
  plcAppendByte(s, (uint8_t)((v >> 8) & 0xFF));
}
void plcAppendU24LE(String &s, uint32_t v) {
  plcAppendByte(s, (uint8_t)(v & 0xFF));
  plcAppendByte(s, (uint8_t)((v >> 8) & 0xFF));
  plcAppendByte(s, (uint8_t)((v >> 16) & 0xFF));
}
uint8_t plcByteAt(const String &s, int i) { return (uint8_t)s.charAt(i); }
uint16_t plcU16At(const String &s, int i) {
  return (uint16_t)plcByteAt(s, i) | ((uint16_t)plcByteAt(s, i + 1) << 8);
}
String plcHexDump(const String &raw) {
  String out;
  for (int i = 0; i < (int)raw.length(); i++) {
    if (i) out += ' ';
    out += plcHex(plcByteAt(raw, i), 2);
  }
  return out;
}

String plcBuildFrame(const String &body) {
  String payload;
  plcAppendU16LE(payload, PLC_MC_MONITOR_TIMER_B);
  payload += body;
  String frame;
  plcAppendByte(frame, PLC_MC_SUBHEADER_REQ_B0);
  plcAppendByte(frame, PLC_MC_SUBHEADER_REQ_B1);
  plcAppendByte(frame, PLC_MC_NETWORK_B);
  plcAppendByte(frame, PLC_MC_PC_B);
  plcAppendByte(frame, PLC_MC_DEST_IO_LO);
  plcAppendByte(frame, PLC_MC_DEST_IO_HI);
  plcAppendByte(frame, PLC_MC_DEST_STATION_B);
  plcAppendU16LE(frame, (uint16_t)payload.length());
  frame += payload;
  return frame;
}

String plcFrameReadWordsBin(uint8_t deviceCodeByte, long deviceNum,
                            uint16_t words) {
  String body;
  plcAppendU16LE(body, PLC_MC_CMD_READ_B);
  plcAppendU16LE(body, PLC_MC_SUB_WORD_B);
  plcAppendU24LE(body, (uint32_t)deviceNum);
  plcAppendByte(body, deviceCodeByte);
  plcAppendU16LE(body, words);
  return plcBuildFrame(body);
}
#endif

String plcBuildPollFrame() {
#if PLC_MC_ASCII
  return plcFrameReadWords(PLC_POLL_DEVICE_CODE, PLC_POLL_DEVICE_NUM,
                           false, PLC_POLL_WORDS);
#else
  return plcFrameReadWordsBin(PLC_MC_DEVICE_CODE_M_B, PLC_POLL_DEVICE_NUM,
                              PLC_POLL_WORDS);
#endif
}

// *** THERE IS NO WRITE FRAME BUILDER, ON PURPOSE ***  [notes §48]

#if PLC_LINK_MODE == PLC_LINK_ETHERNET
String        plcRxBuf;
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

bool plcSend(const String &frame) {
  if (!plcEnsureConnected()) return false;
  plcSendAttempts++;
  if (plcDebug) {
#if PLC_MC_ASCII
    sendFeedback("[PLC_TX] " + frame);
#else
    sendFeedback("[PLC_TX] " + plcHexDump(frame));
#endif
  }
  plcClient.print(frame);
  plcClient.flush();
  plcRxBuf = "";
  plcTxnActive = true;
  plcTxnSentAt = millis();
  return true;
}

void plcOnGoodRead(uint16_t word) {
  bool first = !plcStatusValid;
  uint16_t previous = plcStatusWord;
  plcStatusWord  = word;
  plcStatusValid = true;
  plcLastPollOk  = millis();
  plcGoodReads++;
  if (first || plcStatusWord != previous) {
    sendFeedback("[PLC_STATE] link=UP socket=OPEN data=" + String(plcDataState())
               + " conn=" + String((unsigned long)plcConnectsOk) + "/"
               + String((unsigned long)plcConnectTries)
               + " word=" + plcHex(plcStatusWord, 4)
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

  long w = plcParseHex(frame, PLC_MC_RES_HEADER_UNITS + 8, 4);
  if (w < 0) {
    sendFeedback("[ERROR] PLC returned unreadable device data.");
    return true;
  }
  plcOnGoodRead((uint16_t)w);
  return true;
}
#else
bool plcConsumeResponse() {
  if ((int)plcRxBuf.length() < PLC_MC_RES_HEADER_UNITS + 2) return false;
  int dataLen = (int)plcU16At(plcRxBuf, PLC_MC_RES_HEADER_UNITS);
  int total = PLC_MC_RES_HEADER_UNITS + 2 + dataLen;
  if ((int)plcRxBuf.length() < total) return false;

  String frame = plcRxBuf.substring(0, total);
  plcRxBuf = plcRxBuf.substring(total);

  if (plcByteAt(frame, 0) != PLC_MC_SUBHEADER_RES_B0
   || plcByteAt(frame, 1) != PLC_MC_SUBHEADER_RES_B1) {
    sendFeedback("[ERROR] PLC response subheader was not D0 00 — the port is "
                 "probably not speaking MC protocol 3E BINARY.");
    return true;
  }

  uint16_t endCode = plcU16At(frame, PLC_MC_RES_HEADER_UNITS + 2);
  if (endCode != 0) {
    sendFeedback("[ERROR] PLC end code " + plcHex((unsigned long)endCode, 4)
               + " — the read was refused. Check that M0 exists and that MC "
                 "protocol is enabled on the port.");
    return true;
  }

  uint16_t w = plcU16At(frame, PLC_MC_RES_HEADER_UNITS + 4);
  plcOnGoodRead(w);
  return true;
}
#endif

void plcServiceRx() {
  bool got = false;
  while (plcClient.available() > 0) {
    char c = (char)plcClient.read();
    if (plcRxBuf.length() < 200) plcRxBuf += c;
    got = true;
  }
  if (got && plcDebug) {
#if PLC_MC_ASCII
    sendFeedback("[PLC_RX] " + plcRxBuf);
#else
    sendFeedback("[PLC_RX] " + plcHexDump(plcRxBuf));
#endif
  }
  if (!plcTxnActive) { plcRxBuf = ""; return; }

  if (plcConsumeResponse()) { plcTxnActive = false; return; }

  if (millis() - plcTxnSentAt >= PLC_TXN_TIMEOUT_MS) {
    plcTxnActive = false;
    plcRxBuf = "";
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

void plcTestReport(const String &raw) {
  if (raw.length() == 0) {
    sendFeedback("[PLC_TEST] RX nothing. The socket is open but the PLC did not "
                 "answer a device read — this is almost always MC protocol not "
                 "enabled on that port, or the Communication Data Code set to "
#if PLC_MC_ASCII
                 "BINARY while this board speaks ASCII."
#else
                 "ASCII while this board speaks BINARY."
#endif
                );
    return;
  }
#if PLC_MC_ASCII
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
#else
  sendFeedback("[PLC_TEST] RX " + plcHexDump(raw));
  if ((int)raw.length() < PLC_MC_RES_HEADER_UNITS + 2
   || plcByteAt(raw, 0) != PLC_MC_SUBHEADER_RES_B0
   || plcByteAt(raw, 1) != PLC_MC_SUBHEADER_RES_B1) {
    sendFeedback("[PLC_TEST] Subheader is not D0 00 — the port is answering, but "
                 "not with MC protocol 3E BINARY.");
    return;
  }
  uint16_t endCode = plcU16At(raw, PLC_MC_RES_HEADER_UNITS + 2);
  if (endCode != 0) {
    sendFeedback("[PLC_TEST] End code " + plcHex((unsigned long)endCode, 4)
               + " — the PLC refused the read. Check that M0..M15 exist and that "
                 "the module permits reads.");
    return;
  }
  plcStatusWord = plcU16At(raw, PLC_MC_RES_HEADER_UNITS + 4);
#endif
  plcStatusValid = true;
  plcLastPollOk = millis();
  sendFeedback("[PLC_TEST] OK — M0..M15 = " + plcHex((unsigned long)plcStatusWord, 4)
             + " | " + plcStatusSummary());
}
#endif

bool plcAllHomeSensors() {
  return plcBit(PLC_M_HOME_Z) && plcBit(PLC_M_HOME_ROT)
      && plcBit(PLC_M_HOME_A1) && plcBit(PLC_M_HOME_A2);
}

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
    if (fabs(delta) < 1e-3) continue;
    int dir = (delta > 0) ? 1 : -1;
    if (dir != ends[i]) continue;
    why = String(names[i]) + " is on " + String(sensor[i])
        + " and the leg would drive it further in (" + String(now[i], 2)
        + " -> " + String(want[i], 2) + ")";
    return true;
  }
  return false;
}

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
  if (plcTxnActive) return;
  unsigned long now = millis();
  unsigned long interval = isHoming ? PLC_POLL_HOMING_MS : plcPollIdleMs;
  if (plcLastPollSent != 0 && (now - plcLastPollSent) < interval) return;
  plcLastPollSent = now;
  plcSend(plcBuildPollFrame());
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
  if (plcStatusValid && plcLastPollOk != lastActedOn) {
    lastActedOn = plcLastPollOk;
    plcServiceHomeSensors();
    plcServiceSensorJogWarning();
    plcServiceHomeState();
  }
#endif
}

void plcAssertHomeRequest() {
  plcHomeRequested = true;
  plcSawRunDuringHome = false;

  digitalWrite(PLC_HOME_REQ_PIN, PLC_HOME_ACTIVE_HIGH ? HIGH : LOW);

#if PLC_LINK_MODE == PLC_LINK_ETHERNET
  plcStatusValid = false;
  sendFeedback("[PLC] HOME request asserted on " PLC_HOME_REQ_PIN_NAME
               " -> X0 (hard-wired) — held until M1 (DONE).");
#elif PLC_LINK_MODE == PLC_LINK_DIGITAL_IO
  sendFeedback("[PLC] HOME request asserted on " PLC_HOME_REQ_PIN_NAME " -> X0.");
#else
  sendFeedback("[PLC] PLACEHOLDER: HOME request raised — no PLC wired.");
#endif
}

bool plcHomeDoneAsserted() {
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
  if (!plcStatusValid) return false;
  if (plcAnyRunBit()) { plcSawRunDuringHome = true; return false; }
  if (!plcSawRunDuringHome) return false;
  if (!plcBit(PLC_M_DONE)) return false;
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

void plcClearHomeRequest() {
  if (!plcHomeRequested) return;
  plcHomeRequested = false;
  digitalWrite(PLC_HOME_REQ_PIN, PLC_HOME_ACTIVE_HIGH ? LOW : HIGH);
  sendFeedback("[PLC] HOME request cleared (" PLC_HOME_REQ_PIN_NAME " off).");
}

void plcNetworkInit() {
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
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
    sendFeedback("[ERROR] HOME timeout: PLC did not return DONE.");
#if PLC_LINK_MODE == PLC_LINK_ETHERNET
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
    String frame = plcBuildPollFrame();
#if PLC_MC_ASCII
    sendFeedback("[PLC_TEST] TX " + frame);
#else
    sendFeedback("[PLC_TEST] TX " + plcHexDump(frame));
#endif
    plcRxBuf = "";
    plcClient.print(frame);
    plcClient.flush();
    unsigned long t0 = millis();
    while (millis() - t0 < PLC_TXN_TIMEOUT_MS * 2) {
      while (plcClient.available() > 0) {
        char c = (char)plcClient.read();
        if (plcRxBuf.length() < 200) plcRxBuf += c;
      }
      if ((int)plcRxBuf.length() >= PLC_MC_RES_HEADER_UNITS + (PLC_MC_ASCII ? 4 : 2)) break;
    }
    plcTestReport(plcRxBuf);
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

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

#if ENABLE_ROT_Z_LIMIT_SENSORS
  pinMode(ROT_LIMIT_CW_PIN,  INPUT);
  pinMode(ROT_LIMIT_CCW_PIN, INPUT);
  pinMode(Z_LIMIT_UP_PIN,    INPUT);
  pinMode(Z_LIMIT_DOWN_PIN,  INPUT);
#endif
  pinMode(PLC_HOME_REQ_PIN, OUTPUT);
  plcHomeRequested = true;
  plcClearHomeRequest();
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
