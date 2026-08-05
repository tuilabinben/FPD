#include "ClearCore.h"
#include <math.h>

// ---- Motor connector mapping (as specified) ----
#define MOTOR_Z   ConnectorM0   // ZM  — Z-axis lift
#define MOTOR_ROT ConnectorM1   // RM  — rotation / turntable
#define MOTOR_A1  ConnectorM2   // AM1 — arm extension, motor 1
#define MOTOR_A2  ConnectorM3   // AM2 — arm extension, motor 2 (ganged with AM1)

#define LED_PIN LED_BUILTIN     // ClearCore: replace digitalWrite(LED_PIN, x)
                                // below with ConnectorIO0.State(x)

// ══════════════════════════════════════════════════════════════
// MOTOR CALIBRATION — PLACEHOLDERS. TUNE ALL OF THESE FOR YOUR REAL
// DRIVETRAIN BEFORE TRUSTING ANY DISTANCE OR ANGLE THIS BOARD REPORTS.
// ══════════════════════════════════════════════════════════════
const double MOTOR_STEPS_PER_REV  = 200.0;   // typical 1.8°/step motor
const double MICROSTEPS_PER_STEP  = 16.0;    // your driver's microstep setting
const double PULSES_PER_MOTOR_REV = MOTOR_STEPS_PER_REV * MICROSTEPS_PER_STEP;

// RM (rotation, M-1) — direct-drive or geared turntable
const double ROT_GEAR_RATIO     = 1.0;      // motor turns per 1 turntable turn
const double PULSES_PER_DEG_ROT = (PULSES_PER_MOTOR_REV * ROT_GEAR_RATIO) / 360.0;

// AM1/AM2 (arm extension, M-2/M-3) — e.g. belt, rack-and-pinion, or leadscrew
const double ARM_MM_PER_MOTOR_REV = 40.0;   // linear travel per motor revolution
const double PULSES_PER_MM_ARM    = PULSES_PER_MOTOR_REV / ARM_MM_PER_MOTOR_REV;

// ZM (Z lift, M-0) — e.g. ballscrew or belt
const double Z_MM_PER_MOTOR_REV = 10.0;     // linear travel per motor revolution
const double PULSES_PER_MM_Z    = PULSES_PER_MOTOR_REV / Z_MM_PER_MOTOR_REV;

// Direction inversion — flip any of these if that axis jogs backwards
// from what the GUI/keys say, or if AM1/AM2 fight each other instead of
// driving the arm together.
const bool INVERT_Z    = false;
const bool INVERT_ROT  = false;
const bool INVERT_ARM1 = false;
const bool INVERT_ARM2 = false; // AM1 and AM2 confirmed on real hardware to
                                 // need the SAME sign to drive the arm
                                 // together — do not re-enable the mirrored
                                 // guess below unless you rewire/regear AM2.
                                 // (previously defaulted true, assuming a
                                 // mirrored dual-motor drive — that guess
                                 // was wrong for this robot: it made AM1
                                 // drive forward while AM2 drove backward.)

// ---- Motion profile (same speeds as v5, now real) ----
const float ROT_SPEED_DEG_PER_SEC = 30.0;
const float ARM_SPEED_MM_PER_SEC  = 20.0;
const float Z_SPEED_MM_PER_SEC    = 15.0;

const float ROT_ACCEL_DEG_PER_SEC2 = 60.0;
const float ARM_ACCEL_MM_PER_SEC2  = 40.0;
const float Z_ACCEL_MM_PER_SEC2    = 30.0;

// Precomputed pulse-domain limits, filled in by motorsInit(). Kept as
// globals so ESTOP can reference the same accel numbers (see below).
int32_t rotVelPulses = 0, rotAccelPulses = 0;
int32_t armVelPulses = 0, armAccelPulses = 0;
int32_t zVelPulses   = 0, zAccelPulses   = 0;

// ESTOP decelerates this many times faster than a normal move. A true
// instant stop (MoveStopAbrupt) is available in ClearCore but is not
// used by default here — see the safety note at the top of this file.
const float ESTOP_DECEL_MULTIPLIER = 3.0;

// Soft travel limits for ARM / Z — PLACEHOLDER VALUES. Tune to your
// real robot's travel, or remove if hard limit switches already
// enforce this at the motor-driver level.
const float ARM_MIN_MM = 0.0,  ARM_MAX_MM = 300.0;
const float JZ_MIN_MM  = 0.0,  JZ_MAX_MM  = 200.0;
// Rotation is continuous (turntable, no hard stop / no slip ring limit).

// ---- Heartbeat / idle self-report ----
const unsigned long ALIVE_INTERVAL_MS = 2000;

// ---- Reporting cadence while moving/jogging (does not affect real
// motion, which runs continuously on the ClearCore regardless) ----
const unsigned long P2P_REPORT_INTERVAL_MS  = 150;
const unsigned long JOG_REPORT_INTERVAL_MS  = 50;
const unsigned long HOME_REPORT_INTERVAL_MS = 100;

// ---- LED activity-flash config ----
const unsigned long LED_FLASH_RX_MS = 12;   // super-fast blip: command received
const unsigned long LED_FLASH_TX_MS = 55;   // short blip: feedback sent to Python

// ================= STATE =================
bool isConnected = false;

// P2P state (Cartesian target as received; ROT/ARM/Z as actually commanded)
bool  isMoving = false;
float p2pStartRotDeg = 0, p2pStartArmMm = 0, p2pStartZMm = 0;
float p2pTargetRotDeg = 0, p2pTargetArmMm = 0, p2pTargetZMm = 0;
unsigned long lastP2pReportTime = 0;

// Jog state
int rotDir = 0;   // -1 CCW, 0 stop, 1 CW
int armDir = 0;   // -1 BACK, 0 stop, 1 FWD
int jzDir  = 0;   // -1 DOWN, 0 stop, 1 UP
unsigned long lastJogReportTime = 0;

// Homing state
bool isHoming = false;
unsigned long lastHomeReportTime = 0;

unsigned long lastAliveTime = 0;

// LED activity-flash state (non-blocking)
bool          ledOn    = false;
unsigned long ledOffAt = 0;


// ══════════════════════════════════════════════════════════════
// LED ACTIVITY INDICATOR (non-blocking — never call delay() here)
// ══════════════════════════════════════════════════════════════

void ledPulse(unsigned long durationMs) {
  digitalWrite(LED_PIN, HIGH);   // ClearCore: ConnectorIO0.State(true);
  ledOn    = true;
  ledOffAt = millis() + durationMs;
}

void serviceLed() {
  if (ledOn && (long)(millis() - ledOffAt) >= 0) {
    digitalWrite(LED_PIN, LOW);  // ClearCore: ConnectorIO0.State(false);
    ledOn = false;
  }
}

void sendFeedback(const String &line) {
  Serial.println(line);
  ledPulse(LED_FLASH_TX_MS);
}


// ══════════════════════════════════════════════════════════════
// KINEMATICS — STCR4000S is cylindrical: rotate + radial arm + lift.
// A Cartesian (X, Y) target maps to real joints via simple polar math.
// ══════════════════════════════════════════════════════════════

// Picks the ROT target nearest to currentDeg that is equivalent (mod
// 360°) to targetPrincipalDeg, so the turntable takes the short way
// around instead of unwinding a full extra turn every move.
float nearestEquivalentAngle(float currentDeg, float targetPrincipalDeg) {
  float diff = fmod(targetPrincipalDeg - currentDeg, 360.0f);
  if (diff > 180.0f)  diff -= 360.0f;
  if (diff < -180.0f) diff += 360.0f;
  return currentDeg + diff;
}

void cartesianToPolar(float x, float y, float currentRotDeg,
                      float &outRotDeg, float &outArmMm) {
  outArmMm = sqrtf(x * x + y * y);
  float principalDeg = atan2f(y, x) * 180.0f / (float)PI;
  outRotDeg = nearestEquivalentAngle(currentRotDeg, principalDeg);
}

void polarToCartesian(float rotDeg, float armMm, float &outX, float &outY) {
  float rad = rotDeg * (float)PI / 180.0f;
  outX = armMm * cosf(rad);
  outY = armMm * sinf(rad);
}

// Progress fraction (0..1) of curV between startV and targetV. An axis
// that wasn't asked to move at all reports "done" (1.0) rather than
// dividing by zero.
float axisFraction(float startV, float targetV, float curV) {
  float span = targetV - startV;
  if (fabs(span) < 0.001f) return 1.0f;
  float f = (curV - startV) / span;
  if (f < 0) f = 0;
  if (f > 1) f = 1;
  return f;
}


// ══════════════════════════════════════════════════════════════
// REAL MOTOR HELPERS
// ══════════════════════════════════════════════════════════════

void motorsInit() {
  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_LOW);
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

  rotVelPulses   = (int32_t)lround(ROT_SPEED_DEG_PER_SEC  * PULSES_PER_DEG_ROT);
  rotAccelPulses = (int32_t)lround(ROT_ACCEL_DEG_PER_SEC2 * PULSES_PER_DEG_ROT);
  armVelPulses   = (int32_t)lround(ARM_SPEED_MM_PER_SEC   * PULSES_PER_MM_ARM);
  armAccelPulses = (int32_t)lround(ARM_ACCEL_MM_PER_SEC2  * PULSES_PER_MM_ARM);
  zVelPulses     = (int32_t)lround(Z_SPEED_MM_PER_SEC     * PULSES_PER_MM_Z);
  zAccelPulses   = (int32_t)lround(Z_ACCEL_MM_PER_SEC2    * PULSES_PER_MM_Z);

  MOTOR_ROT.VelMax(rotVelPulses);
  MOTOR_ROT.AccelMax(rotAccelPulses);
  MOTOR_A1.VelMax(armVelPulses);
  MOTOR_A1.AccelMax(armAccelPulses);
  MOTOR_A2.VelMax(armVelPulses);
  MOTOR_A2.AccelMax(armAccelPulses);
  MOTOR_Z.VelMax(zVelPulses);
  MOTOR_Z.AccelMax(zAccelPulses);

  // These lines may be uncommented if your motors have inverted I/O
  // polarity (mirrors the stock StepAndDirection.ino example):
  //MOTOR_Z.PolarityInvertSDEnable(true);
  //MOTOR_ROT.PolarityInvertSDEnable(true);
  //MOTOR_A1.PolarityInvertSDEnable(true);
  //MOTOR_A2.PolarityInvertSDEnable(true);

  MOTOR_Z.EnableRequest(true);
  MOTOR_ROT.EnableRequest(true);
  MOTOR_A1.EnableRequest(true);
  MOTOR_A2.EnableRequest(true);
}

// ---- position read/write, in real-world units, invert-flag aware ----

float rotReadDeg() {
  return ((INVERT_ROT ? -1 : 1) * (float)MOTOR_ROT.PositionRefCommanded()) / PULSES_PER_DEG_ROT;
}
float armReadMm() {
  // AM1 is the reference; AM1/AM2 are expected to stay in lockstep.
  return ((INVERT_ARM1 ? -1 : 1) * (float)MOTOR_A1.PositionRefCommanded()) / PULSES_PER_MM_ARM;
}
float zReadMm() {
  return ((INVERT_Z ? -1 : 1) * (float)MOTOR_Z.PositionRefCommanded()) / PULSES_PER_MM_Z;
}

void rotMoveAbsolute(float deg) {
  int32_t p = (int32_t)lround(deg * PULSES_PER_DEG_ROT);
  MOTOR_ROT.Move((INVERT_ROT ? -p : p), StepGenerator::MOVE_TARGET_ABSOLUTE);
}
void armMoveAbsolute(float mm) {
  int32_t p = (int32_t)lround(mm * PULSES_PER_MM_ARM);
  MOTOR_A1.Move((INVERT_ARM1 ? -p : p), StepGenerator::MOVE_TARGET_ABSOLUTE);
  MOTOR_A2.Move((INVERT_ARM2 ? -p : p), StepGenerator::MOVE_TARGET_ABSOLUTE);
}
void zMoveAbsolute(float mm) {
  int32_t p = (int32_t)lround(mm * PULSES_PER_MM_Z);
  MOTOR_Z.Move((INVERT_Z ? -p : p), StepGenerator::MOVE_TARGET_ABSOLUTE);
}

void rotJogVelocity(int direction) {
  int32_t v = direction * rotVelPulses;
  MOTOR_ROT.MoveVelocity(INVERT_ROT ? -v : v);
}
void armJogVelocity(int direction) {
  int32_t v = direction * armVelPulses;
  MOTOR_A1.MoveVelocity(INVERT_ARM1 ? -v : v);
  MOTOR_A2.MoveVelocity(INVERT_ARM2 ? -v : v);
}
void zJogVelocity(int direction) {
  int32_t v = direction * zVelPulses;
  MOTOR_Z.MoveVelocity(INVERT_Z ? -v : v);
}

bool motionStepsComplete() {
  return MOTOR_ROT.StepsComplete() && MOTOR_Z.StepsComplete() &&
         MOTOR_A1.StepsComplete() && MOTOR_A2.StepsComplete();
}

// Normal decel-stop (used for STOP and for mode-switch interlocks).
void stopAllMotorsDecel() {
  MOTOR_Z.MoveStopDecel();
  MOTOR_ROT.MoveStopDecel();
  MOTOR_A1.MoveStopDecel();
  MOTOR_A2.MoveStopDecel();
}

// Faster (but still ramped, not instant) stop for ESTOP. See the
// safety note at the top of the file for why this isn't MoveStopAbrupt().
void stopAllMotorsEstop() {
  MOTOR_Z.MoveStopDecel((uint32_t)(zAccelPulses * ESTOP_DECEL_MULTIPLIER));
  MOTOR_ROT.MoveStopDecel((uint32_t)(rotAccelPulses * ESTOP_DECEL_MULTIPLIER));
  MOTOR_A1.MoveStopDecel((uint32_t)(armAccelPulses * ESTOP_DECEL_MULTIPLIER));
  MOTOR_A2.MoveStopDecel((uint32_t)(armAccelPulses * ESTOP_DECEL_MULTIPLIER));
}


// ══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // Wait for serial port (needed on some boards)

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  motorsInit();

  Serial.println("[BOOT] P2P + Joystick Robot Controller v6 (STCR4000S) ready.");
  Serial.println("[BOOT] Waiting for Python GUI connection (PING)...");
}


// ══════════════════════════════════════════════════════════════
void loop() {

  // 1. READ & DISPATCH ----------------------------------------------------
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    dispatchCommand(command);
  }

  // 2. P2P MOVE — poll for real completion, report telemetry periodically ---
  if (isMoving) {
    if (motionStepsComplete()) {
      isMoving = false;
      lastAliveTime = millis(); // avoid an immediate stale ALIVE right after completion
      sendFeedback("DA DEN DIEM DICH THANH CONG");
    } else if (millis() - lastP2pReportTime >= P2P_REPORT_INTERVAL_MS) {
      lastP2pReportTime = millis();
      reportP2pProgress();
    }
  }

  // 3. HOMING — same real-motion polling, reusing the JOG telemetry line ---
  if (isHoming) {
    if (motionStepsComplete()) {
      isHoming = false;
      lastAliveTime = millis();
      sendFeedback("[HOME] Homing complete. ROT=0 ARM=0 Z=0");
    } else if (millis() - lastHomeReportTime >= HOME_REPORT_INTERVAL_MS) {
      lastHomeReportTime = millis();
      reportJogPos();
    }
  }

  // 4. JOG — real motors are already moving (MoveVelocity); this just
  //    reports telemetry and enforces soft limits periodically -------------
  bool jogHeld = (rotDir != 0 || armDir != 0 || jzDir != 0);
  if (!isHoming && jogHeld && (millis() - lastJogReportTime >= JOG_REPORT_INTERVAL_MS)) {
    lastJogReportTime = millis();
    checkJogSoftLimits();
    reportJogPos();
  }

  // 5. IDLE SELF-REPORT (ALIVE) ------------------------------------------------
  bool trulyIdle = isConnected && !isMoving && !isHoming && !jogHeld;
  if (trulyIdle && (millis() - lastAliveTime >= ALIVE_INTERVAL_MS)) {
    lastAliveTime = millis();
    sendFeedback("[ALIVE] uptime: " + String(millis() / 1000) + "s");
  }

  // 6. LED SERVICE — turns off any RX/TX flash once its pulse has elapsed.
  serviceLed();
}


// ══════════════════════════════════════════════════════════════
// COMMAND DISPATCH
// ══════════════════════════════════════════════════════════════

void dispatchCommand(String &command) {
  // Super-fast flash on every command RECEIVED.
  ledPulse(LED_FLASH_RX_MS);

  if      (command == "PING")            handlePing();
  else if (command == "BYE")             handleBye();
  else if (command.startsWith("START:")) handleP2pStart(command);
  else if (command == "STOP")            handleStop();
  else if (command == "ROT_CW")          jogRot(1);
  else if (command == "ROT_CCW")         jogRot(-1);
  else if (command == "ROT_STOP")        jogRotStop();
  else if (command == "ARM_FWD")         jogArm(1);
  else if (command == "ARM_BACK")        jogArm(-1);
  else if (command == "ARM_STOP")        jogArmStop();
  else if (command == "Z_UP")            jogZ(1);
  else if (command == "Z_DOWN")          jogZ(-1);
  else if (command == "Z_STOP")          jogZStop();
  else if (command == "HOME")            handleHome();
  else if (command == "ESTOP")           handleEstop();
}


void handlePing() {
  // Always reply first — even mid P2P-move or mid-jog — so the Python
  // heartbeat never times out just because the board is busy.
  sendFeedback("PONG");

  if (!isConnected) {
    isConnected = true;
    sendFeedback("[CONNECTED] Python GUI handshake success.");
  }
}


void handleBye() {
  isConnected = false;
  isMoving    = false;
  isHoming    = false;
  rotDir = armDir = jzDir = 0;
  stopAllMotorsDecel();

  sendFeedback("[BYE] Python GUI disconnected.");

  // Deliberate full-dark, not just another activity blip.
  digitalWrite(LED_PIN, LOW);  // ClearCore: ConnectorIO0.State(false);
  ledOn = false;
}


void handleStop() {
  isMoving = false;
  isHoming = false;
  rotDir = armDir = jzDir = 0;
  stopAllMotorsDecel();
  sendFeedback("DUNG KHAN CAP");
}


void handleEstop() {
  isMoving = false;
  isHoming = false;
  rotDir = armDir = jzDir = 0;
  stopAllMotorsEstop();
  sendFeedback("DUNG KHAN CAP");
}


// ---- P2P (Cartesian in, polar/real-joint out) -----------------------------

void handleP2pStart(String &command) {
  if (!isConnected) {
    sendFeedback("[ERROR] Received START but not connected (no PING yet).");
    return;
  }

  // Interlock: a P2P move cancels any active jog or homing. The P2P
  // move below re-commands all three real joints, which pre-empts an
  // in-progress homing move on its own — but a single jog axis would
  // NOT touch the other two joints, so those need an explicit stop.
  if (rotDir != 0 || armDir != 0 || jzDir != 0) {
    rotDir = armDir = jzDir = 0;
    stopAllMotorsDecel();
    sendFeedback("[WARN] Jog motion canceled: P2P START received.");
  }
  if (isHoming) {
    isHoming = false;
    sendFeedback("[WARN] Homing canceled: P2P START received.");
  }

  String data = command.substring(6); // Strip "START:"
  float x0, y0, z0, x1, y1, z1;
  int parsed = sscanf(data.c_str(), "%f,%f,%f,%f,%f,%f",
                      &x0, &y0, &z0, &x1, &y1, &z1);
  if (parsed != 6) {
    sendFeedback("[ERROR] START parse failed. Expected: START:X0,Y0,Z0,X1,Y1,Z1");
    return;
  }

  // Baseline for the progress % is the REAL current position, not the
  // caller-supplied X0/Y0/Z0 hint — keeps the telemetry honest even if
  // the GUI's "start point" doesn't match where the robot actually is.
  p2pStartRotDeg = rotReadDeg();
  p2pStartArmMm  = armReadMm();
  p2pStartZMm    = zReadMm();

  cartesianToPolar(x1, y1, p2pStartRotDeg, p2pTargetRotDeg, p2pTargetArmMm);
  p2pTargetArmMm = constrain(p2pTargetArmMm, ARM_MIN_MM, ARM_MAX_MM);
  p2pTargetZMm   = constrain(z1, JZ_MIN_MM, JZ_MAX_MM);

  rotMoveAbsolute(p2pTargetRotDeg);
  armMoveAbsolute(p2pTargetArmMm);
  zMoveAbsolute(p2pTargetZMm);
  isMoving = true;
  lastP2pReportTime = millis();

  Serial.print("[START] P2P move (");
  Serial.print(x0); Serial.print(", ");
  Serial.print(y0); Serial.print(", ");
  Serial.print(z0); Serial.print(") -> (");
  Serial.print(x1); Serial.print(", ");
  Serial.print(y1); Serial.print(", ");
  Serial.print(z1); Serial.print(") [Cartesian mm] => ROT ");
  Serial.print(p2pTargetRotDeg, 1); Serial.print(" deg, ARM ");
  Serial.print(p2pTargetArmMm, 1); Serial.print(" mm, Z ");
  Serial.print(p2pTargetZMm, 1); Serial.println(" mm");
  ledPulse(LED_FLASH_TX_MS);
}


void reportP2pProgress() {
  float curRot = rotReadDeg();
  float curArm = armReadMm();
  float curZ   = zReadMm();

  float curX, curY;
  polarToCartesian(curRot, curArm, curX, curY);

  float fRot = axisFraction(p2pStartRotDeg, p2pTargetRotDeg, curRot);
  float fArm = axisFraction(p2pStartArmMm,  p2pTargetArmMm,  curArm);
  float fZ   = axisFraction(p2pStartZMm,    p2pTargetZMm,    curZ);
  int pct = (int)constrain((int)round(((fRot + fArm + fZ) / 3.0f) * 100.0f), 0, 100);

  Serial.print("[CLEARCORE POS] Vi tri hien tai -> X: ");
  Serial.print(curX, 2);
  Serial.print(" mm | Y: ");
  Serial.print(curY, 2);
  Serial.print(" mm | Z: ");
  Serial.print(curZ, 2);
  Serial.print(" mm (");
  Serial.print(pct);
  Serial.println("%)");
  ledPulse(LED_FLASH_TX_MS);
}


// ---- Jog (drives the real motors directly via MoveVelocity) --------------

void jogRot(int direction) {
  if (isMoving) {
    isMoving = false;
    stopAllMotorsDecel();
    sendFeedback("[WARN] P2P move canceled: jog command received.");
  }
  if (isHoming) {
    isHoming = false;
    stopAllMotorsDecel();
    sendFeedback("[WARN] Homing canceled: jog command received.");
  }
  rotDir = direction;
  rotJogVelocity(direction);
}
void jogArm(int direction) {
  if (isMoving) {
    isMoving = false;
    stopAllMotorsDecel();
    sendFeedback("[WARN] P2P move canceled: jog command received.");
  }
  if (isHoming) {
    isHoming = false;
    stopAllMotorsDecel();
    sendFeedback("[WARN] Homing canceled: jog command received.");
  }
  armDir = direction;
  armJogVelocity(direction);
}
void jogZ(int direction) {
  if (isMoving) {
    isMoving = false;
    stopAllMotorsDecel();
    sendFeedback("[WARN] P2P move canceled: jog command received.");
  }
  if (isHoming) {
    isHoming = false;
    stopAllMotorsDecel();
    sendFeedback("[WARN] Homing canceled: jog command received.");
  }
  jzDir = direction;
  zJogVelocity(direction);
}

void jogRotStop() { rotDir = 0; rotJogVelocity(0); }
void jogArmStop() { armDir = 0; armJogVelocity(0); }
void jogZStop()   { jzDir  = 0; zJogVelocity(0); }

// Auto-stops ARM/Z jog if it would cross a soft limit. ROT is continuous
// (turntable), so it has no limit to enforce.
void checkJogSoftLimits() {
  if (armDir != 0) {
    float a = armReadMm();
    if ((armDir > 0 && a >= ARM_MAX_MM) || (armDir < 0 && a <= ARM_MIN_MM)) {
      jogArmStop();
      sendFeedback("[WARN] ARM soft limit reached — jog auto-stopped.");
    }
  }
  if (jzDir != 0) {
    float z = zReadMm();
    if ((jzDir > 0 && z >= JZ_MAX_MM) || (jzDir < 0 && z <= JZ_MIN_MM)) {
      jogZStop();
      sendFeedback("[WARN] Z soft limit reached — jog auto-stopped.");
    }
  }
}

void reportJogPos() {
  Serial.print("[JOG POS] ROT: ");
  Serial.print(rotReadDeg(), 2);
  Serial.print(" deg | ARM: ");
  Serial.print(armReadMm(), 2);
  Serial.print(" mm | Z: ");
  Serial.print(zReadMm(), 2);
  Serial.println(" mm");
  ledPulse(LED_FLASH_TX_MS);
}


// ---- Homing ------------------------------------------------------------------

void handleHome() {
  if (!isConnected) {
    sendFeedback("[ERROR] Received HOME but not connected (no PING yet).");
    return;
  }

  rotDir = armDir = jzDir = 0;  // clear jog flags — the Move()s below
                                // pre-empt any in-progress jog velocity
  isMoving = false;             // and pre-empt any in-progress P2P move

  rotMoveAbsolute(0.0f);
  armMoveAbsolute(0.0f);
  zMoveAbsolute(0.0f);
  isHoming = true;
  lastHomeReportTime = millis();

  sendFeedback("[HOME] Homing started.");
}
