/*
 * ============================================================
 * P2P + Joystick Robot Motion Controller — Arduino Firmware v5
 * ============================================================
 * Compatible with: robot_controller_apple.py (Python GUI, Apple-style UI)
 * Baud rate: 115200
 *
 * Merges two control modes on ONE serial link:
 *   - P2P mode     : absolute X/Y/Z target moves (Cartesian, mm)
 *   - Joystick mode: held-key jog on ROT (deg) / ARM (mm) / Z (mm)
 *
 * IMPORTANT ASSUMPTION — READ BEFORE REAL HARDWARE USE:
 *   ROT/ARM/Z (jog axes) and X/Y/Z (P2P axes) are tracked as TWO
 *   SEPARATE, independently-simulated axis sets below. If your real
 *   STCR4000S kinematics mean these are actually the same physical
 *   motors (e.g. a cylindrical/SCARA arm where jog and P2P should
 *   drive the same joints), you must add the forward/inverse
 *   kinematics transform yourself and unify the state variables.
 *   Shipping this as-is against real motors without resolving that
 *   mapping risks the two "modes" fighting over the same joint.
 *
 * SAFETY DESIGN (defense in depth — enforced here AND in Python):
 *   - Starting a P2P move cancels any active jog or homing
 *   - Starting a jog command cancels any active P2P move or homing
 *   - STOP halts P2P motion AND jog AND homing (not just P2P)
 *   - ESTOP halts everything too (same effect as STOP here; kept as
 *     a distinct command so the GUI's dedicated panic button/key has
 *     its own explicit wire-level intent, and so Python's redundant
 *     3x-send convention has a stable command to repeat)
 *   - PING always replies PONG immediately, even mid-motion, so the
 *     Python heartbeat never times out just because the board is busy
 *
 * LED ACTIVITY INDICATOR (non-blocking, requested by senior):
 *   The board's status LED (LED_PIN / ClearCore IO0) is now a pure
 *   ACTIVITY light instead of a steady "connected" light:
 *     - Every command RECEIVED from Python triggers a very short,
 *       fast flash (see LED_FLASH_RX_MS) — the faster the commands
 *       arrive (e.g. held jog keys), the faster it flickers.
 *     - Every feedback line SENT to Python triggers a slightly longer
 *       flash (see LED_FLASH_TX_MS), so you can visually confirm the
 *       board is actively reporting back.
 *   Both flashes are done with millis()-based timing (ledPulse() /
 *   the LED section of loop()) — never delay() — so motion stepping,
 *   serial reads, and heartbeats are never blocked by the LED.
 *   A steady "connected" LED is no longer used: with feedback flowing
 *   every ALIVE_INTERVAL_MS while idle, a live connection now shows
 *   itself as a slow, regular flicker; a dead one goes fully dark.
 *
 * PROTOCOL — Python -> Board:
 *   PING                          heartbeat probe
 *   BYE                           graceful disconnect, IO0 off
 *   START:X0,Y0,Z0,X1,Y1,Z1       P2P absolute move
 *   STOP                          halt everything
 *   ROT_CW / ROT_CCW / ROT_STOP   jog rotation axis
 *   ARM_FWD / ARM_BACK / ARM_STOP jog arm-extension axis
 *   Z_UP / Z_DOWN / Z_STOP        jog Z axis
 *   HOME                          ramp ROT/ARM/Z back to 0
 *   ESTOP                         halt everything (panic)
 *
 * PROTOCOL — Board -> Python:
 *   PONG
 *   [CONNECTED] IO0 handshake success.
 *   [ALIVE] uptime: Xs
 *   [CLEARCORE POS] Vi tri hien tai -> X: F mm | Y: F mm | Z: F mm (P%)
 *   [JOG POS] ROT: F deg | ARM: F mm | Z: F mm
 *   DA DEN DIEM DICH THANH CONG      (P2P move complete)
 *   DUNG KHAN CAP                    (stop/estop acknowledged)
 *   [HOME] Homing started.
 *   [HOME] Homing complete. ROT=0 ARM=0 Z=0
 *   [WARN] ...   (an interlock auto-canceled a conflicting motion)
 *   [ERROR] ...  (malformed or out-of-sequence command)
 * ============================================================
 */

#define LED_PIN LED_BUILTIN   // ClearCore: replace digitalWrite(LED_PIN, x)
                              // calls below with ConnectorIO0.State(x)
                              // (x is a bool: true = on, false = off)

// ---- LED activity-flash config ----
const unsigned long LED_FLASH_RX_MS = 12;   // super-fast blip: command received
const unsigned long LED_FLASH_TX_MS = 55;   // short blip: feedback sent to Python

// ---- P2P motion config ----
const int           P2P_TOTAL_STEPS      = 20;
const unsigned long P2P_STEP_INTERVAL_MS = 150;

// ---- Jog motion config ----
const unsigned long JOG_UPDATE_INTERVAL_MS = 50;
const float ROT_SPEED_DEG_PER_SEC = 30.0;
const float ARM_SPEED_MM_PER_SEC  = 20.0;
const float Z_SPEED_MM_PER_SEC    = 15.0;

// Soft travel limits for ARM / Z jog — PLACEHOLDER VALUES.
// Tune to your real robot's travel, or remove if limit switches / the
// motor driver already enforce this at a lower level.
const float ARM_MIN_MM = 0.0,  ARM_MAX_MM = 300.0;
const float JZ_MIN_MM  = 0.0,  JZ_MAX_MM  = 200.0;
// Rotation assumed continuous (no limit / slip ring). If your turntable
// has a hard stop, add a constrain() or wraparound here too.

// ---- Homing config ----
const unsigned long HOME_STEP_INTERVAL_MS = 40;
const int           HOME_STEPS = 30;

// ---- Heartbeat / idle self-report ----
const unsigned long ALIVE_INTERVAL_MS = 2000;

// ================= STATE =================
bool isConnected = false;

// P2P state
bool  isMoving = false;
float startX = 0, startY = 0, startZ = 0;
float targetX = 0, targetY = 0, targetZ = 0;
float currentX = 0, currentY = 0, currentZ = 0;
int   p2pStep = 0;
unsigned long lastP2pStepTime = 0;

// Jog state
int   rotDir = 0;   // -1 CCW, 0 stop, 1 CW
int   armDir = 0;   // -1 BACK, 0 stop, 1 FWD
int   jzDir  = 0;   // -1 DOWN, 0 stop, 1 UP
float rotPos = 0.0, armPos = 0.0, jzPos = 0.0;
unsigned long lastJogUpdateTime = 0;

// Homing state
bool  isHoming = false;
float homeStartRot = 0, homeStartArm = 0, homeStartZ = 0;
int   homeStep = 0;
unsigned long lastHomeStepTime = 0;

unsigned long lastAliveTime = 0;

// LED activity-flash state (non-blocking)
bool          ledOn        = false;
unsigned long ledOffAt     = 0;


// ══════════════════════════════════════════════════════════════
// LED ACTIVITY INDICATOR (non-blocking — never call delay() here)
// ══════════════════════════════════════════════════════════════

// Turn the LED on now and schedule it to turn off `durationMs` from now.
// Calling this again before it turns off just extends/refreshes the
// pulse, which is exactly what you want for back-to-back RX/TX events.
void ledPulse(unsigned long durationMs) {
  digitalWrite(LED_PIN, HIGH);   // ClearCore: ConnectorIO0.State(true);
  ledOn    = true;
  ledOffAt = millis() + durationMs;
}

// Call every loop() iteration: turns the LED back off once its
// scheduled pulse duration has elapsed. Non-blocking. The subtraction
// form (rather than millis() >= ledOffAt) stays correct even across
// the ~50-day millis() rollover.
void serviceLed() {
  if (ledOn && (long)(millis() - ledOffAt) >= 0) {
    digitalWrite(LED_PIN, LOW);  // ClearCore: ConnectorIO0.State(false);
    ledOn = false;
  }
}

// Send one line of feedback to Python AND flash the LED for it.
// Use this instead of a bare Serial.println() for every status/telemetry
// line the board reports, so the LED always flickers on outgoing data.
void sendFeedback(const String &line) {
  Serial.println(line);
  ledPulse(LED_FLASH_TX_MS);
}


// ══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // Wait for serial port (needed on some boards)

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("[BOOT] P2P + Joystick Robot Controller v5 ready.");
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

  // 2. P2P STEP -------------------------------------------------------------
  if (isMoving && (millis() - lastP2pStepTime >= P2P_STEP_INTERVAL_MS)) {
    lastP2pStepTime = millis();
    executeP2pStep();
  }

  // 3. HOMING STEP ------------------------------------------------------------
  if (isHoming && (millis() - lastHomeStepTime >= HOME_STEP_INTERVAL_MS)) {
    lastHomeStepTime = millis();
    executeHomeStep();
  }

  // 4. JOG STEP (only when not homing, and at least one axis held) ------------
  bool jogHeld = (rotDir != 0 || armDir != 0 || jzDir != 0);
  if (!isHoming && jogHeld && (millis() - lastJogUpdateTime >= JOG_UPDATE_INTERVAL_MS)) {
    lastJogUpdateTime = millis();
    executeJogStep();
  }

  // 5. IDLE SELF-REPORT (ALIVE) ------------------------------------------------
  bool trulyIdle = isConnected && !isMoving && !isHoming && !jogHeld;
  if (trulyIdle && (millis() - lastAliveTime >= ALIVE_INTERVAL_MS)) {
    lastAliveTime = millis();
    sendFeedback("[ALIVE] uptime: " + String(millis() / 1000) + "s");
  }

  // 6. LED SERVICE — turns off any RX/TX flash once its pulse has elapsed.
  //    Must run every iteration, unconditionally, so flashes never "stick".
  serviceLed();
}


// ══════════════════════════════════════════════════════════════
// COMMAND DISPATCH
// ══════════════════════════════════════════════════════════════

void dispatchCommand(String &command) {
  // Super-fast flash on every command RECEIVED — the busier the link,
  // the faster this flickers. Non-blocking (see ledPulse()/serviceLed()).
  ledPulse(LED_FLASH_RX_MS);

  if      (command == "PING")            handlePing();
  else if (command == "BYE")             handleBye();
  else if (command.startsWith("START:")) handleP2pStart(command);
  else if (command == "STOP")            handleStop();
  else if (command == "ROT_CW")          startJog(rotDir, 1, "ROT_CW");
  else if (command == "ROT_CCW")         startJog(rotDir, -1, "ROT_CCW");
  else if (command == "ROT_STOP")        rotDir = 0;
  else if (command == "ARM_FWD")         startJog(armDir, 1, "ARM_FWD");
  else if (command == "ARM_BACK")        startJog(armDir, -1, "ARM_BACK");
  else if (command == "ARM_STOP")        armDir = 0;
  else if (command == "Z_UP")            startJog(jzDir, 1, "Z_UP");
  else if (command == "Z_DOWN")          startJog(jzDir, -1, "Z_DOWN");
  else if (command == "Z_STOP")          jzDir = 0;
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

  sendFeedback("[BYE] Python GUI disconnected.");

  // Deliberate full-dark, not just another activity blip — overrides
  // whatever pulse sendFeedback() just scheduled.
  digitalWrite(LED_PIN, LOW);  // ClearCore: ConnectorIO0.State(false);
  ledOn = false;
}


void handleStop() {
  isMoving = false;
  isHoming = false;
  rotDir = armDir = jzDir = 0;
  sendFeedback("DUNG KHAN CAP");
}


void handleEstop() {
  isMoving = false;
  isHoming = false;
  rotDir = armDir = jzDir = 0;
  sendFeedback("DUNG KHAN CAP");
}


// ---- P2P -----------------------------------------------------------------

void handleP2pStart(String &command) {
  if (!isConnected) {
    sendFeedback("[ERROR] Received START but not connected (no PING yet).");
    return;
  }

  // Interlock: a P2P move cancels any active jog or homing
  if (rotDir != 0 || armDir != 0 || jzDir != 0) {
    rotDir = armDir = jzDir = 0;
    sendFeedback("[WARN] Jog motion canceled: P2P START received.");
  }
  if (isHoming) {
    isHoming = false;
    sendFeedback("[WARN] Homing canceled: P2P START received.");
  }

  String data = command.substring(6); // Strip "START:"
  int parsed = sscanf(data.c_str(), "%f,%f,%f,%f,%f,%f",
                      &startX, &startY, &startZ,
                      &targetX, &targetY, &targetZ);
  if (parsed != 6) {
    sendFeedback("[ERROR] START parse failed. Expected: START:X0,Y0,Z0,X1,Y1,Z1");
    return;
  }

  currentX = startX; currentY = startY; currentZ = startZ;
  p2pStep = 0;
  isMoving = true;
  lastP2pStepTime = millis();

  Serial.print("[START] P2P move (");
  Serial.print(startX); Serial.print(", ");
  Serial.print(startY); Serial.print(", ");
  Serial.print(startZ); Serial.print(") -> (");
  Serial.print(targetX); Serial.print(", ");
  Serial.print(targetY); Serial.print(", ");
  Serial.print(targetZ); Serial.println(")");
  ledPulse(LED_FLASH_TX_MS);
}


void executeP2pStep() {
  p2pStep++;
  float progress = (float)p2pStep / P2P_TOTAL_STEPS;
  int   pct      = (int)(progress * 100);

  currentX = startX + (targetX - startX) * progress;
  currentY = startY + (targetY - startY) * progress;
  currentZ = startZ + (targetZ - startZ) * progress;

  Serial.print("[CLEARCORE POS] Vi tri hien tai -> X: ");
  Serial.print(currentX, 2);
  Serial.print(" mm | Y: ");
  Serial.print(currentY, 2);
  Serial.print(" mm | Z: ");
  Serial.print(currentZ, 2);
  Serial.print(" mm (");
  Serial.print(pct);
  Serial.println("%)");
  ledPulse(LED_FLASH_TX_MS);

  if (p2pStep >= P2P_TOTAL_STEPS) {
    isMoving = false;
    lastAliveTime = millis(); // avoid an immediate stale ALIVE right after completion
    sendFeedback("DA DEN DIEM DICH THANH CONG");
  }
}


// ---- Jog -------------------------------------------------------------------

void startJog(int &axisDir, int direction, const char *label) {
  // Interlock: a jog command cancels any active P2P move or homing
  if (isMoving) {
    isMoving = false;
    sendFeedback("[WARN] P2P move canceled: jog command received.");
  }
  if (isHoming) {
    isHoming = false;
    sendFeedback("[WARN] Homing canceled: jog command received.");
  }
  axisDir = direction;
}


void executeJogStep() {
  float dt = JOG_UPDATE_INTERVAL_MS / 1000.0;

  if (rotDir != 0) {
    rotPos += rotDir * ROT_SPEED_DEG_PER_SEC * dt;
    // No limit applied — see header note on continuous rotation.
  }
  if (armDir != 0) {
    armPos += armDir * ARM_SPEED_MM_PER_SEC * dt;
    armPos = constrain(armPos, ARM_MIN_MM, ARM_MAX_MM);
  }
  if (jzDir != 0) {
    jzPos += jzDir * Z_SPEED_MM_PER_SEC * dt;
    jzPos = constrain(jzPos, JZ_MIN_MM, JZ_MAX_MM);
  }

  reportJogPos();
}


void reportJogPos() {
  Serial.print("[JOG POS] ROT: ");
  Serial.print(rotPos, 2);
  Serial.print(" deg | ARM: ");
  Serial.print(armPos, 2);
  Serial.print(" mm | Z: ");
  Serial.print(jzPos, 2);
  Serial.println(" mm");
  ledPulse(LED_FLASH_TX_MS);
}


// ---- Homing ------------------------------------------------------------------

void handleHome() {
  if (!isConnected) {
    sendFeedback("[ERROR] Received HOME but not connected (no PING yet).");
    return;
  }

  rotDir = armDir = jzDir = 0;  // cancel any held jog first
  isMoving = false;             // cancel any P2P move

  homeStartRot = rotPos;
  homeStartArm = armPos;
  homeStartZ   = jzPos;
  homeStep = 0;
  isHoming = true;
  lastHomeStepTime = millis();

  sendFeedback("[HOME] Homing started.");
}


void executeHomeStep() {
  homeStep++;
  float progress = (float)homeStep / HOME_STEPS;

  rotPos = homeStartRot + (0.0 - homeStartRot) * progress;
  armPos = homeStartArm + (0.0 - homeStartArm) * progress;
  jzPos  = homeStartZ   + (0.0 - homeStartZ)   * progress;

  reportJogPos();

  if (homeStep >= HOME_STEPS) {
    rotPos = 0.0; armPos = 0.0; jzPos = 0.0;
    isHoming = false;
    lastAliveTime = millis();
    sendFeedback("[HOME] Homing complete. ROT=0 ARM=0 Z=0");
  }
}
