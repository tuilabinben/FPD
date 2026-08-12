// Compiles the ClearCore firmware against the desktop shim and asserts on
// what it actually sends back. Run with tests/run_tests.sh.
#include <vector>
#include <string>
std::vector<std::string> OUT;
#include "stub/ClearCore.h"
SerialC Serial;
MotorConn ConnectorM0, ConnectorM1, ConnectorM2, ConnectorM3;
MotorManager MotorMgr;
unsigned long MOCK_MILLIS = 0;
// -1 = "never written". A HOME test that passes because the pin happened
// to start at the right level would prove nothing, so the array starts at
// a value digitalWrite can never produce.
int PIN_LEVEL[64];
static const bool PIN_LEVEL_INIT = [] {
  for (int i = 0; i < 64; i++) PIN_LEVEL[i] = -1;
  return true;
}();
std::vector<std::string> ETH_TX;
std::string ETH_RX;
bool ETH_CONNECTED = true;
int  ETH_LINK = 1;
EthernetStub Ethernet;
#include "../RobotMotionController_v9_ClearCore/RobotMotionController_v9_ClearCore.ino"

int PASS = 0, FAIL = 0;
void check(bool c, const char *m) {
  if (c) { PASS++; printf("  OK   %s\n", m); }
  else   { FAIL++; printf("  FAIL %s\n", m); }
}
bool saw(const char *n) {
  for (auto &l : OUT) if (l.find(n) != std::string::npos) return true;
  return false;
}
void run(const char *c) { OUT.clear(); handleCommand(String(c)); }
void dump() { for (auto &l : OUT) printf("       | %s\n", l.c_str()); }

// ---- PLC harness helpers ----
std::string lastTx() { return ETH_TX.empty() ? std::string() : ETH_TX.back(); }
void clearTx() { ETH_TX.clear(); }
void advance(unsigned long ms) { MOCK_MILLIS += ms; }

// Builds the reply a real PLC sends to a 1-word batch read: header, then a
// data length (end code + one word), end code 0000, then the word. Tracks
// PLC_MC_ASCII so every OTHER test in this file — HOME sequencing, sensor
// decode, all of it — can call plcPoll()/plcReply() without caring which
// wire format the board is compiled for.
std::string readReply(uint16_t word) {
#if PLC_MC_ASCII
  char buf[64];
  snprintf(buf, sizeof buf, "D00000FF03FF0000080000%04X", word);
  return buf;
#else
  std::string out;
  auto pushB   = [&](uint8_t b)  { out.push_back((char)b); };
  auto pushU16 = [&](uint16_t v) { pushB(v & 0xFF); pushB((v >> 8) & 0xFF); };
  pushB(0xD0); pushB(0x00);           // subheader
  pushB(0x00);                        // network
  pushB(0xFF);                        // pc
  pushB(0xFF); pushB(0x03);           // io, low byte first (03FF)
  pushB(0x00);                        // station
  pushU16(4);                         // data length: end code(2) + word(2)
  pushU16(0);                         // end code 0
  pushU16(word);
  return out;
#endif
}
// Builds the reply a PLC sends when it REFUSES a read: header + end code,
// no device data — a real error reply carries none.
std::string errorReply(uint16_t endCode) {
#if PLC_MC_ASCII
  char buf[32];
  snprintf(buf, sizeof buf, "D00000FF03FF0000" "0004" "%04X", (unsigned)endCode);
  return buf;
#else
  std::string out;
  auto pushB   = [&](uint8_t b)  { out.push_back((char)b); };
  auto pushU16 = [&](uint16_t v) { pushB(v & 0xFF); pushB((v >> 8) & 0xFF); };
  pushB(0xD0); pushB(0x00);
  pushB(0x00);
  pushB(0xFF);
  pushB(0xFF); pushB(0x03);
  pushB(0x00);
  pushU16(2);                         // data length: end code only
  pushU16(endCode);
  return out;
#endif
}
// No writeReply(): the board never writes a PLC device any more. The HOME
// request is a wire on IO-0, so there is no write to answer.

// Feeds a reply and lets the firmware collect it.
void plcReply(const std::string &frame) {
  ETH_RX += frame;
  advance(1);
  servicePlc();
}
// One complete poll cycle: let the poll go out, answer it with `word`.
//
// Any request still in flight is closed out first. The firmware allows one
// outstanding transaction at a time, so a poll left unanswered by an
// earlier test would otherwise block the next one and the assertion would
// be reading a stale status word.
void plcPoll(uint16_t word) {
  if (plcTxnActive) plcReply(readReply(0));
  advance(PLC_POLL_MS + 1);
  servicePlc();              // sends the read request
  plcReply(readReply(word));  // answers it
}
#define BIT(n) (1u << (n))

int main() {
  motorsInit();

  printf("\n=== A. speed model: 150 RPM master, per-axis %% ===\n");
  run("PROFILE");
  check(fabs(masterRpm - 150.0) < 0.01, "master 150 RPM");
  check(fabs(armPct - 125.0) < 0.01, "AM 125%");
  check(fabs(rotPct - 75.0)  < 0.01, "RM 75%");
  check(fabs(zPct   - 50.0)  < 0.01, "ZM 50%");
  check(fabs(armMotorRpmActual - 187.5) < 0.05, "AM -> 187.5 motor RPM");
  check(fabs(rotVelDegS - 23.74) < 0.02, "RM -> 23.74 deg/s (i_RM 28.4375)");
  check(fabs(zVelMmS - 18.75) < 0.02, "ZM -> 18.75 mm/s (20 mm/rev)");
  run("SET_SPEED:150,375,75,125,50,75,125,50");
  check(saw("[MOTION_OK]"), "the GUI's exact wire message is accepted (8 fields)");
  run("SET_SPEED:150,375,900,900,900,900,900,900");
  check(!saw("[ERROR]"), "no percentage cap, including the new accel fields");

  printf("\n  -- acceleration %% is independent of speed %% --\n");
  run("SET_SPEED:150,375,75,125,50,75,125,50");   // accel% == speed%, today's default shape
  float velAtAcc75 = rotVelDegS, accAtAcc75 = rotAccDegS2;
  run("SET_SPEED:150,375,75,125,50,10,10,10");    // same speeds, accel% dropped
  check(fabs(rotVelDegS - velAtAcc75) < 1e-3,
        "RM velocity is unchanged by the new accel field");
  check(rotAccDegS2 < accAtAcc75 - 1e-3,
        "  ...but RM acceleration drops when only rotAccPct drops");
  check(fabs(rotAccPct - 10.0) < 0.01, "rotAccPct is stored independently");
  check(fabs(rotPct - 75.0) < 0.01, "  ...rotPct itself is untouched");

  printf("\n  -- a stale 5-arg SET_SPEED is refused, not misread --\n");
  run("SET_SPEED:150,375,75,125,50");
  check(saw("[ERROR] SET_SPEED needs"), "the old GUI's 5-field form is now refused");
  run("SET_SPEED:150,375,75,125,50,75,125,50");   // restore for anything after this

  printf("\n=== B. THE ARM ANGLE IS ROTATION FROM HOME ===\n");
  check(fabs(FOLD_ANGLE_HOME_DEG - 0.0) < 1e-9, "home is 0 deg, not 60");
  check(fabs(FOLD_ANGLE_MIN_DEG - 0.0) < 1e-9, "travel starts at 0");
  check(fabs(FOLD_ANGLE_MAX_DEG - 120.0) < 1e-9, "and ends at 120 (was 180)");
  check(fabs(ARM_ZERO_CAD_DEG - 60.0) < 1e-9, "0 deg from home == th3_cad 60");
  check(fabs(currentA1() - 0.0) < 1e-6, "A1M reads 0 at the reference pose");
  check(fabs(currentA2() - 0.0) < 1e-6, "A2M too");

  printf("\n  -- the reach curve is unchanged, only its labels moved --\n");
  check(fabs(reachFromFoldAngle(0.0)   - 133.2) < 0.05, "0 deg   -> 133.2 mm (retracted)");
  check(fabs(reachFromFoldAngle(120.0) - 613.2) < 0.05, "120 deg -> 613.2 mm (straight)");
  check(fabs(reachFromFoldAngle(91.72) - 575.0) < 0.5,  "91.72   -> 575 mm (JEL drawing)");
  check(fabs(foldAngleFromReach(133.2) - 0.0)   < 0.05, "133.2 mm -> 0 deg");
  check(fabs(foldAngleFromReach(613.2) - 120.0) < 0.05, "613.2 mm -> 120 deg");
  check(fabs(foldAngleFromReach(reachFromFoldAngle(37.5)) - 37.5) < 1e-6,
        "the pair round-trips");
  check(fabs(FOLD_SINGULARITY_WARN_DEG - 110.0) < 1e-9,
        "the singularity warning moved with the frame (110, was 170)");

  printf("\n  -- reachBandFor finds extremes INSIDE the band, in the new frame --\n");
  { double lo, hi;
    reachBandFor(0.0, 120.0, lo, hi);
    check(fabs(lo - 133.2) < 0.05 && fabs(hi - 613.2) < 0.05,
          "the normal band 0..120 -> 133.2..613.2 mm");
    // cos peaks where fold+60 is a multiple of 180, i.e. at 120, 300, -60.
    reachBandFor(-260.0, 480.0, lo, hi);
    check(fabs(lo - (-26.8)) < 0.5 && fabs(hi - 613.2) < 0.05,
          "a wide taught band -260..480 spans the WHOLE curve, not just its ends");
    reachBandFor(119.0, 121.0, lo, hi);
    check(fabs(hi - 613.2) < 0.05,
          "a band straddling 120 still finds the peak between its endpoints"); }

  printf("\n=== C. taught boundaries: no envelope, unordered ===\n");
  isHomed = true;
  run("RESET_LIMITS");
  check(fabs(limA1Min - 0.0) < 1e-9 && fabs(limA1Max - 240.0) < 1e-9,
        "factory elbow band is 0..240 MOTOR deg (= fold 0..120 at ratio 2)");
  run("SET_LIMIT:A1,MAX,1000");
  check(fabs(limA1Max - 1000.0) < 1e-6, "1000 deg accepted - there is NO ceiling");
  check(!saw("physical envelope"), "  ...and no envelope complaint");
  run("SET_LIMIT:A1,MIN,-5000");
  check(fabs(limA1Min + 5000.0) < 1e-6, "-5000 deg accepted - no floor either");
  run("RESET_LIMITS");
  run("SET_LIMIT:A1,MIN,800"); run("SET_LIMIT:A1,MAX,100");
  check(!saw("must stay at least"), "an inverted pair is NOT rejected");
  check(fabs(limA1Min - 800.0) < 1e-6 && fabs(limA1Max - 100.0) < 1e-6,
        "  ...BOTH taught numbers survive exactly as captured");
  { double lo, hi; armBand(1, lo, hi);
    check(fabs(lo - 100.0) < 1e-6 && fabs(hi - 800.0) < 1e-6,
          "  ...and the band READS 100..800 whatever the teaching order"); }
  run("RESET_LIMITS"); run("SET_LIMIT:A2,MIN,70"); run("SET_LIMIT:A2,MAX,70");
  check(saw("no room to move"), "both ends on ONE position is refused");
  run("SET_LIMIT:A2,MAX,70.5");
  check(fabs(limA2Max - 70.5) < 1e-6, "a 0.5 deg span IS allowed");
  run("RESET_LIMITS");
  run("SET_LIMIT:Z,MAX,9000");   check(saw("physical envelope"), "ZM keeps its envelope");
  run("SET_LIMIT:ROT,MAX,-500"); check(saw("physical envelope"), "RM keeps its envelope");
  run("SET_LIMIT:Z,MIN,50"); run("SET_LIMIT:Z,MAX,40");
  check(saw("must stay at least"), "ZM keeps its ordering rule");
  run("RESET_LIMITS");
  run("SET_LIMIT_HERE:A1,MIN");
  check(saw("captured from the current position"), "SET_LIMIT_HERE works on an elbow");
  check(saw("A1 MIN = 0.00"), "  ...and captures 0.00, the live A1M_POS");

  printf("\n=== D. IK still solves in the new frame ===\n");
  run("RESET_LIMITS");
  { IkResult r = solveIkFrogleg(1, 300, 0, 514.3 + 120);
    check(r.ok, "a 300 mm target solves");
    check(r.th3 > 0.0 && r.th3 < 120.0, "  ...to an elbow angle inside 0..120");
    check(fabs(reachFromFoldAngle(r.th3) - 300.0) < 0.5,
          "  ...that really does put the wafer centre at 300 mm"); }
  run("SET_LIMIT:A1,MIN,-260"); run("SET_LIMIT:A1,MAX,480");
  { IkResult r = solveIkFrogleg(1, 300, 0, 514.3 + 120);
    check(r.ok, "and still solves with a taught band spanning the whole curve"); }
  run("RESET_LIMITS"); run("SET_LIMIT:A1,MAX,40");
  a1Dir = 1; serviceArmSoftLimit(a1Dir, 45.0f, 1);
  check(a1Dir == 0, "jog stops at a taught elbow boundary");
  a1Dir = 0;

  printf("\n  -- per-axis enforcement really does release ONE axis --\n");
  // The button in the GUI says ENFORCED / NOT ENFORCED, so the board has to
  // mean it per axis. Both elbows are given the same tight ceiling and only
  // A1's switch is thrown: A1 must run through it and A2 must still stop.
  run("SET_LIMIT:A2,MAX,40");
  run("SET_LIMIT_ENFORCE:A1,0");
  check(saw("[LIMIT_ENFORCE] A1 = 0"), "SET_LIMIT_ENFORCE:A1,0 is accepted");
  check(saw("[WARN]"), "  ...loudly, because it makes the machine less safe");
  a1Dir = 1; serviceArmSoftLimit(a1Dir, 45.0f, 1);
  check(a1Dir == 1, "  ...and A1 is no longer stopped at its boundary");
  a2Dir = 1; serviceArmSoftLimit(a2Dir, 45.0f, 2);
  check(a2Dir == 0, "  ...while A2, untouched, still stops at its own");
  a1Dir = a2Dir = 0;
  { String why;
    check(jointTargetIsLegal(10, 0, 500, 20, why),
          "  ...a target past A1's boundary is accepted too, not refused by a "
          "limit the board has been told to stop applying");
    check(!jointTargetIsLegal(10, 0, 20, 500, why),
          "  ...while the same target on A2 is still refused"); }
  run("SET_LIMIT:A1,MAX,90");
  check(saw("[LIMIT_SET] A1 MAX = 90.00"),
        "  ...a switched-off boundary is still EDITABLE — teaching one is "
        "exactly what you do while it is off");
  run("SET_LIMIT_ENFORCE:A1,1");
  a1Dir = 1; serviceArmSoftLimit(a1Dir, 95.0f, 1);
  check(a1Dir == 0, "  ...and switching it back on re-arms A1 immediately");
  a1Dir = 0;
  // The master switch is an AND, not an override: it must not re-arm an
  // axis the operator switched off on its own.
  run("SET_LIMIT_ENFORCE:A1,0"); run("SET_LIMITS_ENABLED:0");
  run("SET_LIMITS_ENABLED:1");
  a1Dir = 1; serviceArmSoftLimit(a1Dir, 95.0f, 1);
  check(a1Dir == 1, "the master switch does NOT re-arm an individually-off axis");
  a1Dir = 0;
  run("SET_LIMIT_ENFORCE:A1,1");
  run("SET_LIMIT_LOCK:A1,1");
  check(saw("SET_LIMIT_LOCK no longer exists"),
        "the retired SET_LIMIT_LOCK is refused, not quietly aliased");
  run("RESET_LIMITS");

  printf("\n=== E. PID / coords ===\n");
  run("SET_PID:1,2,3,60");
  check(saw("[PARAMS_OK]"), "SET_PID works");
  check(fabs(currentKp - 1.0) < 1e-6 && fabs(currentN - 60.0) < 1e-6,
        "  ...and the gains actually land");
  run("PID_RESET");
  check(saw("[PARAMS_OK]"), "PID_RESET works");
  check(fabs(currentKp - PID_PRESET_KP) < 1e-6, "  ...restoring the report preset");
  run("RESET_COORD");      check(saw("[COORD"), "RESET_COORD works");

  printf("\n=== F. MATLAB parity: solve_ik_frogleg from mophong_init.m ===\n");
  // The verbatim MATLAB helper, transcribed. If the board's IK and this
  // ever disagree, one of them has drifted from the Simscape model that
  // generated the geometry — which is the thing that must not happen
  // quietly, because the simulation is where the numbers are validated.
  auto matlabIk = [](double X, double Y, double Z,
                     double &d1, double &th2, double &th3_cad) {
    const double Zoff = 388.0 + 50.0 + 46.5 + 24.8 + 5.0;   // 514.3
    d1 = Z - Zoff;
    if (d1 < 0) d1 = 0; else if (d1 > 285) d1 = 285;
    th2 = atan2(Y, X) * RAD_TO_DEG;
    double R = sqrt(X * X + Y * Y);
    double c = (R - (45.0 + 248.2)) / (160.0 + 160.0);
    if (c > 1) c = 1; else if (c < -1) c = -1;
    th3_cad = 180.0 - acos(c) * RAD_TO_DEG;
  };
  run("RESET_LIMITS");
  { double worstD1 = 0, worstRot = 0, worstTh3 = 0;
    int solved = 0;
    for (double r = 140; r <= 610; r += 10) {
      for (double a = 0; a <= 340; a += 20) {
        for (double dz = 0; dz <= 280; dz += 70) {
          double X = r * cos(a * DEG_TO_RAD), Y = r * sin(a * DEG_TO_RAD);
          double Z = 514.3 + dz;
          IkResult got = solveIkFrogleg(1, X, Y, Z);
          if (!got.ok) continue;
          solved++;
          double md1, mth2, mth3;
          matlabIk(X, Y, Z, md1, mth2, mth3);
          worstD1  = max(worstD1,  fabs(got.d1  - md1));
          // RM's zero moved to its CCW stop, so the board reports a
          // bearing in [0, 360) where MATLAB's atan2 reports (-180, 180].
          // The GEOMETRY must still be identical, so the two may differ
          // only by a whole turn — anything else is a real drift.
          double dRot = fmod(fabs(got.th2 - mth2), 360.0);
          if (dRot > 180.0) dRot = 360.0 - dRot;
          worstRot = max(worstRot, dRot);
          // The board reports rotation from home; MATLAB reports th3_cad.
          // ARM_ZERO_CAD_DEG is the whole of the difference.
          worstTh3 = max(worstTh3, fabs((got.th3 + ARM_ZERO_CAD_DEG) - mth3));
        }
      }
    }
    printf("       | %d poses compared, worst error d1=%.2e rot=%.2e th3=%.2e\n",
           solved, worstD1, worstRot, worstTh3);
    check(solved > 500, "the sweep actually solved a few hundred poses");
    check(worstD1  < 1e-9, "d1 matches mophong_init.m to machine precision");
    check(worstRot < 1e-9, "th2 matches mophong_init.m modulo a whole turn");
    check(worstTh3 < 1e-9, "th3 matches, once ARM_ZERO_CAD_DEG is added back"); }

  check(fabs(FOLD_ANGLE_SPEC_MAX_DEG - 91.72) < 0.01,
        "FOLD_ANGLE_SPEC_MAX_DEG is 91.72 (from-home), not 151.72 (th3_cad)");
  check(fabs(reachFromFoldAngle(FOLD_ANGLE_SPEC_MAX_DEG) - 575.0) < 0.5,
        "  ...and really does land on the drawing's 575 mm");

  printf("\n  -- the idle arm HOLDS, it does not snap home --\n");
  // 45, not 560: ikToJoints takes Z FROM HOME now. 560 used to be an
  // absolute height and is 275 mm off the top of a 285 mm stroke in the
  // new frame — which is exactly why the frame change had to be visible
  // here rather than silently reinterpreted.
  { MOTOR_A2.PositionRefSet(0);
    float d1, rot, a1, a2;
    OUT.clear();
    bool ok = ikToJoints(1, 300, 0, 45, d1, rot, a1, a2);
    check(ok, "a single-arm IK still solves");
    check(fabs(a2 - currentA2()) < 1e-6,
          "A2M's target is its CURRENT angle, not FOLD_ANGLE_HOME_DEG"); }

  printf("\n  -- a taught boundary bites WITHOUT a reference, and cannot trap --\n");
  // The reported bug: A1M taught to -341.89, machine never homed, jog ran
  // to -427 because softLimitsActive() waited for isHomed.
  run("RESET_LIMITS");
  isHomed = false;
  run("SET_LIMIT:A1,MIN,-341.89"); run("SET_LIMIT:A1,MAX,902.14");
  MOTOR_A1.PositionRefSet((int32_t)lround(-341.0 * PULSES_PER_DEG_ARM_MOTOR));
  a1Dir = -1; serviceArmSoftLimit(a1Dir, -342.0f, 1);
  check(a1Dir == 0, "unreferenced, retracting past the taught -341.89 IS stopped");
  a1Dir = 1;  serviceArmSoftLimit(a1Dir, -342.0f, 1);
  check(a1Dir == 1, "  ...while extending back INTO the band is allowed");
  // Sitting far outside a boundary taught in an earlier session.
  a1Dir = -1; serviceArmSoftLimit(a1Dir, -500.0f, 1);
  check(a1Dir == 0, "  ...outside the band, going further out is refused");
  a1Dir = 1;  serviceArmSoftLimit(a1Dir, -500.0f, 1);
  check(a1Dir == 1, "  ...and coming back is not — the axis is never trapped");
  a1Dir = 0;
  MOTOR_A1.PositionRefSet(0);
  isHomed = true;
  run("RESET_LIMITS");

  printf("\n  -- there is NO structural reach floor; YOUR band is the limit --\n");
  // 133.2 mm was R(fold = 0). It assumed the elbow's zero really is the
  // folded home pose, through an armGearRatio nobody has measured — a
  // guess, and one that rejected radii the arm can physically hold.
  run("RESET_LIMITS");
  run("SET_LIMIT_ENFORCE:A1,0");
  { IkResult r = solveIkFromHome(1, 50, 0, 45);
    check(r.ok, "r = 50 mm solves with A1's boundary switched off");
    check(r.th3 < 0.0, "  ...to a NEGATIVE fold angle, which is not an error"); }
  { IkResult r = solveIkFromHome(1, 700, 0, 45);
    check(!r.ok, "700 mm is still refused — no elbow angle reaches it");
    check(std::string(r.error.c_str()).find("no solution") != std::string::npos,
          "  ...as arithmetic, not as an opinion about the envelope"); }
  run("SET_LIMIT_ENFORCE:A1,1");
  run("SET_LIMIT:A1,MIN,10"); run("SET_LIMIT:A1,MAX,230");
  { IkResult r = solveIkFromHome(1, 50, 0, 45);
    check(!r.ok, "with the boundary back on, 50 mm is outside the taught band");
    check(std::string(r.error.c_str()).find("YOU taught") != std::string::npos,
          "  ...and the message names it as YOUR limit, not fixed structure"); }
  { // A2's own band is untouched, so it answers for itself.
    run("SET_LIMIT_ENFORCE:A2,0");
    IkResult r = solveIkFromHome(2, 50, 0, 45);
    check(r.ok, "and the switch is per arm — A2 still solves 50 mm");
    run("SET_LIMIT_ENFORCE:A2,1"); }
  run("RESET_LIMITS");

  printf("\n  -- HOME is the P2P reference: X 0, Y 0, Z 0 --\n");
  run("RESET_LIMITS");
  { // Z on the wire is height above HOME, so d1 comes back equal to it.
    IkResult r = solveIkFromHome(1, 300, 0, 45);
    check(r.ok && fabs(r.d1 - 45.0) < 1e-6,
          "Z from HOME lands on d1 unchanged — Z 0 IS the bottom of the stroke");
    // Same carriage travel, second deck: d1 must be IDENTICAL, because one
    // carriage lifts both. The 9 mm is in the absolute height, not in d1.
    IkResult r2 = solveIkFromHome(2, 300, 0, 45);
    check(r2.ok && fabs(r2.d1 - r.d1) < 1e-6,
          "  ...and is the same d1 on arm 2 — one carriage, one Z");
    // The absolute frame underneath is untouched, which is what keeps the
    // MATLAB parity sweep below meaningful.
    IkResult abs1 = solveIkFrogleg(1, 300, 0, 514.3 + 45);
    check(fabs(abs1.d1 - r.d1) < 1e-6,
          "  ...and matches the absolute call it delegates to"); }
  { IkResult neg = solveIkFromHome(1, 300, 0, -5);
    check(!neg.ok, "a NEGATIVE Z is refused — there is nothing below HOME");
    check(std::string(neg.error.c_str()).find("from HOME") != std::string::npos,
          "  ...and the message is in the frame the operator typed in"); }
  { IkResult tall = solveIkFromHome(1, 300, 0, 560);
    check(!tall.ok,
          "560 is refused too — it was a valid ABSOLUTE height and is 275 mm "
          "off the top of the stroke from HOME"); }
  OUT.clear();
  run("LOAD_XYZ_BOTH:300,0,45,250,0,36");
  check(saw("must be EQUAL"),
        "LOAD_XYZ_BOTH refuses the old Za-Zb=9 form — from HOME they are equal");
  OUT.clear();
  run("LOAD_XYZ_BOTH:300,0,45,250,0,45");
  check(saw("[LOADED]"), "  ...and accepts one shared Z for both arms");
  OUT.clear();
  run("FK:45,0,0,0,1");
  check(saw("(from HOME)"),
        "FK answers in the same frame IK accepts, with Zabs alongside");

  printf("\n=== G. PLC link: MC protocol 3E %s frames ===\n",
         PLC_MC_ASCII ? "ASCII" : "BINARY");
  check(PLC_LINK_MODE == PLC_LINK_ETHERNET, "the Ethernet link is the compiled default");
  check(PLC_PORT == 1025, "port 1025, from the PLC configuration screen");
  check(PLC_MC_ASCII == 0,
        "BINARY, not ASCII — read directly off the PLC's own Ethernet "
        "Configuration screen (Own Node Settings -> Communication Data Code "
        "= Binary). A mismatch here is not a partial failure: the PLC "
        "silently drops every frame in the wrong format, which is exactly "
        "what the machine did before this was corrected.");

  printf("\n  -- frame construction --\n");
#if PLC_MC_ASCII
  { String f = plcFrameReadWords("M*", 0, false, 1);
    check(std::string(f.c_str()) == "5000" "00" "FF" "03FF" "00"
                                   "0018" "0002" "0401" "0000" "M*000000" "0001",
          "batch read of M0, 1 word, is byte-for-byte the 3E ASCII frame");
    // The length field is the character count from the monitoring timer
    // on. Hand-counting it is how these frames get silently rejected.
    long declared = plcParseHex(f, PLC_MC_RES_HEADER_UNITS, 4);
    check(declared == f.length() - 18,
          "  ...and its declared length matches the real payload length"); }
#else
  { // Verified independently against a byte-level probe script sent from a
    // PC on the same subnet, not just derived from the spec by hand.
    String f = plcBuildPollFrame();
    static const uint8_t expected[] = {
      0x50, 0x00, 0x00, 0xFF, 0xFF, 0x03, 0x00,        // header, 7 bytes
      0x0C, 0x00,                                      // length = 12 (LE)
      0x02, 0x00,                                      // monitoring timer
      0x01, 0x04,                                      // command 0401 (LE)
      0x00, 0x00,                                      // subcommand 0000 (LE)
      0x00, 0x00, 0x00,                                // device number 0, 3 bytes LE
      0x90,                                             // device code, M
      0x01, 0x00,                                      // 1 word (LE)
    };
    bool match = f.length() == sizeof(expected);
    for (int i = 0; match && i < (int)f.length(); i++) {
      if (plcByteAt(f, i) != expected[i]) match = false;
    }
    check(match, "batch read of M0, 1 word, is byte-for-byte the 3E BINARY frame");
    // The length field is the BYTE count from the monitoring timer on —
    // the binary analogue of the ASCII character-count check above.
    check(plcU16At(f, PLC_MC_RES_HEADER_UNITS)
              == f.length() - (PLC_MC_RES_HEADER_UNITS + 2),
          "  ...and its declared length matches the real payload length"); }
#endif
  // There is deliberately no write-frame test, because there is no write
  // frame — the code would not compile if one were called. The HOME
  // request is a wire from IO-0 into X0; writing X0 over MC protocol never
  // worked reliably, because the PLC refreshes X from its input terminals
  // every scan and overwrites whatever was written. python_check.py
  // asserts the builder's absence in the source text.
  check(PLC_HOME_REQ_PIN == IO0, "the HOME request line is IO-0, in every link mode");

  printf("\n  -- the status word decodes to the right devices --\n");
  ETH_CONNECTED = true; ETH_RX.clear(); clearTx();
  advance(10);
  plcPoll(BIT(1) | BIT(7) | BIT(12));
  check(plcStatusValid, "a poll reply is accepted");
  check(plcBit(PLC_M_DONE), "M1 (DONE) decoded");
  check(plcBit(PLC_M_HOME_A1), "M7 (OutR1 home sensor) decoded");
  check(plcBit(PLC_M_RUN_A1), "M12 (Run A1M) decoded");
  check(!plcBit(PLC_M_HOME_Z) && !plcBit(PLC_M_RUN_Z), "M5 and M10 correctly clear");
  check(plcAnyRunBit(), "  ...and a single run bit counts as running");

  printf("\n  -- a PLC error end code is reported, not swallowed --\n");
  OUT.clear(); clearTx(); ETH_RX.clear();
  advance(PLC_POLL_MS + 1); servicePlc();
  plcReply(errorReply(0x2401));
  check(saw("end code 2401"), "end code 2401 is logged verbatim");

  printf("\n=== H. HOME handshake: IO-0 wire out, M10..M13 then M1 back ===\n");
  OUT.clear(); clearTx(); ETH_RX.clear();
  PIN_LEVEL[IO0] = -1;
  isHomed = false;
  beginHoming();
  check(isHoming, "HOME asserts");
  check(PIN_LEVEL[IO0] == HIGH, "  ...by driving IO-0 HIGH, a wire into X0");
  check(saw("[PLC] HOME request asserted on IO-0"), "  ...and says so");
  // Nothing goes out on the socket. The request is not a packet, and the
  // poll that follows must not be mistaken for one.
  check(lastTx().find("1401") == std::string::npos,
        "  ...and sends NO write frame — the link is read-only");

  // DONE already latched from a previous cycle must NOT end this one.
  OUT.clear();
  plcPoll(BIT(PLC_M_DONE));
  serviceHoming();
  check(isHoming, "a stale latched M1 does NOT complete the home");
  check(!isHomed, "  ...and does not set the reference either");

  // PLC starts driving: run bits come on.
  plcPoll(BIT(PLC_M_RUN_Z) | BIT(PLC_M_RUN_ROT) | BIT(PLC_M_DONE));
  serviceHoming();
  check(isHoming, "still homing while the run bits are on");

  // Run bits drop and DONE is set -> finished.
  OUT.clear(); clearTx();
  plcPoll(BIT(PLC_M_DONE));
  serviceHoming();
  check(!isHoming, "run bits all off + M1 set completes the home");
  check(isHomed, "  ...and THAT is what sets the reference");
  check(saw("[HOME] Homing complete"), "  ...and reports it");
  check(PIN_LEVEL[IO0] == LOW,
        "  ...and IO-0 is released, not left latched asking for another home");

  printf("\n  -- the poll speeds up WHILE homing, or the run bits are missed --\n");
  // The completion gate needs to SEE M10..M13 come on and go off. At the
  // 5 s idle rate a home sequence shorter than one interval finishes
  // between two polls, plcSawRunDuringHome never gets set, and a home that
  // physically succeeded fails on the 30 s timeout — intermittently.
  check(PLC_POLL_IDLE_MS == 5000, "idle polling is 5 s, as asked");
  check(PLC_POLL_HOMING_MS < PLC_POLL_IDLE_MS,
        "  ...but homing polls faster, and that is a correctness requirement");
  OUT.clear(); clearTx(); ETH_RX.clear();
  isHomed = false;
  beginHoming();
  { // Idle interval has NOT elapsed, but a home is running, so a poll goes out.
    if (plcTxnActive) plcReply(readReply(0));
    clearTx();
    advance(PLC_POLL_HOMING_MS + 1);
    servicePlc();
    check(!lastTx().empty(),
          "a poll goes out after 200 ms while homing, not 5 s"); }
  plcReply(readReply(0));
  finishHoming(false, "test cleanup");
  { // Idle again: the same 200 ms must now buy nothing.
    if (plcTxnActive) plcReply(readReply(0));
    advance(PLC_POLL_IDLE_MS + 1); servicePlc(); plcReply(readReply(0));
    clearTx();
    advance(PLC_POLL_HOMING_MS + 1);
    servicePlc();
    check(lastTx().empty(),
          "  ...and back to the slow rate once the home is over"); }

  printf("\n  -- run bits without DONE times out instead of hanging --\n");
  OUT.clear(); clearTx(); ETH_RX.clear();
  isHomed = false;
  beginHoming();
  plcPoll(BIT(PLC_M_RUN_A1));
  serviceHoming();
  check(isHoming, "waiting");
  advance(PLC_HOME_TIMEOUT_MS + 10);
  plcPoll(0);
  serviceHoming();
  check(!isHoming, "the home gives up on the timeout");
  check(!isHomed, "  ...without claiming a reference");
  check(saw("[HOME] FAILED"), "  ...and says FAILED");
  check(PIN_LEVEL[IO0] == LOW,
        "  ...and drops IO-0 on the way out. A failed home that left the line "
        "asserted would re-home the moment the PLC was ready");

  printf("\n  -- a dead socket cannot strand the request asserted --\n");
  // This is the defect the wire fixes. The old code cleared X0 with an
  // Ethernet write, which a dropped socket could refuse; the PLC then kept
  // the request latched and re-homed the machine when the link returned.
  OUT.clear(); clearTx();
  isHomed = false;
  beginHoming();
  check(PIN_LEVEL[IO0] == HIGH, "request asserted");
  ETH_CONNECTED = false;
  finishHoming(false, "cable pulled");
  check(PIN_LEVEL[IO0] == LOW, "  ...and released even with the socket down");
  ETH_CONNECTED = true;

  printf("\n=== I. all four sensors work; JOG warns, P2P refuses ===\n");
  run("RESET_LIMITS");
  isHomed = true;
  OUT.clear(); ETH_RX.clear(); clearTx();
  for (int i = 0; i < 4; i++) { plcHomeSensorPrev[i] = false; plcJogWarned[i] = false; }
  plcHomeStatePrev = false;
  double a1MaxBefore = limA1Max, zMinBefore = limD1Min, rotMinBefore = limRotMin;

  printf("\n  -- the four sit at OPPOSITE ends --\n");
  check(PLC_SENSOR_END_Z == -1 && PLC_SENSOR_END_ROT == -1,
        "M5 and M6 mark the MINIMUM of their axis (down, CCW)");
  check(PLC_SENSOR_END_A1 == +1 && PLC_SENSOR_END_A2 == +1,
        "M7 and M8 mark the MAXIMUM of theirs (arms fully extended)");

  printf("\n  -- JOG only warns, it never stops the axis --\n");
  jzDir = -1; rotDir = -1; a1Dir = 1; a2Dir = 1;
  plcPoll(BIT(PLC_M_HOME_Z) | BIT(PLC_M_HOME_ROT)
        | BIT(PLC_M_HOME_A1) | BIT(PLC_M_HOME_A2));
  check(jzDir == -1 && rotDir == -1 && a1Dir == 1 && a2Dir == 1,
        "jogging INTO all four covered sensors is NOT blocked");
  check(saw("[WARN] ZM is jogging INTO M5 MinZ"), "  ...ZM warns");
  check(saw("[WARN] A1M is jogging INTO M7 OutR1"), "  ...and A1M warns");
  check(!saw("[LIMIT]"), "  ...and nothing is reported as a limit");

  // Warned once per entry, not once per poll.
  OUT.clear();
  plcPoll(BIT(PLC_M_HOME_Z) | BIT(PLC_M_HOME_ROT)
        | BIT(PLC_M_HOME_A1) | BIT(PLC_M_HOME_A2));
  check(!saw("jogging INTO"), "a held jog does not re-warn on every poll");
  // Jogging the other way is not warned about at all.
  OUT.clear();
  jzDir = 1; rotDir = 1; a1Dir = -1; a2Dir = -1;
  plcPoll(BIT(PLC_M_HOME_Z) | BIT(PLC_M_HOME_ROT)
        | BIT(PLC_M_HOME_A1) | BIT(PLC_M_HOME_A2));
  check(!saw("jogging INTO"), "  ...and jogging AWAY is silent");
  jzDir = 0; rotDir = 0; a1Dir = 0; a2Dir = 0;

  printf("\n  -- P2P refuses a leg that drives further into a covered sensor --\n");
  { String why;
    // Motors read 0 in the stub, so a NEGATIVE d1 target drives into M5.
    check(runLegBlockedBySensor(-10.0f, 0.0f, 0.0f, 0.0f, why),
          "with M5 covered, a leg that lowers ZM is refused");
    check(std::string(why.c_str()).find("M5 MinZ") != std::string::npos,
          "  ...and the reason names the sensor");
    why = String("");
    check(!runLegBlockedBySensor(50.0f, 0.0f, 0.0f, 0.0f, why),
          "  ...but a leg that RAISES ZM is allowed");
    // M7 is at the far end, so a POSITIVE a1 target drives into it.
    check(runLegBlockedBySensor(0.0f, 0.0f, 30.0f, 0.0f, why),
          "with M7 covered, a leg that extends A1M is refused");
    check(!runLegBlockedBySensor(0.0f, 0.0f, -30.0f, 0.0f, why),
          "  ...and retracting A1M is allowed — the opposite end");
    check(!runLegBlockedBySensor(0.0f, 0.0f, 0.0f, 0.0f, why),
          "an axis that is not moving is never blocked"); }

  printf("\n  -- and RUN actually stops --\n");
  OUT.clear();
  { float d1, rot, a1, a2;
    isMoving = false;
    beginRunLeg(PHASE_TO_A, -10.0f, 0.0f, 0.0f, 0.0f);
    check(!isMoving && runPhase == PHASE_NONE,
          "beginRunLeg abandons the program rather than starting the leg");
    check(saw("[ERROR] RUN stopped"), "  ...and says why");
    check(saw("Jog that axis off its sensor"), "  ...and how to clear it"); }

  // With every sensor clear, a leg starts normally.
  OUT.clear();
  plcPoll(0);
  { String why;
    check(!runLegBlockedBySensor(-10.0f, 0.0f, 30.0f, 0.0f, why),
          "with the sensors clear, nothing is refused"); }

  // Still no boundary is ever written from a sensor.
  check(fabs(limA1Max - a1MaxBefore) < 1e-9
        && fabs(limD1Min - zMinBefore) < 1e-9
        && fabs(limRotMin - rotMinBefore) < 1e-9,
        "no sensor writes a working boundary");
  check(!saw("[PLC_LIMIT_SET]"), "  ...and there is no such message");

  printf("\n  -- HOME STATE: M5+M6 on, M7+M8 off -> reset the coordinates --\n");
  OUT.clear(); ETH_RX.clear(); clearTx();
  isHomed = false;
  for (int i = 0; i < 4; i++) plcHomeSensorPrev[i] = false;
  plcHomeStatePrev = false;
  MOTOR_Z.PositionRefSet(4321);          // pretend the counters have drifted
  plcPoll(BIT(PLC_M_HOME_Z));            // only one sensor -> not home yet
  check(!isHomed, "M5 alone is not the HOME state");
  plcPoll(BIT(PLC_M_HOME_Z) | BIT(PLC_M_HOME_ROT));
  check(plcHomeStateActive(), "M5 + M6 with M7/M8 clear IS the HOME state");
  check(isHomed, "  ...and it sets the reference");
  check(saw("[COORD_RESET]"), "  ...by zeroing the coordinates");
  check(saw("[PLC_HOME] HOME STATE"), "  ...and saying which condition fired");

  // A broken sensor reading ON must not count as home.
  OUT.clear();
  plcHomeStatePrev = false; isHomed = false;
  plcPoll(BIT(PLC_M_HOME_Z) | BIT(PLC_M_HOME_ROT) | BIT(PLC_M_HOME_A1));
  check(!plcHomeStateActive(),
        "M7 covered blocks the HOME state — at home the arms are pulled IN");
  check(!isHomed, "  ...so nothing is zeroed");

  printf("\n  -- it latches ONCE, not on every poll --\n");
  OUT.clear();
  plcHomeStatePrev = false; isHomed = false;
  plcPoll(BIT(PLC_M_HOME_Z) | BIT(PLC_M_HOME_ROT));
  check(saw("[COORD_RESET]"), "first entry into HOME state resets");
  OUT.clear();
  plcPoll(BIT(PLC_M_HOME_Z) | BIT(PLC_M_HOME_ROT));   // still held
  check(!saw("[COORD_RESET]"),
        "  ...and holding there does NOT keep re-zeroing, which would eat real motion");

  printf("\n  -- and never while the machine is moving --\n");
  OUT.clear();
  plcHomeStatePrev = false;
  plcPoll(0);                                  // leave home state
  jzDir = 1;                                   // now jogging
  plcPoll(BIT(PLC_M_HOME_Z) | BIT(PLC_M_HOME_ROT));
  check(saw("NOT reset"), "a HOME state reached mid-jog refuses to zero");
  jzDir = 0;
  OUT.clear();
  plcPoll(BIT(PLC_M_HOME_Z) | BIT(PLC_M_HOME_ROT));
  check(saw("[COORD_RESET]"), "  ...and latches as soon as motion stops");
  run("RESET_LIMITS");

  printf("\n  -- diagnostics --\n");
  OUT.clear();
  run("PLC_STATUS");
  check(saw("[PLC_STATE]"), "PLC_STATUS answers");
  OUT.clear();
  run("PLC_RECONNECT");
  check(saw("[PLC] Socket dropped"), "PLC_RECONNECT answers");

  printf("\n  -- an absent PLC fails on a timeout, it does not block --\n");
  ETH_CONNECTED = false; ETH_RX.clear(); clearTx();
  OUT.clear();
  plcClient.stop();
  plcLastConnectTry = 0;
  // Long enough for BOTH the reconnect rate limit and the idle poll
  // interval. The poll is what triggers the connect attempt, and idle
  // polling is 5 s now, so advancing only past PLC_RECONNECT_MS left the
  // board with nothing to do and the assertion read a silence it had
  // caused itself.
  advance((PLC_RECONNECT_MS > PLC_POLL_IDLE_MS ? PLC_RECONNECT_MS
                                               : PLC_POLL_IDLE_MS) + 1);
  servicePlc();
  check(saw("[ERROR] PLC unreachable"), "an unreachable PLC is reported once");
  OUT.clear();
  advance(1);
  servicePlc();
  check(!saw("[ERROR] PLC unreachable"), "  ...and not again on the next pass");
  ETH_CONNECTED = true;

  printf("\n=== J. gearing constants ===\n");
  check(fabs(ROT_GEAR_RATIO_DEF - 4.375 * 6.5) < 1e-9, "i_RM = 4.375 * 6.5");
  check(fabs(Z_MM_PER_MOTOR_REV - 20.0) < 1e-9, "ZM 20 mm/rev");

  printf("\n=== K. RM gear ratio has a runtime calibration escape hatch ===\n");
  // The stub's MotorConn does not persist PositionRefSet (always reads back
  // 0), so this exercises the real, load-bearing arithmetic -- pulses per
  // RM degree -- rather than a position readback the stub cannot provide.
  check(fabs(rotGearRatio - ROT_GEAR_RATIO_DEF) < 1e-9,
        "rotGearRatio starts at the modelled default");
  double pulsesAtDefault = pulsesPerDegRot();
  run("SET_ROT_RATIO:14.21875");   // half the default
  check(fabs(rotGearRatio - 14.21875) < 1e-6, "SET_ROT_RATIO changes the runtime ratio");
  check(saw("[ROT_RATIO]"), "  ...and confirms it");
  check(saw("Re-check"), "  ...and warns the taught RM limits need re-checking");
  check(fabs(pulsesPerDegRot() - pulsesAtDefault / 2.0) < 1e-3,
        "  ...and pulses-per-RM-degree halves with it");
  run("SET_ROT_RATIO:0.0001");
  check(saw("[ERROR]"), "out-of-range ratio refused");
  check(fabs(rotGearRatio - 14.21875) < 1e-6, "  ...and the ratio is unchanged");
  run("SET_ROT_RATIO:20");
  run("ROT_RATIO");
  check(saw("[ROT_RATIO] 20.0000"), "the bare query reports the live ratio");
  run("SET_ROT_RATIO:28.4375");    // restore for anything running after this

  printf("\n=== L. RESET_POSITION: no PLC, sensor block skippable, limits checked ===\n");
  OUT.clear(); isMoving = false; isHoming = false; runPhase = PHASE_NONE;
  clearTx();
  run("RESET_POSITION");
  check(isMoving && runPhase == PHASE_RESET_HOME,
        "RESET_POSITION starts a run-leg-style move");
  check(!isHoming, "  ...and never touches the PLC homing state");
  check(lastTx().empty(), "  ...and sends nothing to the PLC socket");
  isMoving = false; runPhase = PHASE_NONE;

  printf("\n  -- refused while something is already moving --\n");
  OUT.clear();
  isMoving = true;
  run("RESET_POSITION");
  check(!(runPhase == PHASE_RESET_HOME), "refused outright while isMoving is already set");
  check(saw("[ERROR] RESET_POSITION refused"), "  ...and says why");
  isMoving = false; runPhase = PHASE_NONE;

  printf("\n  -- beginRunLeg's skipSensorBlock actually skips the check --\n");
  // The stub's MotorConn cannot hold a nonzero position (PositionRefSet is a
  // no-op, PositionRefCommanded always reads 0), so RESET_POSITION's own
  // fixed (0,0,0,0) target can never demonstrate a blocked-vs-skipped
  // difference here -- with "now" always 0, the delta to (0,0,0,0) is
  // always 0, "not moving that axis". This tests the shared primitive
  // beginResetPosition() calls, with a target that DOES produce a delta.
  OUT.clear(); isMoving = false; runPhase = PHASE_NONE;
  plcPoll(BIT(PLC_M_HOME_Z));                          // M5 covered
  beginRunLeg(PHASE_TO_A, -50.0f, 0.0f, 0.0f, 0.0f);   // driving further into M5
  check(!isMoving && runPhase == PHASE_NONE,
        "without skip, a leg driving further into a covered sensor is refused");
  check(saw("[ERROR] RUN stopped"), "  ...and says why");
  OUT.clear();
  beginRunLeg(PHASE_RESET_HOME, -50.0f, 0.0f, 0.0f, 0.0f, /*skipSensorBlock=*/true);
  check(runPhase == PHASE_RESET_HOME,
        "  ...but WITH skipSensorBlock=true, the identical leg proceeds");
  isMoving = false; runPhase = PHASE_NONE;
  plcPoll(0);                                           // clear the sensor bit

  printf("\n  -- taught soft limits are still checked, unlike the sensor block --\n");
  OUT.clear();
  run("SET_LIMITS_ENABLED:1"); run("SET_LIMIT_ENFORCE:ROT,1");
  run("SET_LIMIT:ROT,MIN,10");           // home (RM=0) now outside the band
  run("RESET_POSITION");
  check(!isMoving && runPhase == PHASE_NONE, "home outside a taught limit is refused");
  check(saw("[ERROR] RESET_POSITION refused"), "  ...and says why");
  run("RESET_LIMITS");                   // restore

  printf("\n  -- completion uses its own message, not [RUN] or [HOME] --\n");
  OUT.clear(); isMoving = false; runPhase = PHASE_NONE;
  run("RESET_POSITION");
  check(isMoving, "sanity: the move actually started");
  OUT.clear();
  serviceRun();          // stub StepsComplete() is always true, so this settles at once
  check(saw("[RESET_POSITION] TARGET REACHED"), "distinct completion message");
  check(!saw("[RUN] TARGET REACHED"), "  ...not the RUN text");
  check(!isMoving && runPhase == PHASE_NONE, "  ...and the move is fully finished");

  printf("\n%s  (%d passed, %d failed)\n",
         FAIL ? "FIRMWARE CHECKS FAILED" : "ALL FIRMWARE CHECKS PASSED",
         PASS, FAIL);
  return FAIL ? 1 : 0;
}
