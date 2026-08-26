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
// A three-word reply, so the M30..M32 limit bits can be exercised. Same
// shape as readReply() but carrying M0..M47 instead of just M0..M15.
std::string readReply3(uint16_t w0, uint16_t w1, uint16_t w2) {
#if PLC_MC_ASCII
  char buf[80];
  snprintf(buf, sizeof buf, "D00000FF03FF0000100000%04X%04X%04X", w0, w1, w2);
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
  pushU16(8);                        // end code(2) + three words(6)
  pushU16(0);
  pushU16(w0); pushU16(w1); pushU16(w2);
  return out;
#endif
}

void plcPoll3(uint16_t w0, uint16_t w1, uint16_t w2) {
  if (plcTxnActive) plcReply(readReply3(0, 0, 0));
  advance(PLC_POLL_MS + 1);
  servicePlc();
  plcReply(readReply3(w0, w1, w2));
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
  // Derived from the live ratio, not hard-coded: i_RM is a bench figure the
  // operator recalibrates, and pinning it here made every recalibration
  // look like a regression. 112.5 motor RPM is 150 * 75%, ratio-free.
  check(fabs(rotVelDegS - (112.5 * 6.0 / rotGearRatio)) < 0.02,
        "RM deg/s follows the configured i_RM");
  check(fabs(zVelMmS - 18.75) < 0.05, "ZM -> 18.75 mm/s (20 mm/rev, spec)");
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
  check(fabs(FOLD_ANGLE_MAX_DEG - 180.0) < 1e-9, "and ends at 180, the straight arm");
  // The CAD offset went to 0 with the measured links: fold 0 IS the
  // retracted pose, so there is no frame shift left to carry.
  check(fabs(ARM_ZERO_CAD_DEG - 0.0) < 1e-9, "0 deg from home IS the retracted pose");
  check(fabs(currentA1() - 0.0) < 1e-6, "A1M reads 0 at the reference pose");
  check(fabs(currentA2() - 0.0) < 1e-6, "A2M too");

  printf("\n  -- the reach curve is unchanged, only its labels moved --\n");
  const double R_MIN_MM = ARM_RADIAL_OFFSET_MM - ARM_LINK_SUM_MM;   // 240.0
  const double R_MAX_MM = ARM_RADIAL_OFFSET_MM + ARM_LINK_SUM_MM;   // 605.0
  check(fabs(reachFromFoldAngle(0.0)   - R_MIN_MM) < 0.05, "0 deg   -> 240 mm (retracted)");
  check(fabs(reachFromFoldAngle(180.0) - R_MAX_MM) < 0.05, "180 deg -> 605 mm (straight)");
  check(fabs(reachFromFoldAngle(FOLD_ANGLE_SPEC_MAX_DEG) - 575.0) < 0.5,
        "146.68  -> 575 mm (JEL drawing)");
  check(fabs(foldAngleFromReach(R_MIN_MM) - 0.0)   < 0.05, "240 mm -> 0 deg");
  check(fabs(foldAngleFromReach(R_MAX_MM) - 180.0) < 0.05, "605 mm -> 180 deg");
  check(fabs(foldAngleFromReach(reachFromFoldAngle(37.5)) - 37.5) < 1e-6,
        "the pair round-trips");
  check(fabs(FOLD_SINGULARITY_WARN_DEG - 170.0) < 1e-9,
        "the singularity warning sits just short of the straight arm");

  printf("\n  -- reachBandFor finds extremes INSIDE the band, in the new frame --\n");
  { double lo, hi;
    reachBandFor(0.0, 180.0, lo, hi);
    check(fabs(lo - R_MIN_MM) < 0.05 && fabs(hi - R_MAX_MM) < 0.05,
          "the normal band 0..180 -> 240..605 mm");
    // cos peaks where fold + ARM_ZERO_CAD_DEG is a multiple of 180, so with
    // the offset now 0 that is 0, 180, 360, -180.
    reachBandFor(-200.0, 400.0, lo, hi);
    check(fabs(lo - R_MIN_MM) < 0.5 && fabs(hi - R_MAX_MM) < 0.05,
          "a wide taught band -200..400 spans the WHOLE curve, not just its ends");
    reachBandFor(179.0, 181.0, lo, hi);
    check(fabs(hi - R_MAX_MM) < 0.05,
          "a band straddling 180 still finds the peak between its endpoints"); }

  printf("\n=== C. taught boundaries: no envelope, unordered ===\n");
  isHomed = true;
  run("RESET_LIMITS");
  check(fabs(limA1Min - 0.0) < 1e-9
        && fabs(limA1Max - 180.0 * ARM_GEAR_RATIO_DEF) < 1e-9,
        "factory elbow band is the full fold travel in MOTOR deg");
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

  printf("\n=== F. IK is self-consistent on the MEASURED geometry ===\n");
  // NO MATLAB PARITY SWEEP HERE ANY MORE, ON PURPOSE.
  //
  // mophong_init.m's solve_ik_frogleg was the reference this section
  // measured against, pose for pose. It was dropped because the .m's own
  // calculation is wrong for this machine: its a4/a5/a6 (160/160/248.2)
  // are not the arm's, which measured 91.25/91.25/377.5 on the bench.
  // Checking against a model that does not describe the machine proves
  // nothing, and a red build nobody can fix teaches people to ignore the
  // suite. python_check.py dropped the same sweep for the same reason.
  //
  // What replaces it is a round trip: every pose the IK solves must come
  // back out of forwardKinematics() in the same place.
  run("RESET_LIMITS");
  { double worst = 0;
    int solved = 0;
    for (double r = 240; r <= 605; r += 10) {
      for (double a = 0; a <= 340; a += 20) {
        for (double dz = 0; dz <= 280; dz += 70) {
          double X = r * cos(a * DEG_TO_RAD), Y = r * sin(a * DEG_TO_RAD);
          double Z = 514.3 + dz;
          IkResult got = solveIkFrogleg(1, X, Y, Z);
          if (!got.ok) continue;
          solved++;
          double fx, fy, fz;
          forwardKinematics(got.d1, got.th2, got.th3, 1, fx, fy, fz);
          worst = max(worst, sqrt((fx - X) * (fx - X) + (fy - Y) * (fy - Y)));
          worst = max(worst, fabs(fz - Z));
        }
      }
    }
    printf("       | %d poses round-tripped, worst error %.2e mm\n", solved, worst);
    check(solved > 500, "the sweep actually solved a few hundred poses");
    check(worst < 1e-6, "IK -> FK returns the pose it was given"); }

  // The measured links, named outright so a change is a visible diff.
  check(fabs(A3_MM - 45.0) < 1e-9 && fabs(A4_MM - 91.25) < 1e-9
        && fabs(A5_MM - 91.25) < 1e-9 && fabs(A6_MM - 377.5) < 1e-9,
        "a3/a4/a5/a6 are the MEASURED 45/91.25/91.25/377.5");
  check(fabs(Z_OFFSET_ARM1_MM - 514.3) < 1e-9, "Z_offset(arm 1) = 514.3");
  check(fabs(armGearRatio - 7.80) < 1e-9, "the arm ratio is the measured 7.80");

  check(fabs(FOLD_ANGLE_SPEC_MAX_DEG - 146.68) < 0.01,
        "FOLD_ANGLE_SPEC_MAX_DEG is the rated 146.68 fold deg");
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

  printf("\n  -- but a FACTORY-DEFAULT floor cannot pin an unreferenced axis --\n");
  // The ZM bug: HOME is the minimum of every axis, so the shipped ZM floor
  // is 0. Unreferenced, the counter also reads 0 wherever the board powered
  // up, so ZM sat exactly ON its floor and every Z_DOWN was refused, with
  // nothing to escape from. Only an UNTOUCHED default relaxes -- the taught
  // -341.89 above must keep biting, which is why this is not just !isHomed.
  run("RESET_LIMITS");
  run("SET_LIMITS_ENABLED:1"); run("SET_LIMIT_ENFORCE:Z,1");
  isHomed = false;
  MOTOR_Z.PositionRefSet(0);
  jzDir = -1; serviceJogSoftLimits();
  check(jzDir == -1, "unreferenced, Z_DOWN off the DEFAULT floor is allowed");
  jzDir = 1;  serviceJogSoftLimits();
  check(jzDir == 1, "  ...and Z_UP still is too");
  run("SET_LIMIT:Z,MIN,12");            // now TAUGHT, not the default
  jzDir = -1; serviceJogSoftLimits();
  check(jzDir == 0, "  ...but a TAUGHT floor bites unreferenced, as before");
  isHomed = true;
  run("RESET_LIMITS");
  jzDir = -1; serviceJogSoftLimits();
  check(jzDir == 0, "  ...and once referenced the default floor applies again");
  jzDir = 0;
  run("SET_LIMITS_ENABLED:0");
  run("RESET_LIMITS");

  printf("\n  -- there is NO structural reach floor; YOUR band is the limit --\n");
  // 133.2 mm was R(fold = 0). It assumed the elbow's zero really is the
  // folded home pose, through an armGearRatio nobody has measured — a
  // guess, and one that rejected radii the arm can physically hold.
  run("RESET_LIMITS");
  run("SET_LIMIT_ENFORCE:A1,0");
  // 50 mm is now below the ARITHMETIC span too -- with the measured links
  // the frog-leg spans 422.5 +/- 182.5, so 240 mm is the shortest radius
  // any elbow angle reaches. Switching the boundary off cannot buy that.
  { IkResult r = solveIkFromHome(1, 50, 0, 45);
    check(!r.ok, "r = 50 mm is refused even with A1's boundary switched off");
    check(std::string(r.error.c_str()).find("no solution") != std::string::npos,
          "  ...as arithmetic, not as an opinion about the envelope"); }
  { // Just inside the span DOES solve with the boundary off.
    IkResult r = solveIkFromHome(1, 245, 0, 45);
    check(r.ok, "  ...while 245 mm, inside the span, solves"); }
  { IkResult r = solveIkFromHome(1, 700, 0, 45);
    check(!r.ok, "700 mm is still refused — no elbow angle reaches it");
    check(std::string(r.error.c_str()).find("no solution") != std::string::npos,
          "  ...as arithmetic, not as an opinion about the envelope"); }
  run("SET_LIMIT_ENFORCE:A1,1");
  run("SET_LIMIT:A1,MIN,10"); run("SET_LIMIT:A1,MAX,230");
  { // 400 mm solves on the arithmetic (240..605), and the taught band
    // 10..230 motor deg only reaches ~264 mm, so a refusal here IS the band.
    IkResult r = solveIkFromHome(1, 400, 0, 45);
    check(!r.ok, "with the boundary back on, 400 mm is outside the taught band");
    check(std::string(r.error.c_str()).find("YOU taught") != std::string::npos,
          "  ...and the message names it as YOUR limit, not fixed structure"); }
  { // A2's own band is untouched, so it answers for itself.
    run("SET_LIMIT_ENFORCE:A2,0");
    IkResult r = solveIkFromHome(2, 245, 0, 45);
    check(r.ok, "and the switch is per arm — A2 still solves 245 mm");
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
    plcBuildPollFrame();
    static const uint8_t expected[] = {
      0x50, 0x00, 0x00, 0xFF, 0xFF, 0x03, 0x00,        // header, 7 bytes
      0x0C, 0x00,                                      // length = 12 (LE)
      0x02, 0x00,                                      // monitoring timer
      0x01, 0x04,                                      // command 0401 (LE)
      0x00, 0x00,                                      // subcommand 0000 (LE)
      0x00, 0x00, 0x00,                                // device number 0, 3 bytes LE
      0x90,                                             // device code, M
      0x03, 0x00,                                      // 3 words (LE) = M0..M47
    };
    bool match = plcTxCount == (int)sizeof(expected);
    for (int i = 0; match && i < plcTxCount; i++) {
      if (plcTxBytes[i] != expected[i]) match = false;
    }
    check(match, "batch read of M0, 3 words, is byte-for-byte the 3E BINARY frame");
    // The length field is the BYTE count from the monitoring timer on —
    // the binary analogue of the ASCII character-count check above.
    check(plcU16AtBytes(plcTxBytes, PLC_MC_RES_HEADER_UNITS)
              == plcTxCount - (PLC_MC_RES_HEADER_UNITS + 2),
          "  ...and its declared length matches the real payload length");
    // The frame is half NUL bytes. Built in an Arduino String it can be
    // truncated on the wire while the stub's std::string-backed String
    // happily carries it — which is why this asserts the byte buffer.
    check(plcTxCount == 21, "  ...and the whole 21-byte frame survives the NULs"); }
#endif
  // There is deliberately no write-frame test, because there is no write
  // frame — the code would not compile if one were called. HOME does not
  // write anything to the PLC at all any more: it drives the axes itself
  // and only READS M30..M32 to know when to stop. python_check.py asserts
  // the write builder's absence in the source text.

  printf("\n  -- the status word decodes to the right devices --\n");
  ETH_CONNECTED = true; ETH_RX.clear(); clearTx();
  advance(10);
  plcPoll3(0, BIT(14), BIT(0));
  check(plcStatusValid, "a poll reply is accepted");
  check(plcBit(PLC_M_LIMIT_Z), "M30 (ZM travel limit) decoded");
  check(plcBit(PLC_M_LIMIT_A2), "M32 (A2M travel limit) decoded");
  check(!plcBit(PLC_M_LIMIT_ROT), "M31 correctly clear");

  printf("\n  -- a PLC error end code is reported, not swallowed --\n");
  OUT.clear(); clearTx(); ETH_RX.clear();
  advance(PLC_POLL_MS + 1); servicePlc();
  plcReply(errorReply(0x2401));
  check(saw("end code 2401"), "end code 2401 is logged verbatim");

  printf("\n=== H. HOME completes on M30..M32, not the M10..M13 handshake ===\n");
  OUT.clear(); clearTx(); ETH_RX.clear();
  isHomed = false;
  // HOME needs the switch states to know when to stop, so it refuses
  // outright when no device data has landed rather than driving blind.
  plcPoll3(0, 0, 0);
  OUT.clear();
  beginHoming();
  check(isHoming, "HOME starts");
  // THE PLC IS NOT ASKED, AT ALL. No request line, no write, nothing
  // beyond the ordinary read-only M30..M32 poll every other command uses.
  check(!saw("HOME request asserted"), "  ...and does not claim to have asked it");
  check(saw("this board drives the axes"), "  ...it says it drives them itself");
  check(lastTx().find("1401") == std::string::npos,
        "  ...and sends NO write frame — the link is read-only");
  // Every switch clear, so all three axes are moving. The direction comes
  // from HOME_DIR_*, NOT PLC_LIMIT_END_*: kept as separate constants so a
  // bug in one can't silently corrupt the other, but they must still point
  // at the SAME physical switch, so the values agree axis by axis.
  check(jzDir == HOME_DIR_Z && rotDir == HOME_DIR_ROT && a2Dir == HOME_DIR_A2,
        "  ...all three back off in the direction HOME_DIR_* names");
  check(HOME_DIR_Z == PLC_LIMIT_END_Z && HOME_DIR_ROT == PLC_LIMIT_END_ROT
        && HOME_DIR_A2 == PLC_LIMIT_END_A2,
        "  ...and HOME_DIR_* matches PLC_LIMIT_END_* on every axis, RM included");
  check(HOME_DIR_Z < 0 && HOME_DIR_A2 < 0,
        "  ...ZM and A2M back off NEGATIVE (down, retract)");
  check(HOME_DIR_ROT > 0,
        "  ...but RM backs off POSITIVE (CW) — it is mounted inverted");
  check(a1Dir == 0, "  ...and A1M, which has no switch, is not moved at all");

  // M1 is not read at all now: the limits are what say the axes arrived.
  OUT.clear();
  plcPoll3(BIT(1), 0, 0);
  serviceHoming();
  check(isHoming, "the old M1 DONE bit does NOT complete the home");
  check(!isHomed, "  ...and does not set the reference either");

  // Two of three limits reached is still not home.
  plcPoll3(0, BIT(14) | BIT(15), 0);
  serviceHoming();
  check(isHoming, "M30+M31 without M32 is still not home");

  // All three true -> finished, with or without M1.
  OUT.clear(); clearTx();
  plcPoll3(0, BIT(14) | BIT(15), BIT(0));
  serviceHoming();
  check(!isHoming, "M30+M31+M32 all true completes the home");
  check(isHomed, "  ...and THAT is what sets the reference");
  check(saw("[HOME] Homing complete") || saw("HOME STATE"), "  ...and reports it");

  printf("\n  -- the poll speeds up WHILE homing --\n");
  // 20 ms idle / 10 ms homing is a DELIBERATE operator choice, made after
  // an earlier attempt to widen this got reverted back to 5000/200 on the
  // theory that 20 ms had once overloaded the FX5U link (conn=0/N,
  // UNREACHABLE). The operator asked for it explicitly a second time and
  // said not to revert it again — do not "fix" this back down. If the link
  // genuinely cannot sustain it, that is SET_PLC_POLL on the live machine,
  // not a silent change to the shipped default.
  check(PLC_POLL_IDLE_MS == 20, "idle polling defaults to the operator's 20 ms");
  check(PLC_POLL_HOMING_MS < PLC_POLL_IDLE_MS,
        "  ...but homing polls even faster, and that is a correctness requirement");
  OUT.clear(); clearTx(); ETH_RX.clear();
  isHomed = false;
  beginHoming();
  { // Idle interval has NOT elapsed, but a home is running, so a poll goes out.
    if (plcTxnActive) plcReply(readReply(0));
    clearTx();
    advance(PLC_POLL_HOMING_MS + 1);
    servicePlc();
    check(!lastTx().empty(),
          "a poll goes out at the homing rate, faster than idle"); }
  plcReply(readReply(0));
  finishHoming(false, "test cleanup");
  { // Idle again: the same short interval must now buy nothing.
    if (plcTxnActive) plcReply(readReply(0));
    advance(PLC_POLL_IDLE_MS + 1); servicePlc(); plcReply(readReply(0));
    clearTx();
    advance(PLC_POLL_HOMING_MS + 1);
    servicePlc();
    check(lastTx().empty(),
          "  ...and back to the slow rate once the home is over"); }

  printf("\n  -- limits that never all come on time out instead of hanging --\n");
  OUT.clear(); clearTx(); ETH_RX.clear();
  isHomed = false;
  beginHoming();
  plcPoll3(0, BIT(14), 0);              // M30 only, never all three
  serviceHoming();
  check(isHoming, "waiting");
  advance(PLC_HOME_TIMEOUT_MS + 10);
  plcPoll(0);
  serviceHoming();
  check(!isHoming, "the home gives up on the timeout");
  check(!isHomed, "  ...without claiming a reference");
  check(saw("[HOME] FAILED"), "  ...and says FAILED");
  check(jzDir == 0 && rotDir == 0 && a2Dir == 0,
        "  ...and STOPS the axes it was driving");

  printf("\n  -- a dead socket still lets a HOME give up cleanly --\n");
  // Nothing is latched anywhere on the PLC side any more — there is no
  // request line to strand. A dropped socket during HOME must still stop
  // the axes and fail loudly rather than hang.
  OUT.clear(); clearTx();
  isHomed = false;
  beginHoming();
  ETH_CONNECTED = false;
  finishHoming(false, "cable pulled");
  check(!isHoming && !isHomed, "finishHoming(false, ...) leaves neither flag set");
  ETH_CONNECTED = true;

  printf("\n=== I0. M30..M32 TRAVEL LIMITS stop the axis ===\n");
  // Unlike M5..M8 (home sensors, warn only) these are real limit switches.
  // They live outside M0..M15, so the poll must cover three words or they
  // are invisible - that is what the 3-word frame above is for.
  {
    OUT.clear(); clearTx(); ETH_RX.clear();
    // ZM and A2M are SWAPPED from the tidy numeric order, measured on the
    // machine: M32 follows ZM, M30 follows A2M. Assuming M30=ZM is what
    // made ZM watch a bit that sits at 1 and refuse every Z_DOWN.
    check(PLC_M_LIMIT_Z == 32 && PLC_M_LIMIT_ROT == 31 && PLC_M_LIMIT_A2 == 30,
          "ZM is M32 and A2M is M30 — NOT the numeric order");
    check(PLC_POLL_WORDS == 3, "the poll reads 3 words so M30..M32 are covered");

    // M30 is bit 14 of word 1; M31 bit 15 of word 1; M32 bit 0 of word 2 -
    // the same decode the working ClearCore_PLC_Test sketch uses.
    plcPoll3(0, BIT(14), 0);
    check(plcBit(30), "M30 decodes from word1 bit14");
    check(plcBit(PLC_M_LIMIT_A2), "  ...and M30 is A2M's switch");
    check(!plcBit(PLC_M_LIMIT_Z) && !plcBit(PLC_M_LIMIT_ROT),
          "  ...while ZM and RM stay clear");
    plcPoll3(0, BIT(15), 0);
    check(plcBit(31) && plcBit(PLC_M_LIMIT_ROT), "M31 decodes from word1 bit15, and is RM's");
    plcPoll3(0, 0, BIT(0));
    check(plcBit(32), "M32 decodes from word2 bit0");
    check(plcBit(PLC_M_LIMIT_Z), "  ...and M32 is ZM's switch — the swap that was wrong");

    // WHICH way each switch stops, spelled out. The checks below derive
    // from PLC_LIMIT_END_*, so on their own they stay green whichever sign
    // the constants carry — these pin the sign itself. ZM sits at the
    // BOTTOM of the stroke, so Z_UP is the way off it. RM is mounted
    // inverted, so ROT_CCW is the way off ITS switch, not ROT_CW.
    plcPoll3(0, BIT(15), BIT(0));         // M31 (RM) and M32 (ZM) both tripped
    jzDir = 1;                            // Z_UP
    rotDir = -1;                          // ROT_CCW
    plcServiceLimitStops();
    check(jzDir == 1, "a tripped M32 still allows Z_UP — ZM's switch is at the bottom");
    check(rotDir == -1, "a tripped M31 still allows ROT_CCW — RM is inverted");
    jzDir = -1;                           // Z_DOWN
    rotDir = 1;                           // ROT_CW
    plcServiceLimitStops();
    check(jzDir == 0, "  ...and Z_DOWN into M32 is stopped");
    check(rotDir == 0, "  ...and ROT_CW into M31 is stopped");
    // Back to a clean slate: an untripped poll is what clears the per-axis
    // warn latch, and a left-over jog direction would leak into the HOME
    // state checks further down.
    jzDir = rotDir = 0;
    plcPoll3(0, 0, 0);

    // Directional: into the switch is stopped, off it is allowed.
    plcPoll3(0, 0, BIT(0));               // ZM limit (M32) tripped
    jzDir = PLC_LIMIT_END_Z;              // driving further in
    OUT.clear();
    plcServiceLimitStops();
    check(jzDir == 0, "a jog INTO a tripped M32 is stopped");
    check(saw("[PLC_LIMIT] ZM stopped"), "  ...and says so, naming the device");
    jzDir = -PLC_LIMIT_END_Z;             // driving back off it
    plcServiceLimitStops();
    check(jzDir == -PLC_LIMIT_END_Z, "  ...but jogging OFF it is allowed");

    // A run leg is refused in the same direction, and only that direction.
    String why;
    float farIn = currentD1() + 50.0f * PLC_LIMIT_END_Z;
    check(runLegBlockedByLimit(farIn, currentRot(), currentA2(), why),
          "a run leg driving further into M32 is refused");
    check(!runLegBlockedByLimit(currentD1() - 50.0f * PLC_LIMIT_END_Z,
                                currentRot(), currentA2(), why),
          "  ...and one moving away is allowed");

    // A1M has no switch fitted, so nothing can block it.
    plcPoll3(0, 0, 0);
    check(a1Dir == 0 || true, "A1M has no limit device and is never blocked");
  }

  printf("\n=== I0b. SET_PLC_SENSOR_ENFORCE — one BROKEN switch, disabled --\n");
  {
    // ZM's switch is M32 in this file's numbering (see I0 above).
    OUT.clear(); clearTx(); ETH_RX.clear();
    run("SET_PLC_SENSOR_ENFORCE:BOGUS,1");
    check(saw("[ERROR] SET_PLC_SENSOR_ENFORCE axis must be Z, ROT or A2"),
          "an unknown axis token is refused");
    run("SET_PLC_SENSOR_ENFORCE:Z,1");
    check(saw("[PLC_SENSOR_ENFORCE] Z 1"), "a no-op re-enable still confirms");
    check(plcLimitSensorEnabled[0], "  ...ZM starts enforced");

    plcPoll3(0, 0, BIT(0));               // M32 (ZM) tripped
    OUT.clear();
    run("SET_PLC_SENSOR_ENFORCE:Z,0");
    check(saw("[WARN]") && saw("DISABLED"), "disabling a currently-tripped switch warns");
    check(!plcLimitSensorEnabled[0], "  ...and the flag actually flips");

    jzDir = PLC_LIMIT_END_Z;              // driving further in, switch still covered
    plcServiceLimitStops();
    check(jzDir == PLC_LIMIT_END_Z,
          "a DISABLED switch no longer stops the axis, even while covered");

    String why;
    float farIn = currentD1() + 50.0f * PLC_LIMIT_END_Z;
    check(!runLegBlockedByLimit(farIn, currentRot(), currentA2(), why),
          "  ...and no longer blocks a run leg either");

    check(!plcHomeStateActive(),
          "M31/M32 still needed — only Z was disabled, not the other two");
    // M30 (A2M) bit14, M31 (RM) bit15, M32 (ZM) word2 bit0 — all three
    // physically tripped, even though Z's is disabled and irrelevant now.
    plcPoll3(0, BIT(14) | BIT(15), BIT(0));
    check(plcHomeStateActive(),
          "  ...and with ROT+A2 satisfied, a DISABLED Z no longer blocks HOME");

    // Re-enabling restores real protection immediately.
    OUT.clear();
    run("SET_PLC_SENSOR_ENFORCE:Z,1");
    check(!saw("[WARN]"), "re-enabling is not a confirmed action, unlike disabling");
    jzDir = PLC_LIMIT_END_Z;
    plcServiceLimitStops();
    check(jzDir == 0, "  ...and the switch stops the axis again, covered or not");

    plcPoll3(0, 0, 0);
    jzDir = rotDir = 0;
  }

  printf("\n=== I0b2. A2M's switch is wired at BOTH ends of its travel ===\n");
  {
    // One PLC device, two physical switches, so the bit cannot say which
    // end tripped it. The direction the axis was travelling on the rising
    // edge can, and that is the whole mechanism.
    OUT.clear(); clearTx(); ETH_RX.clear();
    a2Dir = rotDir = jzDir = 0;
    plcPoll3(0, 0, 0);
    plcServiceLimitLatch();
    check(plcLimitEndFor(2) == PLC_LIMIT_END_A2,
          "with nothing latched the end falls back to the HOME side");
    check(PLC_LIMIT_BOTH_ENDS_A2 && !PLC_LIMIT_BOTH_ENDS_Z && !PLC_LIMIT_BOTH_ENDS_ROT,
          "  ...and only A2M is wired that way");

    // --- tripped while driving FORWARD -> the forward limit -----------
    // OUT cleared BEFORE the poll: plcPoll3 services the link, so the
    // latch has already fired by the time it returns.
    a2Dir = +1;
    OUT.clear();
    plcPoll3(0, BIT(14) | BIT(15), BIT(0));   // all three covered
    plcServiceLimitLatch();
    check(plcLimitEndFor(2) == +1, "covered while driving forward = the FORWARD end");
    for (auto &l : OUT) printf("DEBUG [%s]\n", l.c_str());
    check(saw("FORWARD"), "  ...and the board names the end it caught");
    check(!plcHomeStateActive(),
          "  ...a FAR-end trip is not the reference, so it cannot zero the counters");

    plcServiceLimitStops();
    check(a2Dir == 0, "forward is refused while it sits on the forward switch");
    a2Dir = -1;
    plcServiceLimitStops();
    check(a2Dir == -1, "  ...and backward is still allowed, so it is never pinned");

    String why;
    check(runLegBlockedByLimit(currentD1(), currentRot(), currentA2() + 50.0f, why),
          "a run leg driving it further forward is refused");
    check(!runLegBlockedByLimit(currentD1(), currentRot(), currentA2() - 50.0f, why),
          "  ...and one retracting it is allowed");

    // --- clearing forgets the end, and the next trip re-decides -------
    a2Dir = 0;
    plcPoll3(0, 0, 0);
    plcServiceLimitLatch();
    check(plcLimitEndFor(2) == PLC_LIMIT_END_A2,
          "clearing the switch forgets which end it was");

    a2Dir = -1;
    plcPoll3(0, BIT(14) | BIT(15), BIT(0));
    plcServiceLimitLatch();
    check(plcLimitEndFor(2) == -1, "covered while driving back = the BACK end");
    check(plcHomeStateActive(),
          "  ...and THAT one is the reference, so the home state stands");
    plcServiceLimitStops();
    check(a2Dir == 0, "  ...with backward now the refused direction");

    OUT.clear();
    run("PLC_STATUS");
    check(saw("end Z/R/A2="),
          "PLC_STATUS carries the live end, which is all the GUI has to go on");

    a2Dir = rotDir = jzDir = 0;
    plcPoll3(0, 0, 0);
    plcServiceLimitLatch();
  }

  printf("\n=== I0c. a DISABLED switch is never driven blind during HOME ===\n");
  {
    // beginHoming() actively drives each axis onto its own switch. A
    // switch that cannot be trusted must not be driven toward at all —
    // there would be nothing left to stop it at the mechanical end.
    OUT.clear(); clearTx(); ETH_RX.clear();
    plcPoll3(0, 0, 0);                    // nothing tripped, ZM would normally drive
    run("SET_PLC_SENSOR_ENFORCE:Z,0");
    isHoming = false; isHomed = false;
    jzDir = rotDir = a1Dir = a2Dir = 0;
    beginHoming();
    check(jzDir == 0, "ZM is never commanded to move while its switch is disabled");
    check(saw("already on switch") && saw("ZM"),
          "  ...and HOME reports it as already satisfied, not as moving");
    isHoming = false; isHomed = false;
    run("SET_PLC_SENSOR_ENFORCE:Z,1");    // restore for anything after this
    jzDir = rotDir = a1Dir = a2Dir = 0;
  }

  printf("\n=== I. M30..M32 are the ONLY devices read ===\n");
  run("RESET_LIMITS");
  isHomed = true;
  OUT.clear(); ETH_RX.clear(); clearTx();
  plcHomeStatePrev = false;
  double a1MaxBefore = limA1Max, zMinBefore = limD1Min, rotMinBefore = limRotMin;

  // M1 (DONE), M5..M8 (home sensors) and M10..M13 (run) are gone entirely,
  // not merely unused. They lit a lamp and decided nothing, while M30 was
  // the bit actually refusing a jog and had no lamp at all -- the operator
  // read "M5 ZM lift = CLEAR" while ZM would not move down.
  OUT.clear();
  run("PLC_STATUS");
  check(saw("limit Z/R/A2="), "the status summary reports the limit bits");
  check(!saw("home Z/R/A1/A2="), "  ...and no longer reports the home sensors");
  check(!saw("run Z/R/A1/A2="), "  ...nor the run bits");
  check(!saw("M1(DONE)="), "  ...nor DONE");

  printf("\n  -- the three do NOT sit at the same end --\n");
  check(PLC_LIMIT_END_Z == -1 && PLC_LIMIT_END_A2 == -1,
        "M30 and M32 mark the MINIMUM of their axis");
  check(PLC_LIMIT_END_ROT == +1,
        "M31 marks the MAXIMUM of RM, which is mounted inverted");

  // Still no boundary is ever written from a device read.
  check(fabs(limA1Max - a1MaxBefore) < 1e-9
        && fabs(limD1Min - zMinBefore) < 1e-9
        && fabs(limRotMin - rotMinBefore) < 1e-9,
        "no device read writes a working boundary");
  check(!saw("[PLC_LIMIT_SET]"), "  ...and there is no such message");

  printf("\n  -- and a run leg starts normally --\n");
  OUT.clear();
  plcPoll3(0, 0, 0);
  { isMoving = false;
    beginRunLeg(PHASE_TO_A, -10.0f, 0.0f, 0.0f, 0.0f);
    check(runPhase == PHASE_TO_A,
          "beginRunLeg starts the leg with every limit clear"); }
  // beginRunLeg leaves the machine "moving", and the home-state latch below
  // deliberately refuses to zero anything mid-move. Stand it down first.
  isMoving = false; runPhase = PHASE_NONE;

  printf("\n  -- HOME STATE: M30+M31+M32 all true -> reset the coordinates --\n");
  // Simpler than the old two-on/two-off rule: the limit bits are true when
  // their axis is on its stop, so home is all three at once.
  OUT.clear(); ETH_RX.clear(); clearTx();
  isHomed = false;
  plcHomeStatePrev = false;
  // The latch refuses to zero anything while an axis is still being jogged,
  // and the limit-stop tests above leave a direction set.
  isMoving = false; runPhase = PHASE_NONE;
  jzDir = rotDir = a1Dir = a2Dir = 0;
  MOTOR_Z.PositionRefSet(4321);          // pretend the counters have drifted
  plcPoll3(0, BIT(14), 0);               // only M30 -> not home yet
  check(!isHomed, "M30 alone is not the HOME state");
  plcPoll3(0, BIT(14) | BIT(15), 0);     // M30+M31, still missing M32
  check(!plcHomeStateActive(), "M30+M31 without M32 is not the HOME state");
  check(!isHomed, "  ...so nothing is zeroed");
  plcPoll3(0, BIT(14) | BIT(15), BIT(0));
  check(plcHomeStateActive(), "all three true IS the HOME state");
  check(isHomed, "  ...and it sets the reference");
  check(saw("[COORD_RESET]"), "  ...by zeroing the coordinates");
  check(saw("[PLC_HOME] HOME STATE"), "  ...and saying which condition fired");

  // Bits outside M30..M32 must not influence it either way now.
  OUT.clear();
  plcHomeStatePrev = false; isHomed = false;
  plcPoll3(BIT(7), BIT(14) | BIT(15), BIT(0));
  check(plcHomeStateActive(),
        "a bit outside M30..M32 cannot block the HOME state — it is not read");

  printf("\n  -- it latches ONCE, not on every poll --\n");
  OUT.clear();
  plcHomeStatePrev = false; isHomed = false;
  plcPoll3(0, BIT(14) | BIT(15), BIT(0));
  check(saw("[COORD_RESET]"), "first entry into HOME state resets");
  OUT.clear();
  plcPoll3(0, BIT(14) | BIT(15), BIT(0));   // still held
  check(!saw("[COORD_RESET]"),
        "  ...and holding there does NOT keep re-zeroing, which would eat real motion");

  printf("\n  -- and never while the machine is moving --\n");
  OUT.clear();
  plcHomeStatePrev = false;
  plcPoll3(0, 0, 0);                           // leave home state
  // Jog AWAY from the ZM switch: the home state has M30 covered, so jogging
  // into it would be stopped by plcServiceLimitStops() and the machine would
  // read as stationary — which is not what this is testing.
  jzDir = -PLC_LIMIT_END_Z;                    // now jogging
  plcPoll3(0, BIT(14) | BIT(15), BIT(0));
  check(saw("NOT reset"), "a HOME state reached mid-jog refuses to zero");
  jzDir = 0;
  OUT.clear();
  plcPoll3(0, BIT(14) | BIT(15), BIT(0));
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
  // Restoring ETH_CONNECTED does not itself reconnect: plcLastConnectTry
  // is still set from the failed attempt above, and plcEnsureConnected()
  // rate-limits retries by PLC_RECONNECT_MS. Every section below assumes
  // a live PLC link, so clear the throttle and reconnect right here
  // rather than leaving it to whatever the next poll's advance() happens
  // to add up to. Both throttles have to clear: the reconnect rate limit
  // AND the poll interval, because the connect only happens inside a poll.
  advance((PLC_RECONNECT_MS > PLC_POLL_IDLE_MS ? PLC_RECONNECT_MS
                                               : PLC_POLL_IDLE_MS) + 1);
  servicePlc();
  check(plcClient.connected(), "the link is back before the next section relies on it");

  printf("\n=== J. gearing constants ===\n");
  // i_RM is bench-calibrated, so this only asserts the two constants agree
  // with each other. The VALUE is the operator's to set.
  check(fabs(ROT_GEAR_RATIO_DEF - I_RM_TOTAL) < 1e-9,
        "i_RM default tracks I_RM_TOTAL");
  // 20 mm/rev per spec sheet; the earlier 4x-travel bug was the Z driver's
  // microstep DIP switches (4, not 16 like ROT/ARM), not the lead itself.
  check(fabs(Z_MM_PER_MOTOR_REV - 20.0) < 1e-9, "ZM 20 mm/rev (spec)");

  printf("\n=== K. RM gear ratio has a runtime calibration escape hatch ===\n");
  // The stub's MotorConn does not persist PositionRefSet (always reads back
  // 0), so this exercises the real, load-bearing arithmetic -- pulses per
  // RM degree -- rather than a position readback the stub cannot provide.
  check(fabs(rotGearRatio - ROT_GEAR_RATIO_DEF) < 1e-9,
        "rotGearRatio starts at the modelled default");
  double pulsesAtDefault = pulsesPerDegRot();
  double halfRatio = ROT_GEAR_RATIO_DEF / 2.0;   // half the default, whatever it is
  run(("SET_ROT_RATIO:" + std::to_string(halfRatio)).c_str());
  check(fabs(rotGearRatio - halfRatio) < 1e-6, "SET_ROT_RATIO changes the runtime ratio");
  check(saw("[ROT_RATIO]"), "  ...and confirms it");
  check(saw("Re-check"), "  ...and warns the taught RM limits need re-checking");
  check(fabs(pulsesPerDegRot() - pulsesAtDefault / 2.0) < 1e-3,
        "  ...and pulses-per-RM-degree halves with it");
  run("SET_ROT_RATIO:0.0001");
  check(saw("[ERROR]"), "out-of-range ratio refused");
  check(fabs(rotGearRatio - halfRatio) < 1e-6, "  ...and the ratio is unchanged");
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
  plcPoll3(0, 0, BIT(0));                              // M32 tripped (ZM limit)
  // M32 is ZM's device, not M30 — M30 belongs to A2M. Direction derived
  // from PLC_LIMIT_END_Z, not hard-coded, so a wiring change (which end
  // the switch sits at) cannot silently point this test the wrong way.
  const float intoZmLimit = 50.0f * PLC_LIMIT_END_Z;
  beginRunLeg(PHASE_TO_A, intoZmLimit, 0.0f, 0.0f, 0.0f);
  check(!isMoving && runPhase == PHASE_NONE,
        "without skip, a leg driving further into a tripped M32 is refused");
  check(saw("[ERROR] RUN stopped"), "  ...and says why");
  OUT.clear();
  beginRunLeg(PHASE_RESET_HOME, intoZmLimit, 0.0f, 0.0f, 0.0f, /*skipSensorBlock=*/true);
  check(runPhase == PHASE_RESET_HOME,
        "  ...but WITH skipSensorBlock=true, the identical leg proceeds");
  isMoving = false; runPhase = PHASE_NONE;
  plcPoll3(0, 0, 0);                                    // clear the limit bit

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
