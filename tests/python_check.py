"""Behaviour checks for the GUI logic. Run with tests/run_tests.sh."""
import json
import os
import re
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "tkstub"))
sys.path.insert(0, os.path.dirname(HERE))

# A Windows console defaults to cp1252, which cannot encode the arrows and
# degree signs in the messages the code under test produces. Printing one
# raised UnicodeEncodeError and killed the run mid-suite, which reads as a
# test failure but is only the terminal. Replace what will not encode.
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

FAIL = []
def check(cond, msg):
    print(("  OK   " if cond else "  FAIL ") + msg)
    if not cond:
        FAIL.append(msg)

import robot_sim.config as C

TMP = os.path.join(HERE, "_tmp")
os.makedirs(TMP, exist_ok=True)
C.SETTINGS_FILE = os.path.join(TMP, "settings.json")
C.LIMIT_PRESETS_FILE = os.path.join(TMP, "presets.json")

import tkinter as tk
from tkinter import messagebox
import robot_sim.ui.settings_dialog as SD
SD.SETTINGS_FILE = C.SETTINGS_FILE
SD.LIMIT_PRESETS_FILE = C.LIMIT_PRESETS_FILE
import robot_sim.keybinds as KB
KB.KEYBINDS_FILE = os.path.join(TMP, "keybinds.json")
KB._active = None
from robot_sim.kinematics import (fold_angle_to_reach, reach_to_fold_angle,
                                  is_near_singularity, solve_ik)
import robot_sim.kinematics as K


# ══════════════════════════════════════════════════════════════════════
print("\n=== 1. THE ARM ANGLE IS ROTATION FROM HOME ===")
check(C.ARM_HOME_DEG == 0.0, "home is 0°, not 60°")
check(C.FOLD_ANGLE_MIN_DEG == 0.0 and C.FOLD_ANGLE_MAX_DEG == 180.0,
      "travel is 0..180° frog-leg (base 0..90°)")
check(C.ARM_ZERO_CAD_DEG == 0.0,
      "HOME IS frog-leg 0 — measured, not the .m's th3_cad 60")
check(abs(C.FOLD_ANGLE_SPEC_MAX_DEG - 146.68) < 0.01,
      "the rated 575 mm reach is fold 146.68°, inside the 180° travel")
check(C.FOLD_ANGLE_SINGULARITY_WARN_DEG == 170.0,
      "singularity warning 10° short of straight, as before")

print("\n  -- the reach curve is the MEASURED one, not mophong_init.m's --")
# Two bench measurements define the whole curve: HOME 240 mm and the arm
# straight at 605 mm. mophong_init.m's 133.2..613.2 was found to be wrong
# on the machine and is no longer the reference.
check(abs(fold_angle_to_reach(0.0) - 240.0) < 0.05, "0°   -> 240 mm (retracted, measured)")
check(abs(fold_angle_to_reach(180.0) - 605.0) < 0.05, "180° -> 605 mm (straight, measured)")
check(abs(fold_angle_to_reach(C.FOLD_ANGLE_SPEC_MAX_DEG) - 575.0) < 0.5,
      "146.68° -> 575 mm (the rated working reach)")
check(abs(reach_to_fold_angle(240.0) - 0.0) < 0.05, "240 mm -> 0°")
check(abs(reach_to_fold_angle(605.0) - 180.0) < 0.05, "605 mm -> 180°")
check(all(abs(reach_to_fold_angle(fold_angle_to_reach(a)) - a) < 1e-9
          for a in (0.0, 12.5, 60.0, 146.68, 179.9)), "the pair round-trips")
check(abs(C.ARM_MIN_REACH_MM - 240.0) < 0.05 and abs(C.ARM_MAX_REACH_MM - 605.0) < 0.05,
      "the reach envelope is the measured 240..605 mm")
check(is_near_singularity(175.0) and not is_near_singularity(165.0),
      "the singularity warning fires just short of straight")

print("\n  -- the base angle is a linear map onto the RATED travel --")
# NOT fold/2. That identity came from the derived 2:1 knee gearing; the
# bench-measured arm reaches its rated 575 mm at fold 146.68°, and THAT is
# the angle that maps to base 90°. Anything past it extrapolates.
check(abs(K.base_angle_from_fold_angle(0.0) - 0.0) < 1e-12, "fold 0   -> base 0° (HOME)")
check(abs(K.base_angle_from_fold_angle(C.FOLD_ANGLE_SPEC_MAX_DEG) - 90.0) < 1e-12,
      "fold 146.68 -> base 90° (rated working reach)")
check(all(abs(K.base_angle_from_fold_angle(f)
              - f * (90.0 / C.FOLD_ANGLE_SPEC_MAX_DEG)) < 1e-12
          for f in (0.0, 37.0, 90.0, 146.68, 180.0)),
      "base is linear in fold across the travel, exactly")
check(all(abs(K.fold_angle_from_base_angle(K.base_angle_from_fold_angle(f)) - f) < 1e-9
          for f in (0.0, 37.0, 90.0, 180.0)), "  ...and it round-trips")

print("\n  -- the angle counts UP as the arm turns out --")
prev = -1e9
for a in (0, 10, 30, 60, 90, 120):
    r = fold_angle_to_reach(a)
    if r <= prev:
        FAIL.append("monotonic")
    prev = r
check("monotonic" not in FAIL, "reach increases with the angle across the whole travel")
d, t2, a1, a2 = solve_ik(300.0, 0.0, 514.3 + 120.0, arm_choice="A1M")
check(0.0 <= a1 <= 120.0, "IK returns an elbow angle inside 0..120°")
check(abs(fold_angle_to_reach(a1) - 300.0) < 0.5,
      "  ...that really does put the wafer centre at 300 mm")

print("\n  -- the elbow boundary DEFAULTS are in MOTOR degrees --")
check([C.LIMIT_FIELDS[k][6] for k in C.ARM_FRAME_V2_RESET_KEYS]
      == [C.DEFAULT_LIM_A_MIN, C.DEFAULT_LIM_A_MAX] * 2,
      "default elbow band is 0..1394 MOTOR° — inset at the far end only")
check(all(C.LIMIT_FIELDS[k][3] == "motor °" for k in C.ARM_FRAME_V2_RESET_KEYS),
      "  ...and they are labelled as motor degrees, not bare degrees")


# ══════════════════════════════════════════════════════════════════════
print("\n=== 2. a settings file from the OLD frame is not loaded quietly ===")

class Loader(SD.SettingsDialogMixin):
    def __init__(self):
        self.settings = {k: C.LIMIT_FIELDS[k][6] for k in C.LIMIT_KEYS}
        for k, spec in C.SPEED_FIELDS.items():
            self.settings[k] = spec[2]
        self._load_settings_file()

def write(payload):
    with open(C.SETTINGS_FILE, "w", encoding="utf-8") as fh:
        json.dump(payload, fh)

# A v1 file: no schema key, elbow limits in the old 60..180 frame.
write({"lim_a1_min": 60.0, "lim_a1_max": 180.0,
       "lim_a2_min": 60.0, "lim_a2_max": 180.0,
       "lim_z_min": 5.0, "lim_z_max": 250.0, "arm_pct": 111.0})
ld = Loader()
notes = " ".join(n[0] for n in ld._pending_settings_notes)
check(ld.settings["lim_a1_min"] == C.DEFAULT_LIM_A_MIN
      and ld.settings["lim_a1_max"] == C.DEFAULT_LIM_A_MAX,
      "old-frame elbow limits are DROPPED, not read in the new unit")
check("teach them again" in notes, "  ...and the operator is told to re-teach")
check(ld.settings["lim_z_min"] == 5.0 and ld.settings["lim_z_max"] == 250.0,
      "the ZM boundaries survive — mm did not change meaning")
check(ld.settings["arm_pct"] == 111.0, "so does the speed percentage")

# A v2 file: taught elbow limits are honoured.
write({C.SETTINGS_SCHEMA_KEY: C.SETTINGS_SCHEMA,
       "lim_a1_min": 4.0, "lim_a1_max": 96.0})
ld = Loader()
check(ld.settings["lim_a1_min"] == 4.0 and ld.settings["lim_a1_max"] == 96.0,
      "a current file's taught limits are loaded")
check("teach them again" not in " ".join(n[0] for n in ld._pending_settings_notes),
      "  ...with no spurious warning")

# Saving stamps the schema, so this migration runs exactly once.
class Saver(Loader):
    def log(self, m, tag="default"): pass
sv = Saver(); sv._save_settings_file()
with open(C.SETTINGS_FILE, encoding="utf-8") as fh:
    check(json.load(fh).get(C.SETTINGS_SCHEMA_KEY) == C.SETTINGS_SCHEMA,
          "saving stamps the schema version")


# ══════════════════════════════════════════════════════════════════════
print("\n=== 3. boundaries: no elbow envelope, unordered pair ===")
check(all(C.LIMIT_FIELDS[k][4] is None and C.LIMIT_FIELDS[k][5] is None
          for k in C.ARM_FRAME_V2_RESET_KEYS), "elbow fields have no envelope")
check(C.LIMIT_FIELDS["lim_z_min"][4] is not None, "ZM keeps one — its scale IS known")
check(set(C.LIMIT_CAPTURE_ONLY) == set(C.ARM_FRAME_V2_RESET_KEYS),
      "the elbow rows are the capture-only ones")

class Coll:
    _read_number = staticmethod(SD.SettingsDialogMixin._read_number)
    _collect_limits = SD.SettingsDialogMixin._collect_limits
    def __init__(self, over=None):
        # RM's two keys show 0..-340, not native 0..340 — same transform
        # _build_limits_tab applies when it first populates the entries.
        self._limit_vars = {
            k: tk.StringVar(value=repr(SD._rot_limit_to_display(k, C.LIMIT_FIELDS[k][6])))
            for k in C.LIMIT_KEYS}
        for k, v in (over or {}).items():
            self._limit_vars[k].set(v)

out, err = Coll({"lim_a1_max": "1000"})._collect_limits()
check(err is None and out["lim_a1_max"] == 1000.0, "1000° accepted on an elbow")
out, err = Coll({"lim_a1_min": "-5000"})._collect_limits()
check(err is None, "-5000° accepted too")
out, err = Coll({"lim_z_max": "9000"})._collect_limits()
check(err and "MECHANICAL" in err, "ZM 9000 mm still refused")
out, err = Coll({"lim_a1_min": "800", "lim_a1_max": "100"})._collect_limits()
check(err is None and out["lim_a1_min"] == 800.0 and out["lim_a1_max"] == 100.0,
      "an inverted elbow pair is accepted and stored RAW")
out, err = Coll({"lim_a1_min": "70", "lim_a1_max": "70"})._collect_limits()
check(err and "no room to move" in err, "both ends on ONE position is refused")
out, err = Coll({"lim_z_min": "200", "lim_z_max": "100"})._collect_limits()
check(err and "must be at least" in err, "ZM keeps its ordering rule")

import robot_sim.core.jog_control as JC
class Pair:
    settings = {k: C.LIMIT_FIELDS[k][6] for k in C.LIMIT_KEYS}
    is_homed = True
    _limit_pair = JC.JogControlMixin._limit_pair
    _axis_bounds = JC.JogControlMixin._axis_bounds
pr = Pair()
check(pr._limit_pair("z") == (0.0, 280.0), "ZM pair reads the settings")
check(pr._limit_pair("a1") == (C.DEFAULT_LIM_A_MIN, C.DEFAULT_LIM_A_MAX),
      "A1 pair defaults to the factory motor° band")
pr.settings["lim_a1_min"], pr.settings["lim_a1_max"] = 800.0, 100.0
check(pr._limit_pair("a1") == (100.0, 800.0),
      "an inverted stored pair reads back SORTED (matches firmware armBand)")
class Legacy:
    settings = {}
    is_homed = True
    _limit_pair = JC.JogControlMixin._limit_pair
check(Legacy()._limit_pair("a1") == (C.ARM_SIM_MIN_DEG, C.ARM_SIM_MAX_DEG),
      "a settings file missing an axis falls back instead of raising mid-jog")


# ══════════════════════════════════════════════════════════════════════
print("\n=== 4. jog key bindings ===")
for gone in ("PRESETS", "PRESET_ORDER", "DEFAULT_PRESET", "risk_score"):
    check(not hasattr(KB, gone), f"{gone} is gone")
check(isinstance(KB.validate(KB.DEFAULT_KEYMAP), list), "validate() returns a plain list")
check(KB.DEFAULT_KEYMAP == {"ROT_CCW": "a", "ROT_CW": "d", "A1_FWD": "i",
                            "A1_BACK": "k", "A2_FWD": "o", "A2_BACK": "l",
                            "Z_UP": "w", "Z_DOWN": "s"},
      "default layout is A/D · I/K · O/L · W/S")
adj = dict(KB.DEFAULT_KEYMAP); adj["ROT_CCW"] = "u"; adj["A1_FWD"] = "y"
check(not KB.validate(adj), "keys side by side on different axes are allowed")
dup = dict(KB.DEFAULT_KEYMAP); dup["Z_UP"] = dup["A1_FWD"]
check(any("bound to both" in x for x in KB.validate(dup)), "duplicate key rejected")
miss = dict(KB.DEFAULT_KEYMAP); miss["Z_UP"] = ""
check(any("Not bound" in x for x in KB.validate(miss)), "unbound action rejected")
# Driven from RESERVED_KEYS itself rather than a hard-coded list. The HOME
# key moved from "h" to "backspace" and this assertion silently started
# testing an ordinary letter — it passed for the wrong reason until the
# reserved set no longer contained "h" at all.
check(bool(KB.RESERVED_KEYS), "there IS a reserved set to check")
for key, what in sorted(KB.RESERVED_KEYS.items()):
    res = dict(KB.DEFAULT_KEYMAP); res["Z_UP"] = key
    check(any("reserved" in x for x in KB.validate(res)),
          f"{what} key ({KB.display_key(key)}) cannot be taken")

custom = dict(KB.DEFAULT_KEYMAP); custom["Z_UP"] = "p"
check(KB.save(custom), "a valid layout saves")
KB._active = None
check(KB.load()["Z_UP"] == "p", "  ...and survives a restart")
check(not KB.save(dup), "an invalid layout is refused, not written")
KB._active = None
check(KB.load()["Z_UP"] == "p", "  ...leaving the good one in place")
with open(KB.KEYBINDS_FILE, "w") as fh:
    fh.write('{"Z_UP": "p"}')
KB._active = None
check(KB.load() == KB.DEFAULT_KEYMAP, "a partial file is discarded WHOLE")
os.remove(KB.KEYBINDS_FILE); KB._active = None


# ══════════════════════════════════════════════════════════════════════
print("\n=== 5. the Settings dialog builds, end to end ===")
REBOUND = [0]; CAPS = [0]; HINTS = [0]
class App(SD.SettingsDialogMixin):
    def __init__(self):
        self.root = tk.Tk()
        self.settings = {"kp": C.DEFAULT_KP, "ki": C.DEFAULT_KI,
                         "kd": C.DEFAULT_KD, "n": C.DEFAULT_N_FILTER}
        for lk in C.PID_LOCK_KEYS:
            self.settings[lk] = False
        # Seeded exactly as app.py seeds them, or a .get(key, True) default
        # would paper over a key the real app forgot to create.
        for ek in C.LIMIT_ENFORCE_KEYS:
            self.settings[ek] = C.DEFAULT_LIMIT_ENFORCED
        self.settings[C.LIMITS_ENABLED_KEY] = C.DEFAULT_LIMITS_ENABLED
        self.settings[C.PLC_LINK_ENABLED_KEY] = C.DEFAULT_PLC_LINK_ENABLED
        for ek in C.PLC_SENSOR_ENFORCE_KEYS:
            self.settings[ek] = C.DEFAULT_PLC_SENSOR_ENFORCED
        for k, spec in C.SPEED_FIELDS.items():
            self.settings[k] = spec[2]
        for k, spec in C.ACCEL_FIELDS.items():
            self.settings[k] = spec[2]
        for k, spec in C.LIMIT_FIELDS.items():
            self.settings[k] = spec[6]
        self._load_settings_file()
        self.is_homed = False
        self.sim_z = self.sim_rot = 0.0
        self.sim_a1 = self.sim_a2 = C.ARM_HOME_DEG
    def send(self, m, log_tx=True): pass
    def log(self, m, tag="default"): pass
    def _hardware_live(self): return False
    def home(self): pass
    def reset_coordinates(self): pass
    # Lives on P2PControlMixin in the real app. Stubbed rather than
    # guarded with getattr in the dialog: APPLY must repaint the panel's
    # reach line, because that line is now the only statement of the
    # working envelope, and a getattr guard would let that wiring rot
    # silently.
    def _refresh_workspace_hint(self): HINTS[0] += 1
    def _bind_keys(self): REBOUND[0] += 1
    def refresh_jog_keycaps(self): CAPS[0] += 1

os.path.exists(C.SETTINGS_FILE) and os.remove(C.SETTINGS_FILE)
app = App(); app.open_settings_dialog()
check(len(app._speed_vars) == len(C.SPEED_FIELDS), "the Speed tab built every field")
check(len(app._accel_vars) == len(C.ACCEL_FIELDS),
      "  ...and every accel field too, independent of speed")
# The accel defaults USED to be copies of the speed ones. They are set from
# the machine now, and the difference is the point: acceleration is what
# decides how far an axis carries on after the key is released, so an axis
# tuned for speed alone overshoots.
check((C.DEFAULT_ROT_PCT, C.DEFAULT_ARM_PCT, C.DEFAULT_Z_PCT) == (50.0, 62.5, 200.0),
      "the speed defaults are the combination that ran stably on the machine")
check((C.DEFAULT_ROT_ACC_PCT, C.DEFAULT_ARM_ACC_PCT, C.DEFAULT_Z_ACC_PCT)
      == (100.0, 70.0, 200.0),
      "  ...and the accel defaults are their own numbers, from the same bench run")
check(C.DEFAULT_ARM_ACC_PCT != C.DEFAULT_ARM_PCT
      and C.DEFAULT_ROT_ACC_PCT != C.DEFAULT_ROT_PCT,
      "  ...NOT copies of the speed percentages - that was the arm's 225 deg coast")

# Both sides have to agree, or the panel advertises a ramp the board will
# not use until the GUI happens to send SET_SPEED.
_ino_src = open(os.path.join(os.path.dirname(HERE),
                             "RobotMotionController_v9_ClearCore",
                             "RobotMotionController_v9_ClearCore.ino"),
                encoding="utf-8").read()
for _name, _want in (("ROT_PCT_DEF", C.DEFAULT_ROT_PCT),
                     ("ARM_PCT_DEF", C.DEFAULT_ARM_PCT),
                     ("Z_PCT_DEF", C.DEFAULT_Z_PCT),
                     ("ROT_ACC_PCT_DEF", C.DEFAULT_ROT_ACC_PCT),
                     ("ARM_ACC_PCT_DEF", C.DEFAULT_ARM_ACC_PCT),
                     ("Z_ACC_PCT_DEF", C.DEFAULT_Z_ACC_PCT)):
    _m = re.search(rf"const float {_name}\s*=\s*([\d.]+)f", _ino_src)
    check(_m is not None and abs(float(_m.group(1)) - _want) < 0.01,
          f"  ...and the firmware's {_name} is the same number the GUI defaults to")
check("float armAccPct     = ARM_ACC_PCT_DEF;" in _ino_src,
      "  ...with the board seeding accel from the ACCEL default, not the speed one")

# _send_speed() must APPEND the 3 accel percentages, never reorder/interleave
# the original 5 -- an old board parsing this positionally must keep reading
# the same 5 fields it always did.
_sent = []
app.send = lambda m, log_tx=True: _sent.append(m)
app.settings.update(rot_pct=75, arm_pct=125, z_pct=50,
                    rot_acc_pct=60, arm_acc_pct=140, z_acc_pct=40)
app._send_speed()
check(_sent[-1] == f"SET_SPEED:{C.MASTER_RPM:g},{C.MASTER_ACC_RPM_S:g},75,125,50,60,140,40",
      "SET_SPEED appends the 3 accel percentages after the original 5, in order")
check(len(app._limit_vars) == len(C.LIMIT_FIELDS), "the Boundaries tab built all 8")
check(len(app._pid_vars) == len(C.PID_FIELDS), "the PID tab built all 4 gains")
check(len(app._kb_draft) == len(KB.ACTION_ORDER), "the Controls tab built all 8 rows")

print("\n  -- SET HERE captures the live A1M_POS --")
app.sim_a1 = 37.5
app._capture_limit_here("lim_a1_max")
check(app._limit_vars["lim_a1_max"].get() == "37.50",
      "it takes exactly what the readout shows, in the from-home frame")
app.sim_a1 = 1000.0
app._capture_limit_here("lim_a1_min")
check(app._limit_vars["lim_a1_min"].get() == "1000.00", "a four-figure position too")
messagebox.CALLS.clear()
app._apply_limits()
check(not [c for c in messagebox.CALLS if c[0] == "error"],
      "  ...and APPLY accepts the pair, in either order")

print("\n  -- key capture --")
class Ev:
    def __init__(self, k): self.keysym = k
app._begin_capture("Z_UP"); app._on_capture_key(Ev("p"))
check(app._kb_draft["Z_UP"] == "p", "capturing a key rebinds that row")
app._begin_capture("Z_DOWN"); app._on_capture_key(Ev("space"))
check(app._kb_capturing == "Z_DOWN", "a reserved key is refused, capture stays open")
app._on_capture_key(Ev("Escape"))
app._begin_capture("Z_DOWN"); app._on_capture_key(Ev("l"))
check(app._kb_draft["A2_BACK"] == "s", "taking a used key SWAPS rather than unbinding")
messagebox.CALLS.clear(); REBOUND[0] = CAPS[0] = 0
app._apply_keybinds()
check(not [c for c in messagebox.CALLS if c[0] == "ask"], "APPLY never asks")
check(REBOUND[0] == 1 and CAPS[0] == 1, "  ...it rebinds and repaints, no restart")


# ══════════════════════════════════════════════════════════════════════
print("\n=== 6. no stale references to the old frame ===")
import glob, re
for path in glob.glob(os.path.join(os.path.dirname(HERE), "robot_sim", "**", "*.py"),
                      recursive=True):
    src = open(path, encoding="utf-8").read()
    name = os.path.basename(path)
    # th3_cad may only survive in the two conversion sites and the config note.
    if "th3_cad" in src and name not in ("kinematics.py", "config.py"):
        FAIL.append(f"th3_cad leaked into {name}")
check(not [f for f in FAIL if "leaked" in f],
      "th3_cad appears only in kinematics.py and config.py")

fw = open(os.path.join(os.path.dirname(HERE), "RobotMotionController_v9_ClearCore",
                       "RobotMotionController_v9_ClearCore.ino"), encoding="utf-8").read()
check("ARM_ZERO_CAD_DEG" in fw, "the firmware carries the same offset constant")
check("const double FOLD_ANGLE_HOME_DEG     = 0.0;" in fw, "  ...and homes at 0°")
check("ARM_LIMITS_UNBOUNDED" in fw, "  ...and documents the elbows as unbounded")
check("void armBand(" in fw, "  ...and sorts the taught pair at point of use")


# ══════════════════════════════════════════════════════════════════════
print("\n=== 7. P2P maths is self-consistent on the MEASURED geometry ===")
import math

# NO MATLAB PARITY SWEEP HERE ANY MORE, ON PURPOSE.
#
# mophong_init.m's solve_ik_frogleg was the reference this section measured
# against, pose for pose. It was dropped because the .m's own calculation is
# wrong for this machine: its a4/a5/a6 (160/160/248.2) are not the arm's,
# which measured 91.25/91.25/377.5 on the bench. Checking against a model
# that does not describe the machine proves nothing, and a red build nobody
# can fix teaches people to ignore the suite.
#
# MATLAB_v4_final stays read-only reference for the FRAME — the Z chain and
# the d1 stroke, which never depended on link length and are still asserted
# below. Do not reinstate the elbow comparison without first correcting the
# .m, which is not ours to edit.
#
# What replaces it is a round trip: every pose IK solves must come back out
# of FK in the same place. That catches the drift the parity sweep existed
# to catch, without an external reference to disagree with.
worst = 0.0
compared = 0
for r_mm in range(240, 606, 10):
    for a_deg in range(0, 341, 20):
        for dz in (0, 70, 140, 210, 280):
            x = r_mm * math.cos(math.radians(a_deg))
            y = r_mm * math.sin(math.radians(a_deg))
            z = 514.3 + dz
            try:
                d1, rot, a1, _a2 = solve_ik(x, y, z, "A1M", idle_deg=0.0)
            except ValueError:
                continue
            compared += 1
            fx, fy, fz = K.forward_kinematics(d1, rot, a1, 0.0, arm="A1M")
            worst = max(worst, math.hypot(fx - x, fy - y), abs(fz - z))

print("       %d poses round-tripped, worst error %.2e mm" % (compared, worst))
check(compared > 500, "the sweep really solved a few hundred poses")
check(worst < 1e-6, "IK -> FK returns the pose it was given, to machine precision")

# The frame constants, which the .m and the machine still agree on.
check(abs(C.Z_OFFSET_ARM1_MM - 514.3) < 1e-9, "Z_offset(arm 1) = 514.3")
check(C.D1_MIN_MM == 0.0 and C.D1_MAX_MM == 285.0, "d1 stroke 0..285")
# The measured links, named outright so a change to them is a visible diff.
check((C.A3_MM, C.A4_MM, C.A5_MM, C.A6_MM) == (45.0, 91.25, 91.25, 377.5),
      "a3/a4/a5/a6 are the MEASURED 45/91.25/91.25/377.5")
check(abs(C.ARM_MIN_REACH_MM - 240.0) < 1e-9 and abs(C.ARM_MAX_REACH_MM - 605.0) < 1e-9,
      "the measured links give the measured 240..605 mm envelope")

print("\n  -- the two documented departures from the .m are still documented --")
# 1. MATLAB clamps an unsolvable target silently; a machine must refuse it.
#    700 mm is past a3+a6+(a4+a5) = 613.2, so NO elbow angle reaches it and
#    acos() would clamp — the one refusal that is arithmetic, not opinion.
raised = False
try:
    solve_ik(700.0, 0.0, 560.0, "A1M", idle_deg=0.0)
except ValueError:
    raised = True
check(raised, "an unsolvable radius RAISES here where MATLAB would clamp")
# The only floor left is arithmetic: with the MEASURED links the frog-leg
# spans 422.5 ± 182.5 mm, so 240 mm is the shortest radius any elbow angle
# reaches. There is no SEPARATE structural floor on top of that — the old
# 133.2 mm one was R(fold = 0°) on an unmeasured gear ratio and is gone.
# The working limit is the operator's taught band, checked at LOAD.
raised = False
try:
    solve_ik(50.0, 0.0, 560.0, "A1M", idle_deg=0.0)
except ValueError:
    raised = True
check(raised, "r = 50 mm is refused — below the arithmetic span, not a taught limit")
# Just inside the span solves, and lands where FK agrees.
d1s, _rs, a1s, _as = solve_ik(C.ARM_MIN_REACH_MM + 1.0, 0.0, 560.0, "A1M",
                              idle_deg=0.0)
check(abs(K.fold_angle_to_reach(a1s) - (C.ARM_MIN_REACH_MM + 1.0)) < 1e-6,
      "  ...while 1 mm inside the span solves and round-trips")
# clamp_like_matlab still CLAMPS instead of raising — that switch is about
# the failure MODE, which is unchanged, not about the .m's link lengths.
d1c, _rc, a1c, _ac = solve_ik(50.0, 0.0, 560.0, "A1M", idle_deg=0.0,
                              clamp_like_matlab=True)
check(abs(K.fold_angle_to_reach(a1c) - C.ARM_MIN_REACH_MM) < 1e-6,
      "  ...and clamp_like_matlab=True clamps to the span instead of raising")
# 2. Per-arm deck heights: the .m uses arm 1's offset for both.
check(abs(C.Z_OFFSET_ARM1_MM - C.Z_OFFSET_ARM2_MM - 9.0) < 1e-9,
      "arm 2's deck is modelled 9 mm lower (the .m uses arm 1's offset for both)")


# ══════════════════════════════════════════════════════════════════════
print("\n=== 8. PLC link: Ethernet reads, HOME goes out on a wire ===")
check(C.PLC_IP == "192.168.3.101" and C.PLC_PORT == 1025,
      "the endpoint from the PLC configuration screen")
check(C.PLC_HOME_REQUEST_DEVICE == "X0", "HOME still arrives at X0, not M21")
check("IO-0" in C.PLC_HOME_REQUEST_SOURCE,
      "  ...but it is DRIVEN from ClearCore's IO-0 terminal, not written")
check(C.PLC_LINK_IS_READ_ONLY, "  ...and the Ethernet link is read-only")
check(dict((d, dirn) for d, _c, dirn in C.PLC_DEVICE_MAP)["X0"] == "wire",
      "  ...so X0's direction in the device map is 'wire', never 'write'")
check(not any(dirn == "write" for _d, _c, dirn in C.PLC_DEVICE_MAP),
      "  ...and NO device in the map is written at all")
# M5..M8 (home sensors), M1 (DONE) and M10..M13 (run) are gone from BOTH
# sides now, not just muted -- see PLC_SENSOR_PANEL below. They lit a lamp
# and decided nothing, while M30 was the bit actually refusing a jog.
check(not hasattr(C, "PLC_HOME_SENSOR_BITS"),
      "the old M5..M8 sensor-bit map is gone from config.py, not just unused")
check([d for d, _c, _dir in C.PLC_DEVICE_MAP if d in ("M10", "M11", "M12", "M13")] == [],
      "the run bits M10..M13 are not in the device map either")
# ZM and A2M are SWAPPED from the tidy numeric order, measured on the
# machine: M32 follows ZM, M30 follows A2M.
check(C.PLC_SENSOR_PANEL == (
        ("M32", "ZM  lift",      "Z",   "Z_DOWN",  -1),
        ("M31", "RM  turntable", "ROT", "ROT_CW",  +1),
        ("M30", "A2M arm 2",     "A2",  "A2_BACK", -1),
      ), "the sensor panel IS the three travel limits, nothing else")
check("const int PLC_M_LIMIT_Z   = 32;" in fw
      and "const int PLC_M_LIMIT_A2  = 30;" in fw,
      "  ...and the firmware agrees ZM is M32 and A2M is M30")

print("\n  -- the GUI and the firmware must agree on the endpoint --")
check(('#define PLC_IP_3 %s' % C.PLC_IP.split(".")[-1]) in fw,
      "the firmware's PLC address ends in the same octet")
check("const uint16_t PLC_PORT = %d;" % C.PLC_PORT in fw,
      "  ...and uses the same port")
check("#define PLC_LINK_MODE PLC_LINK_ETHERNET" in fw,
      "  ...and the Ethernet link is the compiled default, not the placeholder")

print("\n  -- the Ethernet link is READ-ONLY; HOME talks to no PLC device --")
# HOME used to assert a wire (ClearCore IO-0 -> PLC X0) and wait for the
# PLC's own home sequence to answer. Nothing on the PLC side ever ran it,
# so HOME just sat there and timed out. The board already owns the motors
# and already reads M30..M32, so it drives each axis onto its own switch
# itself now — no request line, no DONE, no PLC-side sequence to wait on.
check("plcAssertHomeRequest" not in fw, "there is no HOME-request wire any more")
check("plcClearHomeRequest" not in fw, "  ...nothing to clear, either")
check("plcHomeDoneAsserted" not in fw, "  ...and no DONE-style completion check")
check("PLC_HOME_REQ_PIN" not in fw, "  ...the dedicated request pin is gone")
check("PLC_HOME_REQ_CODE" not in fw and "PLC_HOME_REQ_NUM" not in fw,
      "  ...and no MC-protocol device was ever named for it")
check("String plcFrameWriteBit(" not in fw,
      "there is NO write-frame builder — not even an unused one")
check("PLC_MC_CMD_WRITE" not in fw and "PLC_MC_SUB_BIT" not in fw,
      "  ...and the write command / bit subcommand codes are gone with it")
check("PLC_MC_CMD_READ" in fw, "the read command is all that remains")
check("bool plcSendPoll() {" in fw,
      "the send path has no isRead flag — every transaction is a read")
check("PLC_CMD_HOME" not in fw,
      "the old ad-hoc \"M2\"/\"DONE\" text protocol is gone")
# HOME drives the axes with the SAME jog primitive as an operator holding
# a key, and stops each one independently the instant its own switch
# reads covered — the request in this session, in the firmware's own words.
home_body = fw.split("void beginHoming() {")[1].split("\nvoid ")[0]
check("homeDirFor(i)" in home_body,
      "beginHoming drives each axis in the direction HOME_DIR_* names")
# NOT PLC_LIMIT_END_*. Which way a covered switch refuses and which way HOME
# goes looking for it are separate facts; sharing one constant meant a wrong
# end sent HOME the wrong way with nowhere separate to correct it.
check("plcLimitEndFor" not in home_body,
      "  ...and NOT from the limit-blocking end, which is a different fact")
# RM is mounted inverted (see PLC_LIMIT_END_ROT), so its switch sits at
# the +1/CW end, not -1 — HOME_DIR_* must match PLC_LIMIT_END_* per axis,
# not carry one blanket sign.
check("const int HOME_DIR_Z   = -1;" in fw and "const int HOME_DIR_ROT = +1;" in fw
      and "const int HOME_DIR_A2  = -1;" in fw,
      "  ...ZM/A2M back off NEGATIVE, RM backs off POSITIVE — it is inverted")
# HOME is not a jog, so no keep-alive arrives: the jog watchdog cancelled
# the move 700 ms in, which is why HOME looked like it did nothing at all.
wd_body = fw.split("void serviceJogWatchdog() {")[1].split("\n}")[0]
check("if (isHoming) return;" in wd_body,
      "the jog watchdog does NOT cancel a HOME-driven move")
check("applyJogVelocities()" in home_body,
      "  ...through the ordinary jog velocity path, nothing PLC-specific")
service_body = fw.split("void serviceHoming() {")[1].split("\nvoid ")[0]
check("plcBit(plcLimitBitFor(i))" in service_body,
      "serviceHoming stops each axis by reading its own switch bit directly")
check("plcHomeStateActive()" in service_body,
      "  ...and completion is M30&&M31&&M32 (via the enforce flags), not a DONE bit")

print("\n  -- M30..M32 are the only PLC devices read, and set NO boundary --")
# M5..M8 (home sensors), M1 (DONE) and M10..M13 (run) are gone entirely now,
# not just muted: they lit a lamp and decided nothing, while M30 was the bit
# actually refusing a jog. Nothing the board reports over PLC ever writes a
# taught boundary either way — that stays the operator's alone.
src_pr_early = open(os.path.join(os.path.dirname(HERE), "robot_sim", "core",
                                 "protocol.py"), encoding="utf-8").read()
check("PLC_LIMIT_SET" not in fw, "the board has no PLC_LIMIT_SET message any more")
check("PLC_LIMIT_DIR" not in fw, "  ...no per-bit blocking direction")
check("PLC_LIMITS_REGISTER_POSITION" not in fw, "  ...and no registration switch")
check("plcServiceHomeSensors" not in fw,
      "the M5..M8 sensor-service function is gone, not just unused")
check("[PLC_HOME]" in fw, "  ...the M30..M32 HOME STATE message is still reported")
check("PLC_LIMIT_SET" not in src_pr_early,
      "the GUI no longer has an adoption path for a board-set boundary")
check("_on_plc_limit_set" not in src_pr_early, "  ...and the handler is gone")
check("[PLC_HOME]" in src_pr_early, "  ...it just logs the sensor transitions")
# The original rule is restored in full: the GUI is the system of record.
check('upper.startswith("[LIMITS")' in src_pr_early,
      "[LIMITS] is still only logged, never parsed back into the settings")
check("_save_settings_file" not in src_pr_early.split("def _on_home_complete")[0],
      "nothing the board says writes the settings file any more")

print("\n=== 9. motor degrees vs frog-leg degrees ===")
from robot_sim.kinematics import motor_deg_to_reach as _mdr
from robot_sim.kinematics import (fold_angle_from_motor_deg, motor_deg_from_fold_angle,
                                  motor_deg_to_reach, reach_to_motor_deg)

# BENCH-MEASURED, not the model's derived 2.0: the arm reaches its rated
# 575 mm where the old ratio put it at 498 mm.
check(C.ARM_GEAR_RATIO == 7.80,
      "the ratio is the measured 7.80, not the model's derived 2.0")
check(abs(fold_angle_from_motor_deg(180.0 * C.ARM_GEAR_RATIO) - 180.0) < 1e-9,
      "motor° / ratio is fold°, at the straight-arm end")
check(abs(motor_deg_from_fold_angle(180.0) - 180.0 * C.ARM_GEAR_RATIO) < 1e-9,
      "and back again")
check(all(abs(fold_angle_from_motor_deg(motor_deg_from_fold_angle(a)) - a) < 1e-12
          for a in (0.0, 17.5, 91.72, 120.0)), "the pair round-trips")

print("\n  -- reach is a function of the FROG-LEG angle, not the motor's --")
check(abs(motor_deg_to_reach(0.0) - C.ARM_MIN_REACH_MM) < 0.05,
      "0 motor° -> 240 mm (retracted)")
check(abs(motor_deg_to_reach(C.ARM_MOTOR_MAX_DEG) - C.ARM_MAX_REACH_MM) < 0.05,
      "the far end of motor travel -> 605 mm (straight)")
# This is the actual bug being fixed: treating the motor number as the arm
# angle put the reported reach at the wrong place entirely.
_m = 60.0 * C.ARM_GEAR_RATIO
check(abs(motor_deg_to_reach(_m) - fold_angle_to_reach(60.0)) < 1e-9,
      "%.0f motor° is fold 60°, R = %.1f mm" % (_m, fold_angle_to_reach(60.0)))
check(abs(motor_deg_to_reach(_m) - fold_angle_to_reach(60.0 * C.ARM_GEAR_RATIO)) > 100.0,
      "  ...and NOT fold %.0f°, far further out — the old bug" % _m)
check(abs(reach_to_motor_deg(575.0)
          - motor_deg_from_fold_angle(C.FOLD_ANGLE_SPEC_MAX_DEG)) < 0.01,
      "575 mm maps back to the rated 146.68 fold° (the JEL drawing figure)")

print("\n  -- the speed figures split the same way --")
check(abs(C.arm_motor_speed_deg_s(150, 125) - 1125.0) < 0.01,
      "AM at 125% of 150 RPM is 1125 MOTOR °/s (exact, no ratio)")
check(abs(C.arm_speed_deg_s(150, 125) - 1125.0 / C.ARM_GEAR_RATIO) < 0.01,
      "  ...which is %.1f frog-leg °/s at ratio %.2f"
      % (1125.0 / C.ARM_GEAR_RATIO, C.ARM_GEAR_RATIO))
check(abs(C.arm_motor_rpm(150, 125) - 187.5) < 0.01,
      "  ...and 187.5 motor RPM, unchanged and still ratio-free")
check(C.SPEED_PREVIEW_BY_KEY["arm_pct"][4] == "motor °/s",
      "the live readout says MOTOR °/s so it cannot be misread")

print("\n  -- taught limits survive a re-calibration --")
# The whole reason limits are stored in motor degrees: changing the ratio
# must not move a boundary the operator taught off the physical stop.
saved_ratio = C.ARM_GEAR_RATIO
# Captured at the 575 mm stop, i.e. the rated fold angle times the ratio.
taught_motor = C.FOLD_ANGLE_SPEC_MAX_DEG * C.ARM_GEAR_RATIO
before = motor_deg_to_reach(taught_motor)
import robot_sim.kinematics as KIN
KIN.ARM_GEAR_RATIO = saved_ratio / 2.0     # pretend the bench says half
after = KIN.motor_deg_to_reach(taught_motor)
KIN.ARM_GEAR_RATIO = saved_ratio
check(abs(before - 575.0) < 0.5,
      "a boundary taught at %.2f motor° reads 575 mm" % taught_motor)
check(abs(after - before) > 50.0,
      "  ...and after re-calibration the SAME stored number means a new reach")
check(taught_motor == C.FOLD_ANGLE_SPEC_MAX_DEG * C.ARM_GEAR_RATIO,
      "  ...but the stored number itself never had to be touched")

print("\n  -- the settings schema was bumped for the unit change --")
check(C.SETTINGS_SCHEMA == 4, "schema is 4 (v4 dropped the centred-frame RM limits)")

print("\n=== 10. HOME resets the coordinate system to standard ===")
src_pr = open(os.path.join(os.path.dirname(HERE), "robot_sim", "core",
                           "protocol.py"), encoding="utf-8").read()
body = src_pr.split("def _on_home_complete")[1].split("def _settings_key_for_axis_end")[0] \
    if "_settings_key_for_axis_end" in src_pr.split("def _on_home_complete")[1] \
    else src_pr.split("def _on_home_complete")[1]
check("if simulated:" not in body.split("self.is_homed = True")[0],
      "the reset is UNCONDITIONAL, not only for the simulated case")
check("current_joints" in body, "the P2P pose copy is reset too, not just the jog sim")
check("_invalidate_loaded_program" in body,
      "a program loaded against the OLD reference is invalidated")
check(C.ARM_HOME_DEG == 0.0, "standard home is 0 motor° on both elbows")
check(C.ROT_HOME_DEG == 0.0 and C.Z_HOME_MM == 0.0, "  ...and 0° RM, 0 mm d1")
check("[COORD_RESET] Coordinates reset to the standard home pose" in fw,
      "the firmware says so on the wire when it zeroes its counters")

print("\n=== 11. English throughout ===")
import glob as _glob
viet = []
for path in (_glob.glob(os.path.join(os.path.dirname(HERE), "robot_sim", "**", "*.py"),
                        recursive=True)
             + [os.path.join(os.path.dirname(HERE), "RobotMotionController_v9_ClearCore",
                             "RobotMotionController_v9_ClearCore.ino")]):
    src = open(path, encoding="utf-8").read()
    for token in ("DA DEN DIEM DICH", "DUNG KHAN CAP", "Báo cáo", "Bảng", " cho {"):
        if token in src:
            # protocol.py keeps the two old wire strings on purpose, so a
            # board still running v8 firmware is still understood.
            if os.path.basename(path) == "protocol.py" and token in (
                    "DA DEN DIEM DICH", "DUNG KHAN CAP"):
                continue
            viet.append("%s in %s" % (token, os.path.basename(path)))
check(not viet, "no Vietnamese left in the live firmware or the GUI (found: %s)" % viet)
check("[RUN] TARGET REACHED" in fw, "the board reports TARGET REACHED in English")
check("[ESTOP] EMERGENCY STOP" in fw, "  ...and EMERGENCY STOP")
check('"TARGET REACHED" in upper or "DA DEN DIEM DICH THANH CONG" in upper' in src_pr,
      "the GUI still accepts the old strings from a v8 board")



# ══════════════════════════════════════════════════════════════════════
print("\n=== 12. boundary defaults sit INSIDE the factory envelope ===")
# The defaults used to be the envelope itself, so on an untaught machine
# the soft limit and the mechanical stop were the same position and the
# soft limit protected nothing.
# The margin is applied at the FAR end only. HOME is the minimum of every
# axis, so insetting the lower end would put the home pose itself outside
# the working envelope and refuse every P2P program.
for key, spec in C.LIMIT_FIELDS.items():
    floor, ceil, default, end = spec[4], spec[5], spec[6], spec[2]
    if floor is None or ceil is None:
        continue
    if end == "MAX":
        check(floor < default < ceil,
              "%s default %.2f is inset from the ceiling %.2f" % (key, default, ceil))
    else:
        check(default == floor,
              "%s default %.2f sits AT the floor, so HOME stays reachable"
              % (key, default))
check(C.DEFAULT_LIM_Z_MIN == 0.0 and C.DEFAULT_LIM_Z_MAX == 280.0,
      "ZM defaults to 0..280 mm — 5 mm clear of the TOP, home at the bottom")
check(C.DEFAULT_LIM_ROT_MIN == 0.0 and C.DEFAULT_LIM_ROT_MAX == 335.0,
      "RM defaults to 0..335° — 5° clear of the CW stop, home at the CCW one")
check(C.DEFAULT_LIM_A_MIN == 0.0
      and abs(C.DEFAULT_LIM_A_MAX
              - (C.ARM_MOTOR_MAX_DEG - C.LIMIT_SAFETY_MARGIN["A1"])) < 1e-9,
      "the elbows default to the full motor travel, inset at the far end only")

print("\n  -- and the HOME pose is inside the default envelope --")
# Otherwise a run, which is HOME -> A -> B -> HOME, is refused before it
# starts. This is the check that caught the far-end-only margin.
for _axis, _lo, _hi, _home in (("ZM", C.DEFAULT_LIM_Z_MIN, C.DEFAULT_LIM_Z_MAX, C.Z_HOME_MM),
                               ("RM", C.DEFAULT_LIM_ROT_MIN, C.DEFAULT_LIM_ROT_MAX, C.ROT_HOME_DEG),
                               ("A1M", C.DEFAULT_LIM_A_MIN, C.DEFAULT_LIM_A_MAX, C.ARM_HOME_DEG),
                               ("A2M", C.DEFAULT_LIM_A_MIN, C.DEFAULT_LIM_A_MAX, C.ARM_HOME_DEG)):
    check(_lo <= _home <= _hi,
          "%s home %.2f is inside its default limits [%.2f, %.2f]"
          % (_axis, _home, _lo, _hi))
check(C.ARM_MOTOR_MIN_DEG == 0.0
      and abs(C.ARM_MOTOR_MAX_DEG - 180.0 * C.ARM_GEAR_RATIO) < 1e-9,
      "  ...while the factory envelope is the full fold travel times the ratio")

print("\n  -- RM's zero is its CCW stop, which is HOME --")
check(C.ROT_MIN_DEG == 0.0 and C.ROT_MAX_DEG == 340.0,
      "RM travels 0..340°, not -170..+170°")
check(C.ROT_HOME_DEG == 0.0, "  ...and HOME is 0, the CCW stop")
from robot_sim.kinematics import rot_from_bearing
check(rot_from_bearing(-5.0) == 355.0,
      "a bearing 5° clockwise of home reads 355, not -5")
check(rot_from_bearing(0.0) == 0.0 and rot_from_bearing(180.0) == 180.0,
      "  ...and a bearing already in range is unchanged")
# +X points along the arm at RM 0, so HOME is a true X0 Y0 reference.
_d, _r, _a1, _a2 = solve_ik(300.0, 0.0, 514.3 + 45.0, "A1M", idle_deg=0.0)
check(abs(_r - 0.0) < 1e-9, "a target straight ahead of HOME solves to RM 0°")
# The 20 deg wedge between 340 and 360 cannot be swept through.
_raised = False
try:
    solve_ik(300.0, -30.0, 514.3 + 45.0, "A1M", idle_deg=0.0)
except ValueError as _e:
    _raised = "340" in str(_e)
check(_raised, "a bearing inside the unreachable 340..360° wedge is refused")
check("ROT_MIN_DEG = 0.0, ROT_MAX_DEG = 340.0" in fw,
      "the firmware carries the same RM range")
check("th2 += 360.0" in fw, "  ...and wraps atan2 into it the same way")

print("\n=== 13. per-axis enforcement, and the master switch above it ===")
# The per-axis control used to be a value LOCK. It answered a question
# ("is this number frozen?") that nobody asks at a safety limit, while the
# question they do ask ("is this boundary switched on?") had no per-axis
# answer at all. The button now reports enforcement.
check(C.LIMIT_ENFORCE_KEYS == ("lim_z_enforced", "lim_rot_enforced",
                               "lim_a1_enforced", "lim_a2_enforced"),
      "one enforcement switch per axis pair, not one per box")
check(set(C.LIMIT_ENFORCE_BY_AXIS) == {"Z", "ROT", "A1", "A2"},
      "  ...keyed by the firmware's own axis tokens")
check(C.DEFAULT_LIMIT_ENFORCED is True,
      "  ...and defaults to ON — a fresh install is not less safe")
check(not hasattr(C, "LIMIT_LOCK_KEYS") and not hasattr(C, "LIMIT_LOCK_BY_AXIS"),
      "the old value-lock keys are GONE, not left beside the new ones")
check(C.LIMITS_ENABLED_KEY == "limits_enabled" and C.DEFAULT_LIMITS_ENABLED is True,
      "the master switch is still a separate key, and defaults to ON")
# The enforcement rename needed no bump — nothing stored changed meaning.
# The schema is at 4 for a DIFFERENT reason (the RM frame), and that bump
# drops only the RM keys, so the taught elbow limits still survive it.
check("lim_a1_min" not in C.ROT_FRAME_V4_RESET_KEYS
      and "lim_a2_max" not in C.ROT_FRAME_V4_RESET_KEYS,
      "  ...and the v4 bump does not touch the taught elbow limits")

src_sd = open(os.path.join(os.path.dirname(HERE), "robot_sim", "ui",
                           "settings_dialog.py"), encoding="utf-8").read()
check("SET_LIMIT_ENFORCE:" in src_sd, "the GUI pushes per-axis enforcement")
check("SET_LIMITS_ENABLED:" in src_sd, "  ...and the master switch too")
check("SET_LIMIT_LOCK" not in src_sd, "  ...and nothing still sends SET_LIMIT_LOCK")
# Values before switches: the board keeps limits in RAM, so this runs on
# every handshake, and arming an axis before its numbers arrive would leave
# a window enforcing whatever the board happened to be holding.
send_body = src_sd.split("def _send_limits")[1].split("def ")[0]
check(send_body.index("SET_LIMIT:{axis}")
      < send_body.index("SET_LIMIT_ENFORCE:{axis}")
      < send_body.index("SET_LIMITS_ENABLED:"),
      "  ...values FIRST, then the per-axis switches, then the master one")
check("SET_PLC_SENSOR_ENFORCE:" in send_body,
      "_send_limits also re-syncs each PLC sensor's boundary on handshake")
check("Locked boundaries were left untouched" not in src_sd,
      "APPLY no longer refuses a pair — a switched-off axis is exactly when "
      "you teach it")
check("NOT ENFORCED" in src_sd and "ENFORCED" in src_sd,
      "the per-axis button says ENFORCED / NOT ENFORCED, not LOCKED")
check("ON (MASTER OFF)" in src_sd,
      "  ...and a third caption for 'on, but the master switch is off', which "
      "would otherwise read ENFORCED on an unprotected machine")
toggle_body = src_sd.split("def _toggle_limit_enforce")[1].split("def ")[0]
check("askyesno" in toggle_body,
      "  ...switching one off is confirmed, like the master switch is")
enable_body = src_sd.split("def _toggle_limits_enabled")[1].split("def ")[0]
check("_refresh_limit_enforce()" in enable_body,
      "  ...and the master switch repaints the four per-axis captions with it")

print("\n  -- and it behaves, not just reads, that way --")
messagebox.CALLS.clear(); messagebox.ANSWER = True
_ek = C.LIMIT_ENFORCE_BY_AXIS["ROT"]
check(app.settings.get(_ek, True) is True, "RM starts enforced")
app._toggle_limit_enforce(_ek)
check(app.settings[_ek] is False, "  ...one click switches RM off")
check([c for c in messagebox.CALLS if c[0] == "ask"],
      "  ...after asking first — this makes the machine less safe")
check(app.settings[C.LIMIT_ENFORCE_BY_AXIS["Z"]] is True,
      "  ...and ZM is untouched, which is the whole point of per-axis")
messagebox.CALLS.clear()
app._toggle_limit_enforce(_ek)
check(app.settings[_ek] is True, "  ...clicking again switches it back on")
check(not [c for c in messagebox.CALLS if c[0] == "ask"],
      "  ...with no confirmation, because making it safer is not an interruption")
messagebox.ANSWER = False
app._toggle_limit_enforce(_ek)
check(app.settings[_ek] is True, "  ...and answering No leaves it enforced")
messagebox.ANSWER = True; messagebox.CALLS.clear()

print("\n  -- PLC sensor boundaries: ONE switch, separate from PLC LINK --")
# Separate from _toggle_limit_enforce (taught soft limits) and from
# _toggle_plc_link (whole socket, HOME included). This is for one
# physically BROKEN switch — the other two axes must be untouched.
plc_toggle_body = src_sd.split("def _toggle_plc_sensor_enforce")[1].split("def ")[0]
check("askyesno" in plc_toggle_body,
      "disabling a PLC sensor's boundary is confirmed, like the others")
_pek = C.PLC_SENSOR_ENFORCE_BY_AXIS["Z"]
check(app.settings.get(_pek, True) is True, "ZM's PLC switch starts enforced")
app._toggle_plc_sensor_enforce(_pek, "M32", "Z")
check(app.settings[_pek] is False, "  ...one click switches it off")
check([c for c in messagebox.CALLS if c[0] == "ask"],
      "  ...after asking first — this removes real protection")
check(app.settings[C.PLC_SENSOR_ENFORCE_BY_AXIS["ROT"]] is True,
      "  ...and RM's switch is untouched — the whole point of per-sensor")
messagebox.CALLS.clear()
app._toggle_plc_sensor_enforce(_pek, "M32", "Z")
check(app.settings[_pek] is True, "  ...clicking again switches it back on")
check(not [c for c in messagebox.CALLS if c[0] == "ask"],
      "  ...with no confirmation, same as re-enabling a soft limit")
messagebox.ANSWER = False
app._toggle_plc_sensor_enforce(_pek, "M32", "Z")
check(app.settings[_pek] is True, "  ...and answering No leaves it enforced")
messagebox.ANSWER = True; messagebox.CALLS.clear()

print("\n  -- the GUI's own jog simulation obeys the same switches --")
src_jc = open(os.path.join(os.path.dirname(HERE), "robot_sim", "core",
                           "jog_control.py"), encoding="utf-8").read()
check("def _axis_enforced(self, axis):" in src_jc,
      "the simulation asks per-axis, not just is_homed")
for call in ('axis="ROT"', 'axis="Z"', 'axis="A1" if is_a1 else "A2"'):
    check(call in src_jc, f"  ...and passes it: {call}")
# Nothing the board reports can reach a boundary: the adoption path was
# removed when M5..M8 became home sensors.
check("PLC_LIMIT_SET" not in open(
        os.path.join(os.path.dirname(HERE), "robot_sim", "core", "protocol.py"),
        encoding="utf-8").read(),
      "no board-reported boundary can switch an axis's enforcement either")

print("\n  -- the firmware ANDs the two, and never lets one re-arm the other --")
check("bool limitsEnabled = true;" in fw, "the master switch defaults to on")
check("bool limZEnforced = true, limRotEnforced = true;" in fw,
      "  ...and so does every per-axis switch")
# A taught boundary applies IMMEDIATELY. It is captured against the same
# counters it is compared with, so it is meaningful in the frame it was
# taught in, reference or not. Waiting for isHomed let a limit taught at
# -300 pass -427.
check("bool softLimitsActive() { return limitsEnabled; }" in fw,
      "softLimitsActive() no longer waits for a reference")
check("return isHomed && limitsEnabled;" not in fw,
      "  ...the old isHomed gate is gone from the code, not just bypassed")
check("return softLimitsActive() && axisEnforced(axis);" in fw,
      "axisLimited() ANDs the per-axis switch on top of it")
check('if (!axisLimited(whichArm == 1 ? "A1" : "A2")) return;' in fw,
      "  ...and the elbow clamp asks per arm, so switching A1 off leaves A2 armed")
check('if (axisLimited("Z")) {' in fw and 'if (axisLimited("ROT")) {' in fw,
      "  ...ZM and RM are gated separately too, not behind one shared return")
check('if (axisEnforced("Z") &&' in fw,
      "target validation skips an axis that is switched off, or it would refuse "
      "a point the machine is willing to drive to")
check("SET_LIMIT_ENFORCE:" in fw, "the board takes the per-axis command")
check("SET_LIMIT_LOCK no longer exists" in fw,
      "  ...and REFUSES the old SET_LIMIT_LOCK rather than aliasing it onto "
      "enforcement, which would mean an old GUI armed an axis believing it had "
      "frozen a value")
check("is LOCKED — unlock it" not in fw, "applyLimit no longer refuses a locked axis")
check("[LIMIT_ENFORCE]" in fw, "the board reports per-axis enforcement")

print("\n=== 14. one coordinate at a time ===")
src_sf = open(os.path.join(os.path.dirname(HERE), "robot_sim", "core",
                           "safety.py"), encoding="utf-8").read()
check("def reset_coordinates(self, axis=None):" in src_sf,
      "reset_coordinates takes an axis")
check('self.send("RESET_COORD" if axis is None else f"RESET_COORD:{axis}")' in src_sf,
      "  ...and sends the per-axis form")
check("RESET COORDINATES" in open(os.path.join(os.path.dirname(HERE), "robot_sim",
        "ui", "p2p_panel.py"), encoding="utf-8").read(),
      "the motion panel offers the reset buttons")
check('upper.startsWith("RESET_COORD:")' in fw, "the board parses RESET_COORD:<axis>")
check("a single-axis reset does not" in fw,
      "  ...and does NOT claim a full reference from one axis")
# The dangerous version of this feature would be a single-axis reset that
# switches the soft limits on against three meaningless counters.
single_block = fw.split('} else if (axis == "Z" || axis == "ROT"')[1].split("} else {")[0]
check("isHomed = true" not in single_block,
      "  ...proved by the single-axis branch never touching isHomed")

print("\n=== 15. HOME is 0 degrees, and nothing says 60 ===")
src_jp = open(os.path.join(os.path.dirname(HERE), "robot_sim", "ui",
                           "jog_panel.py"), encoding="utf-8").read()
check('value="60.00 deg"' not in src_jp,
      "the jog readout no longer STARTS at 60.00 — that was the 60° jump")
# Primary card moved from raw motor deg to the derived BASE angle (the
# operator's working 0..90 scale) — motor deg alone was the old bug this
# section title refers to (shown as if it were the arm angle).
check('value="0.00 base deg"' in src_jp, "  ...it starts at 0.00 base deg")
check("A1M_BASE" in src_jp and "A2M_BASE" in src_jp,
      "  ...and the cards say BASE so the number cannot be misread as raw motor deg")

print("\n  -- and the readout actually computes base angle, not motor deg --")
class Readout:
    _update_jog_readout = JC.JogControlMixin._update_jog_readout
    def _refresh_p2p_pose_readout(self): pass
    def __init__(self, a1, a2):
        self.sim_rot = 0.0
        self.sim_a1 = a1
        self.sim_a2 = a2
        self.sim_z = 0.0
        for v in ("rot_pos_v", "a1_pos_v", "a2_pos_v", "jz_pos_v",
                  "a1_reach_v", "a2_reach_v"):
            setattr(self, v, tk.StringVar())
ro = Readout(a1=C.FOLD_ANGLE_SPEC_MAX_DEG * C.ARM_GEAR_RATIO, a2=0.0)
ro._update_jog_readout()
check(ro.a1_pos_v.get() == "90.00 base deg",
      "A1M at the rated fold angle reads 90.00 base deg, not the raw motor figure")
check(ro.a2_pos_v.get() == "0.00 base deg", "  ...and A2M at home reads 0.00 base deg")
# The BOARD's telemetry path writes the same cards. It used to write them
# itself, in motor degrees, AFTER the jog readout had written base degrees
# -- so the panel showed the base angle only until the next [POS] line
# landed, which is what "A1M_BASE 0.00 motor deg" on the machine was.
import robot_sim.core.p2p_control as _P2C_EARLY
class Telemetry:
    _update_p2p_telemetry = _P2C_EARLY.P2PControlMixin._update_p2p_telemetry
    _update_jog_readout = JC.JogControlMixin._update_jog_readout
    def _refresh_p2p_pose_readout(self): self.p2p_repaints += 1
    def _set_progress(self, pct): pass
    def __init__(self):
        self.current_joints = [0.0, 0.0, 0.0, 0.0]
        self.p2p_repaints = 0
        for v in ("rot_pos_v", "a1_pos_v", "a2_pos_v", "jz_pos_v",
                  "a1_reach_v", "a2_reach_v"):
            setattr(self, v, tk.StringVar())
    sim_z = property(lambda self: self.current_joints[0])
    sim_rot = property(lambda self: self.current_joints[1])
    sim_a1 = property(lambda self: self.current_joints[2])
    sim_a2 = property(lambda self: self.current_joints[3])
tm = Telemetry()
tm._update_p2p_telemetry(0.0, 0.0, C.FOLD_ANGLE_SPEC_MAX_DEG * C.ARM_GEAR_RATIO,
                         0.0, pct=50)
check(tm.a1_pos_v.get() == "90.00 base deg" and tm.a2_pos_v.get() == "0.00 base deg",
      "board telemetry writes the SAME base-angle cards as the jog readout")
check(tm.p2p_repaints == 1,
      "  ...through one readout, repainting each panel once, so they cannot drift")

check("th3_cad, 60 deg = retracted" not in fw,
      "the boot banner no longer announces the th3_cad convention")
check("HOME IS 0" in fw, "  ...it says HOME IS 0")
check(C.ARM_HOME_DEG == 0.0 and C.FOLD_ANGLE_HOME_DEG == 0.0,
      "home is 0 in both the motor frame and the fold frame")
# The CAD offset went to 0 with the measured geometry: fold 0 IS the
# retracted pose, so there is no frame shift left to carry.
check(C.ARM_ZERO_CAD_DEG == 0.0,
      "ARM_ZERO_CAD_DEG is 0 — fold 0 is the retracted pose outright")
check(abs(fold_angle_to_reach(0.0) - C.ARM_MIN_REACH_MM) < 0.05,
      "  ...and fold 0° really is the 240 mm retracted reach")



# ══════════════════════════════════════════════════════════════════════
print("\n=== 16. HOME is on BACKSPACE, and the binding matches the reservation ===")
check(KB.HOME_KEY == "BackSpace", "HOME_KEY is the Tk keysym BackSpace")
check(KB.HOME_KEY in KB.RESERVED_KEYS,
      "  ...and that exact keysym is the one that is RESERVED")
check(KB.RESERVED_KEYS[KB.HOME_KEY] == "HOME", "  ...reserved AS home")
check("h" not in KB.RESERVED_KEYS and "H" not in KB.RESERVED_KEYS,
      "H is no longer reserved, so a jog axis may use it")
# The bug this pins: RESERVED_KEYS said backspace while core/keyboard.py
# still bound "h"/"H". Homing fired on a letter that was not protected, so
# one keypress could jog an axis AND start a homing cycle.
src_kbd = open(os.path.join(os.path.dirname(HERE), "robot_sim", "core",
                            "keyboard.py"), encoding="utf-8").read()
check('bind_all(f"<KeyPress-{keybinds.HOME_KEY}>"' in src_kbd,
      "the binder reads HOME_KEY instead of a literal")
check('for k in ("h", "H"):' not in src_kbd, "  ...and no longer binds h/H")
check(KB.display_key("BackSpace") == "BKSP", "it draws as BKSP on the pads")
check("BKSP = HOME" in KB.to_hint(KB.DEFAULT_KEYMAP),
      "  ...and the hint line under the pads says BKSP, not H")
# Binding a jog axis to backspace must be refused.
res = dict(KB.DEFAULT_KEYMAP); res["Z_UP"] = "BackSpace"
check(any("reserved" in x for x in KB.validate(res)),
      "a jog axis cannot take BackSpace")
# Sanity: every reserved key is spelled as a real Tk keysym, i.e. it is
# either a known display name or a plain single character.
odd = [k for k in KB.RESERVED_KEYS
       if k not in KB.KEY_DISPLAY and len(k) != 1]
check(not odd, "every reserved key is a Tk keysym we can render (odd: %s)" % odd)



# ══════════════════════════════════════════════════════════════════════
print("\n=== 17. stale firmware cannot inject the old 60 deg frame ===")
import robot_sim.core.protocol as PR2
import robot_sim.ui.sensor_panel as SP

class Board:
    """Just enough app for the telemetry parser."""
    def __init__(self):
        self.sim_rot = self.sim_a1 = self.sim_a2 = self.sim_z = 0.0
        self.logged = []
        self._warned_old_elbow_frame = False
        self.status_var = tk.StringVar(value="")
    def log(self, msg, tag="default"): self.logged.append((msg, tag))
    def _update_jog_readout(self): pass
    _parse_hardware_response = PR2.ProtocolMixin._parse_hardware_response
    _board_reports_motor_degrees = PR2.ProtocolMixin._board_reports_motor_degrees

# A pre-v9.2 board: no FOLD1/FOLD2 fields, elbow reading 60 at home.
b = Board()
b._parse_hardware_response(
    "[JOG POS] ROT: 12.00 deg | A1M: 60.00 deg | A2M: 60.00 deg | Z: 30.00 mm")
check(b.sim_a1 == 0.0 and b.sim_a2 == 0.0,
      "an old board's 60 deg elbow values are NOT adopted")
check(b.sim_rot == 12.0 and b.sim_z == 30.0,
      "  ...while RM and ZM, whose frame did not change, still update")
check(any("older than v9.2" in m for m, _t in b.logged),
      "  ...and the operator is told to re-flash")
check(any(t == "error" for _m, t in b.logged), "  ...as an error, not a whisper")
n_before = len(b.logged)
b._parse_hardware_response(
    "[JOG POS] ROT: 13.00 deg | A1M: 60.00 deg | A2M: 60.00 deg | Z: 31.00 mm")
check(len(b.logged) == n_before,
      "  ...warned ONCE per connection, not on every telemetry line")

# A v9.2 board: FOLD1/FOLD2 present, elbow in motor degrees.
b2 = Board()
b2._parse_hardware_response(
    "[JOG POS] ROT: 12.00 deg | A1M: 150.00 deg | A2M: 40.00 deg | Z: 30.00 mm"
    " | FOLD1: 75.00 deg | FOLD2: 20.00 deg | R1: 519.5 mm | R2: 219.4 mm")
check(b2.sim_a1 == 150.0 and b2.sim_a2 == 40.0,
      "a current board's motor degrees ARE adopted")
check(not any("older than v9.2" in m for m, _t in b2.logged), "  ...with no warning")
check("FOLD1:" in fw and "FOLD2:" in fw and "R1:" in fw,
      "the firmware really does append FOLD1/FOLD2/R1")

print("\n=== 17b. real hardware RUN completion actually unlocks the GUI ===")
# Regression: "[RUN] TARGET REACHED" starts with "[RUN]", and the generic
# [RUN]-progress branch used to swallow it and return before the dedicated
# completion check further down ever saw it -- so on real hardware, RUN
# never unlocked the GUI. Only the simulated path exercised completion.
rb = Board()
rb.is_running = True
rb.motion_locked = True
rb.progress_var = tk.IntVar(value=0)
rb.progress_pct_var = tk.StringVar(value="0%")
rb._set_motion_locked = lambda v: setattr(rb, "motion_locked", v)
rb._set_progress = lambda pct: (rb.progress_var.set(pct),
                                rb.progress_pct_var.set(f"{pct}%"))
rb._parse_hardware_response("[RUN] TARGET REACHED")
check(rb.is_running is False, "[RUN] TARGET REACHED actually clears is_running")
check(rb.motion_locked is False, "  ...and unlocks motion")
check(rb.progress_var.get() == 100, "  ...and completes the progress bar")
check("READY" in rb.status_var.get(), "  ...and the status line says READY")

# An ordinary in-progress [RUN] line must still be progress, not completion.
rb2 = Board()
rb2.is_running = True
rb2._parse_hardware_response("[RUN] Moving to point A...")
check(rb2.is_running is True,
      "an ordinary [RUN] progress line is NOT treated as completion")

print("\n=== 17c. RESET_POSITION: moves the machine, but is not a PLC reference ===")
import robot_sim.core.safety as SF

check(hasattr(SF.SafetyMixin, "reset_position"), "reset_position exists on SafetyMixin")

class RP(SF.SafetyMixin, PR2.ProtocolMixin):
    """Just enough app for reset_position() and its completion handler."""
    def __init__(self):
        self.sim_rot = self.sim_a1 = self.sim_a2 = self.sim_z = 0.0
        self.current_joints = [10.0, 20.0, 30.0, 40.0]   # deliberately not home
        self.logged = []
        self.status_var = tk.StringVar(value="")
        self.jog_status_var = tk.StringVar(value="")
        self.motion_locked = False
        self.is_running = False
        self.jog_active = False
        self.sent = []
        self.jog_dot = types.SimpleNamespace(itemconfig=lambda *a, **k: None)
        self._jog_dot_id = 1
    def log(self, msg, tag="default"): self.logged.append((msg, tag))
    def send(self, m, log_tx=True): self.sent.append(m)
    def _hardware_live(self): return False
    def _release_all_jog_axes(self, send_stop=True): pass
    def _cancel_job(self, name): pass
    def _schedule(self, name, ms, fn, *a): setattr(self, name, fn)   # fire synchronously
    def _set_motion_locked(self, v): self.motion_locked = bool(v)
    def _update_jog_readout(self): pass
    def _update_p2p_telemetry(self, *a, **k): pass

import unittest.mock as _mock
rp = RP()
with _mock.patch.object(messagebox, "askyesno", return_value=True):
    rp.reset_position()
check("RESET_POSITION" in rp.sent, "reset_position() sends RESET_POSITION on the wire")
check(rp.motion_locked, "  ...and locks motion like home() does")
check(not hasattr(rp, "is_homing") or not getattr(rp, "is_homing", False),
      "  ...but never claims the PLC-specific homing state")

# The distinct completion message unlocks motion and adopts the home pose --
# but, unlike _on_home_complete(), does NOT claim is_homed (no PLC reference
# was re-anchored) and must be dispatched before the generic "TARGET
# REACHED" catch-all (same shadowing risk [RUN] hit -- see 17b).
rp._parse_hardware_response("[RESET_POSITION] TARGET REACHED")
check(not rp.motion_locked, "the completion message unlocks motion")
check(rp.current_joints == [C.Z_HOME_MM, C.ROT_HOME_DEG, C.ARM_HOME_DEG, C.ARM_HOME_DEG],
      "  ...and the pose is updated to home")
check(getattr(rp, "is_homed", "unset") == "unset",
      "  ...without silently claiming a full PLC reference (is_homed untouched)")
check("READY" in rp.status_var.get(), "  ...and the status line says READY")

# A mid-progress [RESET_POSITION] line (no TARGET REACHED yet) must not be
# mistaken for completion.
rp2 = RP()
rp2.motion_locked = True
rp2._parse_hardware_response("[RESET_POSITION] Moving to (0,0,0,0)...")
check(rp2.motion_locked, "a progress line does not unlock motion early")
check("RESETTING POSITION" in rp2.status_var.get(), "  ...and just updates the status text")
check("RUNNING" in rb2.status_var.get(), "  ...and just updates the running text")

print("\n=== 18. coordinate reset lives in section 3, MOTION CONTROL ===")
_uidir = os.path.join(os.path.dirname(HERE), "robot_sim", "ui")
src_p2p = open(os.path.join(_uidir, "p2p_panel.py"), encoding="utf-8").read()
src_jog = open(os.path.join(_uidir, "jog_panel.py"), encoding="utf-8").read()
src_cr = open(os.path.join(_uidir, "coord_reset.py"), encoding="utf-8").read()
check("RESET COORDINATES" in src_cr, "the buttons are on the motion panel")
check("coord_reset_buttons" in src_cr, "  ...tracked as a group")
check("self.motion_lock_widgets += buttons" in src_cr,
      "  ...and locked during a move, so a counter cannot be zeroed mid-motion")
for axis in ('"Z"', '"ROT"', '"A1"', '"A2"'):
    check(axis in src_cr, "  ...with a per-axis button for %s" % axis)
# BOTH motion panels carry the row. Declaring the reference is a jogging
# action, so having it in P2P only forced a mode switch — which auto-stops
# motion — to finish a job started in JOG.
check("self._build_coord_reset_row(" in src_p2p,
      "  ...and the P2P panel builds it from the shared builder")
check("self._build_coord_reset_row(" in src_jog,
      "  ...and so does the JOYSTICK panel")
# One builder, so a change cannot land on one panel and miss the other.
check("RoundedButton" not in src_jog.split("_build_coord_reset_row")[1][:400],
      "  ...neither panel hand-rolls a second copy of the row")
check("self.coord_reset_buttons +=" in src_cr and
      "self.coord_reset_buttons = buttons" not in src_cr,
      "  ...the shared list is extended, not reassigned, so both rows lock")
src_app = open(os.path.join(os.path.dirname(HERE), "robot_sim", "app.py"),
               encoding="utf-8").read()
check(src_app.count("self.coord_reset_buttons = []") == 2,
      "  ...and it is emptied on init AND on a theme rebuild, like "
      "motion_lock_widgets")
check("RESET ONE AXIS" not in src_sd and "RoundedButton(actions" not in src_sd,
      "and the buttons are GONE from the Settings dialog, not duplicated")
check("command=self.reset_coordinates" not in src_sd,
      "  ...nothing in the dialog still calls reset_coordinates")
check("moved to section 3" in src_sd, "  ...with a note saying where they went")

print("\n  -- RESET POSITION rides the same shared row --")
check("command=self.reset_position" in src_cr,
      "the row also builds a RESET POSITION button")
check("reset_pos_btn" in src_cr and "buttons.append(reset_pos_btn)" in src_cr,
      "  ...tracked in the same buttons list, so it locks and is counted too")
# Visually distinct: it MOVES the machine, unlike the other five buttons
# in this row, which only declare a position.
check("ACCENT_RED" in src_cr.split('text="RESET POS"')[1][:200],
      "  ...styled distinctly (red), not blended into the no-move buttons")



# ══════════════════════════════════════════════════════════════════════
print("\n=== 19. the third lamp is the PLC Ethernet link, not the heartbeat ===")
src_lay = open(os.path.join(os.path.dirname(HERE), "robot_sim", "ui",
                            "layout.py"), encoding="utf-8").read()
check('make_status_led(leds, "HEARTBEAT' not in src_lay,
      "no HEARTBEAT lamp is constructed any more")
check("plc_led_card" in src_lay and 'f"PLC {PLC_IP}"' in src_lay,
      "  ...replaced by a lamp labelled with the PLC's address")
check(set(C.PLC_LED_STATES)
      == {"unknown", "connected", "no_reply", "unreachable", "disabled"},
      "five states: unknown / connected / no_reply / unreachable / disabled")

# No module may still write to the retired lamp.
import glob as _g
stale = [os.path.basename(f) for f in _g.glob(
            os.path.join(os.path.dirname(HERE), "robot_sim", "**", "*.py"),
            recursive=True)
         if "hb_led_card" in open(f, encoding="utf-8").read()]
check(not stale, "nothing writes to hb_led_card any more (stale: %s)" % stale)

# The heartbeat MECHANISM must survive: it is what notices a dead board.
src_hb = open(os.path.join(os.path.dirname(HERE), "robot_sim", "core",
                           "heartbeat.py"), encoding="utf-8").read()
check("MISSED_BEAT_LIMIT" in src_hb and "_on_ping_timeout" in src_hb,
      "the PING/PONG keep-alive still runs")
check("self.send(\"STOP\")" in src_hb,
      "  ...and a lost link still triggers the all-stop")
check('self.send("PLC_STATUS", log_tx=False)' in src_hb,
      "the same tick polls the board for its PLC state, so there is one cadence")
check(C.PLC_STATUS_POLL_MS == C.HEARTBEAT_INTERVAL_MS,
      "  ...at the heartbeat interval")

print("\n  -- the lamp follows the board, and logs only on a CHANGE --")
class Lamp:
    def __init__(self):
        self.logged = []
        self.state_seen = []
        self._plc_led_state = None
        self.status_var = tk.StringVar(value="")
        self.plc_led_card = None      # set_led is skipped, state still tracks
        self.plc_sensor_state = {b: False for b, *_r in C.PLC_SENSOR_PANEL}
        self.plc_sensor_end = {b: e for b, _l, _a, _c, e in C.PLC_SENSOR_PANEL}
        self.plc_sensor_lamps = {}
        self.rot_limit = {"ROT_CW": False, "ROT_CCW": False}
        self.z_limit = {"Z_UP": False, "Z_DOWN": False}
        self.jog_active = set()
        self.motion_locked = False
        self.is_running = False
        self._plc_home_state_prev = False
    def log(self, msg, tag="default"): self.logged.append((msg, tag))
    _parse_hardware_response = PR2.ProtocolMixin._parse_hardware_response
    _on_plc_state = PR2.ProtocolMixin._on_plc_state
    _set_plc_led = PR2.ProtocolMixin._set_plc_led
    _PLC_STATE_RE = PR2.ProtocolMixin._PLC_STATE_RE
    _PLC_DATA_RE = PR2.ProtocolMixin._PLC_DATA_RE
    _PLC_CONN_RE = PR2.ProtocolMixin._PLC_CONN_RE
    _PLC_HOME_BITS_RE = PR2.ProtocolMixin._PLC_HOME_BITS_RE
    _PLC_LIMIT_END_RE = PR2.ProtocolMixin._PLC_LIMIT_END_RE
    _read_plc_limit_ends = PR2.ProtocolMixin._read_plc_limit_ends
    plc_sensor_end_for = SP.SensorPanelMixin.plc_sensor_end_for
    plc_sensor_at_home_end = SP.SensorPanelMixin.plc_sensor_at_home_end
    _set_plc_sensor = PR2.ProtocolMixin._set_plc_sensor
    plc_sensor_covered_for_jog = PR2.ProtocolMixin.plc_sensor_covered_for_jog
    warn_if_jogging_into_sensor = PR2.ProtocolMixin.warn_if_jogging_into_sensor
    _latch_home_state_if_new = PR2.ProtocolMixin._latch_home_state_if_new
    plc_home_state = SP.SensorPanelMixin.plc_home_state
    _refresh_plc_sensor_lamps = lambda self: None
    _adopt_home_state_reset = lambda self: None

L = Lamp()
cases = [
    ("[PLC_STATE] link=DOWN socket=CLOSED word=---- timeouts=0", "unreachable"),
    # A TCP socket opening is NOT "connected" — see section 27.
    ("[PLC] TCP socket open to 192.168.3.101:1025 (attempt 1).", "no_reply"),
    ("[PLC_STATE] link=UP socket=OPEN data=NONE word=---- timeouts=3", "no_reply"),
    ("[PLC_STATE] link=UP socket=OPEN data=OK word=0082 timeouts=0 | M1(DONE)=1",
     "connected"),
    ("[ERROR] PLC unreachable at 192.168.3.101:1025 — check the cable", "unreachable"),
    # SET_PLC_LINK:0 — must read as "off on purpose", not as a fault, even
    # though socket=CLOSED and data=NONE look identical to a dead cable.
    ("[PLC_STATE] link=DISABLED socket=CLOSED data=NONE conn=0/0 word=---- "
     "timeouts=0 | LINK DISABLED", "disabled"),
    # And a real fault afterward must still overrule "disabled" — the state
    # is derived fresh from each line's own link= field, not sticky.
    ("[PLC_STATE] link=DOWN socket=CLOSED word=---- timeouts=0", "unreachable"),
]
for line, want in cases:
    L._parse_hardware_response(line)
    check(L._plc_led_state == want,
          "%-46s -> %s" % (line.split("]")[0] + "] " + line.split()[1][:28], want))

# Repeating an unchanged state must not add a log line: this is polled
# every 3 s, so a line per poll would bury the log.
L._plc_led_state = "connected"
n = len(L.logged)
for _ in range(5):
    L._parse_hardware_response("[PLC_STATE] link=UP socket=OPEN word=0082 timeouts=0")
check(len(L.logged) == n, "an unchanged state is not re-logged on every poll")
check("[PLC_STATE]" not in " ".join(m for m, _t in L.logged),
      "  ...and the raw poll reply is never spammed into the log")

# A dropped serial link must clear the lamp: whatever the board last said
# about the PLC is stale once the board is gone.
check("_plc_link_lost()" in src_hb,
      "losing the board resets the lamp AND marks the sensors unknown")
src_sl = open(os.path.join(os.path.dirname(HERE), "robot_sim", "core",
                           "serial_link.py"), encoding="utf-8").read()
check(src_sl.count("_plc_link_lost()") >= 2,
      "  ...as does connecting and disconnecting the COM port")


print("\n=== 20. HOME is the P2P reference: X 0, Y 0, Z 0 ===")
import robot_sim.kinematics as K
# X and Y are measured from the TURNTABLE AXIS and are signed, because RM
# can put the arm behind the machine. Z is the lift's travel UP from HOME
# and cannot be negative — HOME is the bottom of the stroke.
check(C.Z_INPUT_MIN_MM == 0.0 and C.Z_INPUT_MAX_MM == C.Z_STROKE_MM,
      "the typed Z runs 0..285, from HOME upward")
check(C.Z_HOME_ABS_MM == C.Z_OFFSET_ARM1_MM,
      "  ...and Z 0 is recorded as arm 1's real deck height, 514.3 mm")
check(0 <= C.DEFAULT_POINT_A[2] <= C.Z_INPUT_MAX_MM
      and 0 <= C.DEFAULT_POINT_B[2] <= C.Z_INPUT_MAX_MM,
      "the startup points are inside the new Z band, not left at 560/650")

# The translation lives in ONE place, at the edge. solve_ik and the MATLAB
# parity sweep stay absolute, because the sweep is what proves this module
# still reproduces mophong_init.m exactly.
check(abs(K.z_abs_from_home(0.0, "A1M") - C.Z_OFFSET_ARM1_MM) < 1e-9,
      "Z 0 from HOME is arm 1's deck at 514.3 mm absolute")
check(abs(K.z_abs_from_home(0.0, "A2M") - C.Z_OFFSET_ARM2_MM) < 1e-9,
      "  ...and arm 2's at 505.3 — the 9 mm lives HERE, not in what you type")
check(abs(K.z_home_from_abs(K.z_abs_from_home(137.0, "A2M"), "A2M") - 137.0) < 1e-9,
      "  ...and the pair round-trips")
# One carriage, so the same typed Z is the same d1 on either arm.
_d1a = K.solve_ik(300, 0, K.z_abs_from_home(45.0, "A1M"), "A1M")[0]
_d1b = K.solve_ik(300, 0, K.z_abs_from_home(45.0, "A2M"), "A2M")[0]
check(abs(_d1a - 45.0) < 1e-9 and abs(_d1b - 45.0) < 1e-9,
      "a typed Z of 45 is d1 = 45 on BOTH arms — one carriage, one Z")
try:
    K.solve_ik(300, 0, K.z_abs_from_home(-5.0, "A1M"), "A1M")
    check(False, "a negative Z is refused")
except ValueError as e:
    check(True, "a negative Z is refused")
    check("from HOME" in str(e) and "never negative" in str(e),
          "  ...in the frame the operator typed in, not as an absolute height")

print("\n  -- BOTH mode: one carriage means ONE Z, not Z and Z-9 --")
src_p2c = open(os.path.join(os.path.dirname(HERE), "robot_sim", "core",
                            "p2p_control.py"), encoding="utf-8").read()
sync_body = src_p2c.split("def _sync_z_for_both_mode")[1].split("\n    def ")[0]
check("- ARM2_Z_DROP_MM" not in sync_body,
      "the BOTH-mode sync no longer subtracts the 9 mm deck drop")
check('self.z1_v.set(f"{z0:g}")' in sync_body,
      "  ...it mirrors Z0 exactly; the drop is applied per arm in the IK")
check("z_abs_from_home(" in src_p2c,
      "LOAD converts the typed Z into the absolute frame before solve_ik")
check("z_home_from_abs(" in src_p2c,
      "  ...and converts back for the telemetry readout")
# The board has to agree, or the same numbers mean different things
# depending on whether they went through the GUI or a bare terminal.
check("IkResult solveIkFromHome(int arm" in fw,
      "the firmware takes Cartesian Z from HOME too")
check("must be EQUAL" in fw,
      "  ...and LOAD_XYZ_BOTH now wants Za == Zb, not Za - Zb = 9")
check("solveIkFrogleg(arm, X, Y, zFromHome + zOffsetForArm(a))" in fw,
      "  ...converting at the edge, leaving solveIkFrogleg absolute so the "
      "MATLAB parity sweep still means something")


print("\n=== 21. the real height is shown under each Z entry ===")
import robot_sim.core.p2p_control as P2C

class Heights:
    _refresh_real_heights = P2C.P2PControlMixin._refresh_real_heights
    def __init__(self, arm="A1M"):
        self.arm_config = arm
        self.z0_v, self.z1_v = tk.StringVar(), tk.StringVar()
        self.z0_real_v, self.z1_real_v = tk.StringVar(), tk.StringVar()

h = Heights("A1M"); h.z0_v.set("45"); h.z1_v.set("135")
h._refresh_real_heights()
check("559.3" in h.z0_real_v.get() and "A1M" in h.z0_real_v.get(),
      "Z 45 from HOME reads as a real height of 559.3 mm on A1M")
check("649.3" in h.z1_real_v.get(), "  ...and Z 135 as 649.3 mm")
h2 = Heights("A2M"); h2.z0_v.set("45"); h2.z1_v.set("45")
h2._refresh_real_heights()
check("550.3" in h2.z0_real_v.get(),
      "the SAME typed Z reads 9 mm lower on A2M — it is per deck")
hb = Heights("BOTH"); hb.z0_v.set("45"); hb.z1_v.set("45")
hb._refresh_real_heights()
check("559.3" in hb.z0_real_v.get() and "550.3" in hb.z1_real_v.get(),
      "in BOTH mode Z0 is arm 1's deck and Z1 is arm 2's, from one carriage")
hbad = Heights("A1M"); hbad.z0_v.set("-"); hbad.z1_v.set("400")
hbad._refresh_real_heights()
check(hbad.z0_real_v.get() == "",
      "a half-typed number (\"-\") shows nothing, not a height from junk")
check("outside" in hbad.z1_real_v.get(),
      "  ...and a Z past the stroke is flagged before you press LOAD")
check("_refresh_real_heights" in src_p2c.split("def set_arm_config")[1].split("\n    def ")[0],
      "switching arms repaints it — the deck changed even if the number did not")

print("\n=== 22. EMERGENCY STOP is on BOTH motion panels ===")
check("EMERGENCY STOP" in src_jog and "self.emergency_stop_all" in src_jog,
      "the joystick panel has an EMERGENCY STOP button again")
check("EMERGENCY STOP" in src_p2p, "  ...and so does P2P")
# One audited stop path. A second implementation is the thing to prevent,
# not a second button.
check(src_jog.count("def emergency_stop") == 0,
      "  ...calling the shared emergency_stop_all, not its own stop logic")
estop_block = src_jog.split("EMERGENCY STOP")[1].split("def ")[0]
check("motion_lock_widgets" not in estop_block,
      "  ...and it is NOT motion-locked: it must work while the machine moves")

print("\n=== 23. the PLC poll is fast always, and even faster while homing ===")
# 20 ms idle / 10 ms homing is a DELIBERATE operator choice, made after an
# earlier widening got silently reverted back to 5000/200 on the theory
# that 20 ms had once overloaded the FX5U link (conn=0/N, UNREACHABLE). The
# operator asked for it a second time and said explicitly not to revert it
# again. Do not "fix" this back down — if the link genuinely cannot sustain
# it, that's SET_PLC_POLL on the live machine, not a silent default change.
check(C.PLC_POLL_IDLE_MS == 20,
      "idle polling defaults to the operator's 20 ms")
check(C.PLC_POLL_HOMING_MS < C.PLC_POLL_IDLE_MS,
      "  ...and homing polls even faster")
check(C.PLC_POLL_MS == C.PLC_POLL_IDLE_MS,
      "  ...with PLC_POLL_MS still naming the idle rate")
check("PLC_POLL_IDLE_DEF_MS = 20;" in fw,
      "the firmware agrees on the idle rate as its DEFAULT")
# Still runtime-adjustable — SLOWER, if a given PLC ever needs it — and not
# persisted: a power cycle returns to the deliberate default.
check("SET_PLC_POLL:" in fw, "  ...and it can still be changed without a re-flash")
check("not persisted" in fw, "  ...without becoming the new default")
check("const unsigned long PLC_POLL_HOMING_MS = 10;" in fw,
      "  ...and on the homing rate")
# Homing still polls faster: HOME completes the moment M30..M32 all read
# true, and the sooner that lands the sooner the axes stop.
check("isHoming ? PLC_POLL_HOMING_MS : plcPollIdleMs" in fw,
      "  ...and picks between them on isHoming, covering the whole cycle")
check(C.PLC_POLL_IDLE_MS < 30000,
      "the idle poll stays well inside the 30 s HOME timeout")
check("if (ms < 1 || ms > 60000)" in fw,
      "SET_PLC_POLL accepts 1 ms..60 s, so the rate is tuned on the machine")


print("\n=== 24. the reach envelope is YOURS, not a structural guess ===")
# The 133.2 mm floor was R(fold = 0): it assumed the elbow's zero really is
# the folded home pose, measured through an unverified ARM_GEAR_RATIO. The
# IK now refuses only radii the geometry cannot solve at all.
check(abs(K.REACH_SOLVABLE_MAX_MM - 605.0) < 1e-9
      and abs(K.REACH_SOLVABLE_MIN_MM - 240.0) < 1e-9,
      "the only hard bound left is a3+a6 ± (a4+a5) = 422.5 ± 182.5 mm")
src_kin = open(os.path.join(os.path.dirname(HERE), "robot_sim",
                            "kinematics.py"), encoding="utf-8").read()
reach_body = src_kin.split("def _check_reach")[1].split("\ndef ")[0]
check("ARM_MIN_REACH_MM" not in reach_body and "ARM_MAX_REACH_MM" not in reach_body,
      "  ...and the structural envelope is gone from the check entirely")
# The band a taught elbow pair really sweeps. NOT min/max of the endpoints:
# reach is a cosine, so once a band crosses an extreme the extreme radius
# is INSIDE the interval, and endpoint-only under-reports it.
_peak = 180.0 * C.ARM_GEAR_RATIO          # motor° at the straight arm
lo, hi = K.reach_band_from_motor_deg(0.0, C.ARM_MOTOR_MAX_DEG)
check(abs(lo - 240.0) < 0.05 and abs(hi - 605.0) < 0.05,
      "the factory motor band sweeps the full 240..605 mm")
lo, hi = K.reach_band_from_motor_deg(_peak - 20.0, _peak + 20.0)
check(abs(hi - 605.0) < 0.05,
      "  ...and a band straddling the straight arm finds the peak BETWEEN its ends")
# fold -260..480 spans the whole curve, so BOTH extremes are interior.
# The endpoint-only version reported 593.9..613.2 here and refused every
# ordinary target — the bug reachBandFor was written to fix.
lo, hi = K.reach_band_from_motor_deg(-2.0 * _peak, 3.0 * _peak)
check(abs(hi - 605.0) < 0.05 and abs(lo - 240.0) < 0.05,
      "  ...and a band spanning the whole curve finds BOTH extremes inside it")

print("\n  -- the panel advertises those limits, and repaints when they change --")
class Hint:
    _refresh_workspace_hint = P2C.P2PControlMixin._refresh_workspace_hint
    _limit_pair = JC.JogControlMixin._limit_pair
    _axis_enforced = JC.JogControlMixin._axis_enforced
    def __init__(self, **over):
        self.settings = {k: C.LIMIT_FIELDS[k][6] for k in C.LIMIT_KEYS}
        for ek in C.LIMIT_ENFORCE_KEYS:
            self.settings[ek] = True
        self.settings[C.LIMITS_ENABLED_KEY] = True
        self.settings.update(over)
        self.workspace_hint_v = tk.StringVar()

hn = Hint(); hn._refresh_workspace_hint()
txt = hn.workspace_hint_v.get()
check("Settings" in txt and "Boundaries" in txt,
      "the hint says where the numbers come from")
check("%.1f" % K.motor_deg_to_reach(C.DEFAULT_LIM_A_MAX) in txt,
      "  ...and quotes the reach from the taught band's far end")
# The default elbow floor now coincides with the structural one, so proving
# the hint is LIVE needs a band that is not the default. A hard-coded
# envelope here would advertise a limit that nothing actually applies.
hn_taught = Hint(**{"lim_a1_min": 40.0, "lim_a1_max": 200.0})
hn_taught._refresh_workspace_hint()
# Only A1M's segment — A2M still has its default band, whose floor really
# is 133.2 mm, so checking the whole string would prove nothing.
a1_seg = hn_taught.workspace_hint_v.get().split("A1M")[1].split("|")[0]
check("133.2" not in a1_seg,
      "  ...and a taught A1M floor of 40 motor° is NOT reported as 133.2 mm")
check("%.1f" % _mdr(40.0) in a1_seg,
      "  ...it reports %.1f mm, the reach at that taught floor" % _mdr(40.0))
hn2 = Hint(**{C.LIMIT_ENFORCE_BY_AXIS["A1"]: False}); hn2._refresh_workspace_hint()
check("A1M not enforced" in hn2.workspace_hint_v.get(),
      "a switched-off axis says so instead of advertising a band")
check("_refresh_workspace_hint" in src_sd.split("def _apply_limits")[1].split("\n    def ")[0],
      "APPLY on the Boundaries tab repaints it")

print("\n  -- LOAD is where the working envelope is enforced --")
viol_body = src_p2c.split("def _limit_violation")[1].split("\n    def ")[0]
check("_limit_pair(" in viol_body,
      "the check SORTS the pair — elbow boundaries are stored as taught")
check("_axis_enforced(" in viol_body,
      "  ...and skips an axis whose enforcement is off")
check("Settings" in viol_body,
      "  ...and the refusal points at the tab that owns the number")
# The firmware has to agree, or the GUI and a bare terminal disagree about
# what is reachable.
check('if (axisEnforced(axisTok)) {' in fw,
      "the board checks the taught reach band only when that arm is enforced")
check("no solution: the frog-leg" in fw,
      "  ...and keeps the arithmetic refusal, which is not switchable")
check('if (axisEnforced("Z") && (d1 <' in fw and 'if (axisEnforced("ROT") && (th2 <' in fw,
      "  ...and the IK's ZM and RM checks respect their switches too")


print("\n=== 25. a taught boundary applies IMMEDIATELY, with no reference ===")
# The reported bug, exactly: A1M taught -341.89..902.14, machine never
# homed, and jog ran to -427.16 because the band was widened by a full
# travel either side while is_homed was false.
class Jogger:
    _axis_bounds = JC.JogControlMixin._axis_bounds
    _axis_enforced = JC.JogControlMixin._axis_enforced
    _floor_is_factory_default = JC.JogControlMixin._floor_is_factory_default
    _limit_pair = JC.JogControlMixin._limit_pair
    _apply_axis_limit = JC.JogControlMixin._apply_axis_limit
    def __init__(self, homed=False, **over):
        self.is_homed = homed
        self.settings = {k: C.LIMIT_FIELDS[k][6] for k in C.LIMIT_KEYS}
        for ek in C.LIMIT_ENFORCE_KEYS:
            self.settings[ek] = True
        self.settings[C.LIMITS_ENABLED_KEY] = True
        self.settings.update(over)

j = Jogger(homed=False, lim_a1_min=-341.89, lim_a1_max=902.14)
lo, hi = j._axis_bounds(*j._limit_pair("a1"), axis="A1")
check(abs(lo - (-341.89)) < 1e-9 and abs(hi - 902.14) < 1e-9,
      "unreferenced, the applied band IS the taught band — no widening")
check(not (lo <= -427.16 <= hi),
      "  ...so -427.16, the value that got through before, is now refused")
jh = Jogger(homed=True, lim_a1_min=-341.89, lim_a1_max=902.14)
check(jh._axis_bounds(*jh._limit_pair("a1"), axis="A1") == (lo, hi),
      "  ...and a referenced machine applies exactly the same band")
joff = Jogger(homed=False, lim_a1_min=-341.89, lim_a1_max=902.14,
              **{C.LIMIT_ENFORCE_BY_AXIS["A1"]: False})
lo_off, hi_off = joff._axis_bounds(*joff._limit_pair("a1"), axis="A1")
check(lo_off < -341.89 and hi_off > 902.14,
      "switching the axis OFF is still the way to jog outside a boundary")

print("\n  -- but a FACTORY-DEFAULT floor cannot pin an unreferenced axis --")
# The ZM bug: HOME is the minimum of every axis, so the shipped floor is 0.
# Unreferenced, the counter also reads 0 wherever the board powered up, so
# ZM sat exactly ON its floor and every Z_DOWN tick clamped straight back —
# readout frozen at 0.00, axis pinned, and the escape rule cannot help
# because sitting on the boundary counts as inside.
jz = Jogger(homed=False)
z_lo, z_hi = jz._axis_bounds(*jz._limit_pair("z"), axis="Z")
check(z_lo < C.DEFAULT_LIM_Z_MIN,
      "unreferenced, the DEFAULT ZM floor is relaxed so the axis can jog down")
check(abs(z_hi - C.DEFAULT_LIM_Z_MAX) < 1e-9,
      "  ...while the ceiling is untouched — it cannot meet the counter origin")
_v, _hit = JC.JogControlMixin._apply_axis_limit(-0.5, 0.0, z_lo, z_hi)
check(_hit is None and _v < 0.0,
      "  ...so a Z_DOWN tick from 0.00 actually moves, instead of freezing")
jzh = Jogger(homed=True)
check(jzh._axis_bounds(*jzh._limit_pair("z"), axis="Z")
      == (C.DEFAULT_LIM_Z_MIN, C.DEFAULT_LIM_Z_MAX),
      "  ...and once referenced the floor applies again, in full")
# Narrow on purpose: a TAUGHT floor is still enforced without a reference,
# which is the -341.89 case above and the whole point of this section.
jzt = Jogger(homed=False, lim_z_min=12.0)
check(jzt._axis_bounds(*jzt._limit_pair("z"), axis="Z")[0] == 12.0,
      "a TAUGHT floor is still enforced unreferenced — only defaults relax")

print("\n  -- and an axis outside its band can always jog back in --")
clamp = JC.JogControlMixin._apply_axis_limit
check(clamp(-350.0, -300.0, -341.89, 902.14) == (-341.89, "low"),
      "crossing the boundary from inside stops ON it")
check(clamp(-500.0, -450.0, -341.89, 902.14) == (-450.0, "low"),
      "  ...starting OUTSIDE and going further out is frozen where it was")
check(clamp(-400.0, -450.0, -341.89, 902.14) == (-400.0, None),
      "  ...but coming BACK toward the band is allowed, so nothing is trapped")
check(clamp(910.0, 905.0, -341.89, 902.14) == (905.0, "high"),
      "  ...same at the top end")
check(clamp(0.0, 0.0, -341.89, 902.14) == (0.0, None),
      "  ...and an in-range value is untouched")

print("\n  -- the messages stopped claiming limits are off --")
src_jc2 = open(os.path.join(os.path.dirname(HERE), "robot_sim", "core",
                            "jog_control.py"), encoding="utf-8").read()
check("soft limits off" not in src_jc2 and "are suspended" not in src_jc2,
      "the GUI no longer says soft limits are suspended without a reference")
check("positions are relative" in src_jc2,
      "  ...it says what a missing reference actually costs instead")
check("soft limits are suspended" not in fw,
      "and neither does the board")



# ══════════════════════════════════════════════════════════════════════
print("\n=== 20. a negative X is reachable, and stale RM limits are dropped ===")
# Bearing 180 deg. Under the old centred frame this was outside
# [-170, 170] and genuinely unreachable; in the 0..340 frame it is fine.
_ok = True
try:
    _d, _r, _a1, _a2 = solve_ik(-300.0, 0.0, 514.3 + 45.0, "A1M", idle_deg=0.0)
except ValueError as _e:
    _ok = False
check(_ok, "X = -300, Y = 0 solves")
check(_ok and abs(_r - 180.0) < 1e-9, "  ...to RM 180°, straight behind HOME")
_ok2 = True
try:
    solve_ik(-250.0, 250.0, 514.3 + 45.0, "A1M", idle_deg=0.0)
except ValueError:
    _ok2 = False
check(_ok2, "and X = -250, Y = 250 solves too")

print("\n  -- a v3 file's centred RM limits are DROPPED, not read as 0..340 --")
# This is the actual "it keeps saying there is a limit" bug: a stored
# -150..150 read in the new frame refuses every bearing past 150 deg,
# which includes every negative X.
check(C.ROT_FRAME_V4_RESET_KEYS == ("lim_rot_min", "lim_rot_max"),
      "the RM pair is the v4 reset set")
write({"lim_rot_min": -150.0, "lim_rot_max": 150.0,
       "lim_z_max": 270.0, "arm_pct": 90.0, "_schema": 3})
ld4 = Loader()
check(ld4.settings["lim_rot_min"] == C.DEFAULT_LIM_ROT_MIN
      and ld4.settings["lim_rot_max"] == C.DEFAULT_LIM_ROT_MAX,
      "a centred RM pair falls back to the full 0..335 travel")
check(ld4.settings["lim_z_max"] == 270.0,
      "  ...while ZM, whose frame did NOT change, is kept")
check(ld4.settings["arm_pct"] == 90.0, "  ...and so are the speeds")
_notes4 = " ".join(n[0] for n in ld4._pending_settings_notes)
check("counter-clockwise" in _notes4 and "negative X" in _notes4,
      "  ...and the operator is told why, and what it broke")
# A current file must be left completely alone.
write({"lim_rot_min": 10.0, "lim_rot_max": 300.0, "_schema": C.SETTINGS_SCHEMA})
ld5 = Loader()
check(ld5.settings["lim_rot_min"] == 10.0 and ld5.settings["lim_rot_max"] == 300.0,
      "a v4 file's RM limits are kept as they are")

print("\n=== 21. the ZM lead is measurable without a re-flash ===")
check("SET_Z_LEAD:" in fw, "the board takes SET_Z_LEAD")
check("double zMmPerRev = Z_MM_PER_REV_DEF;" in fw, "  ...into a runtime variable")
check("double pulsesPerMmZ()" in fw, "  ...and pulses/mm is derived from it")
# Nothing may compute with the old constant any more, or a re-calibration
# would be half-applied: the position would re-scale but the velocity, or
# the move, would not.
_body = fw.split("const double Z_MM_PER_MOTOR_REV = Z_MM_PER_REV_DEF;")[1]
check("PULSES_PER_MM_Z" not in _body,
      "the fixed PULSES_PER_MM_Z is gone from every calculation")
check(_body.count("pulsesPerMmZ()") >= 4,
      "  ...and position, move, velocity and accel all use the live figure")
# The board now carries the SPEC lead (20 mm/rev) and its own microstep
# constant for ZM; the GUI side is still the one asking for a bench check.
check("Z_MICROSTEPS_PER_STEP" in fw and "PULSES_PER_MOTOR_REV_Z" in fw,
      "ZM has its own pulses-per-rev, so the spec lead can be entered as-is")
check("MEASURE THIS" in open(
        os.path.join(os.path.dirname(HERE), "robot_sim", "config.py"),
        encoding="utf-8").read(),
      "  ...and the GUI still asks for the bench figure")
# The proportionality the operator can act on: 3x too far means 3x the lead.
check(abs(C.Z_MM_PER_MOTOR_REV - 20.0) < 1e-9,
      "the assumed lead is still 20 mm/rev until somebody measures it")

print("\n=== 21b. the RM gear ratio is measurable without a re-flash ===")
check("SET_ROT_RATIO:" in fw, "the board takes SET_ROT_RATIO")
check("double rotGearRatio = ROT_GEAR_RATIO_DEF;" in fw, "  ...into a runtime variable")
check("double pulsesPerDegRot()" in fw, "  ...and pulses/RM-degree is derived from it")
# Nothing may compute with the old fixed constant any more, same trap as
# section 21: a half-applied recalibration would re-scale position but not
# velocity, or the reverse.
_rot_body = fw.split("double rotGearRatio = ROT_GEAR_RATIO_DEF;")[1]
check("PULSES_PER_DEG_ROT" not in _rot_body,
      "the fixed PULSES_PER_DEG_ROT is gone from every calculation")
check(_rot_body.count("pulsesPerDegRot") >= 5,
      "  ...position, move and velocity/accel all use the live figure")
check('"[ROT_RATIO]"' in fw or "[ROT_RATIO]" in fw,
      "recalibrating confirms itself on the wire")
check("Re-check" in fw.split("SET_ROT_RATIO:")[1][:1200],
      "  ...and warns the taught RM limits (stored in RM degrees) need re-checking "
      "-- unlike the elbow, whose taught limits are motor degrees and unaffected")
check(abs(C.I_RM_TOTAL - 6.5) < 1e-9,
      "GUI-side mirror constant is the measured 6.5")
check("SET_ROT_RATIO" in fw,
      "  ...and the board takes a runtime recalibration, so it never live-syncs")



# ══════════════════════════════════════════════════════════════════════
print("\n=== 22. the Oxy workspace board ===")
import robot_sim.ui.xy_board as XB

class Board(XB.XYBoardMixin):
    """Only what the board reads: the entry vars, the shared pose and the
    RM band."""
    def __init__(self, **over):
        self.x0_v = tk.StringVar(value="300")
        self.y0_v = tk.StringVar(value="0")
        self.x1_v = tk.StringVar(value="250")
        self.y1_v = tk.StringVar(value="250")
        self.current_joints = [0.0, 0.0, 0.0, 0.0]
        self.arm_config = "A1M"
        self.xy_hint_v = tk.StringVar()
        self.xy_canvas = tk.Canvas()
        self._band = over.get("band", (C.ROT_MIN_DEG, C.ROT_MAX_DEG))
    def _limit_pair(self, axis):
        return self._band

b = Board(); b._refresh_xy_board()
kinds = [i[0] for i in b.xy_canvas.items]
check(kinds.count("oval") >= 4,
      "the annulus, HOME and the live dot are drawn")
check(XB.BOARD_PX >= 360,
      "the board is big enough to read (%d px)" % XB.BOARD_PX)
_rings = [t for t in [i[2].get("text") for i in b.xy_canvas.items if i[0] == "text"]
          if t in ("200", "300", "400", "500", "600")]
check(len(_rings) >= 4, "radius rings are drawn and labelled in mm")
check(all(C.ARM_MIN_REACH_MM < float(t) < C.ARM_MAX_REACH_MM for t in _rings),
      "  ...and only rings inside the reachable annulus")
check("arc" in kinds, "the unreachable RM wedge is drawn")
texts = [i[2].get("text") for i in b.xy_canvas.items if i[0] == "text"]
check("A" in texts and "B" in texts, "A and B are labelled")
check("HOME" in texts, "  ...and so is HOME")
check("RM 0°" in texts, "  ...and RM 0 is marked, since the whole frame hangs off it")
from robot_sim.theme import ACCENT_CYAN
check(any(i[0] == "line" and i[2].get("fill") == ACCENT_CYAN
          for i in b.xy_canvas.items),
      "the A→B chord is drawn")

print("\n  -- the scale is FIXED, so two runs can be compared by eye --")
_cx, _cy, _k = b._xy_scale()
b.x0_v.set("600"); b._refresh_xy_board()
check(b._xy_scale() == (_cx, _cy, _k),
      "moving a point does not rescale the plot")
check(abs(_k * C.ARM_MAX_REACH_MM - (XB.BOARD_PX / 2.0 - XB.BOARD_PAD)) < 1e-9,
      "  ...and the full outer reach always fits")

print("\n  -- screen Y is inverted, and RM 0 is +X --")
_px, _py = b._xy_to_px(100.0, 0.0)
check(_px > _cx and abs(_py - _cy) < 1e-9, "+X plots to the right")
_px2, _py2 = b._xy_to_px(0.0, 100.0)
check(_py2 < _cy, "  ...and +Y plots UP, not down")

print("\n  -- it never raises on input the operator is still typing --")
for bad in ("", "-", "abc", "1e", ",", "-."):
    b.x0_v.set(bad)
    try:
        b._refresh_xy_board()
        ok = True
    except Exception as e:      # noqa: BLE001 - any raise is the failure
        ok = False
    check(ok, "a half-typed X of %r does not break the board" % bad)
check(b._xy_points() is None, "  ...and simply draws no A/B while unparseable")

print("\n  -- the live dot is the SHARED pose, not a copy --")
b2 = Board()
b2.current_joints[1] = 90.0                      # RM 90 -> straight along +Y
b2.current_joints[2] = C.ARM_GEAR_RATIO * 60.0   # fold 60
_r60 = K.fold_angle_to_reach(60.0)
lx, ly = b2._xy_live_point()
check(abs(lx) < 1e-6 and abs(ly - _r60) < 0.1,
      "the dot follows current_joints (RM 90°, R %.2f mm)" % _r60)

print("\n  -- and it says the chord is not the tool path --")
b3 = Board(); b3._refresh_xy_board()
hint = b3.xy_hint_v.get().lower()
check("joint" in hint and ("bow" in hint or "chord" in hint),
      "the caption warns the real path bows away from the straight line")

src_p2p2 = open(os.path.join(os.path.dirname(HERE), "robot_sim", "ui",
                             "p2p_panel.py"), encoding="utf-8").read()
check("_build_xy_board" in src_p2p2, "the P2P panel builds the board")
_panel_body = src_p2p2.split("def _build_p2p_panel")[1].split("\n    def ")[0]
check(_panel_body.index("_build_coordinate_inputs(left_col)")
      < _panel_body.index("_build_xy_board(right_col)"),
      "  ...AFTER the entry boxes it reads, or it would first paint empty")
# Its own right-hand column now, not a full-width strip under the numbers:
# it moved so it could grow without squeezing (or being squeezed by) the
# entry boxes -- the old side-column layout clipped the workspace circle,
# which is exactly why it had been full-width before this.
#
# The split is grid-based (column 0 weight=0, column 1 weight=1), not
# nested pack fill/expand -- grid's column weights are unambiguous, so
# the left column always gets exactly its natural width, whatever `split`
# is stretched to.
check("split.grid_columnconfigure(0, weight=0)" in _panel_body,
      "  ...the left column is pinned to its natural width (grid weight=0)")
check("split.grid_columnconfigure(1, weight=1)" in _panel_body,
      "  ...and the board's column absorbs all the leftover space")
check('left_col.grid(row=0, column=0' in _panel_body,
      "  ...entry boxes in column 0")
check('right_col.grid(row=0, column=1' in _panel_body,
      "  ...board in column 1")

# Verified live: grid weight=0 alone was NOT enough. "Natural width" is
# still whatever the column's widest child asks for, and the HOME-frame
# caption + workspace-hint labels are long, single-line sentences with no
# wraplength -- an unwrapped Label reports its full text width as its
# natural size, which made the LEFT column itself thousands of pixels
# wide (cards sat left-aligned inside a secretly enormous column) and
# pushed the board off the right edge with no horizontal scrollbar to
# reach it. wraplength is what actually caps it.
_coord_body2 = src_p2p2.split("def _build_coordinate_inputs")[1].split("\n    def ")[0]
check(_coord_body2.count("wraplength=") >= 2,
      "the HOME caption and workspace-hint labels are wrapped, not left to "
      "report their full sentence as the column's natural width")
check("_refresh_xy_board" in src_p2p2, "  ...and repaints it on every keystroke")
check("_refresh_xy_board" in src_p2c,
      "the live dot is repainted from the pose readout path")

print("\n  -- Point A sits above Point B, not beside it --")
_coord_body = src_p2p2.split("def _build_coordinate_inputs")[1].split("\n    def ")[0]
check(_coord_body.index("POINT A") < _coord_body.index("POINT B"),
      "Point A is built before Point B")
check(_coord_body.count('side="top"') >= 2,
      "  ...and both point blocks stack top-to-bottom")
check('side="left"' not in _coord_body,
      "  ...neither one packs itself into a side-by-side slot any more")

print("\n  -- the real joint-space orbit is drawn, not just the chord --")
from robot_sim.kinematics import sample_joint_path as _sjp
_orbit_pts = _sjp((0, 0, 0, 0), (100, 90, 120, 0), n=10)
check(len(_orbit_pts) == 11, "sample_joint_path returns n+1 points")
check(_orbit_pts[0] != _orbit_pts[-1], "the endpoints differ for a real move")
b5 = Board(); b5._refresh_xy_board()
check(any(i[0] == "line" and i[2].get("dash") == (4, 2)
          for i in b5.xy_canvas.items),
      "an orbit polyline is drawn, distinct (dashed) from the solid chord")
# Must never raise on half-typed input, same contract as _xy_points.
for _bad in ("", "-", "abc"):
    b5.x1_v.set(_bad)
    try:
        b5._refresh_xy_board(); _ok = True
    except Exception:
        _ok = False
    check(_ok, "half-typed input does not break the orbit sampler either")
b5.x1_v.set(f"{C.DEFAULT_POINT_B[0]:g}")
check(b5._xy_orbit_points() is not None,
      "  ...and a valid A/B does produce an orbit again")



# ══════════════════════════════════════════════════════════════════════
print("\n=== 23. M30..M32 are the only sensors, and they are travel limits ===")
# M5..M8 are gone, not just muted — see the config.py comment: they lit a
# lamp and decided nothing, while a limit bit was refusing the jog.
check(len(C.PLC_SENSOR_PANEL) == 3, "the panel is M30, M31, M32 — nothing else")
_axis = {b: a for b, _l, a, _c, _e in C.PLC_SENSOR_PANEL}
check(_axis["M32"] == "Z" and _axis["M30"] == "A2",
      "M32 is ZM's device and M30 is A2M's — NOT the numeric order")
_ends = {b: e for b, _l, _a, _c, e in C.PLC_SENSOR_PANEL}
check(_ends["M32"] == -1 and _ends["M30"] == -1,
      "M32 (ZM) and M30 (A2M) sit at the MINIMUM of their axis")
check(_ends["M31"] == +1,
      "M31 (RM) sits at the MAXIMUM — RM is mounted inverted")
_cmds = {b: c for b, _l, _a, c, _e in C.PLC_SENSOR_PANEL}
check(_cmds["M32"] == "Z_DOWN" and _cmds["M31"] == "ROT_CW"
      and _cmds["M30"] == "A2_BACK",
      "the jog command that drives INTO each switch matches its end")
check(C.PLC_SENSOR_JOINT_INDEX == {"Z": 0, "ROT": 1, "A1": 2, "A2": 3},
      "each sensor maps to its axis in the shared pose")
# HOME is now defined the same way the board homes: all three limits true.
check(C.PLC_HOME_STATE_ON_BITS == ("M30", "M31", "M32")
      and C.PLC_HOME_STATE_CLEAR_BITS == (),
      "HOME STATE is all three limit bits true, same as the board")

print("\n=== 24. JOG warns, P2P refuses ===")
class Sensed:
    """Enough app for both paths."""
    def __init__(self, covered=(), pose=(0.0, 0.0, 0.0, 0.0), ends=None):
        self.plc_sensor_state = {b: (b in covered) for b, *_r in C.PLC_SENSOR_PANEL}
        self.current_joints = list(pose)
        self.logged = []
        self.__init_ends__(ends)
    def log(self, m, tag="default"): self.logged.append((m, tag))
    def __init_ends__(self, ends=None):
        self.plc_sensor_end = {b: e for b, _l, _a, _c, e in C.PLC_SENSOR_PANEL}
        if ends:
            self.plc_sensor_end.update(ends)
    plc_sensor_covered_for_jog = PR2.ProtocolMixin.plc_sensor_covered_for_jog
    warn_if_jogging_into_sensor = PR2.ProtocolMixin.warn_if_jogging_into_sensor
    _sensor_violation = P2C.P2PControlMixin._sensor_violation
    plc_sensor_end_for = SP.SensorPanelMixin.plc_sensor_end_for
    plc_sensor_at_home_end = SP.SensorPanelMixin.plc_sensor_at_home_end
    _PLC_LIMIT_END_RE = PR2.ProtocolMixin._PLC_LIMIT_END_RE
    _read_plc_limit_ends = PR2.ProtocolMixin._read_plc_limit_ends

print("\n  -- jog: a warning, and nothing is blocked --")
sj = Sensed(covered=("M32", "M31"))
sj.warn_if_jogging_into_sensor("Z_DOWN")
check(any("driving INTO M32" in m for m, _t in sj.logged), "jogging DOWN into M32 warns")
check(any(t == "warn" for _m, t in sj.logged), "  ...as a warning, not an error")
check(any("not blocked" in m for m, _t in sj.logged), "  ...and says it is not blocked")
sj.logged.clear()
sj.warn_if_jogging_into_sensor("ROT_CW")
check(any("driving INTO M31" in m for m, _t in sj.logged),
      "turning CW into M31 warns — the opposite end from M32")
sj.logged.clear()
sj.warn_if_jogging_into_sensor("Z_UP")
check(not sj.logged, "jogging AWAY from a covered sensor is silent")
sj.warn_if_jogging_into_sensor("ROT_CCW")
check(not sj.logged, "  ...and so is turning CCW off M31")
sc = Sensed()          # nothing covered
sc.warn_if_jogging_into_sensor("Z_DOWN")
check(not sc.logged, "and a clear sensor never warns")
# Jog must not consult the sensors through the limit latches any more.
src_pr3 = open(os.path.join(os.path.dirname(HERE), "robot_sim", "core",
                            "protocol.py"), encoding="utf-8").read()
check("_apply_plc_sensor_blocks" not in src_pr3,
      "the sensor->limit-latch coupling is gone, not just unused")
src_jc2 = open(os.path.join(os.path.dirname(HERE), "robot_sim", "core",
                            "jog_control.py"), encoding="utf-8").read()
check("warn_if_jogging_into_sensor" in src_jc2, "jog_start warns on press")
check(src_jc2.index("warn_if_jogging_into_sensor")
      < src_jc2.index("self.jog_active.add(command)"),
      "  ...before the axis is added, and WITHOUT returning early")

print("\n  -- P2P: refused, and only when the move makes it worse --")
sp = Sensed(covered=("M32",), pose=(50.0, 100.0, 100.0, 100.0))
check(sp._sensor_violation(20.0, 100.0, 100.0, 100.0) is not None,
      "with M32 covered, lowering ZM is refused")
check("M32" in sp._sensor_violation(20.0, 100.0, 100.0, 100.0),
      "  ...and the message names the device")
check(sp._sensor_violation(80.0, 100.0, 100.0, 100.0) is None,
      "  ...but RAISING ZM is allowed — it moves off the sensor")
check(sp._sensor_violation(50.0, 100.0, 100.0, 100.0) is None,
      "  ...and an axis that does not move is never refused")
sp7 = Sensed(covered=("M31",), pose=(50.0, 100.0, 100.0, 100.0))
check(sp7._sensor_violation(50.0, 140.0, 100.0, 100.0) is not None,
      "with M31 covered, turning RM further CW is refused")
check(sp7._sensor_violation(50.0, 60.0, 100.0, 100.0) is None,
      "  ...and turning CCW is allowed — M31 is at the max end")
check(sp7._sensor_violation(50.0, 100.0, 140.0, 100.0) is None,
      "  ...and A1M is unaffected, having no limit device at all")
sp0 = Sensed(pose=(50.0, 100.0, 100.0, 100.0))
check(sp0._sensor_violation(0.0, 0.0, 0.0, 0.0) is None,
      "with every sensor clear, nothing is refused")
check("_sensor_violation" in src_p2c, "LOAD runs the check")
check("Blocked by a PLC sensor" in src_p2c, "  ...and says so distinctly")

print("\n  -- A2M's switch is wired at BOTH ends, and the board says which --")
check(C.PLC_SENSOR_BOTH_ENDS == frozenset({"M30"}),
      "only A2M's device is wired at both ends of its travel")
check(C.PLC_SENSOR_JOG_CMD[("A2", -1)] == "A2_BACK"
      and C.PLC_SENSOR_JOG_CMD[("A2", +1)] == "A2_FWD",
      "  ...so it has a jog command for each end, not one")
# The GUI never works the end out for itself: between two polls it cannot
# see the edge the board latches on, and a wrong guess refuses the ONE
# direction that comes off the switch.
sf = Sensed(covered=("M30",), pose=(50.0, 100.0, 100.0, 100.0), ends={"M30": +1})
check(sf._sensor_violation(50.0, 100.0, 100.0, 140.0) is not None,
      "latched FORWARD: a leg extending A2M further is refused")
check(sf._sensor_violation(50.0, 100.0, 100.0, 60.0) is None,
      "  ...and one retracting it is allowed, so the arm is never pinned")
check("MAX" in sf._sensor_violation(50.0, 100.0, 100.0, 140.0),
      "  ...and the message names the end, since the device alone cannot")
sb = Sensed(covered=("M30",), pose=(50.0, 100.0, 100.0, 100.0))
check(sb._sensor_violation(50.0, 100.0, 100.0, 60.0) is not None
      and sb._sensor_violation(50.0, 100.0, 100.0, 140.0) is None,
      "unlatched falls back to the BACK end, which is what HOME looks like")
check(sf.plc_sensor_covered_for_jog("A2_FWD") == "M30"
      and sf.plc_sensor_covered_for_jog("A2_BACK") is None,
      "jog warns on A2_FWD while it sits on the forward switch")
check(sb.plc_sensor_covered_for_jog("A2_BACK") == "M30"
      and sb.plc_sensor_covered_for_jog("A2_FWD") is None,
      "  ...and on A2_BACK at the back one -- the opposite command")
check(sf.plc_sensor_at_home_end("M30") is False
      and sb.plc_sensor_at_home_end("M30") is True,
      "only the BACK end is the reference: a fully extended arm is not home")
sf.warn_if_jogging_into_sensor("A2_FWD")
check(any("not blocked" in m for m, _t in sf.logged),
      "jog still only WARNS at either end -- it is how you come off a switch")
sw = Sensed(covered=("M30",))
sw._read_plc_limit_ends("limit Z/R/A2=001 end Z/R/A2=-++ enforce Z/R/A2=111")
check(sw.plc_sensor_end_for("M30") == +1
      and sw.plc_sensor_end_for("M31") == +1
      and sw.plc_sensor_end_for("M32") == -1,
      "the board's end field is read positionally as M32/M31/M30, like the bits")
sw._read_plc_limit_ends("NO DEVICE DATA | limit Z/R/A2=??? end Z/R/A2=???")
check(sw.plc_sensor_end_for("M30") == +1,
      "  ...and '?' leaves the last known end alone rather than inventing one")
sold = Sensed(covered=("M30",), ends={"M30": +1})
sold._read_plc_limit_ends("limit Z/R/A2=001 enforce Z/R/A2=111")
check(sold.plc_sensor_end_for("M30") == +1,
      "  ...a board too old to send the field changes nothing")
check("PLC_LIMIT_BOTH_ENDS_A2  = true" in fw
      and "PLC_LIMIT_BOTH_ENDS_Z   = false" in fw,
      "the board is the one that knows A2M's switch has two ends")
check(fw.index("plcServiceLimitLatch();") < fw.index("plcServiceLimitStops();\n"),
      "  ...and it latches the end BEFORE the stop zeroes the direction it reads")

print("\n  -- the firmware splits it the same way --")
check("PLC_LIMIT_END_Z   = -1" in fw and "PLC_LIMIT_END_ROT = +1" in fw,
      "the board agrees on which end each limit is at")
check("runLegBlockedByLimit" in fw, "it refuses a P2P leg")
check("PLC_SENSOR_BLOCKS_" not in fw,
      "the old jog-blocking table is gone, not left beside the new one")
# The whole M1/M5..M8/M10..M13 apparatus is gone from the board, not muted.
for _dead in ("PLC_M_HOME_Z", "PLC_M_RUN_Z", "PLC_M_DONE", "plcAllHomeSensors",
              "plcServiceHomeSensors", "plcServiceSensorJogWarning",
              "runLegBlockedBySensor", "PLC_SENSOR_END_Z"):
    check(_dead not in fw, "  ...and %s is gone from the firmware" % _dead)



# ══════════════════════════════════════════════════════════════════════
print("\n=== 25. a dead PLC link must NOT look like four clear sensors ===")
# The reported bug: M6 was physically ON and the panel said CLEAR. The lamps
# started at CLEAR and only moved when a poll landed, so "no device data"
# and "nothing is covered" rendered identically — on a safety display the
# failure read as good news.
class Panel(SP.SensorPanelMixin):
    def __init__(self):
        self.plc_sensor_state = {b: False for b, *_r in C.PLC_SENSOR_PANEL}
        self.plc_sensor_end = {b: e for b, _l, _a, _c, e in C.PLC_SENSOR_PANEL}
        self.plc_sensor_data_seen = False
        self._plc_sensor_seen_at = None
        self.plc_sensor_lamps = {}
        self.plc_home_state_lamps = []
        self.logged = []
        self.status_var = tk.StringVar()
    def log(self, m, tag="default"): self.logged.append((m, tag))
    _mark_plc_sensors_seen = PR2.ProtocolMixin._mark_plc_sensors_seen
    _mark_plc_sensors_unknown = PR2.ProtocolMixin._mark_plc_sensors_unknown

pn = Panel()
check(not pn.plc_sensors_known(), "before any device read, the sensors are UNKNOWN")
check(not pn.plc_home_state(),
      "  ...and HOME STATE is false, so nothing auto-resets on a dead link")
pn._mark_plc_sensors_seen()
check(pn.plc_sensors_known(), "a landed read makes them known")
check(C.PLC_SENSOR_UNKNOWN_TEXT == "NO DATA",
      "the unknown state has its own caption, not CLEAR")
src_sp = open(os.path.join(os.path.dirname(HERE), "robot_sim", "ui",
                           "sensor_panel.py"), encoding="utf-8").read()
check("PLC_SENSOR_UNKNOWN_TEXT, ACCENT_PURPLE" in src_sp,
      "  ...and the lamps are BUILT in it, not in CLEAR")
# ...and not in COVERED's colour either. They were both ACCENT_ORANGE, so a
# dead link painted every lamp the same shade as a tripped limit.
check("ACCENT_ORANGE" not in src_sp.split("if not known:")[1].split("elif")[0],
      "  ...and UNKNOWN is a different colour from COVERED, not just different text")
check(src_sp.count("plc_sensors_known()") >= 2,
      "both the lamps and the home state gate on it")

print("\n  -- and the board reports unknown as a field, not by omission --")
check('limit Z/R/A2=???' in fw,
      "the board sends ??? when it has no device data")
check("NO DEVICE DATA" in fw, "  ...and says so in words too")
# The old summary returned bare "no data" with NO bit field, so the GUI's
# regex found nothing and simply never updated — silence read as CLEAR.
check('return String("no data")' not in fw,
      "the silent 'no data' summary is gone")
b_un = Board2 = None
class Lamp2:
    def __init__(self):
        self.plc_sensor_state = {b: False for b, *_r in C.PLC_SENSOR_PANEL}
        self.plc_sensor_end = {b: e for b, _l, _a, _c, e in C.PLC_SENSOR_PANEL}
        self.plc_sensor_data_seen = True
        self._plc_sensor_seen_at = None
        self.plc_sensor_lamps = {}
        self.plc_home_state_lamps = []
        self.logged = []
        self.status_var = tk.StringVar()
        self._plc_led_state = None
        self.plc_led_card = None
        self.motion_locked = False
        self.is_running = False
        self.jog_active = set()
        self._plc_home_state_prev = False
    def log(self, m, tag="default"): self.logged.append((m, tag))
    _parse_hardware_response = PR2.ProtocolMixin._parse_hardware_response
    _on_plc_state = PR2.ProtocolMixin._on_plc_state
    _set_plc_led = PR2.ProtocolMixin._set_plc_led
    _PLC_STATE_RE = PR2.ProtocolMixin._PLC_STATE_RE
    _PLC_DATA_RE = PR2.ProtocolMixin._PLC_DATA_RE
    _PLC_CONN_RE = PR2.ProtocolMixin._PLC_CONN_RE
    _PLC_HOME_BITS_RE = PR2.ProtocolMixin._PLC_HOME_BITS_RE
    _PLC_LIMIT_END_RE = PR2.ProtocolMixin._PLC_LIMIT_END_RE
    _read_plc_limit_ends = PR2.ProtocolMixin._read_plc_limit_ends
    plc_sensor_end_for = SP.SensorPanelMixin.plc_sensor_end_for
    plc_sensor_at_home_end = SP.SensorPanelMixin.plc_sensor_at_home_end
    _set_plc_sensor = PR2.ProtocolMixin._set_plc_sensor
    _mark_plc_sensors_seen = PR2.ProtocolMixin._mark_plc_sensors_seen
    _mark_plc_sensors_unknown = PR2.ProtocolMixin._mark_plc_sensors_unknown
    _latch_home_state_if_new = PR2.ProtocolMixin._latch_home_state_if_new
    _adopt_home_state_reset = lambda self: None
    plc_home_state = SP.SensorPanelMixin.plc_home_state
    plc_sensors_known = SP.SensorPanelMixin.plc_sensors_known
    _refresh_plc_sensor_lamps = lambda self: None

lu = Lamp2()
lu._parse_hardware_response(
    "[PLC_STATE] link=UP socket=OPEN word=---- timeouts=4 | NO DEVICE DATA | "
    "limit Z/R/A2=???")
check(not lu.plc_sensors_known(),
      "a '????' reply marks the sensors unknown rather than clear")
check(any("UNKNOWN" in m for m, _t in lu.logged),
      "  ...and it is logged as a fault")
check(any(t == "error" for _m, t in lu.logged), "  ...as an error")

print("\n  -- SET_PLC_LINK:0's own line marks the sensors unknown too --")
ld = Lamp2()
ld._parse_hardware_response(
    "[PLC_STATE] link=DISABLED socket=CLOSED data=NONE conn=0/0 word=---- "
    "timeouts=0 | LINK DISABLED — SET_PLC_LINK:1 to re-enable | limit Z/R/A2=???")
check(ld._plc_led_state == "disabled",
      "the lamp reads DISABLED, not NO REPLY or UNREACHABLE")
check(not ld.plc_sensors_known(),
      "  ...and the sensors go unknown through the SAME '???' path as a dead link")

print("\n  -- a real reading still lands, with M6 covered --")
lr = Lamp2()
lr._parse_hardware_response(
    "[PLC_STATE] link=UP socket=OPEN word=0000 timeouts=0 | limit Z/R/A2=010")
check(lr.plc_sensors_known(), "a real bit field is accepted")
check(lr.plc_sensor_state["M31"] is True,
      "M31 covered is read as COVERED — the reported symptom")
check(lr.plc_sensor_state["M32"] is False and lr.plc_sensor_state["M30"] is False,
      "  ...and the others stay clear")
# The wire field is ordered by AXIS (Z/R/A2) and the devices are NOT in
# numeric order, so a positional decode of M30/M31/M32 puts ZM's lamp on
# A2M's switch. That is the bug this pins.
lz = Lamp2()
lz._parse_hardware_response(
    "[PLC_STATE] link=UP socket=OPEN word=0000 timeouts=0 | limit Z/R/A2=100")
check(lz.plc_sensor_state["M32"] is True,
      "the FIRST field position is ZM, and ZM is M32")
check(lz.plc_sensor_state["M30"] is False,
      "  ...not M30, which is A2M and sits in the LAST position")

print("\n=== 26. PLC wire diagnostics ===")
check("PLC_TEST" in fw, "PLC_TEST does one read and reports the outcome")
for phrase, why in (
        ("Ethernet link:", "reports whether the PHY has a cable"),
        ("TCP connect:", "  ...whether the socket opened"),
        ("[PLC_TEST] TX ", "  ...the exact frame sent"),
        ("RX nothing", "  ...and distinguishes no answer"),
        ("not with MC protocol 3E ASCII", "  ...from a wrong protocol"),
        ("End code", "  ...from a refusal")):
    check(phrase in fw, why)
check("PLC_DEBUG:" in fw and "[PLC_TX]" in fw and "[PLC_RX]" in fw,
      "PLC_DEBUG echoes every frame verbatim")
check("[PLC_COUNTS]" in fw, "PLC_STATUS reports connect/send/read/timeout counts")
check("NO device read has EVER succeeded" in fw,
      "  ...and says plainly when nothing has ever been read")

print("\n  -- HOME names the real cause instead of just timing out --")
check("plcGoodReads == 0" in fw, "HOME warns UP FRONT if no read has succeeded")
check("it could not have seen the switches at all" in fw,
      "  ...and on timeout blames the link, not the PLC")
check("a switch is stuck, broken, or the" in fw and "mechanically obstructed" in fw,
      "  ...or the physical switch/mechanism, when reads work but a switch never came ON")
check("SET_PLC_SENSOR_ENFORCE" in fw,
      "  ...and points at the escape hatch for a switch known to be broken")



# ══════════════════════════════════════════════════════════════════════
print("\n=== 27. the PLC lamp must not FLAP with the socket ===")
# Reported: the lamp alternated CONNECTED / UNREACHABLE every few seconds.
# It was not lying about the socket — the socket really was cycling:
#   TCP connect OK -> poll -> no reply -> 800 ms timeout -> socket dropped
#   to resync -> 3 s later reconnect -> repeat.
# The lamp was answering "is a socket open" when the operator is asking
# "is device data arriving". It now reports data=, so the same fault sits
# steadily on NO REPLY instead of blinking.
check("data=" in fw, "the board reports a data= state")
check("plcDataState" in fw, "  ...from a dedicated helper")
for token in ("NONE", "STALE", "OK"):
    check('return "%s"' % token in fw, "  ...with a %s case" % token)
check("plcDataStaleMs" in fw, "  ...and a staleness window")

class Flap:
    def __init__(self):
        self._plc_led_state = None
        self.transitions = []
        self.status_var = tk.StringVar()
        self.plc_led_card = None
        self.plc_sensor_state = {b: False for b, *_r in C.PLC_SENSOR_PANEL}
        self.plc_sensor_end = {b: e for b, _l, _a, _c, e in C.PLC_SENSOR_PANEL}
        self.plc_sensor_data_seen = False
        self._plc_sensor_seen_at = None
        self.plc_sensor_lamps = {}
        self.plc_home_state_lamps = []
        self.logged = []
        self.motion_locked = False
        self.is_running = False
        self.jog_active = set()
        self._plc_home_state_prev = False
    def log(self, m, tag="default"): self.logged.append((m, tag))
    _parse_hardware_response = PR2.ProtocolMixin._parse_hardware_response
    _on_plc_state = PR2.ProtocolMixin._on_plc_state
    _PLC_STATE_RE = PR2.ProtocolMixin._PLC_STATE_RE
    _PLC_DATA_RE = PR2.ProtocolMixin._PLC_DATA_RE
    _PLC_CONN_RE = PR2.ProtocolMixin._PLC_CONN_RE
    _PLC_HOME_BITS_RE = PR2.ProtocolMixin._PLC_HOME_BITS_RE
    _PLC_LIMIT_END_RE = PR2.ProtocolMixin._PLC_LIMIT_END_RE
    _read_plc_limit_ends = PR2.ProtocolMixin._read_plc_limit_ends
    plc_sensor_end_for = SP.SensorPanelMixin.plc_sensor_end_for
    plc_sensor_at_home_end = SP.SensorPanelMixin.plc_sensor_at_home_end
    _set_plc_sensor = PR2.ProtocolMixin._set_plc_sensor
    _mark_plc_sensors_seen = PR2.ProtocolMixin._mark_plc_sensors_seen
    _mark_plc_sensors_unknown = PR2.ProtocolMixin._mark_plc_sensors_unknown
    _latch_home_state_if_new = PR2.ProtocolMixin._latch_home_state_if_new
    _adopt_home_state_reset = lambda self: None
    plc_home_state = SP.SensorPanelMixin.plc_home_state
    plc_sensors_known = SP.SensorPanelMixin.plc_sensors_known
    _refresh_plc_sensor_lamps = lambda self: None
    def _set_plc_led(self, state, detail=""):
        if state != self._plc_led_state:
            self.transitions.append(state)
        PR2.ProtocolMixin._set_plc_led(self, state, detail)

# Replay the exact cycle the machine produced: socket opens, read times
# out, socket drops, socket reopens — three times over.
fl = Flap()
for _ in range(3):
    fl._parse_hardware_response("[PLC] TCP socket open to 192.168.3.101:1025 (attempt 2).")
    fl._parse_hardware_response(
        "[PLC] No reply within 800 ms (7 so far). The socket is open but the PLC "
        "is not answering device reads.")
    fl._parse_hardware_response(
        "[PLC_STATE] link=DOWN socket=CLOSED data=NONE conn=3/3 word=---- timeouts=7")
check("connected" not in fl.transitions,
      "the lamp NEVER reads CONNECTED while no device data has arrived")
check(fl.transitions.count("no_reply") == 1,
      "  ...and settles on NO REPLY exactly once instead of flapping (%s)"
      % fl.transitions)

# And a real read still promotes it.
fl._parse_hardware_response(
    "[PLC_STATE] link=UP socket=OPEN data=OK word=0040 timeouts=7 | M1(DONE)=0 "
    "home Z/R/A1/A2=0100 run Z/R/A1/A2=0000")
check(fl._plc_led_state == "connected",
      "a landed device read DOES promote it to CONNECTED")
# Data that stops is not the same as a closed socket.
fl._parse_hardware_response(
    "[PLC_STATE] link=UP socket=OPEN data=STALE word=0040 timeouts=9")
check(fl._plc_led_state == "no_reply",
      "reads that worked and stopped read NO REPLY, not UNREACHABLE")
# A genuinely closed socket with no data is still UNREACHABLE.
# conn=0/N — never opened a socket at all. THAT is unreachable.
fl._parse_hardware_response(
    "[PLC_STATE] link=DOWN socket=CLOSED data=NONE conn=0/9 word=---- timeouts=9")
check(fl._plc_led_state == "unreachable",
      "a socket that has NEVER opened is UNREACHABLE — cable or address")
# conn=5/9 — it opens fine, it just answers nothing. Not unreachable.
fl._parse_hardware_response(
    "[PLC_STATE] link=DOWN socket=CLOSED data=NONE conn=5/9 word=---- timeouts=9")
check(fl._plc_led_state == "no_reply",
      "  ...but one that has opened before is NO REPLY, not UNREACHABLE")

print("\n  -- the board says the flapping IS the fault --")
check("opening and closing every few seconds IS this" in fw,
      "the log explains the symptom rather than leaving it to be guessed")
check("Cable and address are fine" in fw,
      "  ...and rules out the things that are NOT wrong")
check("TCP socket open to" in fw,
      "the connect message says 'socket', not 'connected'")
check("does NOT mean device reads work" in fw, "  ...and says so explicitly")
check("plcLastConnectLog" in fw,
      "and it is rate limited, or reconnecting every 3 s buries the log")
check("conn=" in fw and "plcConnectsOk" in fw,
      "the board reports how many connects SUCCEEDED, not just the current socket")


print("\n=== 28. ENTER runs P2P, and Return is a reserved key ===")
import robot_sim.core.keyboard as KBM

check(KB.RESERVED_KEYS.get("Return") is not None,
      "Return is reserved, so no jog action can be bound to it")


class RunKeyApp(KBM.KeyboardMixin):
    def __init__(self):
        self.root = tk.Tk()
        self.mode = "P2P"
        self.motion_locked = False
        self.jog_pads = {}
        self._bound_jog_keys = ()
        self.ran = 0
        self.homed = 0

    def p2p_run_program(self): self.ran += 1
    def home(self): self.homed += 1
    def jog_start(self, c): pass
    def jog_stop(self, c, s=None): pass
    def emergency_stop_all(self): pass


app = RunKeyApp()
app._bind_keys()
check("<KeyPress-Return>" in app.root.bindings, "ENTER is bound app-wide")
app.root.bindings["<KeyPress-Return>"](None)
check(app.ran == 1, "  ...and running it fires p2p_run_program()")

app.mode = "JOG"
app.root.bindings["<KeyPress-Return>"](None)
check(app.ran == 1, "  ...but not while in JOYSTICK mode")

app.mode = "P2P"
app.root._focus = tk.Entry(app.root)
app.root.bindings["<KeyPress-Return>"](None)
check(app.ran == 1, "  ...or while a coordinate box (or any entry) has focus")

app.root._focus = None
app.root.bindings["<KeyPress-Return>"](None)
check(app.ran == 2, "  ...and fires again once focus clears")

# BackSpace/HOME is unaffected by this change — still gated on JOG mode,
# not P2P, the opposite of ENTER/RUN.
app.mode = "JOG"
app.root.bindings["<KeyPress-BackSpace>"](None)
check(app.homed == 1, "HOME still fires from JOYSTICK mode")
app.mode = "P2P"
app.root.bindings["<KeyPress-BackSpace>"](None)
check(app.homed == 1, "  ...and still refuses from P2P")


print("\n=== 29. the Xbox controller is GONE, not merely unwired ===")
# Removed on request: jog is keyboard and on-screen pads only. Asserted as
# an ABSENCE, the way PLC_SENSOR_BLOCKS_* is, because a half-removal --
# module deleted but the mixin still listed, or the reverse -- would fail
# at import time on the machine rather than here.
import inspect as _inspect
check(not os.path.exists(os.path.join(os.path.dirname(HERE), "robot_sim",
                                      "core", "gamepad_control.py")),
      "gamepad_control.py is deleted, not left orphaned in the tree")
import robot_sim.core as _CORE
check("GamepadMixin" not in _CORE.__all__ and not hasattr(_CORE, "GamepadMixin"),
      "  ...and core no longer imports or exports GamepadMixin")
import robot_sim.app as _APP
_app_src = _inspect.getsource(_APP)
check("Gamepad" not in _app_src and "_init_gamepad" not in _app_src,
      "the app neither mixes it in nor starts a poll loop")
import robot_sim.core.safety as _SF
check("_gamepad_quit" not in _inspect.getsource(_SF),
      "  ...and shutdown no longer tries to close a controller")
# The jog commands themselves are untouched: the pad was only ever an
# extra input onto jog_start()/jog_stop(), so removing it takes no axis
# with it.
check(set(C.JOG_STOP_COMMAND) >= {"A1_FWD", "A1_BACK", "A2_FWD", "A2_BACK",
                                  "ROT_CW", "ROT_CCW", "Z_UP", "Z_DOWN"},
      "all eight jog commands survive -- only the input path went")


print("\n=== 30. telemetry is parsed, never logged ===")
# The board reports the pose every 50 ms while an axis is held. Writing
# those into the event log cost 87 ms of every second on real Tk -- an
# insert, a see("end") that forces a scroll and repaint, and once the
# 800-line cap is reached an index scan and a delete, twenty times a
# second. It also buried every line the operator actually needed.
import robot_sim.core.serial_link as _SL

check(_SL.is_telemetry("[JOG POS] ROT: 1.00 deg | A1M: 2.00 deg"),
      "the 20 Hz jog pose is telemetry")
check(_SL.is_telemetry("[CLEARCORE POS] D1: 0.00 | ROT: 0.00"),
      "  ...so is the run pose")
for _line in ("[ERROR] SCAN refused", "[WARN] no calibration", "[LIMIT] Z_UP",
              "[PLC_STATE] link=UP", "[HOME] ROT reached its switch",
              "[RUN] TARGET REACHED", "PONG"):
    check(not _SL.is_telemetry(_line),
          f"  ...but {_line.split(']')[0] + ']' if ']' in _line else _line} still reaches the log")

_sl_src = _inspect.getsource(_SL.SerialLinkMixin._listen_hardware_response)
check("if not is_telemetry(raw):" in _sl_src,
      "the RX pump skips the log for telemetry, and only for the log")
check("self._parse_hardware_response(raw)" in _sl_src
      and _sl_src.index("self._parse_hardware_response(raw)")
          > _sl_src.index("if not is_telemetry(raw):"),
      "  ...every line is still PARSED, so the readout is as live as it was")
# The TX side already worked this way; this is the same rule, applied to
# the half that never got it.
import robot_sim.core.jog_control as _JC
check('self.send("JOG_HB", log_tx=False)' in _inspect.getsource(_JC),
      "the keep-alive was already kept out of the log for the same reason")


print("\n" + ("ALL PYTHON CHECKS PASSED" if not FAIL else "FAILURES: %s" % FAIL))
sys.exit(1 if FAIL else 0)
