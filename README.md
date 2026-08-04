Robot Motion Controller — P2P + Joystick
=============================================================================
Merges two control modes on one shared connection layer:
  - POINT TO POINT : absolute X/Y/Z target motion 
  - JOYSTICK        : held-key jog on ROT/ARM/Z axes

SHARED ACROSS BOTH MODES:
  - One connection panel + 3-LED status (COM PORT / CLEARCORE IO0 / HEARTBEAT)
  - PING/PONG heartbeat every HEARTBEAT_INTERVAL_MS with missed-beat tracking
  - One unified Event Log (>> sent / << received, color-tagged)
  - A software-only simulation fallback when no hardware is confirmed, so
    both modes remain fully testable/demoable with nothing plugged in

SAFETY DESIGN (defense in depth — enforced in BOTH layers independently):
  - Switching mode in the GUI auto-stops any active jog axis + any P2P move
  - Firmware also refuses to run a P2P move while a jog axis is held (and
    vice versa) — a backstop in case the GUI and firmware ever desync
  - There is exactly ONE emergency-stop code path (emergency_stop_all):
    the P2P panel's button, the Joystick panel's button, and the Space key
    all call it. One audited function beats three similar ones.
  - PING always gets a PONG reply immediately, even mid-motion, so the
    heartbeat never times out just because the board is busy

ASSUMPTION TO VERIFY BEFORE REAL HARDWARE USE:
  ROT/ARM/Z (jog, cylindrical-style) and X/Y/Z (P2P, Cartesian) are kept as
  TWO SEPARATE simulated axis sets, both here and in the firmware. If your
  real STCR4000S kinematics mean these are the same physical joints, you
  need to add the forward/inverse kinematics transform yourself — seeding
  that transform without knowing the real chain would silently misreport
  position. See RobotMotionController_v4.ino header for the matching note.

PROTOCOL (Python -> ClearCore):
  PING / BYE / START:X0,Y0,Z0,X1,Y1,Z1 / STOP
  ROT_CW / ROT_CCW / ROT_STOP
  ARM_FWD / ARM_BACK / ARM_STOP
  Z_UP / Z_DOWN / Z_STOP
  HOME / ESTOP

PROTOCOL (ClearCore -> Python):
  PONG | [ALIVE] uptime: Xs
  [CLEARCORE POS] Vi tri hien tai -> X: F mm | Y: F mm | Z: F mm (P%)
  [JOG POS] ROT: F deg | ARM: F mm | Z: F mm
  DA DEN DIEM DICH THANH CONG | DUNG KHAN CAP
  [HOME] Homing started. | [HOME] Homing complete. ROT=0 ARM=0 Z=0
