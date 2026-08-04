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

LED ACTIVITY INDICATOR:
    The board's status LED (LED_PIN / ClearCore IO0) is now a pure
    ACTIVITY light instead of a steady "connected" light:
    - Every command RECEIVED from Python triggers a very short,
        fast flash (see LED_FLASH_RX_MS) — the faster the commands
      arrive (e.g. held jog keys), the faster it flickers.
    - Every feedback line SENT to Python triggers a slightly longer
      flash (see LED_FLASH_TX_MS), so you can visually confirm the
      board is actively reporting back.
      
  Both flashes are done with millis()-based timing (ledPulse() /
the LED section of loop()) — never delay() — so motion stepping,
serial reads, and heartbeats are never blocked by the LED.

  A steady "connected" LED is no longer used: with feedback flowing
every ALIVE_INTERVAL_MS while idle, a live connection now shows
itself as a slow, regular flicker; a dead one goes fully dark.
  
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
