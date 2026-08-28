"""Serial link to the board, and a simulator for when there is no board.

The simulator is not a toy: it emits the SAME lines the firmware emits, at
roughly the same rate, so every path in the UI -- parsing, plotting, the
progress counters, saving -- is exercised whether or not hardware is
plugged in. That is how the main console is built too, and it is why a
demo can be given away from the machine.
"""

import math
import random
import time

try:
    import serial
    import serial.tools.list_ports as list_ports
    HAS_SERIAL = True
except ImportError:                       # pragma: no cover
    serial = None
    list_ports = None
    HAS_SERIAL = False


def available_ports():
    if not HAS_SERIAL:
        return []
    return [p.device for p in list_ports.comports()]


class SerialLink:
    """Line-oriented wrapper. Never raises at the caller: a dropped USB
    lead reports itself as a disconnect, because a traceback in the middle
    of a scan loses the points already collected."""

    def __init__(self, on_line, on_error):
        self._port = None
        self._buf = ""
        self._on_line = on_line
        self._on_error = on_error

    @property
    def is_open(self):
        return self._port is not None

    def open(self, port, baud):
        if not HAS_SERIAL:
            self._on_error("pyserial is not installed - running simulated.")
            return False
        try:
            # write_timeout matters as much as the read timeout: without it
            # a stalled link blocks the Tk thread and freezes the window.
            self._port = serial.Serial(port=port, baudrate=int(baud),
                                       timeout=0.05, write_timeout=0.5)
            self._buf = ""
            return True
        except Exception as exc:           # pragma: no cover - hardware path
            self._on_error(f"Could not open {port}: {exc}")
            self._port = None
            return False

    def close(self):
        if self._port is None:
            return
        try:
            self._port.close()
        except Exception:                  # pragma: no cover
            pass
        self._port = None

    def send(self, text):
        if self._port is None:
            return False
        try:
            self._port.write((text + "\n").encode("ascii", "ignore"))
            return True
        except Exception as exc:           # pragma: no cover - hardware path
            self._on_error(f"Write failed: {exc}")
            self.close()
            return False

    def poll(self):
        """Reads whatever has arrived and hands over COMPLETE lines only.

        A partial line is kept in the buffer. Splitting a [SCAN_PT] across
        two reads and parsing both halves would put a bogus point in the
        middle of the cloud, which is far harder to spot than a missing one.
        """
        if self._port is None:
            return
        try:
            waiting = self._port.in_waiting
            if not waiting:
                return
            self._buf += self._port.read(waiting).decode("ascii", "ignore")
        except Exception as exc:           # pragma: no cover - hardware path
            self._on_error(f"Read failed: {exc}")
            self.close()
            return
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            line = line.strip()
            if line:
                self._on_line(line)


class SimulatedBoard:
    """Answers the scan commands with the firmware's own vocabulary.

    Shapes a slightly off-centre elliptical chamber with a notch in it, so
    the polar plot shows something recognisable rather than a circle that
    would look the same however wrong the plotting was.
    """

    POINT_INTERVAL_S = 0.004

    def __init__(self, on_line):
        self._on_line = on_line
        self.running = False
        self._reset()

    def _reset(self):
        self.layer = 0
        self.layers = 0
        self.deg = 0.0
        self.deg_step = 1.0
        self.sweep = 340.0
        self.z = 0.0
        self.z_step = 0.0
        self.direction = -1
        self.ref_deg = 340.0
        self._travelled = 0.0
        self._next_at = 0.0
        self.sensor = "ULTRASONIC"

    # -- commands ------------------------------------------------------
    def send(self, text):
        text = text.strip()
        upper = text.upper()
        if upper.startswith("SCAN_START:"):
            self._start(text.split(":", 1)[1])
        elif upper == "SCAN_STOP":
            if self.running:
                self.running = False
                self._on_line("[SCAN_ABORT] stopped by the operator")
            else:
                self._on_line("[SCAN_STATUS] phase=IDLE - nothing to stop")
        elif upper == "ESTOP":
            if self.running:
                self.running = False
                self._on_line("[SCAN_ABORT] emergency stop")
            self._on_line("[ESTOP] EMERGENCY STOP")
        elif upper == "SCAN_READ":
            self._on_line(f"[SCAN_READ] {self._distance(0.0, 0.0):.2f} mm "
                          f"({self.sensor})")
        elif upper.startswith("SET_SCAN_SENSOR:"):
            kind = upper.split(":", 1)[1].strip()
            if kind in ("ULTRASONIC", "ANALOG"):
                self.sensor = kind
                self._on_line(f"[SCAN_SENSOR] {kind}")
            else:
                self._on_line("[ERROR] SET_SCAN_SENSOR takes ULTRASONIC or ANALOG")
        elif upper == "SCAN_STATUS":
            phase = "SWEEP" if self.running else "IDLE"
            self._on_line(f"[SCAN_STATUS] phase={phase} sensor={self.sensor} "
                          f"layer={self.layer}/{self.layers} points=0 cal=0.00000,0.00")
        elif upper == "PING":
            self._on_line("PONG")
        return True

    def _start(self, payload):
        parts = payload.split(",")
        if len(parts) < 3:
            self._on_line("[ERROR] SCAN_START needs zStepMm,degStep,layers[,sweepDeg]")
            return
        self.z_step = float(parts[0])
        self.deg_step = float(parts[1])
        self.layers = int(parts[2])
        self.sweep = float(parts[3]) if len(parts) > 3 else 340.0
        self.layer = 1
        self.z = 0.0
        self.running = True
        self._next_at = time.monotonic()
        self._on_line(f"[SCAN_BEGIN] sensor={self.sensor} layers={self.layers} "
                      f"zStep={self.z_step:.2f} degStep={self.deg_step:.2f} "
                      f"sweep={self.sweep:.1f} fromZ=0.00")
        # The board turns to the RM switch first and sweeps from there. The
        # switch sits at the CW end, so the first layer runs BACKWARDS from
        # it and the next one comes back -- see _next_layer().
        self._on_line("[SCAN_SEEK] turning RM to its switch to reference the sweep...")
        self.ref_deg = self.sweep
        self._on_line(f"[SCAN_REF] RM on its switch at {self.ref_deg:.2f} deg "
                      f"- sweeping from here")
        self._begin_layer(-1)

    def _begin_layer(self, direction):
        self.direction = direction
        self.deg = self.ref_deg if direction < 0 else 0.0
        self._travelled = 0.0
        sign = "+" if direction > 0 else "-"
        self._on_line(f"[SCAN_LAYER] {self.layer}/{self.layers} z={self.z:.2f} mm "
                      f"dir={sign} from={self.deg:.2f}")

    def _next_layer(self):
        """Alternates direction, and re-references whenever a layer ends on
        the switch -- which is what the board does, and why angle error
        cannot accumulate over a tall scan."""
        if self.direction > 0:
            self.ref_deg = self.sweep        # the switch is where the switch is
            self._on_line(f"[SCAN_REF] RM back on its switch at "
                          f"{self.ref_deg:.2f} deg")
        if self.layer >= self.layers:
            self.running = False
            self._on_line(f"[SCAN_DONE] {self.layers} layers")
            return
        self.layer += 1
        self.z += self.z_step
        self._begin_layer(-self.direction)

    # -- the model ------------------------------------------------------
    def _distance(self, deg, z):
        rad = math.radians(deg)
        r = 300.0 + 90.0 * math.cos(rad) + 40.0 * math.sin(2 * rad)
        r += 0.25 * z                                   # walls lean outward
        if 150.0 <= deg <= 172.0:                       # a doorway
            r += 160.0
        if random.random() < 0.02:                      # the odd lost echo
            return -1.0
        return r + random.gauss(0.0, 1.5)

    # -- driven from the UI timer ---------------------------------------
    def poll(self):
        if not self.running:
            return
        now = time.monotonic()
        guard = 0
        while self.running and now >= self._next_at and guard < 400:
            guard += 1
            self._next_at += self.POINT_INTERVAL_S
            self._on_line(f"[SCAN_PT] {self.layer},{self.deg:.2f},"
                          f"{self._distance(self.deg, self.z):.2f}")
            self.deg += self.direction * self.deg_step
            self._travelled += self.deg_step
            # A return leg ends on the SWITCH, not on a degree count -- that
            # is the whole point of turning back to it, and it is why the
            # start angle cannot drift over a tall scan. An outward leg has
            # nothing to stop it but the count.
            if self.direction > 0 and self.deg >= self.sweep:
                self._next_layer()
            elif self._travelled > self.sweep:
                self._next_layer()
