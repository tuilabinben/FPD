"""COM-port discovery, connect/disconnect, and the TX/RX pump."""

from tkinter import messagebox

from .. import serial_backend
from ..config import (
    DEFAULT_COM_PORT,
    PING_TIMEOUT_MS,
    SERIAL_POLL_MS,
)
from ..theme import ACCENT_GREEN, ACCENT_ORANGE, ACCENT_RED, TEXT_MUTED
from ..widgets import set_led

NO_PORT_LABEL = "No COM ports"

# TELEMETRY IS PARSED, NEVER LOGGED.
#
# The board reports the pose every 50 ms while an axis is held, and again
# throughout a P2P run. Writing those into the event log cost more than the
# jog itself: 20 inserts a second, each with a see("end") that forces Tk to
# scroll and repaint, and — once the 800-line cap is reached, which takes
# about 40 seconds of jogging — an index scan and a delete on every one of
# them. The arm kept moving, because the board drives it; what lagged was
# the readout, and it got worse the longer the key was held.
#
# They are still parsed, so the readout is as live as it ever was. The same
# rule the TX side already follows: JOG_HB goes out with log_tx=False for
# exactly this reason, and the RX side simply never got the same treatment.
# The scanner learned it too — see the [SCAN_PT] note in its _on_line().
#
# Anything the operator would want to READ still logs. This list is only for
# lines that repeat many times a second and say the same thing each time.
# [SCAN_PT] too: 341 points a layer. Store takes them, plot draws on a
# timer, the log would add a line per point nobody reads.
TELEMETRY_PREFIXES = ("[JOG POS]", "[CLEARCORE POS]", "[SCAN_PT]")


def is_telemetry(line):
    return line.startswith(TELEMETRY_PREFIXES)


class SerialLinkMixin:
    def refresh_com_ports(self):
        if not serial_backend.HAS_SERIAL:
            self.log("WARNING: the 'pyserial' library is not installed. "
                     "Run: pip install pyserial", tag="error")
            self.com_combo["values"] = [NO_PORT_LABEL]
            self.com_combo.set(NO_PORT_LABEL)
            return

        ports = serial_backend.available_ports()
        if not ports:
            self.com_combo["values"] = [NO_PORT_LABEL]
            self.com_combo.set(NO_PORT_LABEL)
            self.log("No COM ports found.")
            return

        previous = self.com_var.get()
        self.com_combo["values"] = ports
        # Keep user's current selection if it survived rescan, instead of
        # always snapping back to default port.
        if previous in ports:
            self.com_combo.set(previous)
        elif DEFAULT_COM_PORT in ports:
            self.com_combo.set(DEFAULT_COM_PORT)
        else:
            self.com_combo.set(ports[0])
        self.log(f"Found {len(ports)} COM port(s): {', '.join(ports)}")

    def toggle_connection(self):
        if self.is_connected:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        if not serial_backend.HAS_SERIAL:
            messagebox.showerror(
                "Missing library",
                "The 'pyserial' library is not installed.\nRun: pip install pyserial")
            return

        port = self.com_var.get()
        baud = self.baud_var.get()
        if not port or port == NO_PORT_LABEL:
            messagebox.showwarning("Warning", "Please select a valid COM port.")
            return

        try:
            self.ser = serial_backend.open_port(port, baud)
        except (serial_backend.SerialException, ValueError, OSError) as e:
            self.log(f"Error connecting to {port}: {e}", tag="error")
            messagebox.showerror(
                "Connection error",
                f"Could not open {port}.\n"
                "Another program may already have the port open.")
            return

        self.is_connected = True
        self._missed_beats = 0
        # Invalidates any RX pump left over from a previous session, so two
        # listener loops can never run at once after a reconnect.
        self._rx_generation += 1

        set_led(self.com_led_card, "OPEN", ACCENT_GREEN)
        set_led(self.hw_led_card, "WAITING...", ACCENT_ORANGE)
        # Unknown until board answers PLC_STATUS.
        self._plc_link_lost()

        self.btn_connect.set_config("DISCONNECT", ACCENT_RED, icon="❌")
        self.status_var.set(f"COM OPEN — Waiting for ClearCore handshake on {port}...")
        self.log(f"Opened {port} at {baud} bps. Sending PING to ClearCore...")

        self._listen_hardware_response(self._rx_generation)
        self.send("PING")
        self._schedule("_ping_timeout_job", PING_TIMEOUT_MS, self._on_ping_timeout)
        self._schedule_next_heartbeat()

    def disconnect_serial(self):
        self.send("BYE")
        self._schedule("_disconnect_job", 100, self._do_disconnect)

    def _do_disconnect(self):
        self._disconnect_job = None
        self._close_port()
        self.is_connected = False
        self.hw_confirmed = False
        self._missed_beats = 0
        self._rx_generation += 1

        self._cancel_jobs("_ping_timeout_job", "_heartbeat_job", "_hb_blink_job")

        # Safety: treats any disconnect as an all-stop.
        self.is_running = False
        self.is_homing = False
        self._cancel_jobs("anim_job", "_jog_sim_job")
        self._release_all_jog_axes(send_stop=False)   # port is already closed
        self._set_motion_locked(False)

        set_led(self.com_led_card, "CLOSED", ACCENT_RED)
        set_led(self.hw_led_card, "NO SIGNAL", ACCENT_RED)
        self._plc_link_lost()

        self.btn_connect.set_config("CONNECT", ACCENT_GREEN, icon="🔌")
        self.status_var.set("DISCONNECTED — Hardware Offline")
        self.log("Sent BYE and disconnected. All motion stopped.")

    def _close_port(self):
        """Closes the port, swallowing driver errors (a yanked USB cable
        makes close() itself raise — old code let that crash the app)."""
        if self.ser is None:
            return
        try:
            if self.ser.is_open:
                self.ser.close()
        except Exception as e:                        # pragma: no cover
            self.log(f"Error closing the port: {e}", tag="warn")
        finally:
            self.ser = None

    def send(self, msg: str, log_tx=True):
        """Writes one newline-terminated command. No-op when offline, so
        every caller can send unconditionally."""
        line = msg.strip()
        if not line:
            return
        if not (self.ser and getattr(self.ser, "is_open", False)):
            return
        try:
            self.ser.write((line + "\n").encode("utf-8"))
            if log_tx:
                self.log(f">> {line}", tag="tx")
        except Exception as e:
            self.log(f"Serial write failed: {e}", tag="error")
            # Write failure means link is gone — drop it instead of
            # silently pretending to still be connected.
            self._on_link_failure()

    _send_serial = send      # backwards-compatible alias

    def _on_link_failure(self):
        if self.is_connected:
            self.log("Lost the physical COM connection — disconnecting.", tag="error")
            self._do_disconnect()

    # Caps lines consumed per tick so a flooded serial buffer (e.g. from
    # Ethernet socket cycling) can't block the Tkinter event loop for
    # hundreds of milliseconds.
    _RX_MAX_LINES_PER_TICK = 20

    def _listen_hardware_response(self, generation):
        if generation != self._rx_generation or not self.is_connected:
            return
        if self.ser and getattr(self.ser, "is_open", False):
            try:
                lines_read = 0
                while self.ser.in_waiting > 0 and lines_read < self._RX_MAX_LINES_PER_TICK:
                    raw = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if not raw:
                        continue
                    lines_read += 1
                    if not is_telemetry(raw):
                        tag = "rx"
                        if raw.startswith("[ERROR]"):
                            tag = "error"
                        elif raw.startswith("[WARN]"):
                            tag = "warn"
                        self.log(f"<< {raw}", tag=tag)
                    self._parse_hardware_response(raw)
            except Exception:
                pass
        self.root.after(SERIAL_POLL_MS, self._listen_hardware_response, generation)
