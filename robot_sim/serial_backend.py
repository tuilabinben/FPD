"""Optional pyserial import, isolated so the rest of the app never has to
guard the import itself."""

try:
    import serial
    import serial.tools.list_ports as list_ports
    HAS_SERIAL = True
    SerialException = serial.SerialException
except ImportError:                                   # pragma: no cover
    serial = None
    list_ports = None
    HAS_SERIAL = False

    class SerialException(Exception):
        """Stand-in so `except SerialException` still parses without pyserial."""


def available_ports():
    """List of COM/tty device names, or [] when pyserial is missing."""
    if not HAS_SERIAL:
        return []
    return [p.device for p in list_ports.comports()]


def open_port(port, baudrate, timeout=0.05, write_timeout=0.5):
    if not HAS_SERIAL:
        raise SerialException("pyserial is not installed")
    # write_timeout matters as much as the read timeout: pyserial's default
    # is None, which blocks forever on a stalled USB link. send() runs on
    # the Tk main thread, so an unbounded write() freezes the whole GUI
    # until the OS driver gives up -- with this set, write() raises instead
    # and send()'s existing except-clause disconnects cleanly.
    return serial.Serial(port=port, baudrate=int(baudrate),
                         timeout=timeout, write_timeout=write_timeout)
