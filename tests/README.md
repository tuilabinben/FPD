# Tests

Two suites, one command:

```
tests/run_tests.sh
```

**`firmware_check.cpp`** compiles `RobotMotionController_v9_ClearCore.ino`
against `stub/ClearCore.h` — a desktop shim, not an emulator — and asserts
on the text the board sends back. The stub captures `Serial.println` into
`OUT` for exactly that reason: an earlier version made `println` a no-op,
which silently turned every `saw(...)` assertion into a tautology. If you
extend the stub, keep that capture working.

**`python_check.py`** exercises the GUI's logic through `tkstub/`, a
headless stand-in for Tk. Widgets record their tree and `StringVar`s
really hold values, which is enough to drive the dialogs; nothing is
rendered. Two stub details are load-bearing:

* `Widget.__getitem__` raises `KeyError` for unknown options. A version
  that returned `None` made `"x" in widget` fall back to iteration that
  never raised `IndexError`, and the tests hung.
* `after_idle` runs the callback immediately, so deferred wiring is
  actually exercised rather than skipped.

`_tmp/` is scratch space for the file-persistence checks and can be
deleted at any time.
