"""High-DPI support.

WHY UI LOOKED LOW-RES: Tk declares itself DPI-unaware. On Windows display above 100% scaling
(every modern laptop, usually 125-150%), OS does NOT let app draw at native res — renders
small window at 96 DPI then BITMAP-STRETCHES to real size.

Every line/glyph/AA corner drawn correctly then blown up w/ nearest-neighbour-ish filtering.
"Low res" = the stretching, not the drawing. Nothing inside app could fix it — damage happens
after Tk finished.

`enable()` tells Windows process handles DPI itself. Windows stops stretching, hands over real
pixel grid. Two consequences to handle:
  1. Text comes out tiny — Tk still assumes 72 points/inch. `tk scaling` fixes for point sizes.
  2. PIXEL-sized things (canvas widgets, explicit widths) stay physically small. `px()` scales
     those.

Result: UI draws at monitor's real resolution instead of upscaled into it.
"""

import sys

#: Multiply hard-coded pixel dims by this. 1.0 until enable() runs — importing module alone
#: has no effect.
SCALE = 1.0

#: DPI designed against. Windows "100%" setting.
BASE_DPI = 96.0


def enable(root=None):
    """Declares DPI awareness, computes SCALE. Safe to call anywhere, returns scale factor.

    Must run BEFORE first widget created, or Tk caches old metrics and window ends up
    half-scaled.
    """
    global SCALE

    if sys.platform == "win32":
        try:
            import ctypes
            # 2 = PROCESS_PER_MONITOR_DPI_AWARE. Preferred: also handles drag to 2nd monitor
            # w/ different scaling.
            ctypes.windll.shcore.SetProcessDpiAwareness(2)
        except Exception:
            try:
                # Windows 7/8 fallback: system-wide awareness only.
                ctypes.windll.user32.SetProcessDPIAware()
            except Exception:
                # Not Windows, or call unavailable. App still runs, stays blurry on scaled
                # display — same as before.
                return SCALE

    if root is not None:
        try:
            dpi = root.winfo_fpixels("1i")
            if dpi > 0:
                SCALE = max(1.0, dpi / BASE_DPI)
                # Tk sizes fonts in points, assumes 72/inch. Real value keeps text crisp AND
                # correctly sized instead of tiny.
                root.tk.call("tk", "scaling", dpi / 72.0)
        except Exception:
            pass
    return SCALE


def px(value):
    """Scales a pixel dimension written for 96-DPI display.

    Rounded to whole pixels: fractional width -> fractional layout -> half-pixel seams, its own
    kind of blurry.
    """
    return int(round(value * SCALE))


def font(spec):
    """Returns font spec UNCHANGED. Deliberately no-op.

    Used to multiply point size by SCALE — wrong, produced text ~1.5x too large on scaled
    display: `tk scaling` (set in enable() above) ALREADY resolves point sizes against real
    DPI, so SCALE on top double-scaled. Symptom: labels overflowing buttons.

    Kept as identity fn rather than deleted so call sites still read "DPI-managed", and no new
    call site can reintroduce double scaling.

    Sizes in POINTS (what this app uses) scale automatically. Only PIXEL dims need px().
    """
    return spec
