"""High-DPI support.

WHY THE UI LOOKED LOW-RESOLUTION
-------------------------------
Tk declares itself DPI-unaware. On a Windows display running above 100%
scaling — which is every modern laptop, usually 125% or 150% — the OS
therefore does NOT let the app draw at native resolution. It lets the app
render a small window at 96 DPI and then BITMAP-STRETCHES the result to
the real size.

So every line, every glyph and every one of our anti-aliased corners was
being drawn correctly and then blown up with nearest-neighbour-ish
filtering. That is what "low res" was: not the drawing, the stretching
after it. Nothing done inside the app could have fixed it, because the
damage happened after Tk had finished.

`enable()` tells Windows the process handles DPI itself. Windows then
stops stretching and hands over the real pixel grid. Two consequences,
both of which have to be dealt with here:

  1. Text would come out tiny, because Tk still assumes 72 points per
     inch. `tk scaling` fixes that for anything sized in points.
  2. Anything sized in PIXELS — our canvas widgets, which are given
     explicit widths — would stay physically small. `px()` scales those.

The result is a UI that draws at the monitor's real resolution instead of
being upscaled into it.
"""

import sys

#: Multiply hard-coded pixel dimensions by this. 1.0 until enable() runs,
#: so importing this module has no effect on its own.
SCALE = 1.0

#: The DPI we were designed against. Windows' "100%" setting.
BASE_DPI = 96.0


def enable(root=None):
    """Declares DPI awareness and computes SCALE. Safe to call anywhere;
    returns the scale factor.

    Must run BEFORE the first widget is created, or Tk caches the old
    metrics and the window ends up half-scaled.
    """
    global SCALE

    if sys.platform == "win32":
        try:
            import ctypes
            # 2 = PROCESS_PER_MONITOR_DPI_AWARE. Preferred, because it
            # also handles being dragged to a second monitor with
            # different scaling.
            ctypes.windll.shcore.SetProcessDpiAwareness(2)
        except Exception:
            try:
                # Windows 7/8 fallback: system-wide awareness only.
                ctypes.windll.user32.SetProcessDPIAware()
            except Exception:
                # Not Windows, or the call is unavailable. The app still
                # runs; it just stays blurry on a scaled display, which
                # is exactly where it was before.
                return SCALE

    if root is not None:
        try:
            dpi = root.winfo_fpixels("1i")
            if dpi > 0:
                SCALE = max(1.0, dpi / BASE_DPI)
                # Tk sizes fonts in points and assumes 72 per inch. Telling
                # it the real value is what keeps text crisp AND correctly
                # sized rather than tiny.
                root.tk.call("tk", "scaling", dpi / 72.0)
        except Exception:
            pass
    return SCALE


def px(value):
    """Scales a pixel dimension that was written for a 96-DPI display.

    Rounded to whole pixels: a canvas widget given a fractional width gets
    a fractional layout, and the half-pixel seams that produces are their
    own kind of blurry.
    """
    return int(round(value * SCALE))


def font(spec):
    """Returns the font spec UNCHANGED. Deliberately a no-op.

    This used to multiply the point size by SCALE, which was wrong and
    produced text roughly 1.5x too large on a scaled display: `tk scaling`
    (set in enable() above) ALREADY makes Tk resolve point sizes against
    the real DPI, so applying SCALE on top scaled everything twice. The
    symptom was labels overflowing their buttons.

    Kept as an identity function rather than deleted so the call sites
    still read as "this size is DPI-managed", and so any new one that
    reaches for it cannot reintroduce the double scaling.

    Sizes in POINTS (positive numbers, which is what this app uses) scale
    automatically. Only PIXEL dimensions need px().
    """
    return spec
