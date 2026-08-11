"""Wheel and trackpad scrolling for the scrollable canvases.

Scrolling arrives in three different shapes and the differences are not
cosmetic — get one wrong and the surface either never moves or lurches:

* **MouseWheel** — one event per wheel notch on Windows, where the delta
  is a multiple of 120. On macOS the same event carries a small raw delta
  (1, 2, 3...) and a trackpad swipe fires dozens of them in a row.
* **Button-4 / Button-5** — X11's wheel. One event per notch, no delta.
* **TouchpadScroll** — a two-finger swipe under Tk 9, a separate event
  type that does not exist under Tk 8.6.

A note on Tk versions, because it cost a long debugging session: the
Homebrew **Tk 9.0.4** build on macOS generates none of these events for a
trackpad — not MouseWheel, not TouchpadScroll — while still delivering
Motion normally. Tk 8.6 on the same machine reports the swipe as a stream
of small MouseWheel deltas. Nothing here can compensate for an event that
is never sent, so the app needs a Tk 8.6 build to scroll on a trackpad.
"""

# One notch of a real wheel. Windows sends 120 per notch; macOS sends a
# small raw count, so a notch there is a handful of these.
NOTCH = 120

# How far one unit of a fine-grained (macOS) delta scrolls. A trackpad
# swipe arrives as a long stream of ±1s, so this is per-event and must
# stay small or a single swipe throws the page to the bottom.
FINE_DELTA_PIXELS = 16

# A wheel notch on X11, which reports no magnitude at all.
NOTCH_PIXELS = 60


def scroll_pixels(canvas, dy):
    """Scroll a canvas by `dy` pixels, positive being downward.

    `yview_scroll` can't do this: a canvas only understands "units" and
    "pages", and one unit is a tenth of the window — far too coarse for a
    trackpad, which reports a few pixels at a time. So the pixel delta is
    converted to a fraction of the scrollregion instead.
    """
    box = canvas.bbox("all")
    if not box:
        return
    total = box[3] - box[1]
    if total <= 0:
        return
    first = canvas.yview()[0]
    canvas.yview_moveto(min(1.0, max(0.0, first + dy / total)))


def wheel_scroll(canvas, event):
    """Scroll a canvas for one MouseWheel / Button-4 / Button-5 event."""
    num = getattr(event, "num", 0)
    if num == 4:
        scroll_pixels(canvas, -NOTCH_PIXELS)
        return
    if num == 5:
        scroll_pixels(canvas, NOTCH_PIXELS)
        return

    try:
        delta = int(event.delta)
    except (AttributeError, TypeError, ValueError):
        return
    if delta == 0:
        return
    if abs(delta) >= NOTCH:
        # Windows / Tk 9: whole notches, scaled to a readable jump.
        scroll_pixels(canvas, -(delta / NOTCH) * NOTCH_PIXELS)
    else:
        # macOS: fine-grained, many events per gesture.
        scroll_pixels(canvas, -delta * FINE_DELTA_PIXELS)


def touchpad_scroll(canvas, event):
    """Scroll a canvas for one Tk 9 TouchpadScroll event.

    Tk packs both axes into `delta`: x in the low 16 bits, y in the high
    16 bits, each a signed 16-bit pixel count. Only y is used — the page
    doesn't scroll sideways.
    """
    try:
        packed = int(event.delta)
    except (AttributeError, TypeError, ValueError):
        return

    def signed(half):
        return half - 0x10000 if half >= 0x8000 else half

    dy = signed((packed >> 16) & 0xFFFF)
    if dy:
        scroll_pixels(canvas, -dy)
