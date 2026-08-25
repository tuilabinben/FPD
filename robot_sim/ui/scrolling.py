"""Wheel and trackpad scrolling for the scrollable canvases.

Scrolling arrives in three shapes, differences not cosmetic — get one
wrong, surface never moves or lurches:

* **MouseWheel** — one event per wheel notch on Windows, delta multiple of
  120. macOS: small raw delta (1,2,3...), trackpad swipe fires dozens in a row.
* **Button-4 / Button-5** — X11's wheel. One event per notch, no delta.
* **TouchpadScroll** — two-finger swipe under Tk 9, separate event type,
  doesn't exist under Tk 8.6.

Tk version note, cost a long debug session: Homebrew **Tk 9.0.4** on macOS
generates NONE of these events for trackpad — not MouseWheel, not
TouchpadScroll — while still delivering Motion normally. Tk 8.6 on same
machine reports swipe as stream of small MouseWheel deltas. Nothing here
compensates for event never sent — app needs Tk 8.6 build for trackpad scroll.
"""

# One notch of real wheel. Windows sends 120/notch; macOS sends small raw
# count, so a notch there is a handful of these.
NOTCH = 120

# How far one unit of fine-grained (macOS) delta scrolls. Trackpad swipe
# arrives as long stream of +/-1s, per-event, must stay small or single
# swipe throws page to bottom.
FINE_DELTA_PIXELS = 16

# Wheel notch on X11, which reports no magnitude at all.
NOTCH_PIXELS = 60


def scroll_pixels(canvas, dy):
    """Scroll canvas by `dy` pixels, positive downward.

    `yview_scroll` can't do this: canvas only understands "units"/"pages",
    one unit = tenth of window — far too coarse for trackpad (few pixels at
    a time). Pixel delta converted to fraction of scrollregion instead.
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
        # Windows/Tk 9: whole notches, scaled to readable jump.
        scroll_pixels(canvas, -(delta / NOTCH) * NOTCH_PIXELS)
    else:
        # macOS: fine-grained, many events per gesture.
        scroll_pixels(canvas, -delta * FINE_DELTA_PIXELS)


def touchpad_scroll(canvas, event):
    """Scroll canvas for one Tk 9 TouchpadScroll event.

    Tk packs both axes into `delta`: x in low 16 bits, y in high 16 bits,
    each signed 16-bit pixel count. Only y used — page doesn't scroll
    sideways.
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
