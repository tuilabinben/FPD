"""Canvas drawing primitives — anti-aliased rounded surfaces.

WHY THE OLD CORNERS LOOKED JAGGED
---------------------------------
The previous version drew rounded rects with
`create_polygon(..., smooth=True)`. That is not a rounded rectangle: Tk
fits a cardinal spline through the points, which bulges the straight
edges inward and produces a slightly lumpy outline. On top of that, the
Tk canvas has no anti-aliasing at all, so every curve came out as hard
stair-steps. Stacking six offset copies of that shape to fake a shadow
multiplied the problem into visible banding.

WHAT IT DOES NOW
----------------
Corners are rendered as an IMAGE, drawn at 4x size and downsampled with
LANCZOS. Averaging 16 subpixels per output pixel is what produces a
genuinely smooth edge — it is real anti-aliasing rather than a smoother
curve that still aliases. Images are cached by their exact parameters,
so hover redraws are a dictionary lookup, not a re-render.

That path needs Pillow. Without it we fall back to arcs plus rectangles,
which is still true circular geometry (unlike the spline) and looks
markedly better than what was there — just without the soft edge.

The style is deliberately FLAT: solid fills, one hairline border, no
shadow stacks. The depth cue is the fill colour changing on hover and
press, which costs nothing visually and cannot band.
"""

import tkinter as tk

from ..theme import mix, shade

try:                                        # pragma: no cover - env dependent
    from PIL import Image, ImageDraw, ImageTk
    # Pillow moved the resampling constants under Image.Resampling in 10
    # and keeps the old aliases only for now, so resolve it once rather
    # than let a future release turn smooth corners into an AttributeError
    # at draw time.
    _LANCZOS = getattr(getattr(Image, "Resampling", Image), "LANCZOS")
    _HAVE_PIL = True
except Exception:                           # pragma: no cover
    _HAVE_PIL = False
    _LANCZOS = None

#: Supersampling factor. 4 means each output pixel averages 16 rendered
#: ones — past this the improvement is not visible but the cost is
#: quadratic, and these images are rebuilt on every hover.
SUPERSAMPLE = 4

#: Rendered images, keyed by their full parameter tuple. Tk also requires
#: a live Python reference to every PhotoImage or it is garbage collected
#: and the canvas silently goes blank, so this cache is load-bearing, not
#: just an optimisation.
_IMAGE_CACHE = {}
_CACHE_LIMIT = 512


def has_antialiasing():
    """True when Pillow is available and corners will be smooth."""
    return _HAVE_PIL


def _cache(key, build):
    img = _IMAGE_CACHE.get(key)
    if img is None:
        if len(_IMAGE_CACHE) > _CACHE_LIMIT:
            _IMAGE_CACHE.clear()
        img = build()
        _IMAGE_CACHE[key] = img
    return img


def _rgb(color):
    c = color.lstrip("#")
    return tuple(int(c[i:i + 2], 16) for i in (0, 2, 4))


# ══════════════════════════════════════════════════════════════════════
# Anti-aliased image builders
# ══════════════════════════════════════════════════════════════════════
def rounded_image(w, h, radius, fill, surface, border=None, border_w=0):
    """Anti-aliased rounded rectangle as a Tk image.

    Composited onto `surface` rather than left transparent: the widget's
    background is a known flat colour, so baking it in avoids relying on
    RGBA compositing and renders identically everywhere.
    """
    w, h = max(1, int(w)), max(1, int(h))
    key = ("rect", w, h, round(radius, 2), fill, surface, border, border_w)

    def build():
        s = SUPERSAMPLE
        img = Image.new("RGB", (w * s, h * s), _rgb(surface))
        d = ImageDraw.Draw(img)
        d.rounded_rectangle(
            [0, 0, w * s - 1, h * s - 1],
            radius=max(0, radius) * s,
            fill=_rgb(fill),
            outline=_rgb(border) if border else None,
            width=int(border_w * s) if border else 0,
        )
        return ImageTk.PhotoImage(img.resize((w, h), _LANCZOS))

    return _cache(key, build)


def circle_image(size, fill, surface, border=None, border_w=0):
    """Anti-aliased filled circle as a Tk image."""
    size = max(1, int(size))
    key = ("circle", size, fill, surface, border, border_w)

    def build():
        s = SUPERSAMPLE
        img = Image.new("RGB", (size * s, size * s), _rgb(surface))
        d = ImageDraw.Draw(img)
        d.ellipse(
            [0, 0, size * s - 1, size * s - 1],
            fill=_rgb(fill),
            outline=_rgb(border) if border else None,
            width=int(border_w * s) if border else 0,
        )
        return ImageTk.PhotoImage(img.resize((size, size), _LANCZOS))

    return _cache(key, build)


# ══════════════════════════════════════════════════════════════════════
# Fallback geometry — used only when Pillow is missing
# ══════════════════════════════════════════════════════════════════════
def _rounded_arcs(canvas, x1, y1, x2, y2, r, fill, outline, width):
    """True circular corners from four arcs plus two rectangles.

    Not anti-aliased, but geometrically correct — unlike the spline this
    replaced, which pinched the straight edges.
    """
    r = max(0, min(r, (x2 - x1) / 2, (y2 - y1) / 2))
    d = r * 2
    opts = {"fill": fill, "outline": fill, "width": 0}
    if r > 0:
        canvas.create_arc(x1, y1, x1 + d, y1 + d, start=90, extent=90,
                          style="pieslice", **opts)
        canvas.create_arc(x2 - d, y1, x2, y1 + d, start=0, extent=90,
                          style="pieslice", **opts)
        canvas.create_arc(x1, y2 - d, x1 + d, y2, start=180, extent=90,
                          style="pieslice", **opts)
        canvas.create_arc(x2 - d, y2 - d, x2, y2, start=270, extent=90,
                          style="pieslice", **opts)
    canvas.create_rectangle(x1 + r, y1, x2 - r, y2, fill=fill, outline=fill, width=0)
    canvas.create_rectangle(x1, y1 + r, x2, y2 - r, fill=fill, outline=fill, width=0)

    if outline and width:
        canvas.create_line(x1 + r, y1, x2 - r, y1, fill=outline, width=width)
        canvas.create_line(x1 + r, y2, x2 - r, y2, fill=outline, width=width)
        canvas.create_line(x1, y1 + r, x1, y2 - r, fill=outline, width=width)
        canvas.create_line(x2, y1 + r, x2, y2 - r, fill=outline, width=width)
        if r > 0:
            for bbox, start in (((x1, y1, x1 + d, y1 + d), 90),
                                ((x2 - d, y1, x2, y1 + d), 0),
                                ((x1, y2 - d, x1 + d, y2), 180),
                                ((x2 - d, y2 - d, x2, y2), 270)):
                canvas.create_arc(*bbox, start=start, extent=90, style="arc",
                                  outline=outline, width=width)


# ══════════════════════════════════════════════════════════════════════
# Public painters — what the widgets actually call
# ══════════════════════════════════════════════════════════════════════
def paint_rounded(canvas, x, y, w, h, radius, fill, surface,
                  border=None, border_w=0, tag=None):
    """Fills a rounded rect on `canvas`, anti-aliased where possible.

    The PhotoImage is stashed on the canvas (`_img_refs`) because Tk keeps
    only a weak reference to images: without this the picture is collected
    the moment the local goes out of scope and the widget renders blank.
    """
    if _HAVE_PIL:
        img = rounded_image(w, h, radius, fill, surface, border, border_w)
        refs = getattr(canvas, "_img_refs", None)
        if refs is None:
            refs = canvas._img_refs = []
        refs.append(img)
        kw = {"image": img, "anchor": "nw"}
        if tag:
            kw["tags"] = tag
        canvas.create_image(x, y, **kw)
    else:
        _rounded_arcs(canvas, x, y, x + w, y + h, radius, fill,
                      border, border_w)


def paint_circle(canvas, x, y, size, fill, surface, border=None, border_w=0):
    if _HAVE_PIL:
        img = circle_image(size, fill, surface, border, border_w)
        refs = getattr(canvas, "_img_refs", None)
        if refs is None:
            refs = canvas._img_refs = []
        refs.append(img)
        canvas.create_image(x, y, image=img, anchor="nw")
    else:
        canvas.create_oval(x, y, x + size, y + size, fill=fill,
                           outline=border or fill,
                           width=border_w if border else 0)


def clear(canvas):
    """Wipes the canvas AND drops its image references.

    Both halves matter: `delete("all")` alone leaks a PhotoImage per
    redraw, and on a jog pad that redraws on every hover the leak is
    continuous.
    """
    canvas.delete("all")
    if hasattr(canvas, "_img_refs"):
        canvas._img_refs = []


# ══════════════════════════════════════════════════════════════════════
# Legacy shims — kept so nothing that still imports these breaks
# ══════════════════════════════════════════════════════════════════════
def rounded_rect_points(x1, y1, x2, y2, r):
    return [x1 + r, y1, x2 - r, y1, x2, y1, x2, y1 + r,
            x2, y2 - r, x2, y2, x2 - r, y2, x1 + r, y2,
            x1, y2, x1, y2 - r, x1, y1 + r, x1, y1]


def draw_rounded_rect(canvas, x1, y1, x2, y2, r, **kwargs):
    fill = kwargs.get("fill") or ""
    outline = kwargs.get("outline") or None
    width = kwargs.get("width", 0)
    if fill:
        _rounded_arcs(canvas, x1, y1, x2, y2, r, fill, outline, width)
    return None


def draw_bevel_rect(canvas, x1, y1, x2, y2, r, color):
    paint_rounded(canvas, x1, y1, x2 - x1, y2 - y1, r, color, color)


def draw_neumorph_raised(canvas, x1, y1, x2, y2, r, surface, spread=1.0,
                         tint=None):
    face = tint if tint is not None else shade(surface, 1.14)
    paint_rounded(canvas, x1, y1, x2 - x1, y2 - y1, r, face, surface,
                  border=mix(face, "#ffffff", 0.06), border_w=1)
    return face


def draw_neumorph_inset(canvas, x1, y1, x2, y2, r, surface, spread=1.0,
                        fill=None):
    face = fill if fill is not None else shade(surface, 0.82)
    paint_rounded(canvas, x1, y1, x2 - x1, y2 - y1, r, face, surface,
                  border=mix(face, "#000000", 0.25), border_w=1)
    return face


def draw_neumorph_circle_raised(canvas, x1, y1, x2, y2, surface, tint=None):
    face = tint if tint is not None else shade(surface, 1.14)
    paint_circle(canvas, x1, y1, x2 - x1, face, surface)
    return face


def draw_neumorph_circle_inset(canvas, x1, y1, x2, y2, surface, fill=None):
    face = fill if fill is not None else shade(surface, 0.82)
    paint_circle(canvas, x1, y1, x2 - x1, face, surface)
    return face


_TK = tk  # keep the import meaningful for type checkers
