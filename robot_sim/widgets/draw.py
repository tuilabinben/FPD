"""Canvas drawing primitives — anti-aliased rounded surfaces.

OLD CORNERS JAGGED: old code used create_polygon(..., smooth=True). Not
real rounded rect — Tk fits cardinal spline thru points, bulges straight
edges inward, lumpy outline. Tk canvas has zero anti-aliasing, curves
came out stair-stepped. Stacking six offset copies for shadow
multiplied into visible banding.

NOW: corners rendered as IMAGE, drawn 4x size, downsampled w/ LANCZOS.
Averaging 16 subpixels/output pixel = real anti-aliasing, not just
smoother curve that still aliases. Images cached by exact params —
hover redraw = dict lookup, not re-render.

Needs Pillow. Without it: fallback to arcs+rectangles — still true
circular geometry (unlike spline), looks much better, just no soft edge.

Style deliberately FLAT: solid fills, one hairline border, no shadow
stacks. Depth cue = fill colour change on hover/press — free, no banding.
"""

import tkinter as tk

from ..theme import mix, shade

try:                                        # pragma: no cover - env dependent
    from PIL import Image, ImageDraw, ImageTk
    # Pillow moved resampling consts under Image.Resampling in 10, old
    # aliases kept only for now — resolve once, avoid future AttributeError
    # at draw time.
    _LANCZOS = getattr(getattr(Image, "Resampling", Image), "LANCZOS")
    _HAVE_PIL = True
except Exception:                           # pragma: no cover
    _HAVE_PIL = False
    _LANCZOS = None

#: Supersampling factor. 4 = each output pixel averages 16 rendered ones —
#: past this, gain invisible but cost quadratic; images rebuilt every hover.
SUPERSAMPLE = 4

#: Rendered images, keyed by full param tuple. Tk needs live Python ref to
#: every PhotoImage or GC collects it and canvas goes blank silently —
#: cache is load-bearing, not just optimisation.
_IMAGE_CACHE = {}
_CACHE_LIMIT = 512


def has_antialiasing():
    """True if Pillow available, corners will be smooth."""
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


def rounded_image(w, h, radius, fill, surface, border=None, border_w=0):
    """Anti-aliased rounded rectangle as Tk image.

    Composited onto `surface`, not left transparent: widget background is
    known flat colour — baking in avoids RGBA compositing, renders
    identically everywhere.
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


# fallback geometry, used only if Pillow missing
def _rounded_arcs(canvas, x1, y1, x2, y2, r, fill, outline, width):
    """True circular corners from four arcs + two rectangles.

    Not anti-aliased, but geometrically correct — unlike spline it
    replaced, which pinched straight edges.
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


def paint_rounded(canvas, x, y, w, h, radius, fill, surface,
                  border=None, border_w=0, tag=None):
    """Fills rounded rect on `canvas`, anti-aliased where possible.

    PhotoImage stashed on canvas (`_img_refs`): Tk keeps only weak ref to
    images — without this, picture collected moment local goes out of
    scope, widget renders blank.
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
    """Wipes canvas AND drops image refs.

    Both halves matter: `delete("all")` alone leaks a PhotoImage per
    redraw — on jog pad redrawing every hover, leak is continuous.
    """
    canvas.delete("all")
    if hasattr(canvas, "_img_refs"):
        canvas._img_refs = []


# legacy shims, kept so old imports don't break
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


_TK = tk  # keeps import meaningful for type checkers
