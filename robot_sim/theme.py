"""Active colour scheme, switchable at runtime.

HOW LIVE SWITCHING WORKS
------------------------
Every widget module does `from ..theme import PANEL_BG`, which copies the
VALUE into that module's namespace at import time. That is why the first
version of this needed a restart: rebinding `theme.PANEL_BG` alone left a
dozen other modules still holding the old string.

`apply_palette()` fixes that in three steps:

  1. Rebind every constant in THIS module from the new palette.
  2. Walk every already-imported `robot_sim.*` module and rebind any
     name that still holds the OLD value. Comparing against the old
     value — rather than just matching by name — is what makes this safe:
     a module that happens to define its own `BORDER` for an unrelated
     reason is left alone, because its value will not match.
  3. Tell the app to rebuild its widgets, since a Tk widget's colours are
     fixed at construction and cannot be restyled in place.

The alternative was `import theme` everywhere and `theme.PANEL_BG` at
every use site. That is arguably cleaner, but it is several hundred edits
across the UI and makes every colour reference noisier to read, for the
same end result.

DESIGN NOTE — what the colours are FOR
--------------------------------------
This is a machine controller, so colour carries meaning before style:

    ACCENT_RED     stop. Never used for anything else.
    ACCENT_ORANGE  warning / above-recommended. Never a primary accent.
    ACCENT_MINT    the primary accent — selection, focus, a live axis.
    axis colours   identity, kept distinguishable by lightness as well as
                   hue so they survive colour-vision deficiency.

The surface ladder (BG / PANEL_BG / SURFACE / SURFACE_HI) is what
separates a control from the card behind it. Flat design has no shadows,
so those steps do the work shadows used to.
"""

import sys

from .palettes import PALETTES, load_active_name, save_active_name


# ══════════════════════════════════════════════════════════════════════
# Colour maths — defined first, the constants below are derived from it
# ══════════════════════════════════════════════════════════════════════
def shade(hex_color, factor):
    """factor < 1 darkens, factor > 1 lightens (clamped per channel)."""
    hex_color = hex_color.lstrip("#")
    r, g, b = (int(hex_color[i:i + 2], 16) for i in (0, 2, 4))
    r, g, b = (max(0, min(255, int(c * factor))) for c in (r, g, b))
    return f"#{r:02x}{g:02x}{b:02x}"


def mix(color_a, color_b, t):
    """Linear blend: t=0 gives `color_a`, t=1 gives `color_b`."""
    a = color_a.lstrip("#")
    b = color_b.lstrip("#")
    t = max(0.0, min(1.0, t))
    out = []
    for i in (0, 2, 4):
        ca, cb = int(a[i:i + 2], 16), int(b[i:i + 2], 16)
        out.append(int(round(ca + (cb - ca) * t)))
    return "#{:02x}{:02x}{:02x}".format(*out)


def luminance(hex_color):
    """Perceived brightness, 0..255. Used by the palette self-test to
    assert that a scheme's surface ladder steps far enough."""
    r, g, b = (int(hex_color.lstrip("#")[i:i + 2], 16) for i in (0, 2, 4))
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


def _tone(base, target_lum):
    """`base` shifted toward white or black until it hits `target_lum`.

    Blending toward pure white/black keeps the hue recognisable while
    moving the lightness freely, which is what lets a tonal ramp be both
    unified (one colour family) and accessible (separated by lightness
    rather than hue). Binary search because luminance is not linear in
    the blend factor.
    """
    have = luminance(base)
    toward = "#ffffff" if target_lum > have else "#000000"
    lo, hi = 0.0, 1.0
    for _ in range(24):
        t = (lo + hi) / 2
        candidate = mix(base, toward, t)
        if (luminance(candidate) < target_lum) == (target_lum > have):
            lo = t
        else:
            hi = t
    return mix(base, toward, (lo + hi) / 2)


# ══════════════════════════════════════════════════════════════════════
# Constant binding
# ══════════════════════════════════════════════════════════════════════
#: Names this module owns. Only these are propagated on a palette change,
#: and only where the receiving module still holds the previous value.
COLOUR_NAMES = (
    "BG", "PANEL_BG", "SURFACE", "SURFACE_HI", "ENTRY_BG", "LED_BG",
    "BORDER", "BORDER_SOFT", "SHADOW_DARK", "SHADOW_LIGHT",
    "ACCENT_MINT", "ACCENT_CYAN", "ACCENT_GREEN", "ACCENT_RED",
    "ACCENT_ORANGE", "ACCENT_PURPLE", "INK_DARK", "INACTIVE_BG",
    "AXIS_X_COLOR", "AXIS_Y_COLOR", "AXIS_Z_COLOR",
    "ROT_COLOR", "ARM_COLOR", "ARM2_COLOR", "JZ_COLOR",
    "TEXT_LIGHT", "TEXT_MUTED", "TEXT_DIM",
    "WARN_BORDER", "ERROR_BORDER", "FOCUS_BORDER", "NEUTRAL_BORDER",
    "HI_CYAN", "HI_MINT", "HI_GREEN", "HI_PURPLE", "HI_ARM2", "HI_ROT",
)


def _bind(name):
    """Computes every colour constant for palette `name` into globals()."""
    p = PALETTES[name]
    g = globals()

    g["PALETTE_NAME"] = name
    g["PALETTE_LABEL"] = p["label"]
    g["IS_DARK"] = p["dark"]

    # surfaces
    g["BG"] = p["BG"]
    g["PANEL_BG"] = p["PANEL_BG"]
    g["SURFACE"] = p["SURFACE"]
    g["SURFACE_HI"] = p["SURFACE_HI"]
    g["ENTRY_BG"] = p["ENTRY_BG"]
    g["LED_BG"] = p["LED_BG"]
    g["BORDER"] = p["BORDER"]
    g["BORDER_SOFT"] = p["BORDER_SOFT"]
    g["INACTIVE_BG"] = p["SURFACE"]

    # accents. ACCENT_MINT is a legacy name meaning "the primary accent",
    # whatever colour the active scheme uses for it.
    g["ACCENT_MINT"] = p["ACCENT"]
    g["ACCENT_CYAN"] = p["ACCENT"]
    g["ACCENT_GREEN"] = p["ACCENT_GREEN"]
    g["ACCENT_RED"] = p["ACCENT_RED"]
    g["ACCENT_ORANGE"] = p["ACCENT_ORANGE"]
    g["ACCENT_PURPLE"] = p["ACCENT_PURPLE"]
    g["INK_DARK"] = p["ACCENT_ON"]

    # ── the motor / axis ramp ────────────────────────────────────────
    # Previously these were six independently chosen hues — red, green,
    # blue, amber, purple, teal — which made the readout row look like it
    # belonged to a different application than everything around it.
    #
    # They are now DERIVED from the scheme's two accents, so the whole UI
    # stays in one colour family. Separation still comes from lightness,
    # not hue, which is what keeps them distinguishable under red-green
    # colour blindness — and a tonal ramp does that naturally, so the
    # unified look and the accessibility requirement pull the same way
    # here rather than against each other.
    # TWO groups, not four separate colours:
    #
    #     A1M + A2M  share the primary accent    (the arms are a pair)
    #     RM  + ZM   share the secondary accent  (the base axes)
    #
    # Four distinct tones was still four things to tell apart in a row of
    # readouts. Grouping them by what they physically are means the colour
    # answers "which kind of axis" at a glance, and the label answers
    # "which one" — which is the division of labour that actually matches
    # how the readouts get scanned.
    #
    # The tones are solved for a TARGET LUMINANCE rather than mixed by a
    # fixed ratio. Fixed ratios were the first attempt and they do not
    # work: how far a 30% blend moves brightness depends entirely on where
    # the accent started, so schemes whose accents happened to be close in
    # lightness collapsed to a 2-unit separation — invisible, and worse
    # than the clashing hues it replaced.
    a1, a2 = p["ACCENT"], p["ACCENT_2"]
    if p["dark"]:
        # Bright: these sit on the darkest plane in the app (LED_BG), so
        # they can afford to be near the top of the range.
        arm_t, base_t = 214, 168
        c_targets = (206, 180, 154)
    else:
        # On a light card the tones must go DOWN, or they vanish.
        arm_t, base_t = 74, 116
        c_targets = (78, 106, 134)

    arms = _tone(a1, arm_t)
    base = _tone(a2, base_t)

    g["ARM_COLOR"] = arms          # A1M
    g["ARM2_COLOR"] = arms         # A2M — same colour, by design
    g["ROT_COLOR"] = base          # RM
    g["JZ_COLOR"] = base           # ZM  — same colour, by design

    g["AXIS_X_COLOR"] = _tone(a1, c_targets[0])
    g["AXIS_Y_COLOR"] = _tone(a2, c_targets[1])
    g["AXIS_Z_COLOR"] = _tone(a2, c_targets[2])

    g["TEXT_LIGHT"] = p["TEXT_LIGHT"]
    g["TEXT_MUTED"] = p["TEXT_MUTED"]
    g["TEXT_DIM"] = p["TEXT_DIM"]

    # Retained for stragglers; the flat style changes fills, not shadows.
    g["SHADOW_DARK"] = "#000000" if p["dark"] else "#9aa0aa"
    g["SHADOW_LIGHT"] = "#2b3037" if p["dark"] else "#ffffff"

    # Field states. Borders, not fills: the point is to draw the eye
    # without making the number itself harder to read.
    g["WARN_BORDER"] = p["ACCENT_ORANGE"]
    g["ERROR_BORDER"] = p["ACCENT_RED"]
    g["FOCUS_BORDER"] = p["ACCENT"]
    g["NEUTRAL_BORDER"] = p["BORDER"]

    # Highlights. Lighten on dark schemes, darken on light ones — a
    # "brighter" hover on a light theme has nowhere to go.
    lift = 1.18 if p["dark"] else 0.88
    g["HI_CYAN"] = shade(p["ACCENT"], lift)
    g["HI_MINT"] = shade(p["ACCENT"], lift)
    g["HI_GREEN"] = shade(p["ACCENT_GREEN"], lift)
    g["HI_PURPLE"] = shade(g["ARM_COLOR"], lift)
    g["HI_ARM2"] = shade(g["ARM2_COLOR"], lift)
    g["HI_ROT"] = shade(g["ROT_COLOR"], lift)


_bind(load_active_name())


# ── corner radii — not palette dependent ─────────────────────────────
# Generous and consistent. Corners are anti-aliased now, so a larger
# radius reads as soft instead of as a longer staircase.
RADIUS_SM = 8       # chips, keycaps
RADIUS_MD = 12      # buttons, fields
RADIUS_LG = 16      # jog pads
RADIUS_CARD = 14    # section cards

# ── type scale ───────────────────────────────────────────────────────
# Sizes are in POINTS, which Tk resolves against the real DPI once
# `tk scaling` is set — so these are NOT multiplied by the DPI factor
# anywhere. Doing both was the bug that made every label overflow its
# button.
#
# Six steps, each with one job. The previous code picked a size per call
# site, which drifted into eleven different sizes and is why some labels
# looked oversized next to their neighbours.
UI_FAMILY = "Segoe UI"
MONO_FAMILY = "Consolas"

FONT_TITLE = (UI_FAMILY, 11, "bold")     # section headings
FONT_LABEL = (UI_FAMILY, 9)              # form labels, body text
FONT_BUTTON = (UI_FAMILY, 9, "bold")     # every button
FONT_SMALL = (UI_FAMILY, 8)              # secondary text
FONT_CAPTION = (UI_FAMILY, 7, "bold")    # all-caps micro headings
FONT_HINT = (UI_FAMILY, 8, "italic")     # hints and help copy

FONT_READOUT = (MONO_FAMILY, 13, "bold")  # the big numeric values
FONT_MONO = (MONO_FAMILY, 9)              # inline numeric text
FONT_ENTRY = (MONO_FAMILY, 10)            # numeric input fields
FONT_GLYPH = (UI_FAMILY, 16, "bold")      # jog-pad arrows
FONT_KEYCAP = (MONO_FAMILY, 8, "bold")    # keycap chips


# ══════════════════════════════════════════════════════════════════════
# Live switching
# ══════════════════════════════════════════════════════════════════════
def _propagate(old_values):
    """Rebinds theme names in every module that imported them.

    Only rewrites an attribute when it still equals the OLD theme value.
    That guard is what makes a blind sweep over `sys.modules` safe: a
    module with its own unrelated `BORDER` keeps it, because the value
    will not match.

    Returns the number of attributes updated, which the self-test asserts
    is non-zero — a silent no-op here would look exactly like a working
    theme switch that simply had nothing to do.
    """
    updated = 0
    for mod_name, module in list(sys.modules.items()):
        if not mod_name.startswith("robot_sim.") or module is None:
            continue
        if module is sys.modules[__name__]:
            continue
        for const in COLOUR_NAMES:
            old = old_values.get(const)
            if old is None:
                continue
            current = getattr(module, const, None)
            if isinstance(current, str) and current == old:
                setattr(module, const, globals()[const])
                updated += 1
    return updated


def apply_palette(name, persist=True):
    """Switches scheme in place. Returns (ok, attributes_updated).

    Does NOT touch widgets — a Tk widget's colours are fixed when it is
    constructed, so the caller has to rebuild the UI afterwards. That
    split is deliberate: this function is pure state, and testable
    without a display.
    """
    if name not in PALETTES:
        return False, 0
    if persist and not save_active_name(name):
        return False, 0

    old_values = {c: globals().get(c) for c in COLOUR_NAMES}
    _bind(name)
    updated = _propagate(old_values)

    # Rendered surfaces are cached by colour, so old entries are simply
    # unreachable rather than wrong — but they are also dead weight.
    try:
        from .widgets import draw as _draw
        _draw._IMAGE_CACHE.clear()
    except Exception:
        pass
    return True, updated


def available_palettes():
    """[(name, label, blurb), ...] for the Appearance picker."""
    from .palettes import PALETTE_ORDER
    return [(n, PALETTES[n]["label"], PALETTES[n]["blurb"])
            for n in PALETTE_ORDER]


# Backwards-compatible private alias used by older call sites.
_shade = shade
