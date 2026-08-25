"""Active colour scheme, switchable at runtime.

HOW LIVE SWITCHING WORKS: every widget module does `from ..theme import PANEL_BG`, copies the
VALUE into that module's namespace at import time. Why first version needed restart: rebinding
theme.PANEL_BG alone left a dozen other modules still holding old string.

apply_palette() fixes in three steps:
  1. Rebind every constant in THIS module from new palette.
  2. Walk every already-imported robot_sim.* module, rebind any name still holding OLD value.
     Comparing against old value (not just name) is what makes blind sweep safe: module w/ its
     own unrelated BORDER left alone, value won't match.
  3. Tell app to rebuild widgets — Tk widget colours fixed at construction, can't restyle in
     place.

Alternative was `import theme` everywhere + `theme.PANEL_BG` at every use site — cleaner
arguably, but several hundred edits across UI, noisier to read, same end result.

DESIGN NOTE — what colours are FOR. Machine controller: colour carries meaning before style.
    ACCENT_RED     stop. Never anything else.
    ACCENT_ORANGE  warning / above-recommended. Never primary accent.
    ACCENT_MINT    primary accent — selection, focus, live axis.
    axis colours   identity, distinguishable by lightness as well as hue, survive colour-vision
                   deficiency.

Surface ladder (BG/PANEL_BG/SURFACE/SURFACE_HI) separates control from card behind it. Flat
design has no shadows, these steps do that work.
"""

import sys

from .palettes import PALETTES, load_active_name, save_active_name


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
    """Perceived brightness, 0..255. Used by palette self-test to assert scheme's surface
    ladder steps far enough."""
    r, g, b = (int(hex_color.lstrip("#")[i:i + 2], 16) for i in (0, 2, 4))
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


def _tone(base, target_lum):
    """`base` shifted toward white or black until it hits `target_lum`.

    Blending toward pure white/black keeps hue recognisable while moving lightness freely —
    lets tonal ramp be both unified (one colour family) and accessible (separated by lightness
    not hue). Binary search since luminance not linear in blend factor.
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


#: Names this module owns. Only these propagated on palette change, only where receiving
#: module still holds previous value.
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

    g["BG"] = p["BG"]
    g["PANEL_BG"] = p["PANEL_BG"]
    g["SURFACE"] = p["SURFACE"]
    g["SURFACE_HI"] = p["SURFACE_HI"]
    g["ENTRY_BG"] = p["ENTRY_BG"]
    g["LED_BG"] = p["LED_BG"]
    g["BORDER"] = p["BORDER"]
    g["BORDER_SOFT"] = p["BORDER_SOFT"]
    g["INACTIVE_BG"] = p["SURFACE"]

    # ACCENT_MINT: legacy name meaning "primary accent", whatever colour active scheme uses.
    g["ACCENT_MINT"] = p["ACCENT"]
    g["ACCENT_CYAN"] = p["ACCENT"]
    g["ACCENT_GREEN"] = p["ACCENT_GREEN"]
    g["ACCENT_RED"] = p["ACCENT_RED"]
    g["ACCENT_ORANGE"] = p["ACCENT_ORANGE"]
    g["ACCENT_PURPLE"] = p["ACCENT_PURPLE"]
    g["INK_DARK"] = p["ACCENT_ON"]

    # Motor/axis ramp. Previously six independently chosen hues (red, green, blue, amber,
    # purple, teal) — readout row looked like a different app.
    #
    # Now DERIVED from scheme's two accents, whole UI stays one colour family. Separation from
    # lightness not hue — keeps distinguishable under red-green colour blindness; tonal ramp
    # does that naturally, unified look and accessibility pull same way here.
    #
    # TWO groups not four separate colours:
    #   A1M + A2M share primary accent   (arms are a pair)
    #   RM  + ZM  share secondary accent (base axes)
    # Four distinct tones was still four things to tell apart in a readout row. Grouping by
    # what they physically are: colour answers "which kind of axis", label answers "which one".
    #
    # Tones solved for TARGET LUMINANCE not mixed by fixed ratio. Fixed ratio was first
    # attempt, doesn't work: how far a 30% blend moves brightness depends on where accent
    # started, so schemes w/ accents close in lightness collapsed to 2-unit separation —
    # invisible, worse than clashing hues it replaced.
    a1, a2 = p["ACCENT"], p["ACCENT_2"]
    if p["dark"]:
        # Bright: sit on darkest plane in app (LED_BG), can afford near top of range.
        arm_t, base_t = 214, 168
        c_targets = (206, 180, 154)
    else:
        # On light card tones must go DOWN or they vanish.
        arm_t, base_t = 74, 116
        c_targets = (78, 106, 134)

    arms = _tone(a1, arm_t)
    base = _tone(a2, base_t)

    g["ARM_COLOR"] = arms          # A1M
    g["ARM2_COLOR"] = arms         # A2M — same colour by design
    g["ROT_COLOR"] = base          # RM
    g["JZ_COLOR"] = base           # ZM — same colour by design

    g["AXIS_X_COLOR"] = _tone(a1, c_targets[0])
    g["AXIS_Y_COLOR"] = _tone(a2, c_targets[1])
    g["AXIS_Z_COLOR"] = _tone(a2, c_targets[2])

    g["TEXT_LIGHT"] = p["TEXT_LIGHT"]
    g["TEXT_MUTED"] = p["TEXT_MUTED"]
    g["TEXT_DIM"] = p["TEXT_DIM"]

    # Retained for stragglers; flat style changes fills not shadows.
    g["SHADOW_DARK"] = "#000000" if p["dark"] else "#9aa0aa"
    g["SHADOW_LIGHT"] = "#2b3037" if p["dark"] else "#ffffff"

    # Field states. Borders not fills — draw eye w/o making number itself harder to read.
    g["WARN_BORDER"] = p["ACCENT_ORANGE"]
    g["ERROR_BORDER"] = p["ACCENT_RED"]
    g["FOCUS_BORDER"] = p["ACCENT"]
    g["NEUTRAL_BORDER"] = p["BORDER"]

    # Highlights: lighten on dark schemes, darken on light ones — "brighter" hover on light
    # theme has nowhere to go.
    lift = 1.18 if p["dark"] else 0.88
    g["HI_CYAN"] = shade(p["ACCENT"], lift)
    g["HI_MINT"] = shade(p["ACCENT"], lift)
    g["HI_GREEN"] = shade(p["ACCENT_GREEN"], lift)
    g["HI_PURPLE"] = shade(g["ARM_COLOR"], lift)
    g["HI_ARM2"] = shade(g["ARM2_COLOR"], lift)
    g["HI_ROT"] = shade(g["ROT_COLOR"], lift)


_bind(load_active_name())


# Corner radii, not palette dependent. Generous+consistent — corners anti-aliased now, larger
# radius reads soft instead of longer staircase.
RADIUS_SM = 8       # chips, keycaps
RADIUS_MD = 12      # buttons, fields
RADIUS_LG = 16      # jog pads
RADIUS_CARD = 14    # section cards

# Type scale. Sizes in POINTS, Tk resolves against real DPI once `tk scaling` set — NOT
# multiplied by DPI factor anywhere (doing both was the bug making labels overflow buttons).
#
# Six steps, each one job. Previous code picked size per call site, drifted into eleven
# different sizes — why some labels looked oversized next to neighbours.
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


def _propagate(old_values):
    """Rebinds theme names in every module that imported them.

    Only rewrites attribute when it still equals OLD theme value — guard makes blind sweep
    over sys.modules safe: module w/ own unrelated BORDER keeps it, value won't match.

    Returns count of attributes updated; self-test asserts non-zero — silent no-op here would
    look exactly like a working theme switch that had nothing to do.
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

    Does NOT touch widgets — Tk widget colours fixed at construction, caller must rebuild UI
    afterwards. Deliberate split: this fn is pure state, testable w/o display.
    """
    if name not in PALETTES:
        return False, 0
    if persist and not save_active_name(name):
        return False, 0

    old_values = {c: globals().get(c) for c in COLOUR_NAMES}
    _bind(name)
    updated = _propagate(old_values)

    # Rendered surfaces cached by colour — old entries unreachable not wrong, but dead weight.
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
