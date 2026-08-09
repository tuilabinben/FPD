"""Named colour schemes.

WHY THIS IS A SEPARATE MODULE
-----------------------------
`theme.py` exports flat module-level constants because every widget reads
them at import time. That is fine, but it means the scheme has to be
chosen BEFORE anything imports theme — so the choice lives here, in a
module with no dependencies, and is persisted in its own tiny file rather
than in machine_settings.json (which is loaded much later, by the app).

RULES EVERY PALETTE MUST FOLLOW
-------------------------------
These are not stylistic. They are what keeps the UI readable as a machine
controller rather than a mood board:

  1. Every surface level must be SEPARABLE from its neighbours, and a
     control must be separable from the card behind it. Flat design has
     no shadows, so those steps do the work shadows used to.

     Note this is NOT "monotonic luminance". Dark schemes do step
     upward — BG < PANEL_BG < SURFACE < SURFACE_HI — but light schemes
     follow the normal light-UI convention instead: the card is the
     LIGHTEST thing, and controls sit slightly darker than it. Requiring
     a single direction would have banned every sane light theme.

  2. ENTRY_BG must read as a well cut into the card, and LED_BG must be
     the most recessed plane of all. That is the operator's only cue for
     "editable" versus "read-only", so it holds in both directions:
     darker than the card on dark schemes, brighter on light ones (where
     white is what "you may type here" looks like).

  3. RED means stop and AMBER means warning, in every scheme. Those two
     are safety signals, not decoration, so no palette may reassign them
     to something merely pretty — and no palette may use amber as its
     primary accent, or a warning stops standing out.

  4. The axis colours must stay mutually distinguishable, INCLUDING for
     red-green colour-vision deficiency. Hue alone does not achieve that,
     so the six are pinned to luminances spaced ~15 apart and only then
     given a hue. That is why they look slightly desaturated: the
     lightness had to be spent on separation rather than on vividness.

  5. Body text needs ≥ 110 luminance contrast against PANEL_BG.
"""

import json
import os

from . import paths

_HERE = paths.user_data_dir()
PALETTE_FILE = os.path.join(_HERE, "appearance.json")


# ══════════════════════════════════════════════════════════════════════
# The schemes
# ══════════════════════════════════════════════════════════════════════
PALETTES = {

    # ── 1 ─────────────────────────────────────────────────────────────
    "graphite": {
        "label": "Graphite",
        "blurb": "Near-monochrome dark. Colour only where it means "
                 "something: an active axis, a warning, a stop.",
        "dark": True,
        "BG": "#0f1011", "PANEL_BG": "#171819", "SURFACE": "#202223",
        "SURFACE_HI": "#2a2c2e", "ENTRY_BG": "#121314", "LED_BG": "#0c0d0e",
        "BORDER": "#26282a", "BORDER_SOFT": "#1d1f20",
        "ACCENT": "#eef1f5", "ACCENT_2": "#8fb6d6", "ACCENT_ON": "#0f1011",
        "ACCENT_GREEN": "#7ed96a", "ACCENT_RED": "#ff5f5a",
        "ACCENT_ORANGE": "#ffc247", "ACCENT_PURPLE": "#9a8fb5",
        "TEXT_LIGHT": "#e6e7e9", "TEXT_MUTED": "#8a8d91", "TEXT_DIM": "#5b5e62",
    },

    # ── 2 ─────────────────────────────────────────────────────────────
    "slate": {
        "label": "Slate Blue",
        "blurb": "Cool neutral greys with one restrained steel blue. "
                 "Calm, and the least tiring over a long shift.",
        "dark": True,
        "BG": "#0e1014", "PANEL_BG": "#161a20", "SURFACE": "#1f242c",
        "SURFACE_HI": "#282e38", "ENTRY_BG": "#121519", "LED_BG": "#0b0d10",
        "BORDER": "#252b34", "BORDER_SOFT": "#1c2128",
        "ACCENT": "#7fb4ff", "ACCENT_2": "#5fd8dd", "ACCENT_ON": "#0b0d10",
        "ACCENT_GREEN": "#5fd69b", "ACCENT_RED": "#ff6b78",
        "ACCENT_ORANGE": "#ffbb5e", "ACCENT_PURPLE": "#9b8fd8",
        "TEXT_LIGHT": "#e4e8ee", "TEXT_MUTED": "#8892a0", "TEXT_DIM": "#59616d",
    },

    # ── 3 ─────────────────────────────────────────────────────────────
    "nord": {
        "label": "Nordic Light",
        "blurb": "Light, low-contrast, muted blue. Reads best in a bright "
                 "room or under shop lighting.",
        "dark": False,
        "BG": "#e8ebf0", "PANEL_BG": "#f2f4f7", "SURFACE": "#e2e6ed",
        "SURFACE_HI": "#d6dbe4", "ENTRY_BG": "#fbfcfd", "LED_BG": "#eceff4",
        "BORDER": "#ccd2dc", "BORDER_SOFT": "#dde1e8",
        "ACCENT": "#2f7fd0", "ACCENT_2": "#1f9a90", "ACCENT_ON": "#ffffff",
        "ACCENT_GREEN": "#4f8a63", "ACCENT_RED": "#c14a54",
        "ACCENT_ORANGE": "#c08434", "ACCENT_PURPLE": "#6f5fa8",
        "TEXT_LIGHT": "#22262c", "TEXT_MUTED": "#5d646e", "TEXT_DIM": "#8b929b",
    },

    # ── 4 ─────────────────────────────────────────────────────────────
    "paper": {
        "label": "Warm Paper",
        "blurb": "Warm off-white with a muted clay accent. The softest "
                 "option — no cold blue anywhere.",
        "dark": False,
        "BG": "#eae6df", "PANEL_BG": "#f5f2ec", "SURFACE": "#e6e2da",
        "SURFACE_HI": "#dbd6cc", "ENTRY_BG": "#fdfbf7", "LED_BG": "#efebe3",
        "BORDER": "#d2cdc2", "BORDER_SOFT": "#e2ded5",
        "ACCENT": "#c96b42", "ACCENT_2": "#a88a48", "ACCENT_ON": "#ffffff",
        "ACCENT_GREEN": "#5d7f52", "ACCENT_RED": "#b34a41",
        "ACCENT_ORANGE": "#b8802f", "ACCENT_PURPLE": "#7a6288",
        "TEXT_LIGHT": "#2b2722", "TEXT_MUTED": "#6a635a", "TEXT_DIM": "#968e83",
    },

    # ── 5 ─────────────────────────────────────────────────────────────
    "carbon": {
        "label": "Carbon Orange",
        "blurb": "Very dark with a single industrial orange. Looks like "
                 "shop equipment — high contrast, easy at a distance.",
        "dark": True,
        "BG": "#0c0c0d", "PANEL_BG": "#141415", "SURFACE": "#1d1d1f",
        "SURFACE_HI": "#272729", "ENTRY_BG": "#101011", "LED_BG": "#0a0a0b",
        "BORDER": "#242426", "BORDER_SOFT": "#1a1a1c",
        "ACCENT": "#ff8038", "ACCENT_2": "#ffb03a", "ACCENT_ON": "#0c0c0d",
        "ACCENT_GREEN": "#69cf6d", "ACCENT_RED": "#ff5555",
        "ACCENT_ORANGE": "#ffc94a", "ACCENT_PURPLE": "#9b8bc4",
        "TEXT_LIGHT": "#e8e8e9", "TEXT_MUTED": "#8c8c8f", "TEXT_DIM": "#5c5c60",
    },

    # ── 6 ─────────────────────────────────────────────────────────────
    "mono": {
        "label": "Pure Mono",
        "blurb": "Greyscale chrome, colour reserved entirely for state. "
                 "The most minimal — nothing is coloured for decoration.",
        "dark": True,
        "BG": "#101010", "PANEL_BG": "#181818", "SURFACE": "#212121",
        "SURFACE_HI": "#2b2b2b", "ENTRY_BG": "#131313", "LED_BG": "#0d0d0d",
        "BORDER": "#272727", "BORDER_SOFT": "#1e1e1e",
        "ACCENT": "#ffffff", "ACCENT_2": "#c4c4c4", "ACCENT_ON": "#101010",
        "ACCENT_GREEN": "#9ae085", "ACCENT_RED": "#ff7a7a",
        "ACCENT_ORANGE": "#ffcc66", "ACCENT_PURPLE": "#a89ec4",
        "TEXT_LIGHT": "#ededed", "TEXT_MUTED": "#8a8a8a", "TEXT_DIM": "#5a5a5a",
    },

    # ── 0 ─────────────────────────────────────────────────────────────
    # The scheme that was in place before this menu existed. Kept so the
    # change is reversible rather than one-way.
    "mint": {
        "label": "Mint (previous)",
        "blurb": "The original dark scheme with a mint primary.",
        "dark": True,
        "BG": "#0e0f12", "PANEL_BG": "#16181d", "SURFACE": "#1e2127",
        "SURFACE_HI": "#262a32", "ENTRY_BG": "#121419", "LED_BG": "#0f1114",
        "BORDER": "#23262d", "BORDER_SOFT": "#1c1f25",
        "ACCENT": "#5ff0c8", "ACCENT_2": "#5fd0ff", "ACCENT_ON": "#0b0d10",
        "ACCENT_GREEN": "#4fe89a", "ACCENT_RED": "#ff5c6c",
        "ACCENT_ORANGE": "#ffb057", "ACCENT_PURPLE": "#9b8cff",
        "TEXT_LIGHT": "#e9ecf1", "TEXT_MUTED": "#8b919c", "TEXT_DIM": "#5a606a",
    },
}

DEFAULT_PALETTE = "graphite"

#: Order used by the picker, most minimal first.
PALETTE_ORDER = ("graphite", "mono", "slate", "nord", "paper", "carbon", "mint")


def load_active_name():
    """Name of the scheme to use. Falls back to the default rather than
    raising: a bad appearance file must never stop the app starting."""
    try:
        with open(PALETTE_FILE, "r", encoding="utf-8") as fh:
            name = json.load(fh).get("palette")
    except Exception:
        return DEFAULT_PALETTE
    return name if name in PALETTES else DEFAULT_PALETTE


def save_active_name(name):
    """Persists the choice. Returns True on success."""
    if name not in PALETTES:
        return False
    try:
        with open(PALETTE_FILE, "w", encoding="utf-8") as fh:
            json.dump({"palette": name}, fh, indent=2)
    except OSError:
        return False
    return True


def active():
    return PALETTES[load_active_name()]
