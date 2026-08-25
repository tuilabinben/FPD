"""Composite-widget factories: sections, LED wells, coordinate cards.

Every container here = RoundedFrame — no hard 90-degree corner anywhere
in app. Rule that matters: anything TYPE IN or PRESS uses lighter
surface, read-only uses darkest (LED_BG). Operator's cue for what's
touchable — holds without exception.
"""

import tkinter as tk

from ..hidpi import px
from ..theme import (
    FONT_BUTTON,
    FONT_CAPTION,
    FONT_ENTRY,
    FONT_LABEL,
    FONT_MONO,
    FONT_READOUT,
    FONT_TITLE,
    ACCENT_MINT,
    BORDER,
    BORDER_SOFT,
    ENTRY_BG,
    INK_DARK,
    LED_BG,
    PANEL_BG,
    RADIUS_CARD,
    RADIUS_MD,
    RADIUS_SM,
    TEXT_LIGHT,
    TEXT_MUTED,
    mix,
)
from .rounded_frame import RoundedFrame


def make_section(parent, title, row, expand_vertically=False):
    """Titled rounded card. Returns (content_frame, title_label)."""
    outer = RoundedFrame(parent, bg=PANEL_BG, radius=RADIUS_CARD,
                         border=BORDER, border_w=1)
    if expand_vertically:
        outer.grid(row=row, column=0, sticky="nsew", padx=px(16), pady=px(8))
        parent.grid_rowconfigure(row, weight=1)
    else:
        outer.grid(row=row, column=0, sticky="ew", padx=px(16), pady=px(6))
    parent.grid_columnconfigure(0, weight=1)

    inner = outer.body
    head = tk.Frame(inner, bg=PANEL_BG)
    head.pack(fill="x", padx=px(14), pady=(px(10), px(2)))
    title_label = tk.Label(head, text=title, bg=PANEL_BG, fg=TEXT_LIGHT,
                           font=FONT_TITLE, anchor="w")
    title_label.pack(side="left")

    tk.Frame(inner, bg=BORDER_SOFT, height=1).pack(
        fill="x", padx=px(14), pady=(px(8), 0))

    content = tk.Frame(inner, bg=PANEL_BG)
    content.pack(fill="both", expand=True, padx=px(14), pady=(px(12), px(12)))
    return content, title_label


def make_status_led(parent, label, initial_text, initial_color):
    """Recessed rounded status indicator. Returns a dict for set_led()."""
    outer = tk.Frame(parent, bg=parent["bg"])
    tk.Label(outer, text=label, bg=parent["bg"], fg=TEXT_MUTED,
             font=FONT_CAPTION).pack(anchor="w",
                                                            pady=(0, px(2)))
    card = RoundedFrame(outer, bg=LED_BG, radius=RADIUS_SM,
                        border=mix(LED_BG, initial_color, 0.35), border_w=1)
    card.pack()
    text_var = tk.StringVar(value=initial_text)
    led_label = tk.Label(card.body, textvariable=text_var, bg=LED_BG,
                         fg=initial_color,
                         font=FONT_MONO,
                         padx=px(12), pady=px(4))
    led_label.pack()
    return {"frame": outer, "card": card, "label": led_label, "var": text_var}


def set_led(led_dict, text, color):
    led_dict["var"].set(text)
    card = led_dict["card"]
    # RoundedFrame draws own outline — recolour goes through widget, not
    # Tk highlight option.
    if isinstance(card, RoundedFrame):
        card.set_border(mix(LED_BG, color, 0.40))
    led_dict["label"].config(fg=color)


def make_coord_card(parent, col, badge_text, badge_color, var, unit="mm",
                    compact=False):
    """Labelled numeric entry for X/Y/Z coordinate inputs.
    Returns tk.Entry so callers can enable/disable it.

    `compact=True` trims padding/entry width for narrow-column layout,
    not stretched full-width — P2P panel's Point A/B cards, once board
    took wide column.
    """
    parent.grid_columnconfigure(col, weight=1)
    card = RoundedFrame(parent, bg=ENTRY_BG, radius=RADIUS_MD,
                        border=BORDER, border_w=1)
    card.grid(row=0, column=col, padx=px(3 if compact else 5),
             pady=px(3), sticky="ew")
    inner = tk.Frame(card.body, bg=ENTRY_BG)
    inner.pack(padx=px(4 if compact else 6), pady=px(4), fill="x")
    tk.Label(inner, text=badge_text, bg=ENTRY_BG, fg=badge_color,
             font=FONT_BUTTON,
             width=3).pack(side="left", padx=(0, px(4)))
    entry = tk.Entry(inner, textvariable=var, bg=ENTRY_BG, fg=TEXT_LIGHT,
                     relief="flat", font=FONT_LABEL,
                     justify="center", width=5 if compact else 7,
                     insertbackground=ACCENT_MINT,
                     disabledbackground=ENTRY_BG, disabledforeground=TEXT_MUTED,
                     highlightthickness=0, bd=0)
    entry.pack(side="left", fill="x", expand=True)
    tk.Label(inner, text=unit, bg=ENTRY_BG, fg=TEXT_MUTED,
             font=FONT_LABEL).pack(side="left", padx=(px(4), 0))
    return entry


def make_led_card(parent, col, title, var, color):
    """Read-only numeric readout, recessed into the panel."""
    parent.grid_columnconfigure(col, weight=1)
    card = RoundedFrame(parent, bg=LED_BG, radius=RADIUS_MD,
                        border=BORDER_SOFT, border_w=1)
    card.grid(row=0, column=col, padx=px(4), sticky="ew")
    tk.Label(card.body, text=title, bg=LED_BG, fg=TEXT_MUTED,
             font=FONT_CAPTION).pack(pady=(px(4), 0))
    tk.Label(card.body, textvariable=var, bg=LED_BG, fg=color,
             font=FONT_READOUT).pack(pady=(0, px(4)))
    return card


def make_well(parent, radius=RADIUS_MD, bg=LED_BG, border=None):
    """Recessed rounded panel for grouped info — speed reference readout,
    help text, saved-limit-set row.

    Returns RoundedFrame; pack into `.body`.
    """
    return RoundedFrame(parent, bg=bg, radius=radius,
                        border=border or BORDER_SOFT, border_w=1)


def make_inset_entry(parent, var, width=10, justify="right",
                     font=FONT_ENTRY):
    """Text field as rounded well.

    Returns (wrapper, entry). Border lives on WRAPPER so callers can
    recolour it (warning/error) without touching entry, without field
    resizing — would make whole form twitch as you type.
    """
    wrap = RoundedFrame(parent, bg=ENTRY_BG, radius=RADIUS_MD,
                        border=BORDER, border_w=2)
    entry = tk.Entry(wrap.body, textvariable=var, bg=ENTRY_BG, fg=TEXT_LIGHT,
                     relief="flat", font=font, width=width,
                     justify=justify, insertbackground=ACCENT_MINT,
                     highlightthickness=0, bd=0,
                     disabledbackground=ENTRY_BG, disabledforeground=TEXT_MUTED,
                     readonlybackground=ENTRY_BG)
    entry.pack(padx=px(8), pady=px(4), fill="x")
    return wrap, entry


def set_entry_border(wrap, color):
    if isinstance(wrap, RoundedFrame):
        wrap.set_border(color)
    else:
        wrap.config(highlightbackground=color, highlightcolor=color)


def make_chip(parent, text, color, bg=PANEL_BG):
    """Small rounded pill used for inline status words."""
    chip = RoundedFrame(parent, bg=mix(bg, color, 0.16), radius=RADIUS_SM,
                        border=mix(bg, color, 0.30), border_w=1)
    tk.Label(chip.body, text=text, bg=mix(bg, color, 0.16), fg=color,
             font=FONT_CAPTION,
             padx=px(6), pady=px(1)).pack()
    return chip


ACTIVE_INK = INK_DARK
