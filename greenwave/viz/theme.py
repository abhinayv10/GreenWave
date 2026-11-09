"""Chart tokens (dataviz reference palette). Light and dark are separately chosen steps, both validated."""
from __future__ import annotations

TOKENS = {
    "light": {
        "surface": "#fcfcfb", "text": "#0b0b0b", "text2": "#52514e", "muted": "#898781",
        "grid": "#e1e0d9", "axis": "#c3c2b7",
        "series1": "#2a78d6", "series2": "#eb6834",          # advised, drive-at-limit
        "band_fill": "rgba(42,120,214,0.14)",
    },
    "dark": {
        "surface": "#1a1a19", "text": "#f0efec", "text2": "#c3c2b7", "muted": "#898781",
        "grid": "#2c2c2a", "axis": "#383835",
        "series1": "#3987e5", "series2": "#d95926",
        "band_fill": "rgba(57,135,229,0.20)",
    },
}
# Status colours are mode-invariant and reserved for meaning (signal go / no-go), always labelled.
STATUS_GOOD = "#0ca30c"
STATUS_CRITICAL = "#d03b3b"

FONT = 'system-ui, -apple-system, "Segoe UI", sans-serif'


def base_layout(mode: str, **kw) -> dict:
    t = TOKENS[mode]
    axis = dict(gridcolor=t["grid"], linecolor=t["axis"], zeroline=False, tickfont=dict(color=t["muted"]),
                title_font=dict(color=t["text2"]))
    layout = dict(
        paper_bgcolor=t["surface"], plot_bgcolor=t["surface"],
        font=dict(family=FONT, color=t["text"], size=13),
        margin=dict(l=60, r=20, t=40, b=50),
        legend=dict(orientation="h", y=1.02, yanchor="bottom", x=0, font=dict(color=t["text2"])),
        xaxis=dict(axis), yaxis=dict(axis),
        hoverlabel=dict(font=dict(family=FONT)),
    )
    layout.update(kw)
    return layout
