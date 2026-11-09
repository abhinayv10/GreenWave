"""Plotly figures: time-space diagram, speed-vs-cost, stops distribution."""
from __future__ import annotations

import plotly.graph_objects as go

from ..optimizer.band import KMH, Plan
from ..signals.base import SignalForecaster
from ..signals.fixed_time import MAIN_THROUGH
from .theme import STATUS_CRITICAL, STATUS_GOOD, TOKENS, base_layout


def time_space(stop_lines: list[tuple[str, float]], fc: SignalForecaster, trips: dict, mode: str = "light",
               t_range: tuple[float, float] | None = None, show_band: bool = True) -> go.Figure:
    """trips: {"advised": TripLog, "drive at limit": TripLog}. Time on x, distance on y."""
    t = TOKENS[mode]
    logs = [lg for lg in trips.values() if lg is not None and lg.t]
    if t_range is None:
        t_range = (min(lg.t[0] for lg in logs) - 10, max(lg.t[-1] for lg in logs) + 10)
    t0, t1 = t_range
    fig = go.Figure()

    # Signal phases: red/yellow underneath, green on top, as thin bars at each stop line.
    red_x, red_y, grn_x, grn_y = [], [], [], []
    for sid, sx in stop_lines:
        red_x += [t0, t1, None]; red_y += [sx, sx, None]
        for w in fc.green_windows(sid, MAIN_THROUGH, t0, t1):
            grn_x += [max(w.start, t0), min(w.end, t1), None]; grn_y += [sx, sx, None]
    fig.add_trace(go.Scatter(x=red_x, y=red_y, mode="lines", line=dict(color=STATUS_CRITICAL, width=5),
                             name="Signal red / yellow", hoverinfo="skip"))
    fig.add_trace(go.Scatter(x=grn_x, y=grn_y, mode="lines", line=dict(color=STATUS_GOOD, width=5),
                             name="Signal green", hoverinfo="skip"))

    # Advised band from the first plan, drawn as a fan of constant-speed paths to the last horizon signal.
    adv = trips.get("advised")
    if show_band and adv is not None and adv.advice and adv.advice[0].advised:
        rec = adv.advice[0]
        lo, hi = rec.advised
        horizon_x = max(sx for sid, sx in stop_lines if sid in rec.expected)
        d = horizon_x - rec.x
        fig.add_trace(go.Scatter(
            x=[rec.t, rec.t + d / hi, rec.t + d / lo, rec.t], y=[rec.x, horizon_x, horizon_x, rec.x],
            fill="toself", fillcolor=t["band_fill"], line=dict(width=0), mode="lines",
            name=f"Advised band {lo / KMH:.0f}–{hi / KMH:.0f} km/h (first plan)", hoverinfo="skip"))

    ends = []
    for name, colour in (("drive at limit", t["series2"]), ("advised", t["series1"])):
        lg = trips.get(name)
        if lg is None or not lg.t:
            continue
        kmh = [v / KMH for v in lg.v]
        fig.add_trace(go.Scatter(
            x=lg.t, y=lg.x, mode="lines", line=dict(color=colour, width=2), name=name.capitalize(),
            customdata=kmh, hovertemplate="t=%{x:.0f}s · x=%{y:.0f}m · %{customdata:.0f} km/h<extra>" + name + "</extra>"))
        ends.append((lg.t[-1], lg.x[-1], name))
    # Direct labels at trip ends; both trips end at the same x, so stack them by arrival order.
    for i, (te, xe, name) in enumerate(sorted(ends)):
        fig.add_annotation(x=te, y=xe, text=name, showarrow=False, xanchor="right", xshift=-6,
                           yshift=10 - 22 * i, font=dict(color=t["text2"], size=12))

    # Signal ids in the right margin, clear of the distance tick labels.
    for sid, sx in stop_lines:
        fig.add_annotation(x=1, xref="paper", y=sx, text=sid, showarrow=False, xanchor="left", xshift=6,
                           font=dict(color=t["muted"], size=11))
    fig.update_layout(**base_layout(mode, height=560, hovermode="closest",
                                    margin=dict(l=60, r=40, t=40, b=50)))
    fig.update_xaxes(title="Time (s)", range=[t0, t1])
    fig.update_yaxes(title="Distance along corridor (m)", showticklabels=True)
    return fig


def speed_cost(plan: Plan, mode: str = "light") -> go.Figure:
    """Cost of every candidate speed in a plan, with the chosen feasible band shaded and target marked."""
    t = TOKENS[mode]
    sp = [e.speed / KMH for e in plan.evaluations]
    fig = go.Figure()
    b = plan.best
    fig.add_vrect(x0=b.v_lo / KMH, x1=b.v_hi / KMH, fillcolor=t["band_fill"], line_width=0)
    fig.add_trace(go.Scatter(
        x=sp, y=[e.cost for e in plan.evaluations], mode="lines", line=dict(color=t["series1"], width=2),
        name="Cost J", customdata=[[e.stops, sum(e.pattern), e.travel_time] for e in plan.evaluations],
        hovertemplate="%{x:.1f} km/h · J=%{y:.0f}<br>%{customdata[1]} greens · %{customdata[0]} stops · "
                      "%{customdata[2]:.0f}s<extra></extra>"))
    fig.add_trace(go.Scatter(x=[b.target / KMH], y=[b.centre.cost], mode="markers",
                             marker=dict(size=10, color=t["series1"], line=dict(color=t["surface"], width=2)),
                             name="Advised target", hoverinfo="skip"))
    fig.add_annotation(x=b.target / KMH, y=b.centre.cost, text=f"target {b.target / KMH:.0f} km/h",
                       showarrow=False, yshift=16, font=dict(color=t["text2"], size=12))
    fig.update_layout(**base_layout(mode, height=340, showlegend=False))
    fig.update_xaxes(title="Candidate cruise speed (km/h)")
    fig.update_yaxes(title="Cost J (equivalent seconds)")
    return fig


def stops_distribution(df, mode: str = "light") -> go.Figure:
    t = TOKENS[mode]
    fig = go.Figure()
    max_s = int(df.stops.max())
    for advised, name, colour in ((False, "Drive at limit", t["series2"]), (True, "Advised", t["series1"])):
        counts = df[df.advised == advised].stops.value_counts().reindex(range(max_s + 1), fill_value=0)
        fig.add_trace(go.Bar(x=list(counts.index), y=list(counts.values), name=name, marker_color=colour,
                             hovertemplate="%{x} stops: %{y} trips<extra>" + name + "</extra>"))
    fig.update_layout(**base_layout(mode, height=320, barmode="group", bargap=0.3, bargroupgap=0.08))
    fig.update_xaxes(title="Stops per trip", dtick=1)
    fig.update_yaxes(title="Trips")
    return fig
