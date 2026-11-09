"""Forecasters for coordinated semi-actuated signals (ADR-0008, ADR-0009).

Structure the forecaster relies on (true for coordinated semi-actuated control): fixed cycle, main green ENDS
at a fixed yield point, and STARTS somewhere in [yield - max_main, nominal start] depending on how much of
its time the cross street used. So the only uncertainty is the start, and it is driven by cross-street demand.

ActuatedForecaster learns, per signal, the distribution of "early return" (nominal start - actual start)
from history, bucketed by recent cross-street flow (the congestion observable). At runtime it is fed the
current cross flow and, optionally, the live signal state (SPaT-like: "is main green now, since when").

RecordedForecaster replays the actual green intervals of a run: the oracle / upper bound.
"""
from __future__ import annotations

import math
from bisect import bisect_left, bisect_right
from dataclasses import dataclass, field

import numpy as np

from ..scenario import Scenario
from ..sim.history import SignalHistory
from .base import GreenWindow
from .fixed_time import MAIN_THROUGH

FLOW_WINDOW_S = 300.0                    # cross flow is measured over the previous 5 minutes
FLOW_BINS = (0.0, 150.0, 300.0, 450.0)   # veh/h (both cross approaches combined) bucket edges


def flow_bucket(vph: float) -> int:
    return max(0, bisect_right(FLOW_BINS, vph) - 1)


def cross_flow(arrivals: list[float], t: float) -> float:
    """veh/h over [t - FLOW_WINDOW_S, t) from sorted arrival times."""
    n = bisect_left(arrivals, t) - bisect_left(arrivals, t - FLOW_WINDOW_S)
    return n * 3600.0 / FLOW_WINDOW_S


@dataclass
class _SignalModel:
    cycle: float
    offset: float          # nominal main green start (mod cycle)
    green: float           # nominal main green; yield = offset + green
    early_by_bucket: dict[int, list[float]] = field(default_factory=dict)
    early_all: list[float] = field(default_factory=list)

    def samples(self, bucket: int) -> list[float]:
        s = self.early_by_bucket.get(bucket, [])
        if len(s) >= 5:
            return s
        # sparse bucket: borrow from neighbours, then fall back to all data
        near = s + self.early_by_bucket.get(bucket - 1, []) + self.early_by_bucket.get(bucket + 1, [])
        return near if len(near) >= 5 else self.early_all


class ActuatedForecaster:
    def __init__(self, sc: Scenario, history: SignalHistory, use_live_state: bool = False,
                 condition_on_demand: bool = True, warmup_s: float = 180.0):
        self.use_live_state = use_live_state
        self.condition_on_demand = condition_on_demand
        self.models: dict[str, _SignalModel] = {}
        self.flow_now: dict[str, float] = {}
        self.live_green_since: dict[str, float | None] = {}
        self.now = 0.0
        C = sc.timing.cycle_s
        for ix in sc.intersections:
            m = _SignalModel(C, ix.offset_s, ix.main_green_s)
            arrivals = sorted(history.cross_arrivals.get(ix.id, []))
            for s, e in history.green[ix.id]:
                if s < warmup_s:
                    continue
                early = (e - ix.main_green_s) - s       # nominal start of this window minus actual start
                if not (-1.0 <= early <= C):            # malformed / transient interval
                    continue
                b = flow_bucket(cross_flow(arrivals, s))
                m.early_by_bucket.setdefault(b, []).append(max(early, 0.0))
                m.early_all.append(max(early, 0.0))
            self.models[ix.id] = m
            self.flow_now[ix.id] = 0.0
            self.live_green_since[ix.id] = None

    # ---- runtime observations (fed by the simulation runner) ----
    def observe(self, t: float, cross_flow_vph: dict[str, float], main_green: dict[str, bool]) -> None:
        self.now = t
        self.flow_now.update(cross_flow_vph)
        for sid, g in main_green.items():
            if g and self.live_green_since.get(sid) is None:
                self.live_green_since[sid] = t
            elif not g:
                self.live_green_since[sid] = None

    # ---- forecasting ----
    def green_windows(self, signal_id: str, movement: str, t_from: float, t_to: float) -> list[GreenWindow]:
        assert movement == MAIN_THROUGH
        m = self.models[signal_id]
        bucket = flow_bucket(self.flow_now[signal_id]) if self.condition_on_demand else -99
        early = m.samples(bucket) if self.condition_on_demand else m.early_all
        out = []
        # windows are indexed by their yield (end) point: end_k = offset + green + k*C
        k = math.floor((t_from - m.offset - m.green) / m.cycle)
        while True:
            nominal = m.offset + k * m.cycle
            end = nominal + m.green
            if nominal - max(early) > t_to:
                break
            if end > t_from:
                out.append(self._window(signal_id, nominal, end, early))
            k += 1
        return out

    def _window(self, sid: str, nominal: float, end: float, early: list[float]) -> GreenWindow:
        starts = [nominal - e for e in early]
        if self.use_live_state:
            since = self.live_green_since.get(sid)
            if since is not None and since <= self.now < end and since >= min(starts) - 1:
                return GreenWindow.exact(since, end)                  # observed: this window has started
            if since is None and self.now < end:
                pending = [s for s in starts if s > self.now]         # not started yet: condition on that
                starts = pending or [max(self.now, nominal)]
        return GreenWindow.from_start_samples(starts, end)


class RecordedForecaster:
    """Exact replay of the green intervals recorded in a run (oracle for actuated signals)."""

    def __init__(self, history: SignalHistory):
        self.iv = {sid: sorted(map(tuple, ivs)) for sid, ivs in history.green.items()}

    def green_windows(self, signal_id: str, movement: str, t_from: float, t_to: float) -> list[GreenWindow]:
        return [GreenWindow.exact(s, e) for s, e in self.iv[signal_id] if e > t_from and s <= t_to]
