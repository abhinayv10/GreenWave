"""Infer signal timing from probe trajectories alone (M4, ADR-0015).

Evidence: a vehicle crosses a stop line, or starts moving out of a queue, only while its own movement is green.
So every probe crossing and queue release is a "green now" sample for its movement: main road or cross street.

1. **Cycle.** Fold every movement's green samples modulo a candidate cycle C. At the true C each movement's red
   is an empty arc; at a wrong C the samples smear over the circle. Score(C) = mean over movements of
   (largest empty arc / C). Multiples of the true cycle score lower (same empty arc, twice the circle).
2. **Main green start** = end of the main road's empty arc: the first main green sample after red. Queues form
   on red, so the first queued vehicle departs right after green in many cycles; folding makes that edge sharp.
3. **Main green end** = start of the cross street's green minus the intergreen (yellow + all-red, a published
   standard value, not inferred). Cross traffic arrives at random, so a cross queue forms every cycle and leaves
   the moment cross green starts: a sharp edge. The main road's own last green sample is *not* used when cross
   evidence exists: on a well-progressed arterial, platoons pass early in green and the end of green is empty
   in both directions, indistinguishable from red (found empirically, ADR-0015). Without cross evidence it is
   the fallback (an underestimate, which is the safe direction).
4. **Actuated.** The coordinated main green always ends at the yield point (cross green then starts), so step 3
   gives the yield point. Each cycle's main start is bracketed by the last cross green sample + intergreen
   (below) and the first main green sample (above); the midpoint is used where both exist. These per-cycle
   intervals replace controller logs as the ActuatedForecaster's history (ADR-0009).

The output is a Scenario with inferred timing, the same thing a timing sheet would give, so the existing
forecasters consume it unchanged (ADR-0003).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
import sumolib

from ..scenario import Scenario
from ..sim.history import SignalHistory
from ..sim.probes import ProbeData

HALT_SPEED = 0.5         # m/s: a report below this means the probe is queued
MAIN, CROSS = "main", "cross"


@dataclass(frozen=True)
class StopLineEvent:
    signal_id: str
    movement: str               # "main" | "cross"
    t_cross: float              # estimated time the probe crossed the stop line
    stopped: bool               # it was queued on the approach before crossing
    t_release: float | None     # when it started moving out of the queue (None if it never stopped)

    def green_samples(self) -> list[float]:
        return [self.t_cross] + ([self.t_release] if self.t_release is not None else [])


def approach_edges(net_path: Path, signal_ids: set[str]) -> dict[str, tuple[str, float, str]]:
    """Edges ending at a signal: edge id -> (signal id, stop-line position = lane length, movement)."""
    net = sumolib.net.readNet(str(net_path))
    out = {}
    for e in net.getEdges():
        eid, to = e.getID(), e.getToNode().getID()
        if to in signal_ids:
            out[eid] = (to, e.getLanes()[0].getLength(), MAIN if eid.startswith(("eb_", "wb_")) else CROSS)
    return out


def extract_events(probes: ProbeData, net_path: Path, signal_ids: set[str],
                   warmup_s: float = 180.0) -> dict[str, list[StopLineEvent]]:
    """Stop-line events from probe traces. Events before `warmup_s` are dropped: at simulation start the
    coordinated controllers have not settled into their cycle yet (the same warm-up as the controller-log
    history, ADR-0009)."""
    appr = approach_edges(net_path, signal_ids)
    events: dict[str, list[StopLineEvent]] = {s: [] for s in signal_ids}
    for tr in probes.traces.values():
        seg_start = 0
        for i in range(1, len(tr)):
            (t0, e0, p0, _), (t1, e1, p1, _) = tr[i - 1], tr[i]
            if e1 == e0:
                continue
            if e0 in appr:
                sid, length, mov = appr[e0]
                before = max(length - p0, 0.0)
                after = p1 if e1.startswith(":") else 0.0      # into the junction, if still inside it
                frac = before / (before + after) if before + after > 0 else 1.0
                t_cross = t0 + frac * (t1 - t0)
                halted = [k for k in range(seg_start, i) if tr[k][3] < HALT_SPEED]
                t_rel = 0.5 * (tr[halted[-1]][0] + tr[halted[-1] + 1][0]) if halted else None
                if t_cross >= warmup_s:
                    events[sid].append(StopLineEvent(sid, mov, t_cross, bool(halted), t_rel))
            seg_start = i
    return events


OUTLIER_SHARE = 0.01     # the "empty" red arc may contain up to this share of samples (stray events)


def _largest_gap(phases: np.ndarray, cycle: float, outlier_share: float = OUTLIER_SHARE) -> tuple[float, float, float]:
    """Largest arc of a circle of length `cycle` containing at most `outlier_share` of the phases: (start, end,
    length). One stray sample inside the red (a mis-timed crossing estimate, a queue creeping forward) would
    otherwise split the red arc in two and move both edges."""
    p = np.sort(phases)
    n = len(p)
    m = min(int(outlier_share * n), n - 1)
    ext = np.append(p, p + cycle)
    arcs = ext[m + 1: m + 1 + n] - p
    i = int(np.argmax(arcs))
    return float(p[i]), float(ext[i + m + 1] % cycle), float(arcs[i])


def _samples(events: dict[str, list[StopLineEvent]], movement: str, min_events: int) -> dict[str, np.ndarray]:
    out = {}
    for sid, evs in events.items():
        s = [x for e in evs if e.movement == movement for x in e.green_samples()]
        if len(s) >= min_events:
            out[sid] = np.array(s)
    return out


def _all_series(events, min_events: int) -> list[np.ndarray]:
    return list(_samples(events, MAIN, min_events).values()) + list(_samples(events, CROSS, min_events).values())


def cycle_score(series: list[np.ndarray], cycle: float) -> float:
    return float(np.mean([_largest_gap(s % cycle, cycle)[2] / cycle for s in series]))


def estimate_cycle(events: dict[str, list[StopLineEvent]], c_min: float = 40.0, c_max: float = 150.0,
                   step: float = 0.05, min_events: int = 10) -> float:
    """Corridor cycle length (coordinated signals share one): coarse scan, then a fine scan around the best."""
    series = _all_series(events, min_events)
    if not series:
        raise ValueError("not enough probe events to estimate a cycle")
    grid = np.arange(c_min, c_max + step / 2, step)
    best = grid[int(np.argmax([cycle_score(series, c) for c in grid]))]
    fine = np.arange(best - 2 * step, best + 2 * step, step / 20)
    return float(fine[int(np.argmax([cycle_score(series, c) for c in fine]))])


@dataclass(frozen=True)
class InferredPhase:
    signal_id: str
    green_start: float     # phase in [0, cycle)
    green_end: float       # phase in [0, cycle)
    end_source: str        # "cross" (cross-street green start - intergreen) | "main" (last main green sample)
    n_main: int
    n_cross: int

    def green(self, cycle: float) -> float:
        return (self.green_end - self.green_start) % cycle


def infer_phases(events: dict[str, list[StopLineEvent]], cycle: float, intergreen_s: float = 5.0,
                 min_events: int = 10) -> dict[str, InferredPhase]:
    main, cross = _samples(events, MAIN, min_events), _samples(events, CROSS, min_events)
    out = {}
    for sid, s in main.items():
        last_main, start, _ = _largest_gap(s % cycle, cycle)
        if sid in cross:
            _, cross_start, _ = _largest_gap(cross[sid] % cycle, cycle)
            end, src = (cross_start - intergreen_s) % cycle, "cross"
        else:
            end, src = last_main, "main"
        out[sid] = InferredPhase(sid, start, end, src, len(s), len(cross.get(sid, [])))
    return out


def infer_fixed_timing(template: Scenario, events: dict[str, list[StopLineEvent]], cycle: float | None = None,
                       intergreen_s: float = 5.0) -> Scenario:
    """A copy of `template` (geometry, demand) whose signal timing comes from probes only.
    Signals without enough probe evidence are dropped from the returned scenario (no plan, no advice there)."""
    c = cycle or estimate_cycle(events)
    ph = infer_phases(events, c, intergreen_s)
    ixs = tuple(replace(ix, offset_s=round(ph[ix.id].green_start, 2), main_green_s=round(ph[ix.id].green(c), 2))
                for ix in template.intersections if ix.id in ph)
    return replace(template, timing=replace(template.timing, cycle_s=c), intersections=ixs)


def infer_actuated_history(template: Scenario, events: dict[str, list[StopLineEvent]], cycle: float | None = None,
                           intergreen_s: float = 5.0, min_cycles: int = 8,
                           cross_arrivals: dict[str, list[float]] | None = None) -> tuple[Scenario, SignalHistory]:
    """Probe-only replacement for controller logs on coordinated-actuated signals.

    Returns (scenario with inferred cycle and yield point, history of per-cycle main green intervals). Each
    interval ends at the inferred yield point. Its start is bracketed: no later than the earliest main green
    sample in that cycle, no earlier than the latest cross green sample + intergreen. Where both bounds exist
    the midpoint is used. `cross_arrivals` (detector counts) can be passed through for demand conditioning;
    probes do not provide them.
    """
    c = cycle or estimate_cycle(events)
    ph = infer_phases(events, c, intergreen_s)
    green: dict[str, list[list[float]]] = {}
    ixs = []
    for ix in template.intersections:
        if ix.id not in ph:
            continue
        y = ph[ix.id].green_end
        upper: dict[int, float] = {}      # earliest main green sample in cycle k: start is no later than this
        lower: dict[int, float] = {}      # latest cross green sample in cycle k + intergreen: start is no earlier
        for e in events[ix.id]:
            for t in e.green_samples():
                if e.movement == MAIN:
                    k = math.ceil((t - y) / c - 1e-9)            # main green of cycle k ends at y + k*c
                    upper[k] = min(upper.get(k, math.inf), t)
                else:
                    k = math.floor((t - y) / c) + 1              # cross green between yield k-1 and main start k
                    lower[k] = max(lower.get(k, -math.inf), t + intergreen_s)
        both = {k: 0.5 * (u + lower[k]) for k, u in upper.items() if k in lower and lower[k] <= u}
        # Cycles bounded on both sides give a near-unbiased start (midpoint). A main sample alone is an upper
        # bound, late by however long green ran before the first probe; use those only when too few cycles
        # have both bounds (ADR-0015).
        starts = both if len(both) >= min_cycles else upper
        ivs = sorted([s, y + k * c] for k, s in starts.items())
        green[ix.id] = ivs
        g = min(e - s for s, e in ivs)                # shortest observed green: every "early return" >= 0
        ixs.append(replace(ix, offset_s=round((y - g) % c, 2), main_green_s=round(g, 2)))
    sc = replace(template, timing=replace(template.timing, cycle_s=c), intersections=tuple(ixs))
    hist = SignalHistory(green, {s: [] for s in green}, {s: list((cross_arrivals or {}).get(s, [])) for s in green},
                         {"source": "probes"})
    return sc, hist
