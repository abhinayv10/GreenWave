"""Green-wave speed-band computation (ADR-0005, ADR-0006).

Method: sweep a fine grid of candidate cruise speeds within [v_min, v_max]. For each speed, simulate the trip
through the next N signals sequentially (stops included: decelerate, wait for green, start-up delay,
re-accelerate). Each speed yields an *outcome pattern* (which signals are crossed on green) and a cost.
A speed band is a maximal run of adjacent grid speeds with the same pattern; the recommended band is the one
with the lowest cost at its centre, with a penalty if it is narrower than a driver can hold.

`pace_intervals_no_stop` is the exact analytic counterpart for the no-stop case (pace = 1/v makes arrival time
linear) and is used to verify the sweep.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..energy import Powertrain, segment_energy, stop_energy
from ..signals.base import GreenWindow, SignalForecaster
from ..signals.fixed_time import MAIN_THROUGH
from ..traffic.queue import QueueModel
from ..vehicle import VehicleParams, arrival_time, speed_at
from .cost import CostWeights

KMH = 1 / 3.6


@dataclass(frozen=True)
class SignalAhead:
    signal_id: str
    distance: float               # metres from the vehicle to the stop line
    movement: str = MAIN_THROUGH


@dataclass(frozen=True)
class BandConfig:
    v_max: float                      # legal limit (m/s), never exceeded
    v_min: float                      # practical minimum (m/s)
    grid_step: float = 0.5 * KMH      # candidate speed resolution
    horizon: int = 4                  # signals considered per plan
    margin_start_s: float = 1.0       # don't target the first second of green (timing error, queue start)
    margin_end_s: float = 2.0         # don't target the last seconds before yellow (dilemma zone)
    driver_tolerance: float = 2.0 * KMH  # +/- speed variation a driver realistically holds
    p_required: float = 0.85          # rely on an uncertain green start only once P(started) >= this

    @property
    def min_band_width(self) -> float:
        """Bands narrower than the driver's +/- tolerance are 'fragile'."""
        return 2 * self.driver_tolerance

    @classmethod
    def for_limit(cls, v_limit: float, **kw) -> "BandConfig":
        return cls(v_max=v_limit, v_min=0.6 * v_limit, **kw)


@dataclass(frozen=True)
class SignalOutcome:
    signal_id: str
    arrival: float        # time the vehicle reaches the stop line (s)
    green: bool           # crossed without stopping
    wait: float           # seconds stationary
    depart: float         # time it crosses the stop line
    energy: float = 0.0   # energy from the previous signal until crossing this one (g or Wh; 0 without a powertrain)


@dataclass(frozen=True)
class SpeedEvaluation:
    speed: float
    outcomes: tuple[SignalOutcome, ...]
    travel_time: float    # from now until crossing the last horizon signal
    stops: int
    wait: float
    cost: float
    energy: float = 0.0   # over the same span as travel_time

    @property
    def pattern(self) -> tuple[bool, ...]:
        return tuple(o.green for o in self.outcomes)


@dataclass(frozen=True)
class SpeedBand:
    v_lo: float           # feasible band: every speed in it gives the same signal outcome
    v_hi: float
    pattern: tuple[bool, ...]
    centre: SpeedEvaluation   # evaluation at the advised target speed (see _target)
    score: float          # cost of `scored` + fragility penalty (lower is better)
    tolerance: float = 0.0
    scored: SpeedEvaluation | None = None   # evaluation the band was ranked on (differs from centre in stop bands)

    @property
    def ranked(self) -> SpeedEvaluation:
        return self.scored or self.centre

    @property
    def width(self) -> float:
        return self.v_hi - self.v_lo

    @property
    def target(self) -> float:
        return self.centre.speed

    @property
    def advised(self) -> tuple[float, float]:
        """The range shown to the driver: target +/- tolerance, clipped to the feasible band."""
        return max(self.v_lo, self.target - self.tolerance), min(self.v_hi, self.target + self.tolerance)


@dataclass
class Plan:
    t0: float
    v0: float
    signals: tuple[SignalAhead, ...]
    evaluations: list[SpeedEvaluation]
    bands: list[SpeedBand]
    best: SpeedBand | None
    explanation: str = ""
    meta: dict = field(default_factory=dict)


def _usable_start(w: GreenWindow, sig: SignalAhead, cfg: BandConfig, queue: QueueModel | None) -> float:
    """Start of the part of a green window the planner relies on: reliable start (probability p_required),
    plus the historical queue clearance at this signal, plus the safety margin."""
    clear = queue.clearance(sig.signal_id) if queue else 0.0
    return w.start_by(cfg.p_required) + clear + cfg.margin_start_s


def _green_at(fc: SignalForecaster, sig: SignalAhead, t: float, cfg: BandConfig,
              queue: QueueModel | None) -> GreenWindow | None:
    for w in fc.green_windows(sig.signal_id, sig.movement, t - 1, t + 1):
        if _usable_start(w, sig, cfg, queue) <= t <= w.end - cfg.margin_end_s:
            return w
    return None


def _release_time(fc: SignalForecaster, sig: SignalAhead, t: float, cfg: BandConfig, queue: QueueModel | None,
                  horizon_s: float = 600) -> float:
    """When a vehicle that reached the signal at t (not on usable green) can expect to proceed:
    expected green start (given it had not started by t) plus the queue ahead of it discharging."""
    clear = queue.clearance(sig.signal_id) if queue else 0.0
    for w in fc.green_windows(sig.signal_id, sig.movement, t, t + horizon_s):
        if t > w.end - cfg.margin_end_s:
            continue                      # inside the end margin: planner does not count on it
        if w.start_samples is not None:
            later = [s for s in w.start_samples if s >= t]
            g = float(np.median(later)) if later else t
        else:
            g = w.start
        return max(g + clear, t)
    raise RuntimeError(f"no green for {sig.signal_id} within {horizon_s}s of t={t}")


def evaluate_speed(v: float, t0: float, v0: float, signals: tuple[SignalAhead, ...], fc: SignalForecaster,
                   cfg: BandConfig, veh: VehicleParams, weights: CostWeights,
                   queue: QueueModel | None = None, pt: Powertrain | None = None) -> SpeedEvaluation:
    """Simulate cruising at v through `signals` (stops included). With a powertrain `pt`, also integrate
    energy over the same trajectory (ADR-0013); without one, energy is 0 and the M1-M2 behaviour is unchanged."""
    t, pos, speed = t0, 0.0, v0
    outcomes, stops, wait, energy = [], 0, 0.0, 0.0
    for sig in signals:
        seg = sig.distance - pos
        t_arr = t + arrival_time(seg, speed, v, veh)
        v_arr = speed_at(seg, speed, v, veh)
        e = segment_energy(pt, seg, speed, v, veh) if pt else 0.0
        if _green_at(fc, sig, t_arr, cfg, queue):
            outcomes.append(SignalOutcome(sig.signal_id, t_arr, True, 0.0, t_arr, e))
            speed = v_arr
            t = t_arr
        else:
            # Braking from cruise to the stop line costs v/(2b) extra seconds vs cruising through.
            t_stopped = t_arr + v_arr / (2 * veh.decel)
            g = _release_time(fc, sig, t_arr, cfg, queue)
            t_dep = max(g + veh.startup_lost_s, t_stopped)
            w = t_dep - t_stopped
            if pt:
                e += stop_energy(pt, v_arr, veh) + pt.idle(w)
            outcomes.append(SignalOutcome(sig.signal_id, t_arr, False, w, t_dep, e))
            stops, wait, speed, t = stops + 1, wait + w, 0.0, t_dep
        energy += e
        pos = sig.distance
    tt = t - t0
    return SpeedEvaluation(v, tuple(outcomes), tt, stops, wait, weights.cost(tt, stops, wait, energy), energy)


def _speed_grid(cfg: BandConfig) -> list[float]:
    n = int(round((cfg.v_max - cfg.v_min) / cfg.grid_step))
    return [cfg.v_min + i * cfg.grid_step for i in range(n + 1)]


def _approach_cost(e: SpeedEvaluation, weights: CostWeights) -> float:
    """Cost from now until the vehicle leaves the first signal it stops at.

    The advice only governs the approach to that stop: once the signal is passed the controller re-plans
    (ADR-0007), so the cruise speed after the stop is never actually used.
    """
    k = next(i for i, o in enumerate(e.outcomes) if not o.green)
    t0 = e.outcomes[-1].depart - e.travel_time
    head = e.outcomes[: k + 1]
    return weights.cost(head[-1].depart - t0, 1, sum(o.wait for o in head), sum(o.energy for o in head))


def _target(run: list[SpeedEvaluation], cfg: BandConfig,
            weights: CostWeights) -> tuple[SpeedEvaluation, SpeedEvaluation]:
    """Advised speed within a band (ADR-0006, ADR-0012). Returns (target, evaluation used to score the band).

    Only speeds that keep +/- tolerance inside the band qualify (bands narrower than 2*tolerance use their centre).
    All-green band: the fastest qualifying speed (faster = less time, same outcome).
    Band with a stop: departure from the red is fixed by the signal, so a faster approach only means a longer
    wait. Pick the speed with the cheapest approach to the first stop (usually the slowest): less idling,
    gentler braking, and often rolling up to a discharging queue instead of stopping. Ties go to the faster speed.
    The band is still scored at its fastest qualifying speed, so ranking between bands is unchanged.
    """
    lo, hi = run[0].speed + cfg.driver_tolerance, run[-1].speed - cfg.driver_tolerance
    inside = [e for e in run if lo - 1e-9 <= e.speed <= hi + 1e-9]
    if not inside:
        mid = run[len(run) // 2]
        return mid, mid
    fastest = inside[-1]
    if all(fastest.pattern):
        return fastest, fastest
    return min(inside, key=lambda e: (round(_approach_cost(e, weights), 6), -e.speed)), fastest


def compute_plan(t0: float, v0: float, signals: list[SignalAhead], fc: SignalForecaster, cfg: BandConfig,
                 veh: VehicleParams, weights: CostWeights, queue: QueueModel | None = None,
                 pt: Powertrain | None = None) -> Plan:
    sigs = tuple(signals[: cfg.horizon])
    if not sigs:
        return Plan(t0, v0, sigs, [], [], None, "No signals ahead: drive at the speed limit.")
    evals = [evaluate_speed(v, t0, v0, sigs, fc, cfg, veh, weights, queue, pt) for v in _speed_grid(cfg)]

    bands: list[SpeedBand] = []
    run = [evals[0]]
    for e in evals[1:] + [None]:
        if e is not None and e.pattern == run[0].pattern:
            run.append(e)
            continue
        target, scored = _target(run, cfg, weights)
        width = run[-1].speed - run[0].speed
        penalty = weights.w_fragile if width < cfg.min_band_width else 0.0
        bands.append(SpeedBand(run[0].speed, run[-1].speed, run[0].pattern, target, scored.cost + penalty,
                               cfg.driver_tolerance, scored))
        if e is not None:
            run = [e]
    best = min(bands, key=lambda b: (b.score, -b.width))
    return Plan(t0, v0, sigs, evals, bands, best, explain(best, sigs, bands))


def explain(best: SpeedBand, sigs: tuple[SignalAhead, ...], bands: list[SpeedBand]) -> str:
    n_green = sum(best.pattern)
    a_lo, a_hi = best.advised
    parts = [f"Hold {a_lo / KMH:.0f}–{a_hi / KMH:.0f} km/h (anything in {best.v_lo / KMH:.0f}–{best.v_hi / KMH:.0f} works): "
             f"green at {n_green} of the next {len(sigs)} signals"]
    if best.centre.stops:
        parts.append(f", {best.centre.stops} stop(s) unavoidable ({best.centre.wait:.0f}s waiting)")
    alt = [b for b in bands if b is not best and b.v_hi > best.v_hi]
    if alt:
        faster = max(alt, key=lambda b: b.v_hi)
        dt = faster.ranked.travel_time - best.ranked.travel_time
        when = f"{abs(dt):.0f}s {'earlier' if dt < 0 else 'later'}" if abs(dt) >= 1 else "at the same time"
        parts.append(f". Driving faster ({faster.v_lo / KMH:.0f}–{faster.v_hi / KMH:.0f} km/h) means "
                     f"{faster.centre.stops} stop(s) and arriving {when}")
    if best.width < 2 * best.tolerance:
        parts.append(". Band is narrow: low robustness")
    return "".join(parts) + "."


def pace_intervals_no_stop(t0: float, signals: tuple[SignalAhead, ...], fc: SignalForecaster,
                           cfg: BandConfig) -> list[tuple[float, float]]:
    """Exact speed intervals (v_lo, v_hi) that cross ALL given signals on green at constant speed from t0.

    Arrival t_i = t0 + d_i * s with pace s = 1/v, so each green window maps to a pace interval; intersect them.
    """
    s_lo, s_hi = 1 / cfg.v_max, 1 / cfg.v_min
    feasible = [(s_lo, s_hi)]
    for sig in signals:
        t_from, t_to = t0 + sig.distance * s_lo, t0 + sig.distance * s_hi
        ivs = []
        for w in fc.green_windows(sig.signal_id, sig.movement, t_from, t_to):
            a, b = w.start + cfg.margin_start_s, w.end - cfg.margin_end_s
            if b > a:
                ivs.append(((a - t0) / sig.distance, (b - t0) / sig.distance))
        feasible = [(max(f0, i0), min(f1, i1)) for f0, f1 in feasible for i0, i1 in ivs if max(f0, i0) <= min(f1, i1)]
    return sorted((1 / s1, 1 / s0) for s0, s1 in feasible)
