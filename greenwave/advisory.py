"""Advisory controller: turns plans into advice and decides when to re-plan (ADR-0007).

Re-plan only when needed:
  1. no plan yet,
  2. a signal was passed (the horizon slides forward),
  3. the current advice no longer achieves the planned outcome from the vehicle's actual state
     (a signal planned as green is now predicted red), i.e. loss of alignment.
Otherwise the advice is kept, even if a marginally better band appears; stable advice beats optimal-but-twitchy.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .optimizer.band import BandConfig, Plan, SignalAhead, compute_plan, evaluate_speed
from .optimizer.cost import CostWeights
from .signals.base import SignalForecaster
from .traffic.queue import QueueModel
from .vehicle import VehicleParams


@dataclass
class AdviceRecord:
    t: float
    x: float
    v: float
    reason: str
    band: tuple[float, float] | None
    advised: tuple[float, float] | None
    target: float
    explanation: str
    expected: dict[str, bool]       # signal_id -> planned green


@dataclass
class AdvisoryController:
    stop_lines: list[tuple[str, float]]     # (signal_id, x) in travel order
    forecaster: SignalForecaster
    cfg: BandConfig
    veh: VehicleParams
    weights: CostWeights
    queue: QueueModel | None = None
    plan: Plan | None = None
    expected: dict[str, bool] = field(default_factory=dict)
    first_ahead: str | None = None
    target: float = 0.0
    log: list[AdviceRecord] = field(default_factory=list)
    first_plan: Plan | None = None

    def signals_ahead(self, x: float) -> list[SignalAhead]:
        return [SignalAhead(sid, sx - x) for sid, sx in self.stop_lines if sx - x > 0.5]

    def tick(self, t: float, x: float, v: float) -> float:
        ahead = self.signals_ahead(x)
        if not ahead:
            if self.first_ahead is not None:
                self.first_ahead = None
                self.target = self.cfg.v_max
                self.log.append(AdviceRecord(t, x, v, "past last signal", None, None, self.target,
                                             "No signals ahead: drive at the speed limit.", {}))
            return self.target

        reason = self._replan_reason(t, x, v, ahead)
        if reason:
            self.plan = compute_plan(t, v, ahead, self.forecaster, self.cfg, self.veh, self.weights, self.queue)
            if self.first_plan is None:
                self.first_plan = self.plan
            b = self.plan.best
            self.target = b.target
            self.expected = {o.signal_id: o.green for o in b.centre.outcomes}
            self.first_ahead = ahead[0].signal_id
            self.log.append(AdviceRecord(t, x, v, reason, (b.v_lo, b.v_hi), b.advised, b.target,
                                         self.plan.explanation, dict(self.expected)))
        return self.target

    def _replan_reason(self, t: float, x: float, v: float, ahead: list[SignalAhead]) -> str | None:
        if self.plan is None:
            return "initial"
        if ahead[0].signal_id != self.first_ahead:
            return "signal passed"
        sigs = tuple(s for s in ahead[: self.cfg.horizon] if s.signal_id in self.expected)
        now = evaluate_speed(self.target, t, v, sigs, self.forecaster, self.cfg, self.veh, self.weights, self.queue)
        for o in now.outcomes:
            if self.expected.get(o.signal_id) and not o.green:
                return f"lost alignment at {o.signal_id}"
        return None
