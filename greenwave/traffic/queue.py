"""Queue clearance model (ADR-0010).

A vehicle reaching the stop line during green is only unimpeded once the queue that formed during red has
crossed the stop line. From history we know, per signal, how long after green start that takes. The planner
treats green as usable only from `green start + clearance_p` onward.

Historical, not live: plans look up to 4 signals (2-3 minutes) ahead, where today's live queue says little
about the queue that will exist when the vehicle arrives.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..sim.history import SignalHistory


@dataclass
class QueueModel:
    clearance_s: dict[str, float]

    @classmethod
    def from_history(cls, history: SignalHistory, quantile: float = 0.8, warmup_s: float = 180.0) -> "QueueModel":
        out = {}
        for sid, rows in history.clearance.items():
            c = [x for s, x in rows if s >= warmup_s]
            out[sid] = float(np.quantile(c, quantile)) if c else 0.0
        return cls(out)

    @classmethod
    def none(cls) -> "QueueModel":
        return cls({})

    def clearance(self, signal_id: str) -> float:
        return self.clearance_s.get(signal_id, 0.0)
