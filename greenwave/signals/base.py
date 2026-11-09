"""The SignalForecaster interface (ADR-0003).

A GreenWindow mirrors SAE J2735 SPaT timing semantics: a likely start/end plus min/max bounds and a
confidence. Fixed-time windows have zero spread. Actuated windows also carry `start_samples`, an empirical
distribution of when green will start, so the planner can ask for the start time it can rely on with
probability p (ADR-0009).
"""
from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class GreenWindow:
    start: float              # likely green start (s, simulation clock)
    end: float                # likely green end (start of yellow)
    start_min: float
    start_max: float
    end_min: float
    end_max: float
    confidence: float = 1.0   # probability the window occurs roughly as described
    start_samples: tuple[float, ...] | None = None   # sorted empirical start times, if uncertain

    @classmethod
    def exact(cls, start: float, end: float) -> "GreenWindow":
        return cls(start, end, start, start, end, end, 1.0)

    @classmethod
    def from_start_samples(cls, samples, end: float, confidence: float = 1.0) -> "GreenWindow":
        s = tuple(sorted(float(x) for x in samples))
        return cls(float(np.median(s)), end, s[0], s[-1], end, end, confidence, s)

    @property
    def duration(self) -> float:
        return self.end - self.start

    @property
    def is_exact(self) -> bool:
        return self.start_samples is None or self.start_min == self.start_max

    def p_started(self, t: float) -> float:
        """P(green has started by t)."""
        if self.start_samples is None:
            return 1.0 if t >= self.start else 0.0
        return bisect_right(self.start_samples, t) / len(self.start_samples)

    def start_by(self, p: float) -> float:
        """Earliest time by which green has started with probability >= p (the start the planner can rely on)."""
        if self.start_samples is None:
            return self.start
        return float(np.quantile(self.start_samples, p, method="higher"))


class SignalForecaster(Protocol):
    def green_windows(self, signal_id: str, movement: str, t_from: float, t_to: float) -> list[GreenWindow]:
        """Green windows for `movement` at `signal_id` overlapping [t_from, t_to], ordered by start."""
        ...
