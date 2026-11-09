"""Fixed-time signal model: a periodic green window per movement."""
from __future__ import annotations

import math
from dataclasses import dataclass

from ..scenario import Scenario
from .base import GreenWindow

MAIN_THROUGH = "main_through"


@dataclass(frozen=True)
class FixedTimePlan:
    cycle_s: float
    offset_s: float      # green starts at t ≡ offset_s (mod cycle_s)
    green_s: float

    def cycle_time(self, t: float) -> float:
        return (t - self.offset_s) % self.cycle_s

    def is_green(self, t: float) -> bool:
        return self.cycle_time(t) < self.green_s

    def green_windows(self, t_from: float, t_to: float) -> list[GreenWindow]:
        k = math.floor((t_from - self.offset_s) / self.cycle_s)
        out = []
        while True:
            start = self.offset_s + k * self.cycle_s
            if start > t_to:
                break
            end = start + self.green_s
            if end > t_from:
                out.append(GreenWindow.exact(start, end))
            k += 1
        return out


class FixedTimeForecaster:
    """Forecaster backed by known fixed-time plans (the timing plan is assumed correct)."""

    def __init__(self, plans: dict[tuple[str, str], FixedTimePlan]):
        self.plans = plans

    @classmethod
    def from_scenario(cls, sc: Scenario) -> "FixedTimeForecaster":
        plans = {
            (ix.id, MAIN_THROUGH): FixedTimePlan(sc.timing.cycle_s, ix.offset_s, ix.main_green_s)
            for ix in sc.intersections
        }
        return cls(plans)

    def plan(self, signal_id: str, movement: str = MAIN_THROUGH) -> FixedTimePlan:
        return self.plans[(signal_id, movement)]

    def green_windows(self, signal_id: str, movement: str, t_from: float, t_to: float) -> list[GreenWindow]:
        return self.plans[(signal_id, movement)].green_windows(t_from, t_to)
