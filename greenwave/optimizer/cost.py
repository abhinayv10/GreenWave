"""Composite cost J = wT*T + wS*S + wW*W (+ wF*F from M3). Units: seconds-equivalent.

Every term is converted to "equivalent seconds" so weights are interpretable:
wS = 15 means "one stop is as bad as 15 s of extra travel time".
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CostWeights:
    w_time: float = 1.0       # per second of travel time over the horizon
    w_stop: float = 15.0      # per stop
    w_wait: float = 0.5       # per second stationary at red (on top of its share of travel time)
    w_fragile: float = 10.0   # penalty if the band is narrower than the driver can hold

    def cost(self, travel_time: float, stops: int, wait: float) -> float:
        return self.w_time * travel_time + self.w_stop * stops + self.w_wait * wait


# M1 uses Balanced only; M3 tunes these and adds the energy term.
PROFILES: dict[str, CostWeights] = {
    "eco": CostWeights(w_time=1.0, w_stop=30.0, w_wait=1.0),
    "balanced": CostWeights(),
    "fastest": CostWeights(w_time=1.0, w_stop=3.0, w_wait=0.1),
}
