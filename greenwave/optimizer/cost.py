"""Composite cost J = wT*T + wS*S + wW*W + wE*E. Units: seconds-equivalent.

Every term is converted to "equivalent seconds" so weights are interpretable:
wS = 15 means "one stop is as bad as 15 s of extra travel time".

Energy (M3, ADR-0013) is valued at monetary parity with time in the balanced profile:
  time   EUR 12/h           -> 0.00333 EUR/s
  petrol EUR 1.80/L, 745 g/L -> 0.00242 EUR/g  -> 0.725 s per g
  power  EUR 0.30/kWh        -> 0.0003 EUR/Wh  -> 0.09 s per Wh
Eco weighs energy 3x that, fastest ignores it.
"""
from __future__ import annotations

from dataclasses import dataclass, replace


@dataclass(frozen=True)
class CostWeights:
    w_time: float = 1.0       # per second of travel time over the horizon
    w_stop: float = 15.0      # per stop
    w_wait: float = 0.5       # per second stationary at red (on top of its share of travel time)
    w_fragile: float = 10.0   # penalty if the band is narrower than the driver can hold
    w_energy: float = 0.0     # per g of fuel (ICE) or Wh of battery energy (EV)

    def cost(self, travel_time: float, stops: int, wait: float, energy: float = 0.0) -> float:
        return (self.w_time * travel_time + self.w_stop * stops + self.w_wait * wait
                + self.w_energy * energy)


# M1-M2 profiles: energy-blind (w_energy = 0). M3 adds energy with `weights_for`.
PROFILES: dict[str, CostWeights] = {
    "eco": CostWeights(w_time=1.0, w_stop=30.0, w_wait=1.0),
    "balanced": CostWeights(),
    "fastest": CostWeights(w_time=1.0, w_stop=3.0, w_wait=0.1),
}

ENERGY_VALUE_S = {"ice": 0.725, "ev": 0.09}             # seconds per g / per Wh at monetary parity
ENERGY_EMPHASIS = {"eco": 3.0, "balanced": 1.0, "fastest": 0.0}


def weights_for(profile: str, powertrain: str) -> CostWeights:
    """M3 profile weights including the energy term for the given powertrain ("ice" | "ev")."""
    return replace(PROFILES[profile], w_energy=ENERGY_EMPHASIS[profile] * ENERGY_VALUE_S[powertrain])
