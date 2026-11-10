"""Signal-aware route cost: roll out the advisory controller in the planner's model (ADR-0013).

A single cruise speed over every signal on a route almost never exists (ADR-0005), so judging a route by one
constant-speed sweep would make long signalised routes look worse than the controller makes them. Instead:
plan with the normal horizon, follow the advised target to the next signal, re-plan there, and repeat;
after the last signal, drive at the limit to the end of the route. That is what the advised vehicle does,
so the prediction is comparable with what it will experience.

`mode="limit"` predicts the unadvised trip (always the speed limit) with the same model.
"""
from __future__ import annotations

from dataclasses import dataclass

from .energy import Powertrain, segment_energy
from .optimizer.band import BandConfig, SignalAhead, compute_plan, evaluate_speed
from .optimizer.cost import CostWeights
from .signals.base import SignalForecaster
from .traffic.queue import QueueModel
from .vehicle import VehicleParams, arrival_time, speed_at


@dataclass(frozen=True)
class RoutePrediction:
    route: str
    mode: str                 # "advised" | "limit"
    travel_time: float        # s, from the decision point to the end of the route
    stops: int
    wait: float
    energy: float             # g (ICE) or Wh (EV)
    cost: float               # composite cost under the weights used


def predict_route(route_id: str, stop_lines: list[tuple[str, float]], length: float, t0: float, v0: float,
                  fc: SignalForecaster, cfg: BandConfig, veh: VehicleParams, weights: CostWeights,
                  pt: Powertrain, queue: QueueModel | None = None, mode: str = "advised") -> RoutePrediction:
    t, pos, speed = t0, 0.0, v0
    stops, wait, energy = 0, 0.0, 0.0
    for i, (sid, sx) in enumerate(stop_lines):
        ahead = [SignalAhead(s, x - pos) for s, x in stop_lines[i:]]
        if mode == "advised":
            target = compute_plan(t, speed, ahead, fc, cfg, veh, weights, queue, pt).best.target
        else:
            target = cfg.v_max
        o = evaluate_speed(target, t, speed, (ahead[0],), fc, cfg, veh, weights, queue, pt).outcomes[0]
        energy += o.energy
        if o.green:
            speed = speed_at(ahead[0].distance, speed, target, veh)
        else:
            stops, wait, speed = stops + 1, wait + o.wait, 0.0
        t, pos = o.depart, sx
    rest = length - pos                     # past the last signal: drive at the limit
    t += arrival_time(rest, speed, cfg.v_max, veh)
    energy += segment_energy(pt, rest, speed, cfg.v_max, veh)
    tt = t - t0
    return RoutePrediction(route_id, mode, tt, stops, wait, energy, weights.cost(tt, stops, wait, energy))
