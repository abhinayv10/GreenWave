"""Routes as the planner and the runner see them (ADR-0013).

A Route is an edge sequence plus the signals on it, located by **along-route distance** to their stop lines.
That replaces the x coordinate the M1-M2 corridor used, so the same code works for any route geometry.

Distances follow SUMO's odometer (`traci.vehicle.getDistance`): normal lanes plus the internal junction lanes
between them. A vehicle's route position is its odometer plus the lane position it departed at (see
`RoutePosition`), so it can be compared directly with `RouteSignal.stop_line`.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import sumolib


@dataclass(frozen=True)
class RouteSignal:
    signal_id: str        # SUMO traffic light id
    approach_edge: str    # route edge whose end is this signal's stop line
    stop_line: float      # along-route distance from the start of the first edge (m)


@dataclass(frozen=True)
class Route:
    id: str
    edges: tuple[str, ...]
    signals: tuple[RouteSignal, ...]
    length: float         # along-route distance to the end of the last edge (m)

    def stop_lines(self) -> list[tuple[str, float]]:
        """(signal_id, stop line distance) in travel order: the form the advisory controller consumes."""
        return [(s.signal_id, s.stop_line) for s in self.signals]

    def signal(self, signal_id: str) -> RouteSignal:
        return next(s for s in self.signals if s.signal_id == signal_id)

    @classmethod
    def from_net(cls, net_path: str | Path, route_id: str, edges: list[str] | tuple[str, ...]) -> "Route":
        net = _read_net(str(net_path))
        pos, signals = 0.0, []
        for i, eid in enumerate(edges):
            lane = net.getEdge(eid).getLanes()[0]
            pos += lane.getLength()
            if i + 1 == len(edges):
                break
            conns = net.getEdge(eid).getConnections(net.getEdge(edges[i + 1]))
            if not conns:
                raise ValueError(f"route {route_id}: no connection {eid} -> {edges[i + 1]}")
            conn = min(conns, key=lambda c: c.getFromLane().getIndex())
            if conn.getTLSID():
                signals.append(RouteSignal(conn.getTLSID(), eid, pos))
            via = conn.getViaLaneID()
            if via:
                pos += net.getLane(via).getLength()
        return cls(route_id, tuple(edges), tuple(signals), pos)


@lru_cache(maxsize=8)
def _read_net(path: str) -> sumolib.net.Net:
    return sumolib.net.readNet(path, withInternal=True)


class RoutePosition:
    """Converts a vehicle's odometer to along-route distance.

    The offset is the lane position at the first observation minus the odometer then. It is fixed once
    taken, so the observation must be made while the vehicle is still on the route's first edge, which
    holds when it is read on the step the vehicle appears.
    """

    def __init__(self) -> None:
        self._offset: dict[str, float] = {}

    def __call__(self, traci, vid: str) -> float:
        d = traci.vehicle.getDistance(vid)
        if vid not in self._offset:
            self._offset[vid] = traci.vehicle.getLanePosition(vid) - d
        return d + self._offset[vid]
