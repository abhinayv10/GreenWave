"""Record what signals and queues actually did during a SUMO run.

A SignalRecorder attaches to any running simulation and logs, per signal:
  - main-street through green intervals (ground truth timing),
  - queue clearance time after each green start (time until no vehicle is halted on the approach),
  - cross-street arrival times (the "congestion" observable that conditions actuated predictions).

`collect_history` runs a simulation with background traffic only. Its output is the "historical data" that
the queue model and the actuated forecaster learn from (use a different seed from evaluation runs).
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

from ..scenario import Scenario
from . import corridor
from .sumo_env import binary

HALT_SPEED = 0.1


@dataclass
class SignalHistory:
    green: dict[str, list[list[float]]] = field(default_factory=dict)       # sid -> [[start, end], ...]
    clearance: dict[str, list[list[float]]] = field(default_factory=dict)   # sid -> [[green_start, seconds], ...]
    cross_arrivals: dict[str, list[float]] = field(default_factory=dict)    # sid -> [t, ...]
    meta: dict = field(default_factory=dict)

    def save(self, path: Path) -> None:
        path.write_text(json.dumps(asdict(self)))

    @classmethod
    def load(cls, path: Path) -> "SignalHistory":
        return cls(**json.loads(Path(path).read_text()))


class SignalRecorder:
    def __init__(self, traci, sc: Scenario, route_edges: list[str] | None = None):
        """Records the signals of `sc` along `route_edges` (default: the corridor's eastbound route; pass the
        route explicitly when the corridor is part of a larger network, ADR-0013)."""
        self.traci = traci
        self.sc = sc
        self.ids = [ix.id for ix in sc.intersections]
        self.hist = SignalHistory({s: [] for s in self.ids}, {s: [] for s in self.ids}, {s: [] for s in self.ids})
        self.link_idx, self.approach = {}, {}
        route = route_edges or corridor.eastbound_route(sc)
        for sid in self.ids:
            for idx, links in enumerate(traci.trafficlight.getControlledLinks(sid)):
                if any(i.startswith("eb_") and o.startswith("eb_") for i, o, _ in links):
                    self.link_idx[sid] = idx
                    break
            edge = next(e for e in route if e.endswith(f"_{sid}"))
            self.approach[sid] = [f"{edge}_{k}" for k in range(sc.main_lanes)]
        self.green_since: dict[str, float | None] = {s: None for s in self.ids}
        self.queued: dict[str, set[str]] = {s: set() for s in self.ids}

    def main_green_now(self, sid: str) -> bool:
        return self.traci.trafficlight.getRedYellowGreenState(sid)[self.link_idx[sid]] in "Gg"

    def _halted_on_approach(self, sid: str) -> set[str]:
        tr = self.traci
        return {v for lane in self.approach[sid] for v in tr.lane.getLastStepVehicleIDs(lane)
                if tr.vehicle.getSpeed(v) < HALT_SPEED}

    def _on_approach(self, sid: str) -> set[str]:
        return {v for lane in self.approach[sid] for v in self.traci.lane.getLastStepVehicleIDs(lane)}

    def step(self) -> None:
        """Queue clearance = time from green start until every vehicle that was queued (or joined the queue
        while it discharged) has crossed the stop line. A vehicle arriving after that is unimpeded."""
        tr = self.traci
        t, dt = tr.simulation.getTime(), tr.simulation.getDeltaT()
        t_state = t - dt   # TraCI convention (ADR-0001): state belongs to the step that began at t - dt
        for vid in tr.simulation.getDepartedIDList():
            parts = vid.split(".")[0].split("_")
            if parts[0] == "bg" and len(parts) >= 3 and parts[1] in self.hist.cross_arrivals:
                self.hist.cross_arrivals[parts[1]].append(t_state)
        for sid in self.ids:
            g = self.main_green_now(sid)
            if g and self.green_since[sid] is None:
                self.green_since[sid] = t_state
                self.queued[sid] = self._halted_on_approach(sid)
                if not self.queued[sid]:
                    self.hist.clearance[sid].append([t_state, 0.0])
            elif not g and self.green_since[sid] is not None:
                self.hist.green[sid].append([self.green_since[sid], t_state])
                if self.queued[sid]:   # queue never cleared during this green
                    self.hist.clearance[sid].append([self.green_since[sid], t_state - self.green_since[sid]])
                    self.queued[sid] = set()
                self.green_since[sid] = None
            if g and self.queued[sid]:
                on = self._on_approach(sid)
                self.queued[sid] = (self.queued[sid] & on) | self._halted_on_approach(sid)
                if not self.queued[sid]:
                    self.hist.clearance[sid].append([self.green_since[sid], t_state - self.green_since[sid]])


def collect_history(sc: Scenario, cfg_path: Path, seed: int, end: float = 3600,
                    route_edges: list[str] | None = None) -> SignalHistory:
    import libsumo as traci

    traci.start([binary("sumo"), "-c", str(cfg_path), "--seed", str(seed), "--end", str(end),
                 "--step-length", "0.5"])
    try:
        rec = SignalRecorder(traci, sc, route_edges)
        while traci.simulation.getTime() < end:
            traci.simulationStep()
            rec.step()
        rec.hist.meta = {"scenario": sc.name, "seed": seed, "end": end}
        return rec.hist
    finally:
        traci.close()
