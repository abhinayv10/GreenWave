"""Probe-vehicle trajectories: what a fleet of connected cars would report (M4, ADR-0015).

A share of ordinary background vehicles (main road and cross streets alike) report (time, edge, lane position,
speed) once per second, i.e. map-matched GPS traces. No signal state, no detectors: signal timing must be
inferred from these traces alone (signals/probe_inferred.py).

Sampling is by a deterministic hash of the vehicle id, so the probes at a lower penetration rate are a subset
of those at a higher one: one recording at the highest rate serves every rate.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path

from .sumo_env import binary


def probe_u(vid: str) -> float:
    """Deterministic pseudo-uniform number in [0, 1) per vehicle id."""
    return int(hashlib.md5(vid.encode()).hexdigest()[:8], 16) / 2 ** 32


def is_background(vid: str) -> bool:
    """Ordinary traffic (main road and cross streets): any flow named bg_*. Test vehicles are never probes."""
    return vid.split(".")[0].split("_")[0] == "bg"


@dataclass
class ProbeData:
    rate: float
    traces: dict[str, list[tuple[float, str, float, float]]] = field(default_factory=dict)  # vid -> (t, edge, pos, v)
    meta: dict = field(default_factory=dict)

    def subsample(self, rate: float) -> "ProbeData":
        if rate > self.rate + 1e-12:
            raise ValueError(f"cannot subsample {self.rate} to a higher rate {rate}")
        return ProbeData(rate, {v: tr for v, tr in self.traces.items() if probe_u(v) < rate}, dict(self.meta))

    def window(self, t0: float, t1: float) -> "ProbeData":
        """Only what was reported in [t0, t1)."""
        out = {v: [r for r in tr if t0 <= r[0] < t1] for v, tr in self.traces.items()}
        return ProbeData(self.rate, {v: tr for v, tr in out.items() if tr}, dict(self.meta, window=[t0, t1]))

    def save(self, path: Path) -> None:
        path.write_text(json.dumps({"rate": self.rate, "traces": self.traces, "meta": self.meta}))

    @classmethod
    def load(cls, path: Path) -> "ProbeData":
        raw = json.loads(Path(path).read_text())
        return cls(raw["rate"], {v: [tuple(r) for r in tr] for v, tr in raw["traces"].items()}, raw["meta"])


def collect_probes(cfg_path: Path, seed: int, end: float = 3600, rate: float = 0.25,
                   interval: float = 1.0) -> ProbeData:
    """Run the background-only simulation and record probe traces at `rate`, one report every `interval` s."""
    import libsumo as traci

    traci.start([binary("sumo"), "-c", str(cfg_path), "--seed", str(seed), "--end", str(end),
                 "--step-length", "0.5"])
    try:
        traces: dict[str, list] = {}
        next_report = 0.0
        while traci.simulation.getTime() < end:
            traci.simulationStep()
            t = traci.simulation.getTime()
            if t + 1e-9 < next_report:
                continue
            next_report = t + interval
            for vid in traci.vehicle.getIDList():
                if is_background(vid) and probe_u(vid) < rate:
                    traces.setdefault(vid, []).append(
                        (t, traci.vehicle.getRoadID(vid), round(traci.vehicle.getLanePosition(vid), 2),
                         round(traci.vehicle.getSpeed(vid), 2)))
        return ProbeData(rate, traces, {"seed": seed, "end": end, "interval": interval})
    finally:
        traci.close()
