"""Run test ("ego") vehicles through a SUMO corridor, with or without speed advice.

Advice is applied with traci.vehicle.setMaxSpeed(target): the vehicle never exceeds the advised speed, but
SUMO's car-following, red-light compliance and acceleration limits stay fully in charge (safety stays in
the simulator, not in our code). A DriverModel sits between advice and vehicle (ADR-0011).

Every run also records the true signal timeline (SignalRecorder), which serves as the oracle for actuated
signals and as live observations for forecasters that use them.
"""
from __future__ import annotations

import random
from dataclasses import dataclass, field
from pathlib import Path

from ..advisory import AdviceRecord, AdvisoryController
from ..optimizer.band import KMH, BandConfig
from ..energy import ICE, Powertrain
from ..optimizer.cost import PROFILES, CostWeights
from ..scenario import Scenario
from ..signals.base import SignalForecaster
from ..traffic.queue import QueueModel
from ..vehicle import VehicleParams
from . import corridor
from .history import SignalHistory, SignalRecorder
from .route import Route, RoutePosition
from .sumo_env import binary

STOP_SPEED = 0.3       # m/s: below this the vehicle counts as stopped
RESUME_SPEED = 2.0     # m/s: must exceed this before another stop is counted (hysteresis)
CRAWL_SPEED = 10 / 3.6         # m/s: below this counts as a near-stop (stop or crawl behind a queue)
CRAWL_RESUME_SPEED = 20 / 3.6  # m/s: must exceed this before another near-stop is counted


def count_events(v: list[float], below: float, resume: float) -> int:
    """Number of times speed drops below `below`, re-armed only after exceeding `resume`."""
    n, armed = 0, False
    for s in v:
        if s < below and armed:
            n, armed = n + 1, False
        elif s > resume:
            armed = True
    return n


def departure_times(n: int = 51, first: int = 300, step: int = 59) -> list[int]:
    """Test-vehicle departures for every experiment (ADR-0014).

    The step must share no factor with the signal cycle (90 s), or departures sample only a few phase
    relationships: a 60 s step hits just 3 distinct phases (gcd 30), so one boundary case is a third of the
    sample. 59 s is coprime with 90, so 51 departures give 51 distinct phases spread over the cycle.
    """
    return [first + k * step for k in range(n)]


@dataclass(frozen=True)
class DriverModel:
    """How a human turns advice into speed.

    compliance: 1 = holds the advised speed; 0 = ignores it (drives at the limit); in between = slows only
                part of the way, i.e. applied = limit - compliance * (limit - advised).
    bias_sd_kmh: each driver has a persistent offset ~ N(0, sd) from the speed they intend.
    reaction_s: delay before a changed advice is acted upon.
    noisy: imperfect speed keeping (SUMO Krauss dawdling, sigma=0.5).
    """
    compliance: float = 1.0
    bias_sd_kmh: float = 0.0
    reaction_s: float = 0.0
    noisy: bool = False

    @property
    def vtype(self) -> str:
        return "ego_noisy" if self.noisy else "ego"


PERFECT = DriverModel()
REALISTIC = DriverModel(compliance=0.8, bias_sd_kmh=2.0, reaction_s=2.0, noisy=True)


@dataclass
class EgoSpec:
    id: str
    depart: float
    advised: bool


@dataclass
class TripLog:
    id: str
    depart: float
    advised: bool
    t: list[float] = field(default_factory=list)
    x: list[float] = field(default_factory=list)
    v: list[float] = field(default_factory=list)
    fuel_mg: float = 0.0
    elec_wh: float = 0.0          # battery energy used (EV test vehicles; regen counts negative)
    arrived: float | None = None
    advice: list[AdviceRecord] = field(default_factory=list)
    first_plan: object = None

    def metrics(self) -> dict:
        stops, wait, moving = 0, 0.0, False
        for i in range(1, len(self.v)):
            dt = self.t[i] - self.t[i - 1]
            if self.v[i] < STOP_SPEED:
                wait += dt
                if moving:
                    stops, moving = stops + 1, False
            elif self.v[i] > RESUME_SPEED:
                moving = True
        changes = sum(1 for a, b in zip(self.advice, self.advice[1:]) if abs(a.target - b.target) > 1 / 3.6)
        return {
            "id": self.id, "depart": self.depart, "advised": self.advised,
            "travel_time": (self.t[-1] - self.t[0]) if self.t else None,
            "stops": stops, "near_stops": count_events(self.v, CRAWL_SPEED, CRAWL_RESUME_SPEED),
            "wait": wait, "fuel_g": self.fuel_mg / 1000, "elec_wh": self.elec_wh,
            "replans": len(self.advice), "advice_changes": changes,
        }


@dataclass
class RunResult:
    logs: dict[str, TripLog]
    history: SignalHistory


def _feed_observations(forecaster, rec: SignalRecorder, t: float) -> None:
    """Give forecasters that can use them the observables a deployed system would have: recent cross-street
    flow (detector counts) and current main-street signal state (SPaT-style)."""
    if not hasattr(forecaster, "observe"):
        return
    from ..signals.actuated import cross_flow
    flows = {sid: cross_flow(arr, t) for sid, arr in rec.hist.cross_arrivals.items()}
    green = {sid: rec.green_since[sid] is not None for sid in rec.ids}
    forecaster.observe(t, flows, green)


def run_corridor(sc: Scenario, cfg_path: Path, egos: list[EgoSpec], forecaster: SignalForecaster,
                 seed: int = 1, profile: str = "balanced", control_interval: float = 1.0,
                 queue: QueueModel | None = None, driver: DriverModel = PERFECT,
                 band_cfg: BandConfig | None = None, use_gui: bool = False,
                 route: Route | None = None, powertrain: Powertrain = ICE,
                 weights: CostWeights | None = None, energy_aware: bool = False) -> RunResult:
    """Drive `egos` along `route` (default: the corridor's eastbound route). Positions are along-route
    distances (ADR-0013), so stop lines and vehicle positions are comparable on any route geometry.

    powertrain picks the SUMO vehicle type (ICE: HBEFA petrol; EV: SUMO Energy model). With energy_aware,
    the planner also predicts energy and `weights` (default: the profile's) may price it (M3).
    """
    import libsumo as traci  # in-process; same API as traci

    route = route or corridor.eastbound(sc, cfg_path.parent / "corridor.net.xml")
    stop_lines = route.stop_lines()
    route_pos = RoutePosition()
    band_cfg = band_cfg or BandConfig.for_limit(sc.speed_limit_ms)
    veh = VehicleParams()
    weights = weights or PROFILES[profile]
    end = max(e.depart for e in egos) + 900
    v_lim = sc.speed_limit_ms

    traci.start([binary("sumo-gui" if use_gui else "sumo"), "-c", str(cfg_path), "--seed", str(seed),
                 "--end", str(end), "--step-length", "0.5"])
    try:
        rec = SignalRecorder(traci, sc, list(route.edges))
        traci.route.add("ego_route", list(route.edges))
        logs = {e.id: TripLog(e.id, e.depart, e.advised) for e in egos}
        ctrls = {e.id: AdvisoryController(stop_lines, forecaster, band_cfg, veh, weights, queue,
                                          powertrain if energy_aware else None)
                 for e in egos if e.advised}
        bias = {e.id: random.Random(f"{seed}-{e.id}").gauss(0, driver.bias_sd_kmh) * KMH for e in egos}
        pending_speed: dict[str, list[tuple[float, float]]] = {e.id: [] for e in egos}
        last_target: dict[str, float] = {}
        for e in egos:
            vtype = driver.vtype if e.advised else "ego"
            if powertrain.kind == "ev":
                vtype = vtype.replace("ego", "ego_ev", 1)
            traci.vehicle.add(e.id, "ego_route", typeID=vtype,
                              depart=str(e.depart), departLane="best", departSpeed="max")
        next_tick = {e.id: 0.0 for e in egos}
        pending = set(logs)
        dt = traci.simulation.getDeltaT()
        while pending and traci.simulation.getTime() < end:
            traci.simulationStep()
            rec.step()
            t = traci.simulation.getTime()
            for vid in traci.simulation.getArrivedIDList():
                if vid in pending:
                    logs[vid].arrived = t
                    pending.discard(vid)
            active = pending & set(traci.vehicle.getIDList())
            if active and ctrls:
                _feed_observations(forecaster, rec, t)
            for vid in active:
                x, v = route_pos(traci, vid), traci.vehicle.getSpeed(vid)
                log = logs[vid]
                log.t.append(t); log.x.append(x); log.v.append(v)
                log.fuel_mg += traci.vehicle.getFuelConsumption(vid) * dt
                if powertrain.kind == "ev":
                    log.elec_wh += traci.vehicle.getElectricityConsumption(vid) * dt   # Wh/s * s
                if vid not in ctrls:
                    continue
                if t >= next_tick[vid]:
                    target = ctrls[vid].tick(t, x, v)
                    if target != last_target.get(vid):
                        applied = v_lim - driver.compliance * (v_lim - target) + bias[vid]
                        applied = min(max(applied, 3.0), v_lim)
                        pending_speed[vid].append((t + driver.reaction_s, applied))
                        last_target[vid] = target
                    next_tick[vid] = t + control_interval
                while pending_speed[vid] and pending_speed[vid][0][0] <= t:
                    traci.vehicle.setMaxSpeed(vid, pending_speed[vid].pop(0)[1])
        for vid, c in ctrls.items():
            logs[vid].advice = c.log
            logs[vid].first_plan = c.first_plan
        rec.hist.meta = {"scenario": sc.name, "seed": seed}
        return RunResult(logs, rec.hist)
    finally:
        traci.close()
