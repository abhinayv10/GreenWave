"""Scenario configuration: the single source of truth for network geometry and signal timing.

Both the SUMO network builder and our own signal models are generated from the same Scenario,
so the simulator and the optimiser can never silently disagree about the signals.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

KMH = 1 / 3.6  # km/h -> m/s


@dataclass(frozen=True)
class Intersection:
    id: str
    x: float                 # stop-line position along the arterial (m)
    main_green_s: float      # arterial through green duration (nominal, i.e. with cross street at max)
    offset_s: float          # nominal main green start: t ≡ offset_s (mod cycle), simulation clock
    cross_demand_factor: float = 1.0   # multiplies the cross-street demand profile at this intersection


@dataclass(frozen=True)
class SignalTiming:
    control: str
    cycle_s: float
    yellow_s: float
    all_red_s: float
    progression_speed_kmh: float
    # Actuated (coordinated semi-actuated) only:
    cross_min_green_s: float = 8.0   # cross street green floor
    max_gap_s: float = 3.0           # cross green ends after this gap with no detected vehicle

    @property
    def is_actuated(self) -> bool:
        return self.control == "actuated"


@dataclass(frozen=True)
class Scenario:
    name: str
    speed_limit_ms: float
    cross_speed_limit_ms: float
    main_lanes: int
    cross_lanes: int
    cross_arm_length_m: float
    corridor_length_m: float
    timing: SignalTiming
    intersections: tuple[Intersection, ...]
    demand: dict = field(default_factory=dict)

    def cross_green_s(self, ix: Intersection) -> float:
        t = self.timing
        g = t.cycle_s - ix.main_green_s - 2 * (t.yellow_s + t.all_red_s)
        if g < 5:
            raise ValueError(f"{ix.id}: cross-street green {g:.1f}s too short for cycle {t.cycle_s}s")
        return g


def load_scenario(path: str | Path) -> Scenario:
    raw = yaml.safe_load(Path(path).read_text())
    st = raw["signal_timing"]
    timing = SignalTiming(
        control=st["control"],
        cycle_s=float(st["cycle_s"]),
        yellow_s=float(st["yellow_s"]),
        all_red_s=float(st["all_red_s"]),
        progression_speed_kmh=float(st["progression_speed_kmh"]),
        cross_min_green_s=float(st.get("cross_min_green_s", 8.0)),
        max_gap_s=float(st.get("max_gap_s", 3.0)),
    )
    if timing.control not in ("fixed", "actuated"):
        raise ValueError(f"control must be fixed or actuated (adaptive is out of scope, ADR-0002): {timing.control}")
    v_prog = timing.progression_speed_kmh * KMH
    intersections = []
    for item in sorted(raw["intersections"], key=lambda i: i["x"]):
        # Ideal progression: green starts when a vehicle leaving x=0 at t=0 at v_prog arrives.
        offset = (item["x"] / v_prog + item.get("offset_shift_s", 0.0)) % timing.cycle_s
        intersections.append(
            Intersection(id=item["id"], x=float(item["x"]),
                         main_green_s=float(item["main_green_s"]), offset_s=round(offset, 1),
                         cross_demand_factor=float(item.get("cross_demand_factor", 1.0)))
        )
    return Scenario(
        name=raw["name"],
        speed_limit_ms=raw["speed_limit_kmh"] * KMH,
        cross_speed_limit_ms=raw["cross_speed_limit_kmh"] * KMH,
        main_lanes=int(raw["main_lanes"]),
        cross_lanes=int(raw["cross_lanes"]),
        cross_arm_length_m=float(raw["cross_arm_length_m"]),
        corridor_length_m=float(raw["corridor_length_m"]),
        timing=timing,
        intersections=tuple(intersections),
        demand=raw.get("demand", {}),
    )
