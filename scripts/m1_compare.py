"""M1 experiment: advised vs unadvised (drive at the limit) test vehicles on the fixed-time corridor.

Test vehicles depart every 60 s over most of an hour, so results cover every phase relationship rather than
one hand-picked departure. Both runs use the same seed and background traffic.

Usage: python scripts/m1_compare.py [scenario.yaml]
"""
from __future__ import annotations

import json
import pickle
import sys
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from greenwave.scenario import load_scenario  # noqa: E402
from greenwave.signals.fixed_time import FixedTimeForecaster  # noqa: E402
from greenwave.sim import corridor  # noqa: E402
from greenwave.sim.ego import EgoSpec, run_corridor  # noqa: E402


def main(scenario_path: Path) -> None:
    sc = load_scenario(scenario_path)
    out = ROOT / "build" / sc.name
    cfg = corridor.build(scenario_path, out)
    fc = FixedTimeForecaster.from_scenario(sc)
    departs = list(range(300, 3301, 60))

    runs = {}
    for advised in (False, True):
        egos = [EgoSpec(f"{'adv' if advised else 'base'}_{d}", d, advised) for d in departs]
        runs[advised] = run_corridor(sc, cfg, egos, fc, seed=1).logs

    rows = [log.metrics() for r in runs.values() for log in r.values() if log.arrived]
    df = pd.DataFrame(rows)
    res_dir = ROOT / "results" / "m1"
    res_dir.mkdir(parents=True, exist_ok=True)
    df.to_csv(res_dir / "trips.csv", index=False)
    with open(res_dir / "runs.pkl", "wb") as f:
        pickle.dump({"scenario": str(scenario_path), "runs": runs}, f)

    g = df.groupby("advised")
    summary = pd.DataFrame({
        "trips": g.size(),
        "stops_mean": g.stops.mean(),
        "zero_stop_trips_%": g.stops.apply(lambda s: 100 * (s == 0).mean()),
        "wait_s_mean": g.wait.mean(),
        "travel_time_s_mean": g.travel_time.mean(),
        "fuel_g_mean": g.fuel_g.mean(),
    }).rename(index={False: "drive at limit", True: "advised"})
    print(summary.round(2).T.to_string())
    adv = df[df.advised]
    print(f"\nadvised: replans/trip {adv.replans.mean():.1f}, advice changes/trip {adv.advice_changes.mean():.1f}")
    (res_dir / "summary.json").write_text(summary.round(3).to_json(indent=2))


if __name__ == "__main__":
    main(Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "scenarios" / "corridor_fixed.yaml")
