"""M2 experiment: robustness to queues, actuated signals, imperfect signal knowledge and real drivers.

For each scenario:
  1. collect history with a DIFFERENT seed (101) than evaluation (1): what the system "knows in advance";
  2. learn the queue model and the actuated forecaster from that history;
  3. run 51 test departures per condition on the evaluation seed and compare.

Usage: python scripts/m2_robustness.py
"""
from __future__ import annotations

import json
import pickle
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from greenwave.analysis import broken_promises  # noqa: E402
from greenwave.optimizer.band import BandConfig  # noqa: E402
from greenwave.scenario import load_scenario  # noqa: E402
from greenwave.signals.actuated import ActuatedForecaster, RecordedForecaster, cross_flow  # noqa: E402
from greenwave.signals.fixed_time import MAIN_THROUGH, FixedTimeForecaster  # noqa: E402
from greenwave.sim import corridor  # noqa: E402
from greenwave.sim.ego import PERFECT, REALISTIC, EgoSpec, run_corridor  # noqa: E402
from greenwave.sim.history import collect_history  # noqa: E402
from greenwave.traffic.queue import QueueModel  # noqa: E402

HIST_SEED, EVAL_SEED = 101, 1
DEPARTS = list(range(300, 3301, 60))
OUT = ROOT / "results" / "m2"


def run(sc, cfg, label, advised, fc=None, queue=None, driver=PERFECT):
    egos = [EgoSpec(f"{'adv' if advised else 'base'}_{d}", d, advised) for d in DEPARTS]
    res = run_corridor(sc, cfg, egos, fc or FixedTimeForecaster.from_scenario(sc), seed=EVAL_SEED,
                       queue=queue, driver=driver)
    sl = corridor.eastbound_stop_lines(sc, cfg.parent / "corridor.net.xml")
    rows = []
    for lg in res.logs.values():
        if lg.arrived:
            m = lg.metrics()
            m["broken_promises"] = broken_promises(lg, sl) if advised else None
            m["condition"] = label
            rows.append(m)
    print(f"  {label:<48} done", flush=True)
    return rows, res


def forecast_quality(sc, fc_factory, truth, lead_s=60.0, p=0.85):
    """Predict each true main-green start `lead_s` ahead; report start error and how often the reliable
    start (start_by(p)) is at or after the true start (it should be, ~p of the time)."""
    errs, covered = [], []
    for ix in sc.intersections:
        for s, e in truth.green[ix.id]:
            if s < 300:
                continue
            fc = fc_factory()
            t_pred = s - lead_s
            flows = {sid: cross_flow(sorted(arr), t_pred) for sid, arr in truth.cross_arrivals.items()}
            green_now = {sid: any(a <= t_pred < b for a, b in truth.green[sid]) for sid in truth.green}
            if hasattr(fc, "observe"):
                fc.observe(t_pred, flows, green_now)
            # match by the (fixed) yield/end time; the predicted start may be far from the true start
            ws = [w for w in fc.green_windows(ix.id, MAIN_THROUGH, s - 1, e) if abs(w.end - e) < 2]
            if not ws:
                continue
            w = ws[0]
            errs.append(w.start - s)
            covered.append(w.start_by(p) >= s - 0.5)
    errs = np.abs(np.array(errs))
    return {"windows": len(errs), "start_MAE_s": float(errs.mean()), "start_p90_err_s": float(np.quantile(errs, .9)),
            "reliable_start_not_early_%": 100 * float(np.mean(covered))}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    all_rows, quality, store = [], {}, {}

    # ---------------- fixed-time corridor ----------------
    sc = load_scenario(ROOT / "scenarios" / "corridor_fixed.yaml")
    cfg = corridor.build(ROOT / "scenarios" / "corridor_fixed.yaml", ROOT / "build" / sc.name)
    hist = collect_history(sc, cfg, HIST_SEED)
    hist.save(ROOT / "build" / sc.name / f"history_{HIST_SEED}.json")
    q = QueueModel.from_history(hist)
    print(f"{sc.name}: queue clearance q80 by signal {q.clearance_s}")
    fx = FixedTimeForecaster.from_scenario(sc)
    for label, kw in [
        ("drive at limit", dict(advised=False)),
        ("advised: exact plan", dict(advised=True, fc=fx)),
        ("advised: exact plan + queue", dict(advised=True, fc=fx, queue=q)),
        ("advised: exact plan + queue, realistic driver", dict(advised=True, fc=fx, queue=q, driver=REALISTIC)),
    ]:
        rows, res = run(sc, cfg, label, **kw)
        all_rows += [dict(r, scenario=sc.name) for r in rows]
        store[(sc.name, label)] = res.logs

    # ---------------- actuated corridor ----------------
    sc = load_scenario(ROOT / "scenarios" / "corridor_actuated.yaml")
    cfg = corridor.build(ROOT / "scenarios" / "corridor_actuated.yaml", ROOT / "build" / sc.name)
    hist = collect_history(sc, cfg, HIST_SEED)
    hist.save(ROOT / "build" / sc.name / f"history_{HIST_SEED}.json")
    q = QueueModel.from_history(hist)
    print(f"{sc.name}: queue clearance q80 by signal {q.clearance_s}")

    base_rows, base_res = run(sc, cfg, "drive at limit", advised=False)
    all_rows += [dict(r, scenario=sc.name) for r in base_rows]
    # True signal timeline on the evaluation seed, recorded long enough to cover every advised trip.
    # Test vehicles only drive on the arterial, while actuation depends on cross-street detectors, so this
    # timeline should match the runs with test vehicles; checked below ("oracle fidelity").
    truth = collect_history(sc, cfg, EVAL_SEED, end=DEPARTS[-1] + 1200)
    same = sum(1 for sid in truth.green for a, b in zip(truth.green[sid], base_res.history.green[sid])
               if abs(a[0] - b[0]) <= 0.6)
    print(f"  truth vs baseline-run timeline: {same} of {sum(len(v) for v in base_res.history.green.values())} "
          f"green starts identical")

    factories = {
        "nominal plan (ignores actuation)": lambda: FixedTimeForecaster.from_scenario(sc),
        "history, unconditioned": lambda: ActuatedForecaster(sc, hist, condition_on_demand=False),
        "history | cross demand": lambda: ActuatedForecaster(sc, hist),
        "history | cross demand + live state": lambda: ActuatedForecaster(sc, hist, use_live_state=True),
        "oracle (recorded truth)": lambda: RecordedForecaster(truth),
    }
    for name, f in factories.items():
        quality[name] = forecast_quality(sc, f, truth)
    for name, f in factories.items():
        rows, res = run(sc, cfg, f"advised: {name} + queue", advised=True, fc=f(), queue=q)
        all_rows += [dict(r, scenario=sc.name) for r in rows]
        store[(sc.name, name)] = res.logs
        if name.startswith("oracle"):   # did advised vehicles change the signals vs the baseline run?
            diff = sum(1 for sid in truth.green for a, b in zip(truth.green[sid], res.history.green[sid])
                       if abs(a[0] - b[0]) > 0.6)
            print(f"  oracle fidelity: {diff} green starts differ between baseline and advised run")
    rows, _ = run(sc, cfg, "advised: history | cross demand + live state + queue, realistic driver",
                  advised=True, fc=factories["history | cross demand + live state"](), queue=q, driver=REALISTIC)
    all_rows += [dict(r, scenario=sc.name) for r in rows]

    # ---------------- report ----------------
    df = pd.DataFrame(all_rows)
    df.to_csv(OUT / "trips.csv", index=False)
    summary = df.groupby(["scenario", "condition"], sort=False).agg(
        stops=("stops", "mean"), zero_stop_pct=("stops", lambda s: 100 * (s == 0).mean()),
        broken_promises=("broken_promises", "mean"), wait_s=("wait", "mean"),
        travel_s=("travel_time", "mean"), fuel_g=("fuel_g", "mean"), replans=("replans", "mean"),
    ).round(2)
    pd.set_option("display.width", 200)
    print("\n", summary.to_string())
    print("\nForecast quality, 60 s ahead (actuated corridor):")
    print(pd.DataFrame(quality).T.round(2).to_string())
    summary.to_csv(OUT / "summary.csv")
    (OUT / "forecast_quality.json").write_text(json.dumps(quality, indent=2))
    with open(OUT / "runs.pkl", "wb") as fh:
        pickle.dump(store, fh)


if __name__ == "__main__":
    main()
