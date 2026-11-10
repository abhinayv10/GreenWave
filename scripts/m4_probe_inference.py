"""M4 experiment: signal timing inferred from probe trajectories only (ADR-0015).

1. Probe traces (a share of ordinary cars, main road and cross streets, 1 report/s) are recorded in the
   history run (seed 101). Timing is inferred from them at several penetration rates and observation windows,
   and compared with the truth: the timing plan (fixed) or the controller logs (actuated).
2. Trips: advice driven by probe-inferred timing vs advice driven by the true plan / controller logs, on the
   evaluation seed (1), 51 departures (ADR-0014). The queue model comes from stop-line detectors in all
   conditions (ADR-0010), so only the signal-timing source differs.

Usage: python scripts/m4_probe_inference.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from greenwave.analysis import broken_promises, forecast_quality  # noqa: E402
from greenwave.scenario import load_scenario  # noqa: E402
from greenwave.signals.actuated import ActuatedForecaster  # noqa: E402
from greenwave.signals.fixed_time import FixedTimeForecaster  # noqa: E402
from greenwave.signals.probe_inferred import (estimate_cycle, extract_events, infer_actuated_history,  # noqa: E402
                                              infer_fixed_timing, infer_phases)
from greenwave.sim import corridor  # noqa: E402
from greenwave.sim.ego import EgoSpec, departure_times, run_corridor  # noqa: E402
from greenwave.sim.history import collect_history  # noqa: E402
from greenwave.sim.probes import collect_probes  # noqa: E402
from greenwave.traffic.queue import QueueModel  # noqa: E402

HIST_SEED, EVAL_SEED = 101, 1
RATES = [0.02, 0.05, 0.10, 0.25]
WINDOWS_S = [900, 1800, 3420]          # observation time after the 180 s warm-up
TRIP_RATES = [0.05, 0.10, 0.25]        # trips use the full observation window
OUT = ROOT / "results" / "m4"


def circ(d: float, c: float) -> float:
    """Signed circular difference in (-c/2, c/2]."""
    return ((d + c / 2) % c) - c / 2


def fixed_accuracy(sc, events) -> dict:
    c = estimate_cycle(events)
    ph = infer_phases(events, c)
    ds, de = [], []
    for ix in sc.intersections:
        if ix.id in ph:
            ds.append(circ(ph[ix.id].green_start - ix.offset_s, c))
            de.append(circ(ph[ix.id].green_end - (ix.offset_s + ix.main_green_s), c))
    ds, de = np.array(ds), np.array(de)
    return {"cycle_err_s": c - sc.timing.cycle_s, "signals": len(ph),
            "end_from_cross_%": 100 * np.mean([p.end_source == "cross" for p in ph.values()]),
            "start_mae_s": np.abs(ds).mean(), "start_max_s": np.abs(ds).max(),
            "end_mae_s": np.abs(de).mean(), "end_max_s": np.abs(de).max(),
            "end_bias_s": de.mean()}


def actuated_accuracy(sc, events, truth_hist) -> dict:
    """Cycle and yield-point error; per-cycle start error against the controller log of the same run."""
    sc_inf, hist = infer_actuated_history(sc, events)
    c = sc_inf.timing.cycle_s
    ye, se, cover = [], [], []
    for ix in sc_inf.intersections:
        true = {round(e, 0): s for s, e in truth_hist.green[ix.id] if s >= 180}
        y_inf = (ix.offset_s + ix.main_green_s) % c
        y_true = next(iter(true)) % c if true else np.nan
        ye.append(circ(y_inf - y_true, c))
        n = 0
        for s, e in hist.green[ix.id]:
            key = min(true, key=lambda k: abs(k - e)) if true else None
            if key is not None and abs(key - e) < 3:
                se.append(s - true[key])
                n += 1
        cover.append(n / max(len(true), 1))
    se = np.array(se)
    return {"cycle_err_s": c - sc.timing.cycle_s, "signals": len(sc_inf.intersections),
            "yield_mae_s": float(np.nanmean(np.abs(ye))), "yield_max_s": float(np.nanmax(np.abs(ye))),
            "cycles_observed_%": 100 * float(np.mean(cover)),
            "start_err_median_s": float(np.median(se)), "start_err_p90_s": float(np.quantile(se, 0.9)),
            "start_early_%": 100 * float(np.mean(se < -0.5))}


def trips(sc, cfg, label, fc, queue, advised=True) -> list[dict]:
    egos = [EgoSpec(f"{'adv' if advised else 'base'}_{d}", d, advised) for d in departure_times()]
    res = run_corridor(sc, cfg, egos, fc, seed=EVAL_SEED, queue=queue)
    sl = corridor.eastbound_stop_lines(sc, cfg.parent / "corridor.net.xml")
    rows = []
    for lg in res.logs.values():
        if lg.arrived:
            m = lg.metrics()
            m.update(condition=label, scenario=sc.name, broken_promises=broken_promises(lg, sl) if advised else None)
            rows.append(m)
    print(f"  {sc.name}: {label:<52} done", flush=True)
    return rows


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    acc_rows, trip_rows, fq = [], [], {}
    for name in ("fixed", "actuated"):
        path = ROOT / "scenarios" / f"corridor_{name}.yaml"
        sc = load_scenario(path)
        cfg = corridor.build(path, ROOT / "build" / sc.name)
        net = cfg.parent / "corridor.net.xml"
        ids = {ix.id for ix in sc.intersections}
        hist = collect_history(sc, cfg, HIST_SEED)                 # controller logs + detector data
        queue = QueueModel.from_history(hist)
        probes = collect_probes(cfg, HIST_SEED, 3600, max(RATES))
        print(f"{sc.name}: {len(probes.traces)} probe vehicles at {max(RATES):.0%}", flush=True)

        # ---- 1. timing accuracy ----
        for rate in RATES:
            for w in WINDOWS_S:
                ev = extract_events(probes.subsample(rate).window(0, 180 + w), net, ids)
                try:
                    a = fixed_accuracy(sc, ev) if name == "fixed" else actuated_accuracy(sc, ev, hist)
                except ValueError as err:          # not enough evidence at all
                    a = {"error": str(err)}
                acc_rows.append(dict(scenario=sc.name, rate=rate, window_min=w / 60,
                                     events_per_signal=sum(len(v) for v in ev.values()) / len(ids), **a))

        # ---- 2. trips ----
        trip_rows += trips(sc, cfg, "drive at limit", FixedTimeForecaster.from_scenario(sc), queue, advised=False)
        full = {r: extract_events(probes.subsample(r), net, ids) for r in TRIP_RATES}
        if name == "fixed":
            trip_rows += trips(sc, cfg, "advised: true timing plan + queue", FixedTimeForecaster.from_scenario(sc), queue)
            for r in TRIP_RATES:
                fc = FixedTimeForecaster.from_scenario(infer_fixed_timing(sc, full[r]))
                trip_rows += trips(sc, cfg, f"advised: probe-inferred plan ({r:.0%}) + queue", fc, queue)
        else:
            truth = collect_history(sc, cfg, EVAL_SEED, end=departure_times()[-1] + 1200)
            factories = {"controller logs | cross demand": lambda: ActuatedForecaster(sc, hist)}
            for r in TRIP_RATES:
                sc_p, h_p = infer_actuated_history(sc, full[r])
                h_pd = infer_actuated_history(sc, full[r], cross_arrivals=hist.cross_arrivals)[1]
                factories[f"probes ({r:.0%}), unconditioned"] = (
                    lambda sc_p=sc_p, h_p=h_p: ActuatedForecaster(sc_p, h_p, condition_on_demand=False))
                factories[f"probes ({r:.0%}) | cross demand (detectors)"] = (
                    lambda sc_p=sc_p, h_pd=h_pd: ActuatedForecaster(sc_p, h_pd))
            for label, f in factories.items():
                fq[label] = forecast_quality(sc, f, truth)
                trip_rows += trips(sc, cfg, f"advised: {label} + queue", f(), queue)

    acc = pd.DataFrame(acc_rows).round(2)
    df = pd.DataFrame(trip_rows)
    summary = df.groupby(["scenario", "condition"], sort=False).agg(
        stops=("stops", "mean"), near_stops=("near_stops", "mean"),
        zero_stop_pct=("stops", lambda s: 100 * (s == 0).mean()), broken_promises=("broken_promises", "mean"),
        wait_s=("wait", "mean"), travel_s=("travel_time", "mean"), fuel_g=("fuel_g", "mean")).round(2)
    pd.set_option("display.width", 220)
    for name in ("corridor_fixed", "corridor_actuated"):
        print(f"\nTiming accuracy, {name}:\n", acc[acc.scenario == name].dropna(axis=1, how="all").to_string(index=False))
    print("\nForecast quality 60 s ahead (actuated):\n", pd.DataFrame(fq).T.round(2).to_string())
    print("\nTrips:\n", summary.to_string())
    acc.to_csv(OUT / "timing_accuracy.csv", index=False)
    df.to_csv(OUT / "trips.csv", index=False)
    summary.to_csv(OUT / "summary.csv")
    (OUT / "forecast_quality.json").write_text(json.dumps(fq, indent=2))


if __name__ == "__main__":
    main()
