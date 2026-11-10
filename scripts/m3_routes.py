"""M3 experiment: signal-aware route choice, energy, profiles and the Pareto front (ADR-0013).

Network: split S -> {A: 8-signal arterial, B: longer 70 km/h bypass with 2 signals} -> merge M.
For every departure (ADR-0014 grid) and both powertrains, every option is driven in SUMO:
    route {A, B} x driving {limit, advised-eco, advised-balanced, advised-fastest}
and, at the moment of departure, the planner predicts each option (controller rollout, routing.py).
Because every option is realised for every departure, any route-choice policy can be evaluated exactly:
    always A / always B          static routing
    signal-aware (raw model)     pick the route with the lowest *predicted* cost at departure
    signal-aware (calibrated)    same, after correcting each option's mean prediction error (learned on history)
    oracle                       pick the route with the lowest *realised* cost (upper bound)
Queue models and prediction calibration are learned from runs on a different seed (101) than evaluation (1).

Usage: python scripts/m3_routes.py
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

from greenwave.energy import POWERTRAINS  # noqa: E402
from greenwave.optimizer.band import BandConfig  # noqa: E402
from greenwave.optimizer.cost import weights_for  # noqa: E402
from greenwave.routing import predict_route  # noqa: E402
from greenwave.signals.fixed_time import FixedTimeForecaster  # noqa: E402
from greenwave.sim import network  # noqa: E402
from greenwave.sim.ego import EgoSpec, departure_times, run_corridor  # noqa: E402
from greenwave.sim.history import collect_history  # noqa: E402
from greenwave.traffic.queue import QueueModel  # noqa: E402
from greenwave.vehicle import VehicleParams  # noqa: E402

HIST_SEED, EVAL_SEED = 101, 1
PROFILES = ["eco", "balanced", "fastest"]
MODES = ["limit"] + PROFILES
OUT = ROOT / "results" / "m3"


def pareto_mask(points: np.ndarray) -> np.ndarray:
    """Non-dominated rows (minimise every column)."""
    keep = np.ones(len(points), bool)
    for i, p in enumerate(points):
        dominated = np.all(points <= p, axis=1) & np.any(points < p, axis=1)
        keep[i] = not dominated.any()
    return keep


def realise(ctx: dict, cfg: Path, departs: list[int], seed: int) -> tuple[pd.DataFrame, dict]:
    """Drive every option (route x mode x powertrain) for every departure in SUMO on `seed`."""
    trips, store = [], {}
    for pt_name, pt in POWERTRAINS.items():
        for k in "AB":
            c = ctx[k]
            for mode in MODES:
                advised = mode != "limit"
                egos = [EgoSpec(f"{k}_{mode}_{d}", d, advised) for d in departs]
                res = run_corridor(c["sc"], cfg, egos, c["fc"], seed=seed, route=c["route"], queue=c["queue"],
                                   powertrain=pt, energy_aware=True,
                                   weights=weights_for(mode, pt_name) if advised else None)
                for lg in res.logs.values():
                    if lg.arrived:
                        m = lg.metrics()
                        m.update(route=k, mode=mode, powertrain=pt_name,
                                 energy=m["elec_wh"] if pt_name == "ev" else m["fuel_g"])
                        trips.append(m)
                store[(pt_name, k, mode)] = res.logs
                print(f"  seed {seed}: {pt_name} route {k} {mode:<9} done", flush=True)
    return pd.DataFrame(trips), store


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    spec = ROOT / "scenarios" / "two_route.yaml"
    cfg = network.build(spec, ROOT / "build" / "two_route")
    nw = network.load_network(spec)
    net = cfg.parent / f"{network.STEM}.net.xml"
    departs = departure_times()
    veh = VehicleParams()

    ctx = {}
    for k in "AB":
        rc = nw[k]
        hist = collect_history(rc.sc, cfg, HIST_SEED, route_edges=rc.edges)
        q = QueueModel.from_history(hist)
        print(f"route {k}: queue clearance q80 {q.clearance_s}", flush=True)
        ctx[k] = dict(sc=rc.sc, route=rc.route(net), queue=q, fc=FixedTimeForecaster.from_scenario(rc.sc),
                      cfg=BandConfig.for_limit(rc.sc.speed_limit_ms))

    # ---- predictions at departure (deterministic: fixed-time signals, queue model from history) ----
    preds = []
    for pt_name, pt in POWERTRAINS.items():
        for k in "AB":
            c = ctx[k]
            sl, length = c["route"].stop_lines(), c["route"].length
            for d in departs:
                for mode in MODES:
                    w = weights_for(mode if mode != "limit" else "balanced", pt_name)
                    p = predict_route(k, sl, length, float(d), c["sc"].speed_limit_ms, c["fc"], c["cfg"], veh, w, pt,
                                      c["queue"], mode="limit" if mode == "limit" else "advised")
                    preds.append(dict(depart=d, route=k, mode=mode, powertrain=pt_name, pred_time=p.travel_time,
                                      pred_stops=p.stops, pred_wait=p.wait, pred_energy=p.energy))
    pr = pd.DataFrame(preds)

    # ---- calibration: mean prediction error per option, measured on the HISTORY seed ----
    # The planner model is idealised (steady cruise, planned stops counted as full stops). Real trips have
    # traffic friction and roll through some planned stops. Like the queue model, the correction is learned
    # from past runs and applied unchanged to the evaluation runs.
    hist_df, _ = realise(ctx, cfg, departs, HIST_SEED)
    h = hist_df.merge(pr, on=["depart", "route", "mode", "powertrain"]).groupby(["powertrain", "route", "mode"])
    calib = pd.DataFrame({"energy_ratio": h.energy.sum() / h.pred_energy.sum(),
                          "time_bias": h.travel_time.mean() - h.pred_time.mean(),
                          "stops_bias": h.stops.mean() - h.pred_stops.mean(),
                          "wait_bias": h.wait.mean() - h.pred_wait.mean()}).reset_index()
    pr = pr.merge(calib, on=["powertrain", "route", "mode"])
    pr["cal_energy"] = pr.pred_energy * pr.energy_ratio
    pr["cal_time"] = pr.pred_time + pr.time_bias
    pr["cal_stops"] = (pr.pred_stops + pr.stops_bias).clip(lower=0)
    pr["cal_wait"] = (pr.pred_wait + pr.wait_bias).clip(lower=0)

    # ---- realised outcomes on the evaluation seed ----
    df, store = realise(ctx, cfg, departs, EVAL_SEED)
    both = df.merge(pr, on=["depart", "route", "mode", "powertrain"])
    # Only departures where all 16 options arrived
    complete = both.groupby(["depart"]).size()
    both = both[both.depart.isin(complete[complete == len(MODES) * 2 * 2].index)]

    # ---- prediction quality ----
    for pre, tcol, ecol in (("raw", "pred_time", "pred_energy"), ("cal", "cal_time", "cal_energy")):
        both[f"{pre}_err_time"] = both[tcol] - both.travel_time
        both[f"{pre}_err_energy_%"] = 100 * (both[ecol] - both.energy) / both.energy
    quality = both.groupby(["powertrain", "route", "mode"], sort=False).agg(
        raw_time_bias_s=("raw_err_time", "mean"), raw_energy_bias_pct=("raw_err_energy_%", "mean"),
        cal_time_bias_s=("cal_err_time", "mean"), cal_time_mae_s=("cal_err_time", lambda e: e.abs().mean()),
        cal_energy_bias_pct=("cal_err_energy_%", "mean"),
        cal_energy_mae_pct=("cal_err_energy_%", lambda e: e.abs().mean()),
        stops_pred=("pred_stops", "mean"), stops_cal=("cal_stops", "mean"), stops_real=("stops", "mean")).round(2)

    # ---- options: realised means ----
    options = both.groupby(["powertrain", "route", "mode"], sort=False).agg(
        stops=("stops", "mean"), near_stops=("near_stops", "mean"), wait_s=("wait", "mean"),
        travel_s=("travel_time", "mean"), energy=("energy", "mean")).round(2)

    # ---- route-choice policies per profile ----
    rows = []
    for pt_name in POWERTRAINS:
        for prof in PROFILES:
            w = weights_for(prof, pt_name)
            sub = both[(both.powertrain == pt_name) & (both["mode"] == prof)].copy()
            sub["real_cost"] = [w.cost(r.travel_time, r.stops, r.wait, r.energy) for r in sub.itertuples()]
            sub["pred_cost"] = [w.cost(r.pred_time, r.pred_stops, r.pred_wait, r.pred_energy)
                                for r in sub.itertuples()]
            sub["cal_cost"] = [w.cost(r.cal_time, r.cal_stops, r.cal_wait, r.cal_energy) for r in sub.itertuples()]
            piv = sub.pivot(index="depart", columns="route")
            pick = {
                "always A": pd.Series("A", index=piv.index),
                "always B": pd.Series("B", index=piv.index),
                "signal-aware (raw model)": piv["pred_cost"].idxmin(axis=1),
                "signal-aware (calibrated)": piv["cal_cost"].idxmin(axis=1),
                "oracle": piv["real_cost"].idxmin(axis=1),
            }
            for pol, choice in pick.items():
                sel = pd.DataFrame([sub[(sub.depart == d) & (sub.route == r)].iloc[0] for d, r in choice.items()])
                rows.append(dict(powertrain=pt_name, profile=prof, policy=pol, share_A=100 * (choice == "A").mean(),
                                 cost=sel.real_cost.mean(), travel_s=sel.travel_time.mean(), stops=sel.stops.mean(),
                                 energy=sel.energy.mean(),
                                 agrees_with_oracle=100 * (choice == pick["oracle"]).mean()))
    policies = pd.DataFrame(rows).round(2)

    # ---- Pareto front (time, energy) per departure over all 8 options ----
    front = []
    for (pt_name, d), g in both.groupby(["powertrain", "depart"]):
        mask = pareto_mask(g[["travel_time", "energy"]].to_numpy())
        for (_, r), on in zip(g.iterrows(), mask):
            front.append(dict(powertrain=pt_name, depart=d, route=r.route, mode=r["mode"], on_front=on))
    fr = pd.DataFrame(front)
    front_share = (fr.groupby(["powertrain", "route", "mode"], sort=False).on_front.mean() * 100).round(1)

    pd.set_option("display.width", 220)
    print("\nRealised options (mean over departures):\n", options.to_string())
    print("\nPrediction calibration learned on the history seed:\n", calib.round(3).to_string(index=False))
    print("\nPrediction quality on the evaluation seed (predicted - realised):\n", quality.to_string())
    print("\nRoute-choice policies (advised driving, per profile):\n", policies.to_string(index=False))
    print("\nShare of departures where the option is on the time-energy Pareto front (%):\n", front_share.to_string())

    df.to_csv(OUT / "trips.csv", index=False)
    pr.to_csv(OUT / "predictions.csv", index=False)
    calib.round(4).to_csv(OUT / "prediction_calibration.csv", index=False)
    options.to_csv(OUT / "options.csv")
    quality.to_csv(OUT / "prediction_quality.csv")
    policies.to_csv(OUT / "policies.csv", index=False)
    front_share.to_csv(OUT / "pareto_share.csv")
    (OUT / "meta.json").write_text(json.dumps({"departures": len(both.depart.unique()), "eval_seed": EVAL_SEED,
                                               "history_seed": HIST_SEED}, indent=2))
    with open(OUT / "runs.pkl", "wb") as fh:
        pickle.dump(store, fh)


if __name__ == "__main__":
    main()
