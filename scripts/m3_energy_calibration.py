"""M3: calibrate the planner's ICE fuel model and validate both energy models against SUMO (ADR-0013).

Test vehicles (ICE and EV) drive both routes of the two-route network under a range of speed caps, so the
data covers 30-70 km/h cruising, stops, idling and accelerations. Then:
  1. ICE: fit idle rate, overrun (fuel cut-off) rate and the Willans line c0 + c1*P to SUMO's HBEFA fuel,
     step by step. These are the constants in greenwave/energy.py.
  2. Both: per trip, compare the energy model applied to the vehicle's own speed trace with SUMO's total.
     This validates the consumption model; the planner's *trajectory* prediction is validated in m3_routes.py.

Usage: python scripts/m3_energy_calibration.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from greenwave.energy import EV, ICE  # noqa: E402
from greenwave.sim import network  # noqa: E402
from greenwave.sim.sumo_env import binary  # noqa: E402

OUT = ROOT / "results" / "m3"
CAPS_KMH = [30, 40, 50, 60, 70, None]      # None = drive at the limit


def collect(seed: int = 7):
    import libsumo as traci

    spec = ROOT / "scenarios" / "two_route.yaml"
    cfg = network.build(spec, ROOT / "build" / "two_route")
    nw = network.load_network(spec)
    traci.start([binary("sumo"), "-c", str(cfg), "--seed", str(seed), "--end", "3000", "--step-length", "0.5"])
    try:
        dt = traci.simulation.getDeltaT()
        cap = {}
        for k in "AB":
            traci.route.add(f"r{k}", nw[k].edges)
        i = 0
        for k in "AB":
            for pt in ("ice", "ev"):
                for j in range(18):
                    vid = f"{k}_{pt}_{j}"
                    cap[vid] = CAPS_KMH[j % len(CAPS_KMH)]
                    traci.vehicle.add(vid, f"r{k}", typeID="ego" if pt == "ice" else "ego_ev",
                                      depart=str(200 + 61 * j + 13 * i), departSpeed="0")
                i += 1
        steps: dict[str, list] = {v: [] for v in cap}
        while traci.simulation.getTime() < 3000 and traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            for vid in traci.vehicle.getIDList():
                if vid not in cap:
                    continue
                if cap[vid] is not None:
                    traci.vehicle.setMaxSpeed(vid, cap[vid] / 3.6)
                rate = (traci.vehicle.getFuelConsumption(vid) / 1000.0 if "_ice_" in vid      # mg/s -> g/s
                        else traci.vehicle.getElectricityConsumption(vid))                    # Wh/s
                steps[vid].append((traci.vehicle.getSpeed(vid), traci.vehicle.getAcceleration(vid), rate))
        return {v: np.array(s) for v, s in steps.items() if len(s)}, dt
    finally:
        traci.close()


def fit_ice(data: dict[str, np.ndarray]) -> dict:
    d = np.vstack([s for v, s in data.items() if "_ice_" in v])
    v, a, f = d[:, 0], d[:, 1], d[:, 2]
    p = np.array([ICE.body.power(vi, ai) for vi, ai in zip(v, a)])
    idle, drive = v <= 0.1, (v > 0.1) & (p > 0)
    over = (v > 0.1) & (p <= 0)
    X = np.column_stack([np.ones(drive.sum()), p[drive] / 1000])
    (c0, c1), *_ = np.linalg.lstsq(X, f[drive], rcond=None)
    pred = np.where(idle, f[idle].mean(), np.where(over, f[over].mean(), c0 + c1 * p / 1000))
    r2 = 1 - ((f - pred) ** 2).sum() / ((f - f.mean()) ** 2).sum()
    return {"idle_gps": float(f[idle].mean()), "overrun_gps": float(f[over].mean()), "c0_gps": float(c0),
            "c1_gps_per_kw": float(c1), "step_R2": float(r2), "steps": int(len(v))}


def validate(data: dict[str, np.ndarray], dt: float) -> dict:
    out = {}
    for kind, pt in (("ice", ICE), ("ev", EV)):
        sim, mod = [], []
        for vid, s in data.items():
            if f"_{kind}_" not in vid:
                continue
            sim.append(s[:, 2].sum() * dt)
            mod.append(sum(pt.rate(v, a) for v, a, _ in s) * dt)
        sim, mod = np.array(sim), np.array(mod)
        err = (mod - sim) / sim
        out[kind] = {"trips": int(len(sim)), "unit": pt.unit, "sim_mean": float(sim.mean()),
                     "model_mean": float(mod.mean()), "bias_%": float(100 * err.mean()),
                     "abs_err_%_mean": float(100 * np.abs(err).mean()), "abs_err_%_max": float(100 * np.abs(err).max()),
                     "corr": float(np.corrcoef(sim, mod)[0, 1])}
    return out


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    data, dt = collect(seed=7)            # fit on one traffic realisation ...
    fit = fit_ice(data)
    print("ICE fit (step level):", {k: round(v, 4) for k, v in fit.items()})
    print("energy.py constants:  ", {k: getattr(ICE, k) for k in ("idle_gps", "overrun_gps", "c0_gps", "c1_gps_per_kw")})
    held_out, dt = collect(seed=11)       # ... validate on another (different background traffic, queues, stops)
    val = validate(held_out, dt)
    for kind, r in val.items():
        print(f"{kind.upper()} per-trip model vs SUMO:", {k: round(v, 3) if isinstance(v, float) else v for k, v in r.items()})
    (OUT / "energy_calibration.json").write_text(json.dumps({"ice_fit": fit, "validation": val}, indent=2))


if __name__ == "__main__":
    main()
