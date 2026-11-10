# Green-Wave Speed Advisory & Signal-Aware Navigation

Computes robust speed *bands* that let a vehicle cross successive signals on green, compares routes by
signal-aware cost, and evaluates it all in SUMO, first on synthetic corridors (L2), then on a real one (L3).

## Setup
```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m pytest -q
```
SUMO comes from PyPI (`eclipse-sumo`); no system install or `SUMO_HOME` needed.

## Build and run the synthetic corridor
```powershell
.\.venv\Scripts\python.exe -m greenwave.sim.corridor scenarios/corridor_fixed.yaml
# headless
.\.venv\Lib\site-packages\sumo\bin\sumo.exe -c build\corridor_fixed\corridor.sumocfg
# with GUI
.\.venv\Lib\site-packages\sumo\bin\sumo-gui.exe -c build\corridor_fixed\corridor.sumocfg
```

## Run the M3 experiments
```powershell
.\.venv\Scripts\python.exe scripts\m3_energy_calibration.py   # fit + held-out check of the energy models
.\.venv\Scripts\python.exe scripts\m3_routes.py               # route choice: 2 routes x 4 driving modes x ICE/EV
```

## Run the M4 experiment
```powershell
.\.venv\Scripts\python.exe scripts\m4_probe_inference.py      # timing from probes: accuracy + trips
```

## Run the M1 experiment and dashboard
```powershell
.\.venv\Scripts\python.exe scripts\m1_compare.py        # SUMO: 51 advised + 51 unadvised test vehicles
.\.venv\Scripts\streamlit.exe run dashboard\app.py
```

## Results so far
All results use 51 test departures, one every 59 s, so each meets the 90 s signal cycle at a different phase
(ADR-0014; an earlier 60 s grid sampled only 3 phases and inflated the effects).

**M1 (fixed-time corridor, exact signal knowledge, 500 veh/h background):**

| | Drive at limit | Advised | Δ |
|---|---|---|---|
| Stops / trip | 1.86 | 1.24 | −33% |
| Near-stops / trip (< 10 km/h) | 2.24 | 1.39 | −38% |
| Waiting at red (s) | 25.4 | 8.3 | −67% |
| Travel time (s) | 334.6 | 349.0 | +4.3% |
| Fuel (g, SUMO HBEFA) | 220.8 | 216.3 | −2.0% |

Where the remaining advised stops happen: 49 of 63 were *planned* (no legal speed avoids them: bad departure
phase at J1, deliberately mistimed J5). 14 were planned green at J5 but the vehicle stopped **behind an
undischarged queue** (median 46 m before the stop line), which motivates M2's queue model. Advice saves stops
and fuel, not time (FIFO). Without a queue model, the slow approach to a planned stop (ADR-0012) loses queue
position to traffic arriving meanwhile, which costs time.

**M2 (robustness; knowledge learned from a different seed than evaluation):**
run with `.\.venv\Scripts\python.exe scripts\m2_robustness.py`; full tables in `results/m2/`.

Fixed-time corridor:

| Condition | Stops | Near-stops | Zero-stop trips | Broken promises* | Wait (s) | Travel (s) | Fuel (g) |
|---|---|---|---|---|---|---|---|
| Drive at limit | 1.86 | 2.24 | 0% | n/a | 25.4 | 334.6 | 220.8 |
| Advised, exact plan (M1) | 1.24 | 1.39 | 12% | 0.27 | 8.3 | 349.0 | 216.3 |
| + queue model | **0.57** | **0.90** | **65%** | **0.00** | 4.3 | 350.9 | 215.4 |
| + queue, realistic driver | 0.98 | 1.18 | 33% | 0.00 | 10.6 | 355.9 | 208.3 |

Actuated corridor:

| Condition | Stops | Near-stops | Zero-stop trips | Wait (s) | Travel (s) | Fuel (g) |
|---|---|---|---|---|---|---|
| Drive at limit | 1.59 | 1.75 | 27% | 14.3 | 311.1 | 208.5 |
| Advised, nominal plan (ignores actuation) | 0.12 | 0.14 | 88% | 1.0 | 337.0 | 209.7 |
| Advised, history (unconditioned) | 0.24 | 0.35 | 84% | 1.7 | 324.5 | 204.6 |
| Advised, history conditioned on cross demand | 0.27 | 0.37 | 78% | 1.3 | 322.8 | 204.0 |
| + live signal state | 0.27 | 0.35 | 78% | 1.2 | 324.3 | 205.9 |
| Advised, oracle (recorded truth) | 0.12 | 0.16 | 88% | 0.8 | 322.7 | 203.1 |
| Demand-conditioned + live, realistic driver | 0.41 | 0.43 | 67% | 2.2 | 327.8 | 195.7 |

On the actuated corridor, every advised condition avoids most stops; what better signal knowledge buys is
**time**: the nominal plan is 14 s slower than history-based forecasts because it waits for green starts that
actuation brings forward.

Near-stops count every drop below 10 km/h (stops included), so a crawl behind a queue counts too (ADR-0012).

\*Stops at a signal the advice had planned as green (trust metric). All advised actuated conditions: 0.00.

Forecasting the main-green start 60 s ahead (361 windows): nominal plan MAE 22.6 s; history 2.4 s (p90 8.5 s);
**history conditioned on cross-street demand 1.06 s (p90 4.0 s)**; reliable start (p = 0.85) not too early in
92% of windows.

**M3 (route choice and energy; ADR-0013):** route A is the fixed-time arterial (3.8 km, 50 km/h, 8 signals),
route B a bypass (6.1 km, 70 km/h, 2 signals). Every option (route × {limit, eco, balanced, fastest} × fuel
car / EV) is driven for every departure; run with `scripts\m3_routes.py`, full tables in `results/m3/`.

The options (means over 51 departures):

| Route, driving | Stops | Travel (s) | Fuel (g) | EV battery (Wh) |
|---|---|---|---|---|
| A at limit | 1.92 | 332.8 | 220.7 | 377 |
| A advised (eco) | 0.37 | 345.5 | 212.8 | 308 |
| B at limit | 0.29 | 327.5 | 286.0 | 645 |
| B advised (eco) | 0.00 | 336.1 | 283.3 | 625 |

- **Advice saves an EV far more energy than a fuel car** on A: −18% vs −4%. An engine burns ~0.5 g/s just to
  run, so a slower trip costs fuel by the second. EV consumption is mostly speed-dependent drag, with no idle loss.
- **The bypass costs an EV twice the energy of the arterial** (fuel car: +33%): drag grows with speed², and B
  is both longer and faster.

Route-choice policies, advised driving. Cost in equivalent seconds per trip; lower is better.

| Powertrain, profile | Always A | Always B | Signal-aware | Oracle |
|---|---|---|---|---|
| Fuel, eco | 823.2 | 952.4 | **823.2** | 823.2 |
| Fuel, balanced | 507.8 | 541.6 | 509.4 | 504.5 |
| Fuel, fastest | 347.8 | 333.9 | **329.3** | 329.1 |
| EV, eco | 443.6 | 504.8 | 450.1 | 440.3 |
| EV, balanced | 382.4 | 392.4 | **376.6** | 374.6 |
| EV, fastest | 347.8 | 333.9 | **329.3** | 329.1 |

Signal-aware routing picks the route with the lowest *predicted* cost at departure. Predictions come from the
advisory controller rolled out in the planner model, corrected by each option's mean error measured on a
history run. Uncorrected, the model underestimates the bypass's energy (−6 to −7%: real trips fluctuate behind
traffic) and overestimates the arterial's stops (planned stops often become roll-throughs). That sent eco fuel
cars onto the bypass on 39% of trips (cost 852 instead of 823). Corrected, mean energy error is below 1% for
fuel cars and below 2.5% for EVs.

Where the best route depends on the departure, signal-aware routing captures most of the gain: 96% of the
oracle's advantage over the best fixed route for *fastest*, 75% for *EV balanced*. Where one route nearly
always wins (eco → A), there is little to gain, and per-trip prediction noise costs 0.3–1.5% (fuel balanced,
EV eco). The gain is mostly time, not energy.

**M4 (signal timing from probe trajectories only; ADR-0015):** a share of ordinary cars (main road and cross
streets) report position and speed once per second. Stop-line crossings and queue releases are "green now"
samples. Folding them over candidate cycle lengths recovers the cycle, then each signal's green. Run with
`scripts\m4_probe_inference.py`; full tables in `results/m4/`.

| Condition | Stops | Broken promises | Travel (s) | Fuel (g) |
|---|---|---|---|---|
| Fixed-time: true timing plan | 0.57 | 0 | 350.9 | 215.4 |
| Fixed-time: probes, 5% of cars, 1 h | 0.51 | 0 | 351.3 | 215.2 |
| Actuated: controller logs | 0.27 | 0 | 322.8 | 204.0 |
| Actuated: probes 10% | 0.27 | 0 | 329.9 | 207.3 |
| Actuated: probes 25% | 0.24 | 0 | 325.0 | 204.4 |

- **Fixed-time timing is recovered almost exactly:** cycle error 0.00 s; green start and end within 0.2 s
  on average from 10% probes over 30 min, or 5% over an hour. Advice from it matches advice from the plan.
- **The end of green needs cross-street probes.** On a progressed arterial, platoons pass early and the end
  of green is empty in both directions. The main road alone estimated green ends 6–27 s early.
- **Actuated signals need more probes:** cycle and yield point are as good as from controller logs, but each
  cycle's start is harder. 10% probes match the logs on stops at +2% travel time; 5% still removes most
  stops (0.22 vs 1.59 at the limit), but trips are 30 s slower.

## Layout
| Path | Role |
|---|---|
| `scenarios/` | Scenario YAML: single source of truth for geometry, signal timing, demand |
| `greenwave/scenario.py` | Loads scenarios, derives progression offsets |
| `greenwave/signals/` | `SignalForecaster` interface + implementations (ADR-0003) |
| `greenwave/sim/` | SUMO network/route/signal generation, test-vehicle runner |
| `greenwave/sim/route.py` | Routes as edge lists + signals at along-route distances (ADR-0013) |
| `greenwave/sim/network.py` | Two-route network (arterial A, bypass B) from `scenarios/two_route.yaml` |
| `greenwave/sim/probes.py` | Probe-vehicle traces: a share of ordinary cars reporting once per second (M4) |
| `greenwave/signals/probe_inferred.py` | Signal timing inferred from probe traces alone (ADR-0015) |
| `greenwave/optimizer/` | Speed-band engine, composite cost, profiles (energy-aware from M3) |
| `greenwave/energy.py` | Fuel (ICE) and battery (EV) models, matched to SUMO (`scripts/m3_energy_calibration.py`) |
| `greenwave/routing.py` | Signal-aware route cost: the advisory controller rolled out in the planner model |
| `greenwave/advisory.py` | Turns plans into advice, decides when to re-plan |
| `greenwave/viz/` | Plotly charts (time-space, speed-vs-cost, stops distribution) |
| `dashboard/` | Streamlit app |
| `scripts/` | Experiments |
| `docs/decisions/` | Architecture Decision Records: why every major choice was made |
| `tests/` | Includes the SUMO-vs-model signal consistency test |

## Milestones
- [x] **M0** Setup, scenario format, fixed-time signal model, SUMO corridor, model↔SUMO consistency
- [x] **M1** Deterministic speed bands, ego vehicle in SUMO following advice, time-space diagram
- [x] **M2** Actuated signals, probabilistic windows, queues, driver model, reliability threshold (p), re-plan triggers
  - fixed: the queue model was right; the advised speed in bands with a planned stop was wrong (ADR-0012)
  - open: dashboard pages for M2 move to M5
- [x] **M3** Two-route network, composite cost, profiles, Pareto front, energy/EV (ADR-0013)
  - found on the way: the 60 s departure grid sampled only 3 signal phases (ADR-0014; M1–M2 re-run)
  - open: in close calls, per-trip prediction noise flips route choices (EV eco −1.5% vs always A): add a
    switching margin or prediction uncertainty; Pareto charts and route pages move to the M5 dashboard
- [x] **M4** Signal timing inferred from probe trajectories (ADR-0015)
  - open: real GPS noise and irregular reporting, time-of-day plan changes, probe-only queue estimation
    (all to test on Lankershim in M6)
- [ ] **M5** Evaluation matrix + Streamlit dashboard
- [ ] **M6** L3: NGSIM Lankershim Blvd (ADR-0004)
