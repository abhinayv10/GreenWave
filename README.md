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

## Run the M1 experiment and dashboard
```powershell
.\.venv\Scripts\python.exe scripts\m1_compare.py        # SUMO: 51 advised + 51 unadvised test vehicles
.\.venv\Scripts\streamlit.exe run dashboard\app.py
```

## Results so far
**M1 (fixed-time corridor, exact signal knowledge, 500 veh/h background, 51 departures each):**

| | Drive at limit | Advised | Δ |
|---|---|---|---|
| Stops / trip | 2.73 | 1.25 | −54% |
| Waiting at red (s) | 36.1 | 19.9 | −45% |
| Travel time (s) | 348.6 | 356.9 | +2.4% |
| Fuel (g, SUMO HBEFA) | 233.4 | 222.5 | −4.7% |

Where the remaining advised stops happen: 51 of 64 were *planned* (no legal speed avoids them: bad departure
phase at J1, deliberately mistimed J5). 13 were planned green but the vehicle stopped **behind an undischarged
queue** (median stop 31 m before the stop line), which motivates M2's queue model. Travel time is about the same,
as the FIFO property predicts: advice saves stops and fuel, not time.

**M2 (robustness; 51 departures per condition; knowledge learned from a different seed than evaluation):**
run with `.\.venv\Scripts\python.exe scripts\m2_robustness.py`; full tables in `results/m2/`.

Fixed-time corridor:

| Condition | Stops | Broken promises* | Wait (s) | Travel (s) | Fuel (g) |
|---|---|---|---|---|---|
| Drive at limit | 2.73 | n/a | 36.1 | 348.6 | 233.4 |
| Advised, exact plan (M1) | 1.25 | 0.25 | 19.9 | 356.9 | 222.5 |
| + queue model | 1.59 | **0.00** | 21.3 | 358.3 | 223.7 |
| + queue, realistic driver | 1.57 | 0.00 | 22.9 | 359.2 | 214.9 |

Actuated corridor:

| Condition | Stops | Zero-stop trips | Wait (s) | Travel (s) | Fuel (g) |
|---|---|---|---|---|---|
| Drive at limit | 2.31 | 2% | 23.9 | 323.0 | 219.9 |
| Advised, nominal plan (ignores actuation) | 0.45 | 67% | 3.1 | 348.1 | 217.6 |
| Advised, history (unconditioned) | 0.63 | 67% | 7.1 | 337.5 | 211.2 |
| Advised, history conditioned on cross demand | 0.69 | 63% | 3.9 | 333.3 | 210.0 |
| + live signal state | 0.65 | 63% | 3.7 | 334.4 | 210.7 |
| Advised, oracle (recorded truth) | 0.63 | 67% | 3.9 | 333.6 | 210.0 |
| Demand-conditioned + live, realistic driver | 0.78 | 57% | 5.7 | 335.3 | 202.1 |

\*Stops at a signal the advice had planned as green (trust metric). All advised actuated conditions: 0.00.

Forecasting the main-green start 60 s ahead (365 windows): nominal plan MAE 22.6 s; history 2.4 s (p90 8.5 s);
**history conditioned on cross-street demand 1.05 s (p90 3.8 s)**; reliable start (p = 0.85) not too early in
92% of windows.

## Layout
| Path | Role |
|---|---|
| `scenarios/` | Scenario YAML: single source of truth for geometry, signal timing, demand |
| `greenwave/scenario.py` | Loads scenarios, derives progression offsets |
| `greenwave/signals/` | `SignalForecaster` interface + implementations (ADR-0003) |
| `greenwave/sim/` | SUMO network/route/signal generation, test-vehicle runner |
| `greenwave/optimizer/` | Speed-band engine, composite cost, profiles |
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
  - open: queue model is over-pessimistic (counts slow-rolls as stops), see results; dashboard pages for M2 move to M5
- [ ] **M3** Two-route network, composite cost, profiles, Pareto front, energy/EV
- [ ] **M4** Signal timing inferred from probe trajectories ("congestion as predictor")
- [ ] **M5** Evaluation matrix + Streamlit dashboard
- [ ] **M6** L3: NGSIM Lankershim Blvd (ADR-0004)
