# ADR-0015: Signal timing inferred from probe trajectories

- **Status:** accepted
- **Date:** 2026-11-09
- **Milestone:** M4

## Context
M1–M3 assume the advisory knows the signal timing: a correct timing plan (fixed-time) or controller logs
(actuated, ADR-0009). Many cities publish neither. A fleet of connected cars reports where it is, though.
**Can the timing be recovered from ordinary vehicles' traces alone, and what does the advice lose?**
ADR-0003 already reserved a `ProbeInferred` source behind the `SignalForecaster` interface.

## Options considered
1. **Per-cycle state estimation** (hidden Markov model of the signal state from each cycle's probes). Natural
   for actuated signals, but at realistic penetration a cycle has 0–3 probes per approach: too little.
2. **Fold over the cycle.** Fixed-time and coordinated signals repeat every cycle C, so all observations,
   from every cycle, can be stacked on one circle of length C. Sparse data per cycle becomes dense data per phase.
3. **Learn from stops and travel-time delays only** (the classic "probe delay" methods). Uses less of the trace.

## Decision
Option 2, with evidence and rules chosen from what failed first:

- **Data:** a share p of all ordinary cars, main road *and* cross streets, report (time, edge, lane position,
  speed) once per second (map-matched GPS). Test vehicles are never probes. No signal state, no detectors.
- **Evidence:** a stop-line crossing or a queue release happens only during its movement's green, so each is a
  "green now" sample for the main road or the cross street.
- **Cycle:** fold every movement's samples at candidate C (40–150 s); the true C leaves each movement's red as
  an empty arc. Score = mean largest-empty-arc fraction. Multiples of C score lower (same arc, longer circle).
- **Main green start:** end of the main road's empty arc (queues release right after green in many cycles).
- **Main green end: from the cross street**, as cross green start minus the intergreen (yellow + all-red; a
  published standard value, taken as known). *First attempt used the main road's last crossing.* On a
  progressed arterial platoons pass early, so the end of green is empty in both directions and looks like red:
  green ends were estimated 6–27 s early at J3, J4, J6, even with 25% probes. Cross traffic arrives at random,
  so a cross queue releases at cross green start every cycle: a sharp edge, even with few probes.
- **Robustness:** the red arc may contain up to 1% of samples (stray events), and events in the first 180 s
  are dropped. That warm-up is the same as the controller-log history's: two cross-street crossings during
  SUMO's start-up, before the coordinated controllers settle, had moved one yield-point estimate by 9.5 s.
- **Actuated:** the yield point (fixed in the cycle) comes from the cross-street rule above. Each cycle's main
  start is *bracketed*: no later than the first main sample, no earlier than the last cross sample +
  intergreen; the midpoint is used where both bounds exist (enough cycles), else the upper bound. *First attempt
  used the upper bound only*: late by a median 5.5 s at 10% (p90 38 s). The advice then waited for greens
  that had already started: trips +34 s and more fuel than driving at the limit.
- **Output:** a `Scenario` with inferred timing (fixed), or that plus a per-cycle green history (actuated). The
  existing `FixedTimeForecaster` / `ActuatedForecaster` consume it unchanged.

## Results (scripts/m4_probe_inference.py; history seed 101, evaluation seed 1, 51 departures)
**Timing accuracy, fixed-time corridor** (8 signals; error vs the timing plan):

| Probes | Observed | Cycle error | Green start MAE | Green end MAE (max) |
|---|---|---|---|---|
| 2% | 57 min | 0.04 s | 0.8 s | 8.0 s (27 s): too few cross probes, falls back to the main road |
| 5% | 30 min | 0.00 s | 0.6 s | 4.3 s (24 s) |
| 5% | 57 min | 0.00 s | 0.4 s | 0.3 s (1.0 s) |
| 10% | 30 min | 0.00 s | 0.2 s | 0.2 s (0.5 s) |
| 25% | 15 min | 0.00 s | 0.2 s | 0.2 s (0.5 s) |

**Trips:**

| Condition | Stops | Broken promises | Wait (s) | Travel (s) | Fuel (g) |
|---|---|---|---|---|---|
| Fixed: drive at limit | 1.86 | – | 25.4 | 334.6 | 220.8 |
| Fixed: true plan + queue | 0.57 | 0 | 4.3 | 350.9 | 215.4 |
| Fixed: probes 5% + queue | 0.51 | 0 | 4.2 | 351.3 | 215.2 |
| Fixed: probes 10% + queue | 0.53 | 0 | 4.2 | 351.2 | 215.3 |
| Actuated: drive at limit | 1.59 | – | 14.3 | 311.1 | 208.5 |
| Actuated: controller logs + demand + queue | 0.27 | 0 | 1.3 | 322.8 | 204.0 |
| Actuated: probes 5% + queue | 0.22 | 0 | 1.2 | 341.1 | 212.3 |
| Actuated: probes 10% + queue | 0.27 | 0 | 1.7 | 329.9 | 207.3 |
| Actuated: probes 25% + queue | 0.24 | 0 | 1.4 | 325.0 | 204.4 |

Actuated main-green start forecast 60 s ahead (MAE): controller logs 1.06 s; probes 5% 11.7 s, 10% 5.0 s
(3.9 s with detector demand conditioning), 25% 3.3 s (2.5 s). The reliable start (p = 0.85) is not too early in
86–100% of windows.

## Consequences
- **Fixed-time: probes are as good as the timing plan** from about 5% penetration with an hour of data, or
  10% with 30 minutes. A city does not need to publish timing sheets for fixed-time advice to work.
- **Actuated: probes recover the structure** (cycle, yield point) as well as controller logs, but the
  per-cycle actuation is harder. 10% probes give the same stops and 2% more travel time than controller
  logs; 5% still removes most stops but costs time and a little fuel.
- **Cross-street probes are essential.** The main road alone cannot see the end of green on a well-progressed
  arterial. A probe source that only covers the main road (e.g. one navigation app's route) needs another
  source for the end of green.
- Assumptions to revisit on real data (M6, Lankershim): exact map-matched positions (real GPS has metres of
  error and irregular reporting), a known intergreen, and timing that does not change during the observation
  window (time-of-day plans would need change-point detection). The queue model still comes from stop-line
  detectors (ADR-0010); probe-only queue estimation is not attempted.
- Inference is fast (folding ~10⁴ samples × 4,000 candidate cycles in seconds), so re-estimating every few
  minutes from a rolling window is practical.
