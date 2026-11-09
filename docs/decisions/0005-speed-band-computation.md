# ADR-0005: Speed bands by speed sweep with sequential stop simulation; rolling 4-signal horizon

- **Status:** accepted
- **Date:** 2026-11-09
- **Milestone:** M1

## Context
We need speed *intervals* (not a single speed) that align with green windows at successive signals, and the
method must still give sensible advice when no speed avoids every stop.

## Options considered
1. **Analytic pace intersection.** With pace s = 1/v, arrival time t_i = t0 + d_i·s is linear, so each green
   window maps to a pace interval and the all-green set is their intersection. Exact and elegant, but returns
   *nothing* when no speed clears every signal, which is common over several signals.
2. **Speed sweep with sequential simulation.** For each candidate speed on a 0.5 km/h grid, simulate the trip
   signal by signal: green → pass; otherwise brake, wait for green + start-up delay, re-accelerate. Handles
   unavoidable stops, initial speed changes (reachability) and any cost function. ~40 speeds × 4 signals is trivial.
3. **Full trajectory optimisation** (eco-approach optimal control). Best energy, but outputs a speed profile a
   human can't follow; deferred.

## Decision
Option 2 as the engine; option 1 kept as an exact reference (`pace_intervals_no_stop`) and a test asserts they
agree. A **band** is a maximal run of adjacent speeds with the same outcome pattern (which signals are crossed
on green). Bands are scored by the composite cost at their advised target. Horizon: next **4** signals,
re-planned as signals are passed (rolling horizon). A single speed over 8+ signals almost never exists.

## Consequences
- Advice always exists, even when stops are unavoidable ("hold 30–34 km/h, 1 stop at J5 is unavoidable").
- Reachability is built in: arrival times include the comfortable speed change from the current speed.
- The vehicle model in the planner (accel 2.0, decel 3.0 m/s², 1 s start-up) must match the SUMO test-vehicle
  vType, or predictions drift from the simulation.
- Horizon length is a tuning parameter to revisit in M5 (longer = more foresight, more forecast error exposure).
