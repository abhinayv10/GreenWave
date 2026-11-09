# ADR-0008: Actuated signals are modelled as coordinated semi-actuated, using SUMO's native controller

- **Status:** accepted
- **Date:** 2026-11-09
- **Milestone:** M2

## Context
"Actuated" covers several control modes. On arterials, the dominant one is **coordinated semi-actuated**:
a fixed cycle; the main street is the coordinated phase and always yields at a fixed point in the cycle; the
cross street is detector-actuated (min green, gap-out, max green); time the cross street does not use returns
to the main street ("early return to green"). Fully free-running actuation (no common cycle) is more common at
isolated intersections and at night.

## Decision
Model actuated corridors as coordinated semi-actuated using SUMO's `type="actuated"` with
`coordinated=true`, `cycleTime`, and `earliestEnd = latestEnd` on the main phase.

Consequence for prediction: **main green END is fixed; main green START is uncertain and only ever earlier
than nominal.** The uncertainty is driven by cross-street demand.

## Implementation findings (each found by measuring, not assumed)
1. **Yield point placement.** With the yield at timeInCycle = main_green, an early-started main green wraps
   the cycle boundary and SUMO intermittently ends it at `minDur` (5 s), splitting greens. Placing the yield at
   timeInCycle = C − 1 (SUMO offset shifted accordingly) keeps each main green within one cycle; fixed.
2. **`jam-threshold`.** A value > 0 is needed so the single cross lane (whose left turn is permissive `g`)
   gets a usable detector. But it also makes detectors ignore vehicles stopped on them longer than the threshold;
   at 30 s, long cross queues became invisible. Set to 100000.
3. **Route sorting.** SUMO reads route files incrementally and assumes departures are sorted. Time-profiled
   cross flows written per intersection were silently dropped (J3 inserted 140 of ~500 vehicles). Flows are now
   sorted by `begin`. (M1 used constant flows starting at 0, so it was unaffected.)

After these fixes (history seed 101): 39–40 cycles/hour per signal, main green ends at the yield point, and the
early return depends clearly on cross demand. Example J3, mean early return: quiet 22.4 s, busy 5.9 s,
moderate 17.2 s.

## Consequences / not covered
- Free-running actuated intersections (both green start and end uncertain, cycle drifts) are **not** modelled
  yet. The forecaster interface supports them (sample-based windows); the controller model does not. Revisit if the
  L3 corridor runs free.
- Pedestrian calls, transit priority and pre-emption are not modelled.
