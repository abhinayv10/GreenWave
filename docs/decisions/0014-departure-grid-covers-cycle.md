# ADR-0014: Test departures must cover the signal cycle (59 s spacing, not 60 s)

- **Status:** accepted
- **Date:** 2026-11-09
- **Milestone:** M3 (found while building it; changes M1–M2 results)

## Context
M1–M2 sent a test vehicle every 60 s, intending to "cover every phase relationship" with the signals. The cycle
is 90 s, and gcd(60, 90) = 30, so the departures met the signals at only **3 distinct phases**, each repeated
17 times. Every average was an average over 3 cases.

Found because route A of the new two-route network (ADR-0013) gave 1.57 stops per trip at the limit while the
same arterial alone gave 2.73. The signals were identical step for step. A's stop lines are 8 m closer to the
start in the network, so vehicles arrive 0.6 s earlier. For one of the 3 phases, that is the difference
between stopping 1 m before J1 as it turns yellow (then again at J2, J3 and J4) and getting through on yellow.
One phase is a third of the sample.

## Options considered
1. **Spacing coprime with the cycle** (59 s): 51 departures, 51 distinct phases spread over the cycle,
   same hour of traffic.
2. **Random departure times:** also covers phases, but changes the sample between runs unless seeded, and
   leaves gaps by chance.
3. **Explicit phase sweep** (depart at t0 + k·cycle/51): perfect coverage, but couples the design to one cycle
   length; breaks when routes have different cycles.

## Decision
Option 1, in one place: `greenwave.sim.ego.departure_times()` (51 departures from 300 s, every 59 s). All
experiment scripts use it.

## Result
With phases covered, route A in the network and the standalone corridor agree (1.92 vs 1.86 stops, 333 vs
335 s, 220.7 vs 220.8 g). M1–M2 re-run on the new grid (fixed corridor):

| | Old grid (3 phases) | New grid (51 phases) |
|---|---|---|
| Stops at the limit | 2.73 | 1.86 |
| Stops advised, exact plan | 1.25 (−54%) | 1.24 (−33%) |
| Stops advised + queue model | 0.65 | 0.57 (−69%) |
| Fuel saving, advised exact plan | −4.6% | −2.0% |

## Consequences
- M1–M2 conclusions hold (fewer stops, a little less fuel, slightly longer trips), but the effect sizes were
  inflated by an unlucky phase. README tables now use the new grid. ADR-0012's before/after table compares
  on the old grid; its conclusion is unchanged.
- Any future change of cycle length must keep the spacing coprime with it (or switch to option 3).
- Sensitivity to sub-second shifts is real: a vehicle arriving at a yellow onset is a boundary case.
  Reported averages should not be read to better than a few percent.
