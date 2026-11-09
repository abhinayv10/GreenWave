# ADR-0010: Queue-aware green windows from historical clearance times

- **Status:** accepted
- **Date:** 2026-11-09
- **Milestone:** M2

## Context
M1: 13 of 64 advised stops were at a signal the advice had planned as green; the vehicle stopped behind a queue
that had not cleared yet (median 31 m before the stop line). Green at the stop line ≠ free passage.

## Options considered
1. **Analytic queue (D/D/1):** queue = arrival rate × red time, clearance = n·h/(1 − λh). Ignores that a
   coordinated corridor delivers platoons *during green* (well-progressed signals have no queue at all).
2. **Live queue length** (connected vehicles / detectors): accurate now, but plans look 2–3 minutes ahead,
   when the queue will be a different one.
3. **Historical clearance per signal:** measure, for each green, the time from green start until every vehicle
   that queued (or joined during discharge) has crossed the stop line; use a quantile.

## Decision
Option 3, q80 per signal; usable green starts at `reliable start + clearance_q80 + 1 s margin`; a stopped
vehicle's release time includes the clearance.

## Measurement detail
A first version measured "until nobody on the approach is halted". That ends when the last queued car *starts
moving*, not when it *clears the stop line*, giving ~0 s everywhere. Fixed to track queued vehicles until they
leave the approach lane. Resulting q80 (fixed corridor): J5 (mistimed) 12 s, J1 5.5 s, J3 3.5 s, well-progressed
signals 0 s, consistent with traffic-flow theory.

## Consequences
- Not conditioned on time of day or demand yet; trivially extendable like ADR-0009's buckets.
- Lane-agnostic: uses the worse of the approach lanes.
