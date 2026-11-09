# ADR-0007: Advice applied as a max-speed cap; event-driven re-planning

- **Status:** accepted
- **Date:** 2026-11-09
- **Milestone:** M1

## Context
In simulation the advice must be "followed" without bypassing the simulator's safety logic, and re-planning must
be intelligent rather than constant (brief §Feature 5).

## Decision
- **Applying advice:** `traci.vehicle.setMaxSpeed(target)`. The vehicle never exceeds the advice, but SUMO's
  car-following, red-light compliance and acceleration limits stay in charge. A queue or a red light still stops
  the car; we do not force speeds. (M2 adds driver noise and partial compliance on top.)
- **Re-plan triggers:** (1) no plan; (2) a signal was passed, so the horizon slides; (3) loss of alignment: from the
  vehicle's *actual* time/position/speed, the current target no longer reaches a signal that was planned green.
  Otherwise the advice is kept even if a marginally better band appears.
- Control interval 1 s; simulation step 0.5 s.

## Consequences
- M1 result: 9.0 re-plans/trip on an 8-signal corridor, i.e. initial + one per signal passed; zero
  "lost alignment" re-plans with exact signal knowledge. 4.0 advice changes/trip, all at signal passages.
- The free-flow alignment check does not see queues. Planned-green stops behind a queue are the main
  remaining failure (see README M1 results), handled in M2 by shifting usable green by the predicted queue
  discharge time.
