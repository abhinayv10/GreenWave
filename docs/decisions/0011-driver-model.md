# ADR-0011: Driver model between advice and vehicle

- **Status:** accepted
- **Date:** 2026-11-09
- **Milestone:** M2

## Context
Real drivers do not hold an advised speed exactly, react late, and only partly comply. Field pilots (Ottawa)
found driver engagement dominated outcomes. Robustness has to be tested against this, not assumed.

## Decision
`DriverModel(compliance, bias_sd_kmh, reaction_s, noisy)` applied in the simulation runner:
- **applied speed** = limit − compliance × (limit − advised) + bias, clipped to [3 m/s, limit];
- **bias** ~ N(0, sd) per driver, persistent over the trip (someone who always drives a bit fast/slow);
- **reaction** delay before a changed advice takes effect;
- **noisy** = SUMO Krauss dawdling sigma 0.5 (imperfect speed keeping, random decelerations).

Preset `REALISTIC` = compliance 0.8, bias sd 2 km/h, reaction 2 s, noisy. The controller does not know the
driver's bias; it sees only position and speed, which is what feeds the loss-of-alignment re-plan trigger.

## Consequences
- Compliance is a population/engagement dial for M5 sweeps (0 to 1).
- Speeding above the limit is not modelled (applied speed is capped at the limit).
