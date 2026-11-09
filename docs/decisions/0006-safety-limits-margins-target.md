# ADR-0006: Hard limits, green-window margins, and choice of the advised speed

- **Status:** accepted
- **Date:** 2026-11-09
- **Milestone:** M1

## Decision
**Hard constraints (filter, never weights):**
- `v_max` = posted limit. Never exceeded.
- `v_min` = 60% of the limit (30 km/h on a 50 road). Slower advice obstructs traffic and invites tailgating.
- Usable green = `[green start + 1 s, green end − 2 s]`. The start margin absorbs timing error and the first
  queued vehicle; the end margin keeps the target out of the yellow/dilemma zone. Yellow is never green.

**Advised speed within a band:** the fastest speed that still leaves ±2 km/h (driver tolerance) inside the band.
The driver is shown `target ± 2 km/h`; the full feasible band is in the explanation.

## Why not the band centre
All speeds in a band produce the same green/red outcome, so the centre buys tolerance nobody needs and costs
time. On the test corridor the centre (35 km/h) was ~15 s slower than the top-with-tolerance (36–38 km/h) for an
identical number of stops. Bands narrower than 2×tolerance are flagged *fragile* and penalised.

## Observed consequence (design check)
On the synthetic corridor the offsets put green **start** exactly at the 40 km/h arrival. With the 1 s start
margin, 40 km/h itself is just outside the band; the usable band is 32–38.9 km/h. Progression "design speed" is
an edge, not a centre. Worth remembering when reading real timing sheets (M6).
