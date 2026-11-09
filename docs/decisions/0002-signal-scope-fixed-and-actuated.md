# ADR-0002: Support fixed-time and actuated signals; exclude adaptive

- **Status:** accepted
- **Date:** 2026-11-09
- **Milestone:** M0

## Context
Signals fall into three predictability classes:
- **Fixed-time** — repeating plan (cycle, splits, offset), possibly changing by time of day. Deterministic once the
  plan and a clock reference are known.
- **Actuated** — phases extend/terminate/skip based on detector demand, within min/max green bounds. Future
  phase changes are a *distribution*.
- **Adaptive** (SCOOT, SCATS, ...) — the plan itself is re-optimised cycle by cycle. Forecasts beyond the current
  cycle are weak; field projects (Ottawa EcoDrive, PrioBike Hamburg) report this as the main cause of bad advice.

## Decision
Support fixed-time and actuated. Adaptive intersections are out of scope; on a real corridor they are flagged
and the advisory is suppressed through them rather than modelled.

## Consequences
- The signal forecaster must represent uncertainty from the start (green windows carry min/likely/max bounds,
  mirroring SAE J2735 SPaT `minEndTime`/`maxEndTime`/`likelyTime`), even though fixed-time windows have zero spread.
- Actuated forecasting is conditioned on demand on the main and cross roads: under saturation actuated phases
  max out and behave almost like fixed-time; at low demand they gap out and become less predictable.
- The real corridor (ADR-0004) must not depend on adaptive control.
