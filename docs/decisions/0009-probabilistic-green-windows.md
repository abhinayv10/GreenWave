# ADR-0009: Uncertain green windows as empirical start-time samples; plan on the p = 0.85 reliable start

- **Status:** accepted
- **Date:** 2026-11-09
- **Milestone:** M2

## Context
For actuated signals the planner cannot know exactly when green starts. Options for representing this:
1. **Min / max bounds only** (J2735-style). Simple, but planning on `start_max` wastes every early return, and
   planning on `start_min` is reckless.
2. **Parametric distribution** (e.g. normal). The real distribution is bounded and multi-modal (often a spike at
   "cross gapped out at min green" plus a tail), so a normal fits badly.
3. **Empirical samples** from history. Exact shape, trivially conditioned (filter the samples), needs enough data.

## Decision
`GreenWindow.start_samples` holds empirical start times. The planner uses
`start_by(p)` = the time by which green has started with probability ≥ p (`p_required = 0.85`), and only counts
an arrival as green from there on. When stopped, expected release = median of the samples still in the future.

**Conditioning (the "congestion as predictor" idea, first form):** `ActuatedForecaster` buckets historical early
returns by cross-street flow over the previous 5 minutes (0–150 / 150–300 / 300–450 / 450+ veh/h, both
directions). At runtime it is fed the current cross flow (detector counts in deployment). Sparse buckets borrow
from neighbours, then fall back to all data.

**Live state (optional):** when the current signal state is available (SPaT-style), a window that has visibly
started becomes exact, and a window that has not started yet is conditioned on "not before now".

## Consequences
- p is a trust dial: higher p = fewer broken promises, fewer usable greens. M5 sweeps it.
- Forecast quality is measured directly (start-time error 60 s ahead, and how often the reliable start is not
  too early) in addition to trip outcomes.
- The forecaster needs history per signal per demand level; on a real corridor that comes from controller
  logs or is inferred from probe data (M4).
