# ADR-0003: All signal knowledge goes through one SignalForecaster interface

- **Status:** accepted
- **Date:** 2026-11-09
- **Milestone:** M0

## Context
Signal information will come from very different places over the project: the true SUMO programme (oracle), a
known timing plan, timings inferred from probe-vehicle trajectories, a demand-conditioned actuated model, and on
the real corridor, published timing sheets. The brief requires that simulated data can be replaced by real data
"without redesigning the optimisation engine".

## Decision
The optimiser never reads signals directly. It asks a `SignalForecaster` for the green windows of a given
signal *movement* over a time horizon. Each window is a `GreenWindow` with likely start/end plus min/max bounds
and a confidence. Implementations: `Oracle`, `FixedTimePlan`, `ProbeInferred`, `ActuatedProbabilistic`.

"Green" means the protected/permissive green of the movement the vehicle will make. **Yellow is not green**:
the advisory never targets arriving during yellow (safety, see brief §7).

## Consequences
- Swapping data sources is a one-line change, and we can measure the cost of imperfect knowledge directly:
  run the same scenario with `Oracle` vs `ProbeInferred` and compare outcomes.
- Signals are keyed by movement (signal id + approach + turn), not by intersection, because a left-turn arrow
  and the through green are different phases. M1 uses through movements on the arterial only.
