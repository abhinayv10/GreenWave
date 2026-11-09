# ADR-0001: Use Eclipse SUMO as the simulation platform

- **Status:** accepted
- **Date:** 2026-11-09
- **Milestone:** M0

## Context
The system must be demonstrated in simulation (L2) and then on one real corridor (L3). Results must be credible
to traffic engineers, and the simulator must support fixed-time *and* actuated signals, background traffic with
queues, per-vehicle control, and emissions/energy estimates.

## Options considered
1. **Own minimal simulator** — total control, fast, trivial to install; but no realistic car-following, queue
   discharge, actuated controllers or emissions, and results would be easy to dismiss as "the model agrees with itself".
2. **Eclipse SUMO** — open source, standard in transport research; microscopic car-following, `static` and
   `actuated` traffic-light logic with detectors, OSM import (`netconvert`), built-in HBEFA/PHEM emission
   models and an electric-vehicle energy model; scriptable from Python via TraCI/libsumo. Steeper learning curve.
3. **Commercial (VISSIM, Aimsun)** — realistic but licensed, closed, not reproducible by others.

## Decision
Use SUMO 1.28.0, installed from PyPI (`eclipse-sumo`, `traci`, `libsumo`, `sumolib`) into the project venv,
so the whole stack is reproducible with `pip install -r requirements.txt` and no system installer or `SUMO_HOME`.

## Consequences
- `traci` (socket, works with `sumo-gui`) for interactive/debug runs; `libsumo` (in-process, much faster) for the
  batch evaluation matrix. Both expose the same API.
- `libsumo` is compiled against Arrow 23, so `pyarrow` is pinned to `23.*` (Streamlit pulls in pyarrow).
- Our own signal model must agree exactly with SUMO's signal behaviour; this is enforced by a test
  (`tests/test_signal_consistency.py`), because every downstream result depends on it.
- **TraCI timing convention (found by that test):** after `simulationStep()`, `getTime()` returns the *end* of
  the step just executed, while signal states (and vehicle states) reflect that step, which began at `t − dt`.
  Ignoring this gives a systematic half-step (0.5 s) error at every phase change. The advisory loop must use
  the same convention when it compares vehicle arrivals to green windows.
- `tlLogic offset` in SUMO uses the same convention as ours: phase 0 starts at t ≡ offset (mod cycle).
