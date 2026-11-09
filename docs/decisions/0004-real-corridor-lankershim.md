# ADR-0004: Real corridor for L3: NGSIM Lankershim Boulevard (Los Angeles)

- **Status:** accepted (provisional until the data is downloaded and inspected)
- **Date:** 2026-11-09
- **Milestone:** M6 (data also used to validate probe inference in M4)

## Context
L3 means running the system on one real corridor. To *evaluate* the system there (not just run it), we need
ground truth for both signals and traffic. We also want to validate the "infer signal timing from vehicle
trajectories" method (M4) against known timings.

## Options considered
1. **NGSIM Lankershim Blvd** — real arterial with signalised intersections; FHWA dataset includes
   vehicle trajectories (10 Hz), **signal timing**, loop detector data, CAD/GIS geometry. Old (2005) and short
   (≈500 m, a handful of intersections), but has ground truth for *both* signals and vehicles.
2. **Hamburg open traffic-light data** — live real signal states via SensorThings API, large network; but no
   vehicle trajectories, and many intersections are adaptive (excluded by ADR-0002).
3. **NGSIM Peachtree St (Atlanta)** — trajectories available; signal timing files not confirmed.
4. **Self-collected local corridor** — most compelling demo, but needs many cycles of manual timing collection.

## Decision
Use **Lankershim Blvd** as the L3 corridor: geometry from OSM/CAD into `netconvert`, signal programmes from the
timing data, traffic demand calibrated from the trajectories, and the trajectories reused as "probe data" to test
signal inference against the real timings. Hamburg remains an optional second demo for live real-signal forecasts.

## Consequences
- Corridor is short; results there demonstrate correctness and transfer, while the bigger statistical evaluation
  stays on synthetic corridors.
- Must verify on download: timing format, whether signals were fixed-time or actuated during collection, and
  clock alignment between timing and trajectory data.

Source: FHWA-HRT-07-029, Lankershim Boulevard Dataset fact sheet.
