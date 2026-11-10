# ADR-0012: Target speed in bands with a stop; near-stop metric

- **Status:** accepted (amends ADR-0006's choice of advised speed)
- **Date:** 2026-11-09
- **Milestone:** M2

## Context
In M2, adding the queue model (ADR-0010) raised stops on the fixed corridor from 1.25 to 1.59 per trip. We first
read this as the queue model being over-pessimistic. Trip-level analysis showed something else: all 13 extra
stops were at J1, which has a queue every cycle (clearance 5.5–6 s in all 38 measured greens).

- **Without** the queue model, the planner promised green at J1. The vehicle arrived at green start, caught
  the discharging queue, slowed to **6–8 km/h** and crossed 6–8 s after green. Not a "stop" by the metric
  (< 0.3 m/s), but in practice a stop.
- **With** the queue model, the planner correctly predicted it could not get through unimpeded and planned a
  stop. But ADR-0006 picks the *fastest* speed in a band, so the vehicle raced to the red at the limit and
  crawled to ~0 km/h. It crossed J1 at the same time as before.

So the queue model was right. Two other things were wrong:
1. ADR-0006 assumes every speed in a band gives the same outcome, so faster always means less time. That
   holds for all-green bands. In a band with a stop, departure is fixed by the signal: every speed has the
   same travel time, and a faster one only means more idling and harder braking.
2. The stop metric could not see a crawl to 7 km/h, so the two conditions were not measured on the same scale.

## Options considered
1. **Lower the queue quantile or shrink the clearance.** Would hide the effect by promising green that the
   queue then blocks. Rejected: it breaks the broken-promises result (0.25 → 0.00) that the queue model exists for.
2. **Pick the lowest-cost speed in the band over the whole horizon.** Tried first; changed nothing. The band's
   single speed also covers the signals *after* the stop, where a faster restart saves time. That outweighed
   the shorter wait, so the planner kept choosing 40.5 of a 33–42.5 km/h band.
3. **Pick the speed with the cheapest approach to the first planned stop.** The controller re-plans as soon as
   that signal is passed (ADR-0007), so the post-stop cruise speed is never used and should not influence the
   choice. Since departure from the red is fixed, this is usually the slow end of the band.
4. **Model a delayed-but-moving arrival behind a discharging queue explicitly** (shockwave model). More
   faithful, but needs per-green queue length and tail position in history; revisit if option 3 is not enough.

## Decision
Option 3. All-green band: fastest speed with ±tolerance inside the band (unchanged). Band with a stop: the
qualifying speed with the lowest cost from now until leaving the first stopped signal, ties to the faster.
Bands are still **ranked** at their fastest qualifying speed, so which band wins does not change.
Add `near_stops`: drops below 10 km/h (stops included), re-armed above 20 km/h. Report it next to `stops`.

## Result (M2 re-run, fixed corridor, 51 departures)
These numbers use the original 60 s departure grid, which sampled only 3 signal phases (ADR-0014). The
before/after comparison is like for like; for current absolute numbers see the README.

| Condition | Stops | Near-stops | Zero-stop trips | Wait (s) | Travel (s) | Fuel (g) |
|---|---|---|---|---|---|---|
| Exact plan + queue, before | 1.59 | 1.67 | 0% | 21.3 | 358.3 | 223.7 |
| Exact plan + queue, after | **0.65** | **1.02** | **61%** | 8.2 | 360.2 | 221.5 |
| Exact plan (no queue model), before | 1.25 | 1.59 | 8% | 19.9 | 356.9 | 222.5 |
| Exact plan (no queue model), after | 1.25 | 1.59 | 8% | 12.6 | 363.6 | 222.6 |

Broken promises stay at 0 with the queue model. Actuated conditions improve as well (e.g. demand-conditioned +
live state: 0.69 → 0.29 stops, wait 4.1 → 1.4 s, travel time unchanged).

## Consequences
- **Without a queue model the slow approach costs time.** Same stops and fuel, 7 s less waiting, but 6.7 s
  longer trips. A vehicle arriving later at a red loses queue position to background traffic arriving
  meanwhile (FIFO), and the release time without a queue model ignores that. With the queue model the
  release is planned after the queue clears, so the cost is +1.9 s. Revisit (option 4, or price the
  arrival rate into the release time) if the queue-free configuration matters again.
- One outlier: the 540 s departure with the queue model has 0 stops but is 40 s slower. Watch for long
  stretches at `v_min` when M3 adds the energy term.
- All-green behaviour and the M1 progression test are unchanged.
- In stop bands the advice is to slow down early (down to `v_min`), which is the eco-approach behaviour GLOSA is
  supposed to produce. Followers are protected by `v_min` = 60% of the limit.
- `near_stops` is the fairer headline comparison between conditions; `stops` stays for continuity.
