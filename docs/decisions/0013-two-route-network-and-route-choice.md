# ADR-0013: Two-route network, signal-aware route choice, energy term and profiles

- **Status:** accepted
- **Date:** 2026-11-09
- **Milestone:** M3

## Context
M1–M2 advise speed on one fixed route. M3 asks the navigation question: **given what we know about the
signals, which route should this vehicle take, and how should it be weighed against energy?** That needs
(1) a network with a real choice, (2) a predicted cost per route that reflects the signals, (3) an energy term
the planner can compute, and (4) user profiles that pick different trade-offs.

Constraint from M1–M2: the whole stack assumes one straight eastbound road where "position" is the x
coordinate (`eastbound_route`, `eastbound_stop_lines`, `getPosition()[0]`, recorder approach edges).

## Options considered

**Network**
1. **Grid** (e.g. 3×3 signals): many routes, but every route shares most signals, so they are not
   distinct choices, and offsets can't be coordinated along every path.
2. **Two parallel routes between one origin and one destination:** route A is the existing 8-signal arterial;
   route B is a longer, faster bypass with few signals. One decision point, two clearly different
   characters, and each route can still be studied with the M1–M2 tools.

**Predicted route cost**
1. **One constant speed over every signal on the route:** cheap, but a single speed over 8 signals almost
   never exists (ADR-0005), so A would look worse than the controller actually makes it.
2. **Roll out the controller in the planner model:** compute_plan with the normal 4-signal horizon, follow
   its target to the next signal, re-plan, repeat. The prediction matches what the advised vehicle will do.

**Energy in the planner**
1. **Use SUMO's emission output:** only available after driving, so it can't rank options in advance.
2. **Analytic energy from the planner trajectory:** rolling + aero resistance while cruising, kinetic
   energy lost per stop, idle while waiting. ICE: a Willans-line fuel model (idle rate + tractive energy / efficiency).
   EV: tractive energy / drive efficiency, minus a regen fraction of braking energy, no idle loss.

## Decision
- **Network:** origin → split S → {A: existing `corridor_fixed` arterial, 3.85 km, 50 km/h, 8 signals;
  B: ramp, bypass at 70 km/h, ramp, ~4.5–5 km, 2 signals} → merge M → destination. B's geometry is tuned
  so that **neither route dominates**: B's mean time at the limit is within ±10% of A's, and B is
  longer and faster, so it costs more energy per km. Fixed-time signals first; actuated
  reuses ADR-0008/0009 later.
- **Route abstraction first:** a `Route` (edge list + signals at along-route stop-line distances) replaces the
  x coordinate everywhere; vehicle position becomes along-route distance (`traci.vehicle.getDistance`).
  The corridor becomes a one-route network, and M2 is re-run as a regression check.
- **Route cost:** controller rollout (option 2) at the decision point, per route. The route is fixed once
  chosen (no switching mid-route: there is only one decision point).
- **Energy:** analytic model (option 2) for ICE and EV, with parameters matched to the SUMO vTypes. It is
  **validated against SUMO** (HBEFA fuel for ICE; SUMO battery device for EV) the way M0 validated the
  signal model: the correlation and bias between predicted and simulated energy per trip go into results.
- **Cost and profiles:** J = w_T·T + w_S·S + w_W·W + w_E·E, still in equivalent seconds. Profiles
  (eco / balanced / fastest) × powertrain (ICE / EV) choose route and band together.
- **Pareto front:** per departure, every option (route × {limit, advised per profile}) gives (time, energy,
  stops). Report the front and where each profile lands on it.

## Implementation notes (decided while building)
- **B geometry** (`scenarios/two_route.yaml`): connectors 400 m across and 1500 m up, straight section
  between them, 6.13 km total; 70 km/h on the whole route, so one `v_max` per route still holds. Signals K1, K2
  on the straight, cycle 90 s, coordinated at 70 km/h. **Calibration (51 departures, at the limit):** B takes
  0.98× A's time and is faster on 31 of 51 departures; B uses ~30% more fuel. Neither route dominates.
- **Test vehicles start at S** (the decision point) and end at M. Background traffic stays on its own route,
  so route A behaves like the standalone corridor (checked: 1.92 vs 1.86 stops, 333 vs 335 s).
- **Energy models** (`greenwave/energy.py`). EV: SUMO's Energy model structure with identical parameters
  (1500 kg, CdA 0.70, Cr 0.01, η 0.9 / regen 0.6, 300 W auxiliaries); the vType is generated from the same
  numbers. ICE: idle 0.49 g/s, fuel cut-off 0.09 g/s while braking, 0.50 + 0.052·P[kW] g/s when driving,
  fitted to HBEFA4 petrol Euro-4 (step R² 0.96). **Held-out per-trip check** (different traffic seed,
  36 trips each, 30–70 km/h): ICE bias −0.2%, mean |error| 2.2%; EV bias +1.8%, mean |error| 1.8%.
- **Energy price in the cost** (`weights_for`): monetary parity with time in *balanced* (time EUR 12/h,
  petrol EUR 1.80/L, electricity EUR 0.30/kWh → 0.725 s per g, 0.09 s per Wh); *eco* ×3, *fastest* ×0.
  M1–M2 profiles stay energy-blind, so their results are unchanged.
- **Policies evaluated** (`scripts/m3_routes.py`): every option (2 routes × {limit, eco, balanced, fastest}
  × {ICE, EV}) is driven for every departure, so always-A, always-B, signal-aware (lowest *predicted* cost)
  and oracle (lowest *realised* cost) are compared on identical trips.

- **Prediction correction** (added after the first run): the raw rollout underestimated B's energy by 6–7%
  (real trips fluctuate behind traffic; the model cruises steadily) and overestimated A's stops (planned stops
  that the slow approach turns into roll-throughs, ADR-0012). That shrank the predicted A/B energy gap by ~40%
  and sent eco fuel cars onto B on 39% of trips. Fix: each option's mean error (energy ratio, time/stops/wait
  bias) is learned on the history seed and applied on the evaluation seed, the same pattern as the queue model.

## Results (51 departures; full tables in results/m3/)
- Corrected predictions, evaluation seed: mean energy error < 1% (fuel) and < 2.5% (EV); time MAE 4–9 s.
- Signal-aware routing vs the best fixed route vs oracle (cost, equivalent s/trip): fastest 329.3 / 333.9 /
  329.1 (both powertrains); EV balanced 376.6 / 382.4 / 374.6; fuel eco 823.2 / 823.2 / 823.2; fuel balanced
  509.4 / 507.8 / 504.5; EV eco 450.1 / 443.6 / 440.3.
- Advice on A saves an EV 18% energy and a fuel car 4%. B costs an EV 2× A's energy, a fuel car +33%.

## Consequences
- Observed: both powertrains choose the same route on eco (A). The ICE/EV difference is in how much
  *advice* is worth (−18% vs −4% energy), not in the route choice.
- In close calls, per-trip prediction noise flips the choice and loses 0.3–1.5% vs the better fixed route.
  Next step: switch away from the usually-better route only when the predicted advantage exceeds the
  option's prediction spread (learned on history).
- Expected, and worth testing: stops cost an EV less (regen), so the eco choice can differ between ICE
  and EV, and the arterial with advice may beat the bypass on energy but not on time.
- Static routing (always the shorter free-flow time) is the baseline. Signal-aware routing should win
  on departures where A's progression is bad.
- The Route refactor touches ego, recorder, analysis and corridor. Results may move slightly, because
  along-route distance includes internal junction lanes and the x coordinate didn't. Any shift beyond
  noise is a bug.
- Revisit the network choice if the two routes turn out to have a fixed winner for every departure
  (no real choice), or if M6 needs a real network.
