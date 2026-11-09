import math

from greenwave.optimizer.band import (KMH, BandConfig, SignalAhead, compute_plan, evaluate_speed,
                                      pace_intervals_no_stop)
from greenwave.optimizer.cost import CostWeights
from greenwave.signals.fixed_time import MAIN_THROUGH, FixedTimeForecaster, FixedTimePlan
from greenwave.vehicle import VehicleParams, arrival_time, speed_at

VEH = VehicleParams()
W = CostWeights()


def test_arrival_time_kinematics():
    # cruise
    assert math.isclose(arrival_time(100, 10, 10, VEH), 10)
    # accelerate 0 -> 10 at 2 m/s2: 5 s over 25 m, then 75 m at 10 m/s
    assert math.isclose(arrival_time(100, 0, 10, VEH), 5 + 7.5)
    # distance shorter than the speed change: 0.5*a*t^2 = 4 -> t = 2
    assert math.isclose(arrival_time(4, 0, 10, VEH), 2)
    assert math.isclose(speed_at(4, 0, 10, VEH), 4)
    # decelerate 14 -> 8 at 3 m/s2: 2 s over 22 m
    assert math.isclose(arrival_time(22, 14, 8, VEH), 2)


def _corridor(offsets, greens, xs, cycle=90):
    plans = {(f"S{i}", MAIN_THROUGH): FixedTimePlan(cycle, o, g) for i, (o, g) in enumerate(zip(offsets, greens))}
    sigs = [SignalAhead(f"S{i}", x) for i, x in enumerate(xs)]
    return FixedTimeForecaster(plans), sigs


def test_sweep_matches_analytic_pace_intervals():
    """For a vehicle already cruising (v0 = v), the all-green band from the sweep equals the exact pace intersection."""
    v_prog = 40 * KMH
    xs = [400, 750, 1300, 1650]
    fc, sigs = _corridor([x / v_prog for x in xs], [50, 45, 50, 40], xs)
    cfg = BandConfig.for_limit(50 * KMH, grid_step=0.1 * KMH)
    exact = pace_intervals_no_stop(0.0, tuple(sigs), fc, cfg)
    assert exact, "a progression band must exist"
    all_green = [e.speed for e in (evaluate_speed(v, 0.0, v, tuple(sigs), fc, cfg, VEH, W)
                                   for v in [cfg.v_min + i * cfg.grid_step for i in range(200)] if v <= cfg.v_max)
                 if all(e.pattern)]
    lo, hi = exact[0]
    assert abs(min(all_green) - lo) <= cfg.grid_step and abs(max(all_green) - hi) <= cfg.grid_step
    # Offsets put green START at the 40 km/h arrival, and the first second of green is excluded (start margin),
    # so the design speed is just above the band's top edge.
    assert hi < v_prog and v_prog - hi < 2 * KMH


def test_plan_prefers_progression_over_speeding():
    """Signals timed for 40 km/h: driving at 50 hits red; the plan must recommend a band containing 40."""
    v_prog = 40 * KMH
    xs = [400, 750, 1300, 1650]
    fc, sigs = _corridor([x / v_prog for x in xs], [50, 45, 50, 40], xs)
    cfg = BandConfig.for_limit(50 * KMH)
    plan = compute_plan(0.0, v_prog, sigs, fc, cfg, VEH, W)
    assert all(plan.best.pattern)
    # target = fastest speed leaving +/- tolerance inside the band
    assert math.isclose(plan.best.target, plan.best.v_hi - cfg.driver_tolerance, abs_tol=cfg.grid_step)
    at_limit = evaluate_speed(cfg.v_max, 0.0, v_prog, tuple(sigs), fc, cfg, VEH, W)
    assert at_limit.stops >= 1
    # FIFO: the stopping vehicle is NOT slower to the last signal, which is why stops (not time) are the win
    assert at_limit.travel_time <= plan.best.centre.travel_time


def test_never_exceeds_limits():
    fc, sigs = _corridor([0, 30], [20, 20], [300, 600])
    cfg = BandConfig.for_limit(50 * KMH)
    plan = compute_plan(5.0, 10.0, sigs, fc, cfg, VEH, W)
    for b in plan.bands:
        assert cfg.v_min - 1e-9 <= b.v_lo <= b.v_hi <= cfg.v_max + 1e-9
