import math

from greenwave.energy import EV, ICE, segment_energy, stop_energy
from greenwave.optimizer.band import KMH, BandConfig, SignalAhead, evaluate_speed
from greenwave.optimizer.cost import CostWeights, weights_for
from greenwave.signals.fixed_time import MAIN_THROUGH, FixedTimeForecaster, FixedTimePlan
from greenwave.vehicle import VehicleParams

VEH = VehicleParams()


def test_ev_cruise_matches_sumo():
    """SUMO 1.28's Energy model reported 1.065 Wh/s for this vehicle cruising at 50 km/h (13.89 m/s)."""
    assert math.isclose(EV.rate(13.89, 0.0), 1.065, rel_tol=0.01)


def test_regimes():
    assert ICE.rate(0.0, 0.0) == ICE.idle_gps                       # idling burns fuel
    assert ICE.rate(13.89, -3.0) == ICE.overrun_gps                 # fuel cut-off while braking
    assert EV.rate(13.89, -3.0) < 0                                 # regen returns energy
    assert EV.rate(0.0, 0.0) == EV.aux_w / 3600                     # stationary EV: auxiliaries only


def test_segment_energy_cruise_and_ramp():
    v = 13.89
    assert math.isclose(segment_energy(ICE, 500, v, v, VEH), ICE.rate(v, 0) * 500 / v, rel_tol=1e-9)
    # accelerating from rest costs more than cruising the same distance
    assert segment_energy(ICE, 500, 0.0, v, VEH) > segment_energy(ICE, 500, v, v, VEH)
    assert stop_energy(EV, v, VEH) < 0 < stop_energy(ICE, v, VEH)


def test_planner_energy_only_with_powertrain():
    fc = FixedTimeForecaster({("S", MAIN_THROUGH): FixedTimePlan(90, 60, 30)})
    sigs = (SignalAhead("S", 300.0),)
    cfg, w = BandConfig.for_limit(50 * KMH), CostWeights()
    blind = evaluate_speed(50 * KMH, 0.0, 50 * KMH, sigs, fc, cfg, VEH, w)
    ice = evaluate_speed(50 * KMH, 0.0, 50 * KMH, sigs, fc, cfg, VEH, w, pt=ICE)
    assert blind.energy == 0 and ice.energy > 0
    assert not ice.outcomes[0].green and ice.outcomes[0].wait > 0     # stops at red, idles: energy includes it
    assert math.isclose(ice.energy, sum(o.energy for o in ice.outcomes))


def test_route_prediction_advice_vs_limit():
    """Signals timed for 40 km/h: at the 50 km/h limit the rollout predicts stops; advised it predicts fewer,
    with less energy, and never a shorter trip than physics allows."""
    from greenwave.routing import predict_route
    xs, v_prog = [400, 750, 1300, 1650], 40 * KMH
    fc = FixedTimeForecaster({(f"S{i}", MAIN_THROUGH): FixedTimePlan(90, x / v_prog, g)
                              for i, (x, g) in enumerate(zip(xs, [50, 45, 50, 40]))})
    sl = [(f"S{i}", float(x)) for i, x in enumerate(xs)]
    cfg = BandConfig.for_limit(50 * KMH)
    w = weights_for("balanced", "ice")
    lim = predict_route("R", sl, 2000.0, 0.0, v_prog, fc, cfg, VEH, w, ICE, mode="limit")
    adv = predict_route("R", sl, 2000.0, 0.0, v_prog, fc, cfg, VEH, w, ICE, mode="advised")
    assert lim.stops >= 1 and adv.stops < lim.stops
    assert adv.energy < lim.energy and adv.cost < lim.cost
    assert adv.travel_time >= 2000.0 / cfg.v_max


def test_profile_energy_weights():
    assert weights_for("fastest", "ice").w_energy == 0
    assert weights_for("eco", "ev").w_energy == 3 * weights_for("balanced", "ev").w_energy > 0
