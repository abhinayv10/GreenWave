from greenwave.optimizer.band import KMH, BandConfig, SignalAhead, evaluate_speed
from greenwave.optimizer.cost import CostWeights
from greenwave.scenario import load_scenario
from greenwave.signals.actuated import ActuatedForecaster, flow_bucket
from greenwave.signals.base import GreenWindow
from greenwave.signals.fixed_time import MAIN_THROUGH
from greenwave.sim.history import SignalHistory
from greenwave.traffic.queue import QueueModel
from greenwave.vehicle import VehicleParams


def test_window_probabilities():
    w = GreenWindow.from_start_samples([10, 12, 14, 16, 30], end=60)
    assert w.start == 14 and w.start_min == 10 and w.start_max == 30
    assert w.p_started(9) == 0 and w.p_started(14) == 0.6 and w.p_started(30) == 1
    assert w.start_by(0.5) == 14 and w.start_by(0.85) == 30
    assert GreenWindow.exact(5, 9).start_by(0.99) == 5


class _OneWindow:
    def __init__(self, w):
        self.w = w

    def green_windows(self, sid, mov, t0, t1):
        return [self.w] if self.w.end > t0 and self.w.start_min <= t1 else []


def test_planner_relies_only_on_probable_green_and_cleared_queue():
    cfg = BandConfig.for_limit(50 * KMH, margin_start_s=0.0)
    veh, wts = VehicleParams(), CostWeights()
    sig = (SignalAhead("S", 100.0),)
    # green certainly by t=20 (samples 5..20); arriving at t=10 is only 50% likely green -> not relied on
    fc = _OneWindow(GreenWindow.from_start_samples([5, 8, 10, 12, 15, 18, 20], end=60))
    assert not evaluate_speed(10.0, 0.0, 10.0, sig, fc, cfg, veh, wts).outcomes[0].green   # arrives t=10
    assert evaluate_speed(100 / 25, 0.0, 100 / 25, sig, fc, cfg, veh, wts).outcomes[0].green  # arrives t=25
    # queue clearance of 10 s pushes the usable start from 20 to 30
    q = QueueModel({"S": 10.0})
    assert not evaluate_speed(100 / 25, 0.0, 100 / 25, sig, fc, cfg, veh, wts, q).outcomes[0].green
    assert evaluate_speed(100 / 32, 0.0, 100 / 32, sig, fc, cfg, veh, wts, q).outcomes[0].green


def test_actuated_forecaster_conditions_on_cross_demand(tmp_path):
    sc = load_scenario("scenarios/corridor_actuated.yaml")
    ix = sc.intersections[0]
    C, nominal0 = sc.timing.cycle_s, ix.offset_s
    # history: when cross flow is low, main green starts 20 s early; when high, 2 s early
    green, arrivals = [], []
    for k in range(4, 40):
        nominal = nominal0 + k * C
        busy = k >= 20
        green.append([nominal - (2 if busy else 20), nominal + ix.main_green_s])
        if busy:   # ~400 veh/h of cross arrivals in the busy half
            arrivals += [nominal - C + i * 9 for i in range(10)]
    hist = SignalHistory({ix.id: green}, {ix.id: []}, {ix.id: sorted(arrivals)})
    sc1 = type(sc)(**{**sc.__dict__, "intersections": (ix,)})
    fc = ActuatedForecaster(sc1, hist, warmup_s=0)
    t_query = nominal0 + 50 * C
    fc.observe(t_query - 100, {ix.id: 0.0}, {ix.id: False})
    quiet = fc.green_windows(ix.id, MAIN_THROUGH, t_query, t_query + 1)[0]
    fc.observe(t_query - 100, {ix.id: 400.0}, {ix.id: False})
    busy = fc.green_windows(ix.id, MAIN_THROUGH, t_query, t_query + 1)[0]
    assert flow_bucket(0) != flow_bucket(400)
    assert quiet.start < busy.start                       # low cross demand -> earlier main green
    assert quiet.end == busy.end                          # yield point is fixed
