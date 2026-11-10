import numpy as np

from greenwave.signals.probe_inferred import (CROSS, MAIN, StopLineEvent, _largest_gap, cycle_score,
                                              estimate_cycle, infer_phases)

C, OFF, G, IG = 90.0, 20.0, 50.0, 5.0     # main green [20, 70), cross green [75, 90+15)


def _events(main_window=(0.0, G), n_cycles=40, per_cycle=3, seed=0):
    """Synthetic probe events. Main crossings fall in [OFF + main_window[0], OFF + main_window[1]); cross
    crossings fall in the cross green, starting with a queue release right at its start."""
    rng = np.random.default_rng(seed)
    cross_green = C - G - 2 * IG
    ev = []
    for k in range(n_cycles):
        base = (k + 3) * C          # cycle boundaries at multiples of C, so phases are as defined above
        for _ in range(per_cycle):
            ev.append(StopLineEvent("S", MAIN, base + OFF + rng.uniform(*main_window), False, None))
        t0 = base + OFF + G + IG
        ev.append(StopLineEvent("S", CROSS, t0 + 1.5 + rng.uniform(0, 3), True, t0 + rng.uniform(0.2, 1.0)))
        ev.append(StopLineEvent("S", CROSS, t0 + rng.uniform(5, cross_green), False, None))
    return {"S": ev}


def test_recovers_cycle_and_green_window():
    ev = _events()
    c = estimate_cycle(ev)
    assert abs(c - C) < 0.05
    ph = infer_phases(ev, c, IG)["S"]
    assert ph.end_source == "cross"
    assert abs(ph.green_start - OFF) < 2.0
    assert abs(ph.green_end - (OFF + G)) < 1.5


def test_multiple_of_cycle_scores_lower():
    series = [np.array([e.t_cross for e in _events()["S"] if e.movement == MAIN])]
    assert cycle_score(series, C) > cycle_score(series, 2 * C)
    assert cycle_score(series, C) > cycle_score(series, C / 2)


def test_progressed_signal_end_comes_from_cross_street():
    """Platoons cross only in the first 20 s of a 50 s green: the main road's last crossing says nothing about
    when green ends; the cross street's green start does."""
    ev = _events(main_window=(0.0, 20.0))
    ph = infer_phases(ev, C, IG)["S"]
    assert abs(ph.green_end - (OFF + G)) < 1.5


def test_single_stray_sample_does_not_split_red():
    p = np.concatenate([np.linspace(20, 70, 150), [45.0 + 40.0]])   # one sample deep in the red
    start, end, _ = _largest_gap(p, C)
    assert abs(start - 70) < 0.5 and abs(end - 20) < 0.5
