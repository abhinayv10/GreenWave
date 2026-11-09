from greenwave.signals.fixed_time import FixedTimePlan


def test_is_green_and_windows():
    p = FixedTimePlan(cycle_s=90, offset_s=30, green_s=40)
    assert p.is_green(30) and p.is_green(69.9) and not p.is_green(70) and not p.is_green(29.9)
    assert p.is_green(120) and p.is_green(-60)          # periodic both ways
    w = p.green_windows(0, 200)
    assert [(g.start, g.end) for g in w] == [(30, 70), (120, 160)]
    # a window already in progress at t_from is included
    assert p.green_windows(50, 60)[0].start == 30
