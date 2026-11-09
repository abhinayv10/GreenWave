"""Trip analysis: where stops happen and whether the advice had promised green there."""
from __future__ import annotations

from .sim.ego import RESUME_SPEED, STOP_SPEED, TripLog


def stop_events(log: TripLog, stop_lines: list[tuple[str, float]]) -> list[dict]:
    """Each stop with the signal it occurred at and what the advice in force had planned there."""
    out, moving = [], False
    for t, x, v in zip(log.t, log.x, log.v):
        if v < STOP_SPEED and moving:
            ahead = [(sid, sx) for sid, sx in stop_lines if sx >= x - 1]
            sid, sx = ahead[0] if ahead else ("END", x)
            recs = [a for a in log.advice if a.t <= t]
            planned_green = recs[-1].expected.get(sid) if recs else None
            out.append({"t": t, "signal": sid, "dist_to_stop_line": sx - x, "planned_green": planned_green})
            moving = False
        elif v > RESUME_SPEED:
            moving = True
    return out


def broken_promises(log: TripLog, stop_lines: list[tuple[str, float]]) -> int:
    """Stops at a signal the advice had planned as green: the failures that cost driver trust."""
    return sum(1 for s in stop_events(log, stop_lines) if s["planned_green"])
