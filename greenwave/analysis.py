"""Trip analysis (where stops happen, broken promises) and forecast quality against a recorded truth."""
from __future__ import annotations

import numpy as np

from .signals.actuated import cross_flow
from .signals.fixed_time import MAIN_THROUGH
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


def forecast_quality(sc, fc_factory, truth, lead_s=60.0, p=0.85):
    """Predict each true main-green start `lead_s` ahead; report start error and how often the reliable
    start (start_by(p)) is at or after the true start (it should be, ~p of the time)."""
    errs, covered = [], []
    for ix in sc.intersections:
        for s, e in truth.green[ix.id]:
            if s < 300:
                continue
            fc = fc_factory()
            t_pred = s - lead_s
            flows = {sid: cross_flow(sorted(arr), t_pred) for sid, arr in truth.cross_arrivals.items()}
            green_now = {sid: any(a <= t_pred < b for a, b in truth.green[sid]) for sid in truth.green}
            if hasattr(fc, "observe"):
                fc.observe(t_pred, flows, green_now)
            # match by the (fixed) yield/end time; the predicted start may be far from the true start
            ws = [w for w in fc.green_windows(ix.id, MAIN_THROUGH, s - 1, e) if abs(w.end - e) < 2]
            if not ws:
                continue
            w = ws[0]
            errs.append(w.start - s)
            covered.append(w.start_by(p) >= s - 0.5)
    errs = np.abs(np.array(errs))
    return {"windows": len(errs), "start_MAE_s": float(errs.mean()), "start_p90_err_s": float(np.quantile(errs, .9)),
            "reliable_start_not_early_%": 100 * float(np.mean(covered))}
