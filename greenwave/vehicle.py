"""Longitudinal vehicle model used by the planner.

Constant-acceleration speed changes, matching SUMO's Krauss model behaviour for an undisturbed vehicle
(v_next = v + a*dt until the target is reached), so planner predictions line up with the simulation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleParams:
    accel: float = 2.0           # m/s^2, comfortable acceleration (must equal the SUMO vType accel)
    decel: float = 3.0           # m/s^2, comfortable deceleration (must equal the SUMO vType decel)
    startup_lost_s: float = 1.0  # reaction + start-up delay after green when first in queue


def arrival_time(d: float, v0: float, v: float, p: VehicleParams) -> float:
    """Time to cover distance d starting at speed v0, changing at comfortable rate to cruise speed v."""
    if d <= 0:
        return 0.0
    if math.isclose(v0, v, abs_tol=1e-6):
        return d / v
    rate = p.accel if v > v0 else p.decel
    d_change = abs(v * v - v0 * v0) / (2 * rate)
    t_change = abs(v - v0) / rate
    if d >= d_change:
        return t_change + (d - d_change) / v
    # target speed not reached before d
    if v > v0:
        return (-v0 + math.sqrt(v0 * v0 + 2 * rate * d)) / rate
    return (v0 - math.sqrt(max(v0 * v0 - 2 * rate * d, 0.0))) / rate


def speed_at(d: float, v0: float, v: float, p: VehicleParams) -> float:
    """Speed after covering distance d (same manoeuvre as arrival_time)."""
    if math.isclose(v0, v, abs_tol=1e-6):
        return v
    rate = p.accel if v > v0 else p.decel
    sign = 1 if v > v0 else -1
    v_d = math.sqrt(max(v0 * v0 + sign * 2 * rate * d, 0.0))
    return min(v_d, v) if v > v0 else max(v_d, v)
