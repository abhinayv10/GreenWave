"""Energy the planner can compute in advance (ADR-0013).

One physics core: tractive power P(v, a) = (m·a + m·g·Cr + ½·ρ·CdA·v²)·v for the test vehicle's body. Two
powertrains turn it into a consumption rate:

- **EV** (Wh/s): P/η_drive when driving, P·η_regen when braking (energy recovered), plus auxiliaries. This is
  the structure of SUMO's `Energy` model, and `ev_vtype_xml()` writes the SUMO vType from the same numbers,
  so planner and simulator share one parameter set.
- **ICE** (g/s): idle rate when stopped, fuel cut-off rate when the power demand is negative, and an affine
  function of power when driving (Willans line). Coefficients are fitted to SUMO's HBEFA4 PC petrol Euro-4
  model by `scripts/m3_energy_calibration.py`, which also reports the fit quality.

Trajectories come from the planner's kinematics (vehicle.py): comfortable speed changes at constant rate,
cruising, braking to a stop, idling. `segment_energy` integrates the rate over those phases.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .vehicle import VehicleParams

G = 9.81
RHO = 1.2


@dataclass(frozen=True)
class Body:
    mass: float = 1500.0          # kg
    cd: float = 0.32
    area: float = 2.2             # m²
    cr: float = 0.01              # rolling resistance coefficient

    def power(self, v: float, a: float) -> float:
        """Tractive power (W) at speed v (m/s) and acceleration a (m/s²). Zero when stationary."""
        if v <= 0.1:
            return 0.0
        return (self.mass * a + self.mass * G * self.cr + 0.5 * RHO * self.cd * self.area * v * v) * v


@dataclass(frozen=True)
class Powertrain:
    kind: str                     # "ice" | "ev"
    body: Body = Body()
    # EV (SUMO Energy model parameters)
    eta_drive: float = 0.9
    eta_regen: float = 0.6
    aux_w: float = 300.0
    # ICE (fitted to SUMO HBEFA4/PC_petrol_Euro-4; see scripts/m3_energy_calibration.py)
    idle_gps: float = 0.4855
    overrun_gps: float = 0.0939
    c0_gps: float = 0.5039
    c1_gps_per_kw: float = 0.0519

    @property
    def unit(self) -> str:
        return "Wh" if self.kind == "ev" else "g"

    def rate(self, v: float, a: float) -> float:
        """Consumption rate: Wh/s (EV) or g/s (ICE)."""
        p = self.body.power(v, a)
        if self.kind == "ev":
            w = (p / self.eta_drive if p > 0 else p * self.eta_regen) + self.aux_w
            return w / 3600.0
        if v <= 0.1:
            return self.idle_gps
        if p <= 0:
            return self.overrun_gps
        return self.c0_gps + self.c1_gps_per_kw * p / 1000.0

    def idle(self, seconds: float) -> float:
        return self.rate(0.0, 0.0) * max(seconds, 0.0)


ICE = Powertrain("ice")
EV = Powertrain("ev")
POWERTRAINS = {"ice": ICE, "ev": EV}


def _ramp(pt: Powertrain, v0: float, v1: float, rate: float, dt: float = 0.25) -> float:
    """Energy of a constant-acceleration speed change v0 -> v1 at |rate| (midpoint rule in time)."""
    if math.isclose(v0, v1, abs_tol=1e-6):
        return 0.0
    a = rate if v1 > v0 else -rate
    t_total = abs(v1 - v0) / rate
    n = max(1, math.ceil(t_total / dt))
    h = t_total / n
    return sum(pt.rate(v0 + a * (i + 0.5) * h, a) for i in range(n)) * h


def segment_energy(pt: Powertrain, d: float, v0: float, v: float, veh: VehicleParams) -> float:
    """Energy to cover d metres starting at v0 and changing at comfortable rate towards cruise speed v,
    then cruising (the same manoeuvre as vehicle.arrival_time)."""
    if d <= 0:
        return 0.0
    rate = veh.accel if v > v0 else veh.decel
    d_change = abs(v * v - v0 * v0) / (2 * rate)
    if d >= d_change:
        return _ramp(pt, v0, v, rate) + pt.rate(v, 0.0) * (d - d_change) / max(v, 0.1)
    # target speed not reached within d
    v_end = math.sqrt(max(v0 * v0 + (2 if v > v0 else -2) * rate * d, 0.0))
    return _ramp(pt, v0, v_end, rate)


def stop_energy(pt: Powertrain, v: float, veh: VehicleParams) -> float:
    """Energy of braking from v to standstill at the comfortable deceleration (negative for an EV: regen)."""
    return _ramp(pt, v, 0.0, veh.decel)


def ev_vtype_xml(vtype_id: str, veh: VehicleParams, sigma: float = 0.0, color: str = "0,0.6,1",
                 pt: Powertrain = EV) -> str:
    """SUMO vType for the EV test vehicle, from the planner's parameters (one source of truth)."""
    b = pt.body
    params = {
        "has.battery.device": "true", "device.battery.capacity": "60000", "device.battery.chargeLevel": "50000",
        "frontSurfaceArea": b.area, "airDragCoefficient": b.cd, "rollDragCoefficient": b.cr,
        "radialDragCoefficient": 0.0, "constantPowerIntake": pt.aux_w,
        "propulsionEfficiency": pt.eta_drive, "recuperationEfficiency": pt.eta_regen,
    }
    inner = "".join(f'\n    <param key="{k}" value="{v}"/>' for k, v in params.items())
    return (f'  <vType id="{vtype_id}" vClass="passenger" sigma="{sigma}" speedFactor="1.0" speedDev="0" '
            f'accel="{veh.accel}" decel="{veh.decel}" color="{color}" emissionClass="Energy/unknown" '
            f'mass="{b.mass}">{inner}\n  </vType>')
