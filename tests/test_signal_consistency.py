"""Our FixedTimePlan must agree with SUMO's actual signal state at every timestep (ADR-0001).

If this fails, every speed band downstream is computed against the wrong signals.
"""
from pathlib import Path

import pytest

from greenwave.scenario import load_scenario
from greenwave.signals.fixed_time import FixedTimeForecaster
from greenwave.sim import corridor
from greenwave.sim.sumo_env import binary

ROOT = Path(__file__).resolve().parents[1]
SCENARIO = ROOT / "scenarios" / "corridor_fixed.yaml"


@pytest.fixture(scope="module")
def sumo_run(tmp_path_factory):
    import libsumo as traci

    out = tmp_path_factory.mktemp("corridor")
    cfg = corridor.build(SCENARIO, out)
    traci.start([binary("sumo"), "-c", str(cfg), "--step-length", "0.5", "--end", "400"])
    yield traci
    traci.close()


def _eastbound_through_index(traci, tls_id: str) -> int:
    for idx, links in enumerate(traci.trafficlight.getControlledLinks(tls_id)):
        for in_lane, out_lane, _via in links:
            if in_lane.startswith("eb_") and out_lane.startswith("eb_"):
                return idx
    raise AssertionError(f"no eastbound through link at {tls_id}")


def test_sumo_matches_fixed_time_model(sumo_run):
    traci = sumo_run
    sc = load_scenario(SCENARIO)
    fc = FixedTimeForecaster.from_scenario(sc)
    ids = [ix.id for ix in sc.intersections]
    for tls in ids:
        assert traci.trafficlight.getProgram(tls) == corridor.SUMO_PROGRAM_ID
    idx = {tls: _eastbound_through_index(traci, tls) for tls in ids}

    # TraCI convention: after simulationStep() the clock reads the END of the step just executed (t), while
    # the reported signal state is the one that was in force DURING that step, i.e. from t - dt.
    dt = traci.simulation.getDeltaT()
    mismatches = []
    while traci.simulation.getTime() < 380:
        traci.simulationStep()
        t = traci.simulation.getTime()
        for tls in ids:
            sumo_green = traci.trafficlight.getRedYellowGreenState(tls)[idx[tls]] in "Gg"
            model_green = fc.plan(tls).is_green(t - dt)
            if sumo_green != model_green:
                mismatches.append((t, tls, sumo_green, model_green))
    assert not mismatches, f"{len(mismatches)} mismatches, first: {mismatches[:5]}"
