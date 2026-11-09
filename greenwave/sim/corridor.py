"""Build a SUMO arterial corridor from a Scenario.

Outputs (in out_dir): nodes/edges (plain XML), net.net.xml, tls.add.xml (our signal programmes),
routes.rou.xml (background demand), corridor.sumocfg.

Usage:  python -m greenwave.sim.corridor scenarios/corridor_fixed.yaml [out_dir]
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from xml.sax.saxutils import quoteattr

import sumolib

from ..scenario import Scenario, load_scenario
from ..vehicle import VehicleParams
from .sumo_env import binary

MAIN_PRIORITY = 3
CROSS_PRIORITY = 1
SUMO_PROGRAM_ID = "gw"


def _chain(sc: Scenario) -> list[tuple[str, float]]:
    return [("W", 0.0)] + [(ix.id, ix.x) for ix in sc.intersections] + [("E", sc.corridor_length_m)]


def write_plain_xml(sc: Scenario, out: Path) -> None:
    a = sc.cross_arm_length_m
    nodes = ['<nodes>']
    for nid, x in _chain(sc):
        ntype = "traffic_light" if nid not in ("W", "E") else "dead_end"
        nodes.append(f'  <node id="{nid}" x="{x}" y="0" type="{ntype}"/>')
    for ix in sc.intersections:
        nodes.append(f'  <node id="{ix.id}N" x="{ix.x}" y="{a}" type="dead_end"/>')
        nodes.append(f'  <node id="{ix.id}S" x="{ix.x}" y="{-a}" type="dead_end"/>')
    nodes.append('</nodes>')
    (out / "corridor.nod.xml").write_text("\n".join(nodes))

    def edge(eid, frm, to, lanes, speed, prio):
        return (f'  <edge id="{eid}" from="{frm}" to="{to}" numLanes="{lanes}" '
                f'speed="{speed:.2f}" priority="{prio}"/>')

    edges = ['<edges>']
    chain = _chain(sc)
    for (a_id, _), (b_id, _) in zip(chain, chain[1:]):
        edges.append(edge(f"eb_{a_id}_{b_id}", a_id, b_id, sc.main_lanes, sc.speed_limit_ms, MAIN_PRIORITY))
        edges.append(edge(f"wb_{b_id}_{a_id}", b_id, a_id, sc.main_lanes, sc.speed_limit_ms, MAIN_PRIORITY))
    for ix in sc.intersections:
        j, v, n = ix.id, sc.cross_speed_limit_ms, sc.cross_lanes
        edges.append(edge(f"{j}_nin", f"{j}N", j, n, v, CROSS_PRIORITY))
        edges.append(edge(f"{j}_nout", j, f"{j}N", n, v, CROSS_PRIORITY))
        edges.append(edge(f"{j}_sin", f"{j}S", j, n, v, CROSS_PRIORITY))
        edges.append(edge(f"{j}_sout", j, f"{j}S", n, v, CROSS_PRIORITY))
    edges.append('</edges>')
    (out / "corridor.edg.xml").write_text("\n".join(edges))


def run_netconvert(out: Path) -> Path:
    net = out / "corridor.net.xml"
    subprocess.run([
        binary("netconvert"),
        "--node-files", str(out / "corridor.nod.xml"),
        "--edge-files", str(out / "corridor.edg.xml"),
        "--output-file", str(net),
        "--no-turnarounds", "true",
        "--tls.default-type", "static",
        "--no-warnings", "true",
    ], check=True)
    return net


def _phase_states(net: sumolib.net.Net, tls_id: str) -> dict[str, str]:
    """Signal state strings for each phase, derived from the junction's link list.

    Main phase: arterial links green (left turns permissive 'g'), cross red. Cross phase: the reverse.
    """
    links: dict[int, tuple[bool, str]] = {}
    for edge in net.getNode(tls_id).getIncoming():
        is_main = edge.getPriority() >= MAIN_PRIORITY
        for lane in edge.getLanes():
            for conn in lane.getOutgoing():
                if conn.getTLSID() == tls_id:
                    links[conn.getTLLinkIndex()] = (is_main, conn.getDirection())
    n = max(links) + 1

    def state(green_main: bool | None, char: str) -> str:
        s = []
        for i in range(n):
            is_main, direction = links[i]
            if green_main is None or is_main != green_main:
                s.append("r")
            elif char == "G":
                s.append("g" if direction in ("l", "L") else "G")
            else:
                s.append(char)
        return "".join(s)

    return {
        "main_green": state(True, "G"), "main_yellow": state(True, "y"),
        "cross_green": state(False, "G"), "cross_yellow": state(False, "y"),
        "all_red": state(None, "r"),
    }


def sumo_offset(our_offset: float, cycle: float) -> float:
    """Convert our offset convention (main green starts at t ≡ offset mod cycle) to SUMO's tlLogic offset.

    Empirically, SUMO starts phase 0 at t ≡ offset (mod cycle), the same convention as ours.
    Verified by tests/test_signal_consistency.py (note the TraCI one-step reporting convention there).
    """
    return our_offset % cycle


def write_tls(sc: Scenario, net_path: Path, out: Path) -> None:
    net = sumolib.net.readNet(str(net_path))
    t = sc.timing
    lines = ['<additional>']
    for ix in sc.intersections:
        st = _phase_states(net, ix.id)
        if t.is_actuated:
            lines += _actuated_tls(sc, ix, st)
            continue
        phases = [
            (ix.main_green_s, st["main_green"]), (t.yellow_s, st["main_yellow"]), (t.all_red_s, st["all_red"]),
            (sc.cross_green_s(ix), st["cross_green"]), (t.yellow_s, st["cross_yellow"]), (t.all_red_s, st["all_red"]),
        ]
        lines.append(f'  <tlLogic id="{ix.id}" type="static" programID="{SUMO_PROGRAM_ID}" '
                     f'offset="{sumo_offset(ix.offset_s, t.cycle_s)}">')
        for dur, s in phases:
            lines.append(f'    <phase duration="{dur}" state="{s}"/>')
        lines.append('  </tlLogic>')
    lines.append('</additional>')
    (out / "tls.add.xml").write_text("\n".join(lines))


def _actuated_tls(sc: Scenario, ix, st: dict[str, str]) -> list[str]:
    """Coordinated semi-actuated control (ADR-0008).

    The main (coordinated) phase always ends at the yield point Y = offset + main_green. The cross phase is
    gap-actuated between cross_min_green and its nominal maximum; time it does not use returns to the main
    street, so main green starts EARLY (never late) relative to nominal.

    SUMO reference: timeInCycle = (t - sumo_offset) % cycle. We place the yield point at timeInCycle = C - 1,
    so the main green always starts AND ends inside one cycle. (With the yield at timeInCycle = main_green,
    an early-started main green wraps the cycle boundary and SUMO intermittently ends it at minDur;
    found empirically, see ADR-0008.)
    """
    t = sc.timing
    g_main, g_cross = ix.main_green_s, sc.cross_green_s(ix)
    yield_ct = t.cycle_s - 1
    sumo_off = (ix.offset_s + g_main - yield_ct) % t.cycle_s
    chain = [n for n, _ in _chain(sc)]
    i = chain.index(ix.id)
    main_in = [f"eb_{chain[i - 1]}_{ix.id}", f"wb_{chain[i + 1]}_{ix.id}"]
    # Main-street detectors must never end the coordinated phase by gap-out; only the yield point ends it.
    # (Without this, an early-started main green gaps out whenever the arterial has a 3 s gap; found empirically.)
    no_gap_out = [f'    <param key="max-gap:{e}_{k}" value="10000"/>' for e in main_in for k in range(sc.main_lanes)]
    return [
        f'  <tlLogic id="{ix.id}" type="actuated" programID="{SUMO_PROGRAM_ID}" offset="{sumo_off}">',
        '    <param key="coordinated" value="true"/>',
        f'    <param key="cycleTime" value="{t.cycle_s}"/>',
        f'    <param key="max-gap" value="{t.max_gap_s}"/>',
        '    <param key="detector-gap" value="2.0"/>',
        '    <param key="jam-threshold" value="100000"/>',  # >0 makes the shared cross lane usable; huge so queued cars are never ignored
        *no_gap_out,
        f'    <phase duration="{g_main}" minDur="5" maxDur="{t.cycle_s}" '
        f'earliestEnd="{yield_ct}" latestEnd="{yield_ct}" state="{st["main_green"]}"/>',
        f'    <phase duration="{t.yellow_s}" state="{st["main_yellow"]}"/>',
        f'    <phase duration="{t.all_red_s}" state="{st["all_red"]}"/>',
        f'    <phase duration="{g_cross}" minDur="{t.cross_min_green_s}" maxDur="{g_cross}" '
        f'state="{st["cross_green"]}"/>',
        f'    <phase duration="{t.yellow_s}" state="{st["cross_yellow"]}"/>',
        f'    <phase duration="{t.all_red_s}" state="{st["all_red"]}"/>',
        '  </tlLogic>',
    ]


def write_routes(sc: Scenario, out: Path, end_s: float = 3600) -> None:
    d = sc.demand
    first, last = sc.intersections[0].id, sc.intersections[-1].id
    veh = VehicleParams()
    rows = [
        '<routes>',
        '  <vType id="car" vClass="passenger" sigma="0.5" speedFactor="1.0" speedDev="0.1"/>',
        # Test vehicles: no driver noise (M1), accel/decel identical to the planner's VehicleParams.
        f'  <vType id="ego" vClass="passenger" sigma="0" speedFactor="1.0" speedDev="0" '
        f'accel="{veh.accel}" decel="{veh.decel}" color="1,0,0"/>',
        # Same vehicle, imperfect human speed keeping (Krauss dawdling), for robustness tests (M2).
        f'  <vType id="ego_noisy" vClass="passenger" sigma="0.5" speedFactor="1.0" speedDev="0" '
        f'accel="{veh.accel}" decel="{veh.decel}" color="1,0,1"/>',
    ]
    flows = [
        ("bg_eb", f"eb_W_{first}", f"eb_{last}_E", 0, end_s, d.get("main_eb", 0)),
        ("bg_wb", f"wb_E_{last}", f"wb_{first}_W", 0, end_s, d.get("main_wb", 0)),
    ]
    # Cross demand: a time profile [[begin, end, veh/h], ...] scaled per intersection, or a constant.
    profile = d.get("cross_profile") or [[0, end_s, d.get("cross_each", 0)]]
    for ix in sc.intersections:
        for k, (b, e, vph) in enumerate(profile):
            q = vph * ix.cross_demand_factor
            flows.append((f"bg_{ix.id}_ns_{k}", f"{ix.id}_nin", f"{ix.id}_sout", b, e, q))
            flows.append((f"bg_{ix.id}_sn_{k}", f"{ix.id}_sin", f"{ix.id}_nout", b, e, q))
    # SUMO reads route files incrementally and assumes departures are sorted; unsorted flows get dropped.
    for fid, frm, to, b, e, vph in sorted(flows, key=lambda f: f[3]):
        if vph > 0:
            rows.append(f'  <flow id="{fid}" type="car" from="{frm}" to="{to}" begin="{b}" end="{e}" '
                        f'vehsPerHour="{vph:.0f}" departLane="best" departSpeed="max"/>')
    rows.append('</routes>')
    (out / "routes.rou.xml").write_text("\n".join(rows))


def write_sumocfg(out: Path) -> Path:
    cfg = out / "corridor.sumocfg"
    cfg.write_text(f"""<configuration>
  <input>
    <net-file value="corridor.net.xml"/>
    <route-files value="routes.rou.xml"/>
    <additional-files value="tls.add.xml"/>
  </input>
  <time><begin value="0"/><end value="3600"/><step-length value="0.5"/></time>
  <report><no-step-log value="true"/><no-warnings value="true"/></report>
</configuration>""")
    return cfg


def eastbound_route(sc: Scenario) -> list[str]:
    chain = _chain(sc)
    return [f"eb_{a}_{b}" for (a, _), (b, _) in zip(chain, chain[1:])]


def eastbound_stop_lines(sc: Scenario, net_path: Path) -> list[tuple[str, float]]:
    """x-coordinate of each eastbound stop line (end of the incoming lane), in corridor order."""
    net = sumolib.net.readNet(str(net_path))
    chain = _chain(sc)
    out = []
    for (a, _), (b, _) in zip(chain, chain[1:]):
        if b == "E":
            continue
        lane = net.getEdge(f"eb_{a}_{b}").getLanes()[0]
        out.append((b, lane.getShape()[-1][0]))
    return out


def build(scenario_path: str | Path, out_dir: str | Path | None = None) -> Path:
    sc = load_scenario(scenario_path)
    out = Path(out_dir) if out_dir else Path("build") / sc.name
    out.mkdir(parents=True, exist_ok=True)
    write_plain_xml(sc, out)
    net = run_netconvert(out)
    write_tls(sc, net, out)
    write_routes(sc, out)
    return write_sumocfg(out)


if __name__ == "__main__":
    cfg = build(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
    print(f"built {cfg}")
