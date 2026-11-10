"""Build a SUMO arterial corridor from a Scenario.

Outputs (in out_dir): nodes/edges (plain XML), net.net.xml, tls.add.xml (our signal programmes),
routes.rou.xml (background demand), corridor.sumocfg.

The pieces are parametric (end-node names, geometry, flow-id prefix) so that a network of several corridors
can be assembled from them (see network.py, ADR-0013). With the defaults they produce the M1-M2 corridor:
a straight road along y = 0 from W (x = 0) to E.

Usage:  python -m greenwave.sim.corridor scenarios/corridor_fixed.yaml [out_dir]
"""
from __future__ import annotations

import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import sumolib

from ..energy import ev_vtype_xml
from ..scenario import Intersection, Scenario, load_scenario
from ..vehicle import VehicleParams
from .route import Route
from .sumo_env import binary

MAIN_PRIORITY = 3
CROSS_PRIORITY = 1
SUMO_PROGRAM_ID = "gw"


@dataclass(frozen=True)
class Polyline:
    """Corridor geometry: along-corridor distance s -> (x, y). Straight segments between points."""
    points: tuple[tuple[float, float], ...]

    @classmethod
    def straight(cls, length: float) -> "Polyline":
        return cls(((0.0, 0.0), (float(length), 0.0)))

    def _segments(self):
        s = 0.0
        for (x0, y0), (x1, y1) in zip(self.points, self.points[1:]):
            d = math.hypot(x1 - x0, y1 - y0)
            yield s, d, (x0, y0), ((x1 - x0) / d, (y1 - y0) / d)
            s += d

    @property
    def length(self) -> float:
        return sum(d for _, d, _, _ in self._segments())

    def at(self, s: float) -> tuple[tuple[float, float], tuple[float, float]]:
        """(point, unit direction) at distance s."""
        segs = list(self._segments())
        for s0, d, (x0, y0), u in segs:
            if s <= s0 + d + 1e-9:
                return (x0 + u[0] * (s - s0), y0 + u[1] * (s - s0)), u
        s0, d, (x0, y0), u = segs[-1]
        return (x0 + u[0] * d, y0 + u[1] * d), u

    def bends_between(self, s_from: float, s_to: float) -> list[tuple[float, float]]:
        """Interior vertices strictly between two distances (an edge's shape between its end nodes)."""
        out, s = [], 0.0
        for (x0, y0), (x1, y1) in zip(self.points, self.points[1:]):
            s += math.hypot(x1 - x0, y1 - y0)
            if s_from + 1e-6 < s < s_to - 1e-6:
                out.append((x1, y1))
        return out


def _chain(sc: Scenario, west: str = "W", east: str = "E") -> list[tuple[str, float]]:
    return [(west, 0.0)] + [(ix.id, ix.x) for ix in sc.intersections] + [(east, sc.corridor_length_m)]


def _fmt(v: float) -> str:
    return repr(float(round(v, 3)))


def corridor_xml(sc: Scenario, west: str = "W", east: str = "E", line: Polyline | None = None,
                 end_type: str = "dead_end") -> tuple[list[str], list[str]]:
    """Node and edge XML lines for one corridor: main road both ways, a signal at each intersection,
    and a cross street (one arm each side, perpendicular to the road) at every signal."""
    line = line or Polyline.straight(sc.corridor_length_m)
    a = sc.cross_arm_length_m
    chain = _chain(sc, west, east)
    nodes = []
    for nid, s in chain:
        (x, y), _ = line.at(s)
        ntype = end_type if nid in (west, east) else "traffic_light"
        nodes.append(f'  <node id="{nid}" x="{_fmt(x)}" y="{_fmt(y)}" type="{ntype}"/>')
    for ix in sc.intersections:
        (x, y), (ux, uy) = line.at(ix.x)
        nx, ny = -uy, ux                         # left of the direction of travel
        nodes.append(f'  <node id="{ix.id}N" x="{_fmt(x + a * nx)}" y="{_fmt(y + a * ny)}" type="dead_end"/>')
        nodes.append(f'  <node id="{ix.id}S" x="{_fmt(x - a * nx)}" y="{_fmt(y - a * ny)}" type="dead_end"/>')

    def edge(eid, frm, to, lanes, speed, prio, shape=()):
        sh = f' shape="{" ".join(f"{_fmt(px)},{_fmt(py)}" for px, py in shape)}"' if shape else ""
        return (f'  <edge id="{eid}" from="{frm}" to="{to}" numLanes="{lanes}" '
                f'speed="{speed:.2f}" priority="{prio}"{sh}/>')

    def full_shape(s0, s1):
        bends = line.bends_between(s0, s1)
        return [line.at(s0)[0], *bends, line.at(s1)[0]] if bends else []

    edges = []
    for (a_id, s0), (b_id, s1) in zip(chain, chain[1:]):
        sh = full_shape(s0, s1)
        edges.append(edge(f"eb_{a_id}_{b_id}", a_id, b_id, sc.main_lanes, sc.speed_limit_ms, MAIN_PRIORITY, sh))
        edges.append(edge(f"wb_{b_id}_{a_id}", b_id, a_id, sc.main_lanes, sc.speed_limit_ms, MAIN_PRIORITY,
                          sh[::-1]))
    for ix in sc.intersections:
        j, v, n = ix.id, sc.cross_speed_limit_ms, sc.cross_lanes
        edges.append(edge(f"{j}_nin", f"{j}N", j, n, v, CROSS_PRIORITY))
        edges.append(edge(f"{j}_nout", j, f"{j}N", n, v, CROSS_PRIORITY))
        edges.append(edge(f"{j}_sin", f"{j}S", j, n, v, CROSS_PRIORITY))
        edges.append(edge(f"{j}_sout", j, f"{j}S", n, v, CROSS_PRIORITY))
    return nodes, edges


def write_plain_xml(sc: Scenario, out: Path, stem: str = "corridor") -> None:
    nodes, edges = corridor_xml(sc)
    (out / f"{stem}.nod.xml").write_text("\n".join(["<nodes>", *nodes, "</nodes>"]))
    (out / f"{stem}.edg.xml").write_text("\n".join(["<edges>", *edges, "</edges>"]))


def run_netconvert(out: Path, stem: str = "corridor") -> Path:
    net = out / f"{stem}.net.xml"
    subprocess.run([
        binary("netconvert"),
        "--node-files", str(out / f"{stem}.nod.xml"),
        "--edge-files", str(out / f"{stem}.edg.xml"),
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


def tls_lines(sc: Scenario, net: sumolib.net.Net, west: str = "W", east: str = "E") -> list[str]:
    t = sc.timing
    lines = []
    for ix in sc.intersections:
        st = _phase_states(net, ix.id)
        if t.is_actuated:
            lines += _actuated_tls(sc, ix, st, west, east)
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
    return lines


def write_tls(sc: Scenario, net_path: Path, out: Path) -> None:
    net = sumolib.net.readNet(str(net_path))
    (out / "tls.add.xml").write_text("\n".join(['<additional>', *tls_lines(sc, net), '</additional>']))


def _actuated_tls(sc: Scenario, ix: Intersection, st: dict[str, str], west: str = "W",
                  east: str = "E") -> list[str]:
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
    chain = [n for n, _ in _chain(sc, west, east)]
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


def vtype_lines() -> list[str]:
    veh = VehicleParams()
    return [
        '  <vType id="car" vClass="passenger" sigma="0.5" speedFactor="1.0" speedDev="0.1"/>',
        # Test vehicles: no driver noise (M1), accel/decel identical to the planner's VehicleParams.
        f'  <vType id="ego" vClass="passenger" sigma="0" speedFactor="1.0" speedDev="0" '
        f'accel="{veh.accel}" decel="{veh.decel}" color="1,0,0"/>',
        # Same vehicle, imperfect human speed keeping (Krauss dawdling), for robustness tests (M2).
        f'  <vType id="ego_noisy" vClass="passenger" sigma="0.5" speedFactor="1.0" speedDev="0" '
        f'accel="{veh.accel}" decel="{veh.decel}" color="1,0,1"/>',
        # Electric versions of both (M3): SUMO Energy model with the planner's parameters (energy.py).
        ev_vtype_xml("ego_ev", veh),
        ev_vtype_xml("ego_ev_noisy", veh, sigma=0.5, color="0.4,0.4,1"),
    ]


def flows(sc: Scenario, west: str = "W", east: str = "E", end_s: float = 3600,
          tag: str = "") -> list[tuple[str, str, str, float, float, float]]:
    """Background flows (id, from, to, begin, end, veh/h). `tag` keeps main-road flow ids unique per corridor;
    cross flows are named bg_<signal>_..., which the SignalRecorder relies on."""
    d = sc.demand
    first, last = sc.intersections[0].id, sc.intersections[-1].id
    out = [
        (f"bg_{tag}eb", f"eb_{west}_{first}", f"eb_{last}_{east}", 0, end_s, d.get("main_eb", 0)),
        (f"bg_{tag}wb", f"wb_{east}_{last}", f"wb_{first}_{west}", 0, end_s, d.get("main_wb", 0)),
    ]
    # Cross demand: a time profile [[begin, end, veh/h], ...] scaled per intersection, or a constant.
    profile = d.get("cross_profile") or [[0, end_s, d.get("cross_each", 0)]]
    for ix in sc.intersections:
        for k, (b, e, vph) in enumerate(profile):
            q = vph * ix.cross_demand_factor
            out.append((f"bg_{ix.id}_ns_{k}", f"{ix.id}_nin", f"{ix.id}_sout", b, e, q))
            out.append((f"bg_{ix.id}_sn_{k}", f"{ix.id}_sin", f"{ix.id}_nout", b, e, q))
    return out


def write_route_file(path: Path, flow_list: list[tuple]) -> None:
    rows = ['<routes>', *vtype_lines()]
    # SUMO reads route files incrementally and assumes departures are sorted; unsorted flows get dropped.
    for fid, frm, to, b, e, vph in sorted(flow_list, key=lambda f: f[3]):
        if vph > 0:
            rows.append(f'  <flow id="{fid}" type="car" from="{frm}" to="{to}" begin="{b}" end="{e}" '
                        f'vehsPerHour="{vph:.0f}" departLane="best" departSpeed="max"/>')
    rows.append('</routes>')
    path.write_text("\n".join(rows))


def write_routes(sc: Scenario, out: Path, end_s: float = 3600) -> None:
    write_route_file(out / "routes.rou.xml", flows(sc, end_s=end_s))


def write_sumocfg(out: Path, stem: str = "corridor") -> Path:
    cfg = out / f"{stem}.sumocfg"
    cfg.write_text(f"""<configuration>
  <input>
    <net-file value="{stem}.net.xml"/>
    <route-files value="routes.rou.xml"/>
    <additional-files value="tls.add.xml"/>
  </input>
  <time><begin value="0"/><end value="3600"/><step-length value="0.5"/></time>
  <report><no-step-log value="true"/><no-warnings value="true"/></report>
</configuration>""")
    return cfg


def eastbound_route(sc: Scenario, west: str = "W", east: str = "E") -> list[str]:
    chain = _chain(sc, west, east)
    return [f"eb_{a}_{b}" for (a, _), (b, _) in zip(chain, chain[1:])]


def eastbound(sc: Scenario, net_path: Path) -> Route:
    """The corridor's eastbound route (ADR-0013). On this straight road, along-route distance equals x."""
    return Route.from_net(net_path, "eb", eastbound_route(sc))


def eastbound_stop_lines(sc: Scenario, net_path: Path) -> list[tuple[str, float]]:
    """Along-route distance of each eastbound stop line, in corridor order."""
    return eastbound(sc, net_path).stop_lines()


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
