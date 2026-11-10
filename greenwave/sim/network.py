"""Two-route network: two corridors sharing a split node S and a merge node M (ADR-0013).

Each route is an ordinary corridor (scenario.py), assembled from the same pieces as corridor.py, so the M1-M2
machinery (signal models, recorder, advisory, runner) applies to each route unchanged.
Background traffic stays on its own route; only test vehicles choose between A and B.

Usage:  python -m greenwave.sim.network scenarios/two_route.yaml [out_dir]
"""
from __future__ import annotations

import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import sumolib
import yaml

from ..scenario import Scenario, load_scenario, scenario_from_dict
from . import corridor
from .corridor import Polyline
from .route import Route

STEM = "network"
SPLIT, MERGE = "S", "M"


@dataclass(frozen=True)
class RouteCorridor:
    key: str                  # "A" or "B"
    sc: Scenario
    line: Polyline
    west: str = SPLIT
    east: str = MERGE

    @property
    def edges(self) -> list[str]:
        return corridor.eastbound_route(self.sc, self.west, self.east)

    def route(self, net_path: Path) -> Route:
        return Route.from_net(net_path, self.key, self.edges)


@dataclass(frozen=True)
class TwoRouteNetwork:
    name: str
    routes: dict[str, RouteCorridor]

    def __getitem__(self, key: str) -> RouteCorridor:
        return self.routes[key]


def load_network(path: str | Path) -> TwoRouteNetwork:
    path = Path(path)
    raw = yaml.safe_load(path.read_text())
    a = load_scenario(path.parent / raw["route_a"])
    length = a.corridor_length_m

    rb = dict(raw["route_b"])
    dx, lift = float(rb.pop("connector_dx_m")), float(rb.pop("lift_m"))
    line_b = Polyline(((0.0, 0.0), (dx, lift), (length - dx, lift), (length, 0.0)))
    ramp = math.hypot(dx, lift)
    ixs = []
    for ix in rb["intersections"]:
        if not dx < ix["x"] < length - dx:
            raise ValueError(f"route B signal {ix['id']} at x={ix['x']} is not on the straight section")
        ixs.append(dict(ix, x=ramp + (ix["x"] - dx)))      # x on the straight -> along-route distance
    rb.update(intersections=ixs, corridor_length_m=line_b.length)
    b = scenario_from_dict(rb)

    clash = {ix.id for ix in a.intersections} & {ix.id for ix in b.intersections}
    if clash:
        raise ValueError(f"signal ids used on both routes: {sorted(clash)}")
    return TwoRouteNetwork(raw["name"], {"A": RouteCorridor("A", a, Polyline.straight(length)),
                                         "B": RouteCorridor("B", b, line_b)})


def build(path: str | Path, out_dir: str | Path | None = None) -> Path:
    nw = load_network(path)
    out = Path(out_dir) if out_dir else Path("build") / nw.name
    out.mkdir(parents=True, exist_ok=True)

    nodes, edges, seen = [], [], set()
    for rc in nw.routes.values():
        n, e = corridor.corridor_xml(rc.sc, rc.west, rc.east, rc.line, end_type="priority")
        for line in n:                                    # S and M are shared by both routes
            nid = re.search(r'id="([^"]+)"', line).group(1)
            if nid not in seen:
                seen.add(nid)
                nodes.append(line)
        edges += e
    (out / f"{STEM}.nod.xml").write_text("\n".join(["<nodes>", *nodes, "</nodes>"]))
    (out / f"{STEM}.edg.xml").write_text("\n".join(["<edges>", *edges, "</edges>"]))
    net_path = corridor.run_netconvert(out, STEM)

    net = sumolib.net.readNet(str(net_path))
    tls = [ln for rc in nw.routes.values() for ln in corridor.tls_lines(rc.sc, net, rc.west, rc.east)]
    (out / "tls.add.xml").write_text("\n".join(["<additional>", *tls, "</additional>"]))
    flows = [f for rc in nw.routes.values() for f in corridor.flows(rc.sc, rc.west, rc.east, tag=f"{rc.key}_")]
    corridor.write_route_file(out / "routes.rou.xml", flows)
    return corridor.write_sumocfg(out, STEM)


if __name__ == "__main__":
    cfg = build(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
    print(f"built {cfg}")
