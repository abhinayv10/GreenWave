"""Streamlit dashboard (M1). Run: .venv\\Scripts\\streamlit run dashboard/app.py

Reads results produced by scripts/m1_compare.py (it does not run SUMO itself).
"""
from __future__ import annotations

import pickle
import sys
from pathlib import Path

import pandas as pd
import streamlit as st

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from greenwave.optimizer.band import KMH  # noqa: E402
from greenwave.scenario import load_scenario  # noqa: E402
from greenwave.signals.fixed_time import FixedTimeForecaster  # noqa: E402
from greenwave.sim import corridor  # noqa: E402
from greenwave.viz import charts  # noqa: E402

st.set_page_config(page_title="Green-Wave Advisory", layout="wide")
RES = ROOT / "results" / "m1"


@st.cache_resource
def load():
    data = pickle.load(open(RES / "runs.pkl", "rb"))
    sc = load_scenario(data["scenario"])
    net = ROOT / "build" / sc.name / "corridor.net.xml"
    return data["runs"], sc, corridor.eastbound_stop_lines(sc, net), FixedTimeForecaster.from_scenario(sc)


if not (RES / "runs.pkl").exists():
    st.error("No results yet. Run: `.venv\\Scripts\\python.exe scripts\\m1_compare.py`")
    st.stop()

runs, sc, stop_lines, fc = load()
df = pd.read_csv(RES / "trips.csv")
mode = "dark" if getattr(st.context, "theme", None) and st.context.theme.type == "dark" else "light"

st.title("Green-wave speed advisory")
st.caption(f"Scenario **{sc.name}** · {len(sc.intersections)} fixed-time signals · cycle {sc.timing.cycle_s:.0f}s · "
           f"limit {sc.speed_limit_ms / KMH:.0f} km/h · signal knowledge: exact (oracle) · profile: balanced")

# Headline: advised vs drive-at-limit across all departures
base, adv = df[~df.advised], df[df.advised]


def delta(a, b):
    return f"{100 * (b - a) / a:+.0f}%" if a else "n/a"


c = st.columns(4)
c[0].metric("Stops per trip", f"{adv.stops.mean():.2f}", delta(base.stops.mean(), adv.stops.mean()), delta_color="inverse")
c[1].metric("Waiting at red (s)", f"{adv.wait.mean():.0f}", delta(base.wait.mean(), adv.wait.mean()), delta_color="inverse")
c[2].metric("Travel time (s)", f"{adv.travel_time.mean():.0f}", delta(base.travel_time.mean(), adv.travel_time.mean()), delta_color="inverse")
c[3].metric("Fuel (g)", f"{adv.fuel_g.mean():.0f}", delta(base.fuel_g.mean(), adv.fuel_g.mean()), delta_color="inverse")
st.caption(f"Advised vs drive-at-limit, mean over {len(adv)} departures (one every 60 s). "
           "Deltas are relative to driving at the limit; lower is better.")

st.divider()
departs = sorted(int(d) for d in df.depart.unique())
dep = st.select_slider("Departure time (s)", options=departs, value=departs[len(departs) // 2])
pair = {"advised": runs[True].get(f"adv_{dep}"), "drive at limit": runs[False].get(f"base_{dep}")}

left, right = st.columns([3, 2])
with left:
    st.subheader("Time-space diagram")
    st.plotly_chart(charts.time_space(stop_lines, fc, pair, mode), width="stretch")
with right:
    a, b = pair["advised"], pair["drive at limit"]
    ma, mb = a.metrics(), b.metrics()
    st.subheader("This departure")
    st.dataframe(pd.DataFrame({
        "Advised": [ma["stops"], round(ma["wait"]), round(ma["travel_time"]), round(ma["fuel_g"])],
        "Drive at limit": [mb["stops"], round(mb["wait"]), round(mb["travel_time"]), round(mb["fuel_g"])],
    }, index=["Stops", "Waiting (s)", "Travel time (s)", "Fuel (g)"]), width="stretch")
    if a.first_plan is not None:
        st.subheader("Why this speed")
        st.info(a.first_plan.explanation)
        st.plotly_chart(charts.speed_cost(a.first_plan, mode), width="stretch")

with st.expander("Advice log for this trip"):
    st.dataframe(pd.DataFrame([{
        "t (s)": round(r.t, 1), "x (m)": round(r.x), "speed (km/h)": round(r.v / KMH),
        "reason": r.reason,
        "advised (km/h)": f"{r.advised[0] / KMH:.0f}–{r.advised[1] / KMH:.0f}" if r.advised else "limit",
        "planned greens": ", ".join(f"{k}:{'G' if v else 'R'}" for k, v in r.expected.items()),
        "explanation": r.explanation,
    } for r in a.advice]), width="stretch", hide_index=True)

st.divider()
st.subheader("Stops per trip across all departures")
st.plotly_chart(charts.stops_distribution(df, mode), width="stretch")
with st.expander("Table view: all trips"):
    st.dataframe(df, width="stretch", hide_index=True)
