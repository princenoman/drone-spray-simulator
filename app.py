import streamlit as st
from typing import List, Tuple
from src.simulator.field import Field
from src.simulator.drone import Drone
from src.simulator.operations import OperationConfig
from src.simulator.simulator import Simulator
from src.planners.lawnmower import LawnmowerPlanner
from src.planners.optimizer import OptimizerPlanner
from src.visualization.plots import (
    plot_path_coverage,
    plot_metrics_comparison,
    plot_time_breakdown,
    plot_flight_summary,
    plot_animated_flight,
)
from config import DEFAULT_FIELD, DEFAULT_POLYGON, DEFAULT_DRONE, DEFAULT_OPS, DEFAULT_HOURS_PER_DAY

st.set_page_config(page_title="Drone Spray Simulator", layout="wide")
st.title("Drone Spray Simulator")

for key in ["base_metrics", "opt_metrics", "base_waypoints", "opt_waypoints", "field", "fwidth", "fheight", "fverts", "best_angle"]:
    if key not in st.session_state:
        st.session_state[key] = None

with st.sidebar:
    st.header("Field Configuration")
    field_shape = st.selectbox("Field Shape", ["Rectangle", "Polygon"], index=0)

    if field_shape == "Rectangle":
        field_width = st.number_input("Field Width (m)", min_value=10.0, value=float(DEFAULT_FIELD.width), step=10.0)
        field_height = st.number_input("Field Height (m)", min_value=10.0, value=float(DEFAULT_FIELD.height), step=10.0)
        field_name = st.text_input("Field Name", value=DEFAULT_FIELD.name)

        presets_rect = st.selectbox("Presets", ["Custom", "Small (200x150)", "Medium (400x300)", "Large (600x450)"])
        if presets_rect == "Small (200x150)":
            field_width = 200.0; field_height = 150.0
        elif presets_rect == "Medium (400x300)":
            field_width = 400.0; field_height = 300.0
        elif presets_rect == "Large (600x450)":
            field_width = 600.0; field_height = 450.0

        field = Field(width=field_width, height=field_height, name=field_name)
    else:
        shape_preset = st.selectbox("Preset Shapes", [
            "Custom", "Triangle", "Hexagon", "L-Shape", "Trapezoid"
        ])
        polygon_presets = {
            "Triangle": [(50, 50), (350, 50), (200, 280)],
            "Hexagon": [(100, 50), (300, 50), (350, 150),
                        (300, 250), (100, 250), (50, 150)],
            "L-Shape": [(50, 50), (250, 50), (250, 150),
                        (350, 150), (350, 250), (50, 250)],
            "Trapezoid": [(100, 50), (300, 50), (350, 250), (50, 250)],
        }
        if shape_preset != "Custom":
            verts = polygon_presets[shape_preset]
            verts_text = "\n".join(f"{x}, {y}" for x, y in verts)
            field_name = f"{shape_preset} Field"
        else:
            verts_text = st.text_area(
                "Vertices (x, y per line)",
                "50, 50\n350, 50\n400, 150\n350, 250\n50, 250\n0, 150",
                height=150,
                help="Enter one vertex per line as 'x, y'"
            )
            field_name = st.text_input("Field Name", "Polygon Field")

        try:
            raw_verts = []
            for line in verts_text.strip().split("\n"):
                line = line.strip()
                if line:
                    parts = line.split(",")
                    x, y = float(parts[0].strip()), float(parts[1].strip())
                    raw_verts.append((x, y))
            field = Field(vertices=raw_verts, name=field_name)
        except Exception:
            field = Field(vertices=[(0, 0), (100, 0), (100, 100), (0, 100)], name="Error")
            st.error("Invalid vertices. Using fallback square.")

    st.header("Drone Configuration")
    drone_speed = st.number_input("Speed (m/s)", min_value=0.5, value=float(DEFAULT_DRONE.speed), step=0.5)
    spray_width = st.number_input("Spray Width (m)", min_value=0.5, value=float(DEFAULT_DRONE.spray_width), step=0.5)
    battery_cap = st.number_input("Battery Capacity (min)", min_value=1.0, value=float(DEFAULT_DRONE.battery_capacity), step=1.0)

    st.header("Operations")
    swap_time = st.number_input("Battery Swap Time (min)", min_value=0.0, value=float(DEFAULT_OPS.battery_swap_time), step=0.5)
    refill_time = st.number_input("Refill Time (min)", min_value=0.0, value=float(DEFAULT_OPS.refill_time), step=0.5)
    to_land_time = st.number_input("Takeoff + Landing (min)", min_value=0.0, value=float(DEFAULT_OPS.takeoff_landing_time), step=0.5)

    st.header("Schedule")
    hours_per_day = st.slider("Operating Hours / Day", min_value=1.0, max_value=24.0, value=float(DEFAULT_HOURS_PER_DAY), step=0.5)

    run_btn = st.button("Run Simulation", type="primary", use_container_width=True)

drone = Drone(speed=drone_speed, spray_width=spray_width, battery_capacity=battery_cap)
ops = OperationConfig(
    battery_swap_time=swap_time,
    refill_time=refill_time,
    takeoff_landing_time=to_land_time,
)

if run_btn:
    base_planner = LawnmowerPlanner(field, drone)
    opt_planner = OptimizerPlanner(field, drone)

    base_waypoints = base_planner.plan()
    opt_waypoints = opt_planner.plan()

    sim = Simulator(field, drone, ops)
    base_metrics = sim.run(base_waypoints, hours_per_day)
    opt_metrics = sim.run(opt_waypoints, hours_per_day)

    st.session_state.base_metrics = base_metrics
    st.session_state.opt_metrics = opt_metrics
    st.session_state.base_waypoints = base_waypoints
    st.session_state.opt_waypoints = opt_waypoints
    st.session_state.field = field
    st.session_state.fwidth = field.width or 400
    st.session_state.fheight = field.height or 300
    st.session_state.fverts = field.vertices
    st.session_state.best_angle = opt_planner._find_best_sweep_direction()

if st.session_state.base_metrics is not None:
    bm = st.session_state.base_metrics
    om = st.session_state.opt_metrics
    bw = st.session_state.base_waypoints
    ow = st.session_state.opt_waypoints
    f = st.session_state.field
    fw = st.session_state.fwidth
    fh = st.session_state.fheight
    fv = st.session_state.fverts

    col1, col2 = st.columns(2)

    with col1:
        st.subheader("Baseline (Lawnmower)")
        c1, c2 = st.columns(2)
        c1.metric("Acres / Day", f"{bm.total_acres:.2f}")
        c2.metric("Field Area", f"{f.area_acres:.2f} ac")
        c1.metric("Flights / Day", str(bm.total_flights))
        c2.metric("Flying Time", f"{bm.time_breakdown['flying']:.1f}%")
        c1.metric("Acres / Hour", f"{bm.acres_per_hour:.2f}")
        c2.metric("Coverage Eff.", f"{bm.total_acres / f.area_acres * 100:.1f}%" if f.area_acres > 0 else "N/A")

        st.plotly_chart(plot_path_coverage(bw, fw, fh, fv, title="Baseline Path"), use_container_width=True)
        st.plotly_chart(plot_time_breakdown(bm, title="Baseline Time Breakdown"), use_container_width=True)
        st.plotly_chart(plot_flight_summary(bm, title="Baseline Flights"), use_container_width=True)

    with col2:
        st.subheader("Optimized")
        c1, c2 = st.columns(2)
        c1.metric("Acres / Day", f"{om.total_acres:.2f}")
        c2.metric("Field Area", f"{f.area_acres:.2f} ac")
        c1.metric("Flights / Day", str(om.total_flights))
        c2.metric("Flying Time", f"{om.time_breakdown['flying']:.1f}%")
        c1.metric("Acres / Hour", f"{om.acres_per_hour:.2f}")
        c2.metric("Coverage Eff.", f"{om.total_acres / f.area_acres * 100:.1f}%" if f.area_acres > 0 else "N/A")

        st.plotly_chart(plot_path_coverage(ow, fw, fh, fv, title="Optimized Path"), use_container_width=True)
        st.plotly_chart(plot_time_breakdown(om, title="Optimized Time Breakdown"), use_container_width=True)
        st.plotly_chart(plot_flight_summary(om, title="Optimized Flights"), use_container_width=True)

    st.divider()
    st.subheader("Results")

    tab_anim, tab_results = st.tabs(["Flight Animation", "All Results"])

    with tab_anim:
        planner_choice = st.radio("Planner", ["Baseline", "Optimized"],
                                  horizontal=True, key="anim_planner")
        flights_data = bm.flights if planner_choice == "Baseline" else om.flights

        if flights_data:
            flight_labels = [f"Flight {i+1} ({fl.area_sprayed_acres:.2f} ac, {fl.flight_time_min:.1f} min)"
                             for i, fl in enumerate(flights_data)]
            sel_flight = st.selectbox("Select Flight", range(len(flights_data)),
                                      format_func=lambda i: flight_labels[i])

            path = flights_data[sel_flight].path
            if len(path) >= 2:
                st.plotly_chart(
                    plot_animated_flight(path, fw, fh, fv,
                                         title=f"Flight {sel_flight + 1} — Step-by-Step Animation"),
                    use_container_width=True
                )
                st.caption("Click **Play** to watch the drone fly. Use the **slider** to jump to any step.")
            else:
                st.info("Selected flight has no valid path.")
        else:
            st.info("No flights completed for this planner.")

    with tab_results:
        st.plotly_chart(plot_metrics_comparison(bm, om), use_container_width=True)

        delta_acres = om.total_acres - bm.total_acres
        delta_pct = (delta_acres / bm.total_acres * 100) if bm.total_acres > 0 else 0
        delta_flights = om.total_flights - bm.total_flights

        c1, c2, c3 = st.columns(3)
        c1.metric("Acres Improvement", f"{delta_acres:+.2f}", f"{delta_pct:+.1f}%")
        c2.metric("Flight Count Change", f"{delta_flights:+d}")
        c3.metric("Efficiency (Opt vs Base)",
                  f"{om.acres_per_hour:.2f} vs {bm.acres_per_hour:.2f} ac/hr")

        st.info(f"Field Type: {'Polygon' if f.is_polygon else 'Rectangle'} | "
                f"Area: {f.area_acres:.2f} acres | "
                f"Best Sweep Angle: {st.session_state.best_angle:.0f}°")

else:
    col1, col2 = st.columns(2)
    with col1:
        st.subheader("Baseline (Lawnmower)")
        st.info("Adjust parameters in the sidebar and click **Run Simulation**")
        if field.is_polygon:
            st.plotly_chart(plot_path_coverage([], field.width or 400, field.height or 300, field.vertices, title="Field Boundary"), use_container_width=True)

    with col2:
        st.subheader("Optimized")
        st.info("Adjust parameters in the sidebar and click **Run Simulation**")
