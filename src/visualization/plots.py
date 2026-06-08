import plotly.graph_objects as go
from plotly.subplots import make_subplots
from typing import List, Tuple, Optional
from src.simulator.metrics import SimulationMetrics


def _add_field_trace(fig, field_width: float, field_height: float,
                     vertices: Optional[List[Tuple[float, float]]] = None):
    if vertices and len(vertices) >= 3:
        xs = [v[0] for v in vertices] + [vertices[0][0]]
        ys = [v[1] for v in vertices] + [vertices[0][1]]
    else:
        xs = [0, field_width, field_width, 0, 0]
        ys = [0, 0, field_height, field_height, 0]

    fig.add_trace(go.Scatter(
        x=xs, y=ys, mode="lines", fill="toself",
        fillcolor="lightgreen", opacity=0.15,
        line=dict(color="green", width=2),
        name="Field", hoverinfo="skip",
    ))


def _add_field_shape(fig, field_width: float, field_height: float,
                     vertices: Optional[List[Tuple[float, float]]] = None):
    if vertices and len(vertices) >= 3:
        xs = [v[0] for v in vertices] + [vertices[0][0]]
        ys = [v[1] for v in vertices] + [vertices[0][1]]
        fig.add_trace(go.Scatter(
            x=xs, y=ys, mode="lines",
            line=dict(color="green", width=2),
            fill="toself", fillcolor="lightgreen", opacity=0.15,
            name="Field Boundary", showlegend=False,
        ))
    else:
        fig.add_shape(
            type="rect", x0=0, y0=0, x1=field_width, y1=field_height,
            line=dict(color="green", width=2),
            fillcolor="lightgreen", opacity=0.1,
        )


def plot_animated_flight(waypoints: List[Tuple[float, float]],
                         field_width: float, field_height: float,
                         vertices: Optional[List[Tuple[float, float]]] = None,
                         title: str = "First Flight Animation") -> go.Figure:
    if len(waypoints) < 2:
        fig = go.Figure()
        fig.add_annotation(text="Not enough waypoints for animation",
                           showarrow=False, x=0.5, y=0.5, xref="paper", yref="paper")
        return fig

    fig = go.Figure()

    _add_field_trace(fig, field_width, field_height, vertices)

    fig.add_trace(go.Scatter(
        x=[waypoints[0][0]], y=[waypoints[0][1]],
        mode="lines+markers",
        marker=dict(size=4, color="blue"),
        line=dict(color="blue", width=2),
        name="Flight Path"
    ))

    fig.add_trace(go.Scatter(
        x=[waypoints[0][0]], y=[waypoints[0][1]],
        mode="markers+text",
        marker=dict(size=14, color="red", symbol="x"),
        text=["Start"], textposition="top center",
        textfont=dict(color="white", size=11),
        name="Drone"
    ))

    _txt_color = "white"

    total = len(waypoints) - 1
    frames = []
    for k in range(total):
        pts = waypoints[:k + 2]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]

        frames.append(go.Frame(
            data=[
                go.Scatter(x=xs, y=ys, mode="lines+markers",
                           marker=dict(size=4, color="blue"),
                           line=dict(color="blue", width=2)),
                go.Scatter(
                    x=[pts[-1][0]], y=[pts[-1][1]],
                    mode="markers+text",
                    marker=dict(size=14, color="red", symbol="x"),
                    text=[f"{k + 1}/{total}"],
                    textposition="top center",
                    textfont=dict(color=_txt_color, size=11),
                ),
            ],
            traces=[1, 2],
            name=f"f{k}",
        ))

    fig.frames = frames

    slider_steps = []
    for k in range(total):
        slider_steps.append(dict(
            method="animate",
            args=[[f"f{k}"], dict(mode="immediate", frame=dict(duration=0, redraw=True))],
            label=str(k + 1),
        ))

    fig.update_layout(
        title=title,
        xaxis_title="Width (m)", yaxis_title="Height (m)",
        xaxis=dict(scaleanchor="y", scaleratio=1),
        height=550,
        showlegend=True,
        sliders=[dict(
            active=0,
            steps=slider_steps,
            transition=dict(duration=0),
            x=0, y=-0.05, len=1.0,
        )],
        updatemenus=[dict(
            type="buttons",
            showactive=False,
            buttons=[
                dict(label="Play", method="animate",
                     args=[None, dict(frame=dict(duration=80, redraw=True),
                                     fromcurrent=True, mode="immediate",
                                     transition=dict(duration=0))],
                     ),
                dict(label="Pause", method="animate",
                     args=[None, dict(frame=dict(duration=0, redraw=True),
                                     mode="immediate")]),
            ],
            font=dict(color="black", size=14),
            bgcolor="white",
            bordercolor="#ccc",
            borderwidth=1,
            x=0.5, y=-0.45, xanchor="center",
        )],
        margin=dict(t=30, b=140),
    )

    return fig


def plot_path_coverage(waypoints: List[Tuple[float, float]],
                       field_width: float, field_height: float,
                       vertices: Optional[List[Tuple[float, float]]] = None,
                       title: str = "Coverage Path") -> go.Figure:
    fig = go.Figure()

    _add_field_shape(fig, field_width, field_height, vertices)

    xs = [p[0] for p in waypoints]
    ys = [p[1] for p in waypoints]

    fig.add_trace(go.Scatter(
        x=xs, y=ys, mode="lines+markers",
        marker=dict(size=4, color="blue"),
        line=dict(color="blue", width=2),
        name="Flight Path"
    ))

    if waypoints:
        fig.add_trace(go.Scatter(
            x=[waypoints[0][0]], y=[waypoints[0][1]],
            mode="markers+text", marker=dict(size=10, color="green"),
            text=["Start"], textposition="top center",
            textfont=dict(color="white", size=11), name="Start"
        ))
        fig.add_trace(go.Scatter(
            x=[waypoints[-1][0]], y=[waypoints[-1][1]],
            mode="markers+text", marker=dict(size=10, color="red"),
            text=["End"], textposition="top center",
            textfont=dict(color="white", size=11), name="End"
        ))

    fig.update_layout(
        title=title,
        xaxis_title="Width (m)", yaxis_title="Height (m)",
        xaxis=dict(scaleanchor="y", scaleratio=1),
        height=500,
        showlegend=True,
    )
    return fig


def plot_metrics_comparison(baseline: SimulationMetrics,
                            optimized: Optional[SimulationMetrics] = None,
                            title: str = "Performance Metrics") -> go.Figure:
    categories = ["Acres/Day", "Flights/Day", "Flying Time (%)", "Acres/Hour"]
    baseline_vals = [
        round(baseline.total_acres, 1),
        baseline.total_flights,
        round(baseline.time_breakdown["flying"], 1),
        round(baseline.acres_per_hour, 2),
    ]

    fig = go.Figure()
    fig.add_trace(go.Bar(name="Baseline", x=categories, y=baseline_vals,
                         marker_color="lightblue"))

    if optimized is not None:
        opt_vals = [
            round(optimized.total_acres, 1),
            optimized.total_flights,
            round(optimized.time_breakdown["flying"], 1),
            round(optimized.acres_per_hour, 2),
        ]
        fig.add_trace(go.Bar(name="Optimized", x=categories, y=opt_vals,
                             marker_color="orange"))

    fig.update_layout(
        title=title,
        barmode="group",
        yaxis_title="Value",
        height=400,
    )
    return fig


def plot_time_breakdown(metrics: SimulationMetrics,
                        title: str = "Time Breakdown") -> go.Figure:
    labels = ["Flying", "Idle (Battery Swap + Refill + T/O)"]
    values = [metrics.total_flying_time_min, metrics.total_idle_time_min]

    colors = ["#2ecc71", "#e74c3c"]
    if sum(values) == 0:
        values = [1, 0]
        colors = ["#cccccc", "#ffffff"]

    fig = go.Figure(data=[go.Pie(labels=labels, values=values,
                                  marker=dict(colors=colors), textinfo="label+percent")])
    fig.update_layout(title=title, height=350)
    return fig


def plot_flight_summary(metrics: SimulationMetrics,
                        title: str = "Flight-by-Flight Area") -> go.Figure:
    if not metrics.flights:
        fig = go.Figure()
        fig.add_annotation(text="No flights completed", showarrow=False,
                           x=0.5, y=0.5, xref="paper", yref="paper")
        fig.update_layout(title=title, height=300)
        return fig

    flights = list(range(1, len(metrics.flights) + 1))
    areas = [f.area_sprayed_acres for f in metrics.flights]
    times = [f.flight_time_min for f in metrics.flights]

    fig = make_subplots(specs=[[{"secondary_y": True}]])
    fig.add_trace(go.Bar(x=flights, y=areas, name="Acres", marker_color="forestgreen"),
                  secondary_y=False)
    fig.add_trace(go.Scatter(x=flights, y=times, name="Flight Time (min)",
                              mode="lines+markers", line=dict(color="orange", width=2)),
                  secondary_y=True)

    fig.update_xaxes(title_text="Flight #", dtick=1)
    fig.update_yaxes(title_text="Acres Sprayed", secondary_y=False)
    fig.update_yaxes(title_text="Flight Time (min)", secondary_y=True)
    fig.update_layout(title=title, height=300)
    return fig
