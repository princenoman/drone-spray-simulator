import plotly.graph_objects as go
from plotly.subplots import make_subplots
from typing import List, Tuple, Optional
from src.simulator.metrics import SimulationMetrics
from src.simulator.field import Field


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


def _get_sweep_reference_vector(waypoints: List[Tuple[float, float]]) -> Tuple[float, float]:
    if len(waypoints) < 2:
        return (1.0, 0.0)
    max_len = -1.0
    ref_vec = (1.0, 0.0)
    for i in range(len(waypoints) - 1):
        dx = waypoints[i+1][0] - waypoints[i][0]
        dy = waypoints[i+1][1] - waypoints[i][1]
        d = (dx*dx + dy*dy) ** 0.5
        if d > max_len:
            max_len = d
            if d > 0:
                ref_vec = (dx / d, dy / d)
    return ref_vec


def _make_tooltip_data(waypoints, drone_speed, spray_width, field, battery_capacity=20.0):
    total = len(waypoints)
    cum_dist = [0.0]
    cum_spray = [0.0]
    actions = ["START"]

    ref_vec = _get_sweep_reference_vector(waypoints)

    for i in range(1, total):
        x1, y1 = waypoints[i - 1]
        x2, y2 = waypoints[i]
        dx = x2 - x1
        dy = y2 - y1
        d = (dx * dx + dy * dy) ** 0.5
        cum_dist.append(cum_dist[-1] + d)

        is_spray = False
        if field is not None and field.contains((x1 + x2) / 2, (y1 + y2) / 2):
            if d > 0:
                dot_prod = abs((dx / d) * ref_vec[0] + (dy / d) * ref_vec[1])
                if dot_prod > 0.99:
                    is_spray = True

        if is_spray:
            cum_spray.append(cum_spray[-1] + d)
        else:
            cum_spray.append(cum_spray[-1])

        if is_spray:
            if abs(dy) > abs(dx):
                actions.append("SPRAY (UP)" if dy > 0 else "SPRAY (DOWN)")
            else:
                actions.append("SPRAY (RIGHT)" if dx > 0 else "SPRAY (LEFT)")
        else:
            actions.append("TURN")

    speed_ms = drone_speed
    spray_m = spray_width
    tooltip_data = []
    for i in range(total):
        t_min = cum_dist[i] / speed_ms / 60
        bat_min = max(0.0, battery_capacity - t_min)
        area_m2 = cum_spray[i] * spray_m
        ac_hr = (area_m2 / 4046.86) / max(t_min / 60, 0.001)

        tooltip_data.append([
            round(t_min, 2),
            round(bat_min, 2),
            round(area_m2, 2),
            round(ac_hr, 4),
            actions[i],
        ])
    return tooltip_data


def plot_animated_flight(waypoints: List[Tuple[float, float]],
                         field_width: float, field_height: float,
                         vertices: Optional[List[Tuple[float, float]]] = None,
                         title: str = "First Flight Animation",
                         reverse: bool = False,
                         drone_speed: float = 5.0,
                         spray_width: float = 10.0,
                         field: Optional[Field] = None,
                         battery_capacity: float = 20.0,
                         all_flights: Optional[List] = None,
                         current_flight_idx: Optional[int] = None) -> go.Figure:
    if reverse:
        waypoints = waypoints[::-1]
    if len(waypoints) < 2:
        fig = go.Figure()
        fig.add_annotation(text="Not enough waypoints for animation",
                           showarrow=False, x=0.5, y=0.5, xref="paper", yref="paper")
        return fig

    tooltip_data = _make_tooltip_data(waypoints, drone_speed, spray_width, field, battery_capacity)
    total = len(waypoints) - 1

    if vertices and len(vertices) >= 3:
        xs = [v[0] for v in vertices]
        ys = [v[1] for v in vertices]
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)
    else:
        xmin, xmax = 0.0, field_width
        ymin, ymax = 0.0, field_height

    x_margin = (xmax - xmin) * 0.05 or 10.0
    y_margin = (ymax - ymin) * 0.05 or 10.0
    x_range = [xmin - x_margin, xmax + x_margin]
    y_range = [ymin - y_margin, ymax + y_margin]

    hover_tmpl = (
        "<br>"
        "&nbsp;&nbsp;<b>— DRONE TELEMETRY —</b>&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Time elapsed:</b> %{customdata[0]:.2f} min&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Battery Remaining:</b> %{customdata[1]:.2f} min&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Covered Area:</b> %{customdata[2]:.1f} m²&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Current Productivity:</b> %{customdata[3]:.2f} ac/hr&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Action State:</b> %{customdata[4]}&nbsp;&nbsp;"
        "<br>"
        "<extra></extra>"
    )

    fig = go.Figure()

    _add_field_trace(fig, field_width, field_height, vertices)

    fig.add_trace(go.Scatter(
        x=[waypoints[0][0]], y=[waypoints[0][1]],
        mode="lines+markers",
        marker=dict(size=4, color="blue"),
        line=dict(color="blue", width=2),
        name="Flight Path", hoverinfo="skip"
    ))

    fig.add_trace(go.Scatter(
        x=[waypoints[0][0]], y=[waypoints[0][1]],
        mode="markers",
        marker=dict(size=14, color="red", symbol="x"),
        name="Drone",
        customdata=[tooltip_data[0]],
        hovertemplate=hover_tmpl,
        hoverlabel=dict(
            bgcolor="#ffffff",
            bordercolor="#dddddd",
            font=dict(family="sans-serif", size=15, color="#333333"),
            align="left",
        ),
    ))

    # Add previous completed flights' stoppage markers as static traces
    if all_flights and current_flight_idx is not None:
        for idx in range(current_flight_idx):
            prev_path = all_flights[idx].path
            if len(prev_path) >= 3:
                prev_stop = prev_path[-2]
                fig.add_trace(go.Scatter(
                    x=[prev_stop[0]], y=[prev_stop[1]],
                    mode="markers+text",
                    marker=dict(size=8, color="#7f8c8d", symbol="square"),
                    text=[f"<b>Flight {idx+1} Stop</b>"], textposition="top center",
                    textfont=dict(color="#7f8c8d", size=10),
                    name=f"Flight {idx+1} Stop",
                    hoverinfo="skip"
                ))

    # Add current flight's stoppage trace (initially empty, will be populated in frames)
    curr_label = f"Flight {current_flight_idx + 1} Stop" if current_flight_idx is not None else "Last Stop"
    if len(waypoints) >= 3:
        fig.add_trace(go.Scatter(
            x=[], y=[],
            mode="markers+text",
            marker=dict(size=10, color="darkorange", symbol="square"),
            text=[f"<b>{curr_label}</b>"], textposition="top center",
            textfont=dict(color="#d35400", size=12),
            name=curr_label,
            hoverinfo="skip"
        ))

    current_stop_idx = 3 + (current_flight_idx if current_flight_idx is not None else 0)

    _txt_color = "white"

    frames = []
    for k in range(total):
        pts = waypoints[:k + 2]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]

        # Show current stoppage marker only after the drone reaches waypoints[-2]
        # which corresponds to step k >= total - 2
        if len(waypoints) >= 3 and k >= total - 2:
            last_stop = waypoints[-2]
            stop_trace = go.Scatter(
                x=[last_stop[0]], y=[last_stop[1]],
                mode="markers+text",
                marker=dict(size=10, color="darkorange", symbol="square"),
                text=[f"<b>{curr_label}</b>"], textposition="top center",
                textfont=dict(color="#d35400", size=12),
                name=curr_label,
                hoverinfo="skip"
            )
        else:
            stop_trace = go.Scatter(
                x=[], y=[],
                mode="markers",
                showlegend=False,
                hoverinfo="skip"
            )

        frames.append(go.Frame(
            data=[
                go.Scatter(x=xs, y=ys, mode="lines+markers",
                           marker=dict(size=4, color="blue"),
                           line=dict(color="blue", width=2)),
                go.Scatter(
                    x=[pts[-1][0]], y=[pts[-1][1]],
                    mode="markers",
                    marker=dict(size=14, color="red", symbol="x"),
                    customdata=[tooltip_data[k + 1]],
                    hovertemplate=hover_tmpl,
                    hoverlabel=dict(
                        bgcolor="#ffffff",
                        bordercolor="#dddddd",
                        font=dict(family="sans-serif", size=15, color="#333333"),
                        align="left",
                    ),
                ),
                stop_trace
            ],
            traces=[1, 2, current_stop_idx],
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
        xaxis=dict(scaleanchor="y", scaleratio=1, range=x_range),
        yaxis=dict(range=y_range),
        height=550,
        showlegend=True,
        hovermode="closest",
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
                       title: str = "Coverage Path",
                       field: Optional[Field] = None,
                       spray_width: float = 10.0) -> go.Figure:
    fig = go.Figure()

    _add_field_shape(fig, field_width, field_height, vertices)

    xs = [p[0] for p in waypoints]
    ys = [p[1] for p in waypoints]

    total = len(waypoints)
    tooltip_data = []

    if total >= 2:
        cum_dist = [0.0]
        cum_spray = [0.0]
        actions = ["START"]
        ref_vec = _get_sweep_reference_vector(waypoints)

        for i in range(1, total):
            x1, y1 = waypoints[i - 1]
            x2, y2 = waypoints[i]
            dx = x2 - x1
            dy = y2 - y1
            d = (dx * dx + dy * dy) ** 0.5
            cum_dist.append(cum_dist[-1] + d)

            is_spray = False
            if field is not None and field.contains((x1 + x2) / 2, (y1 + y2) / 2):
                if d > 0:
                    dot_prod = abs((dx / d) * ref_vec[0] + (dy / d) * ref_vec[1])
                    if dot_prod > 0.99:
                        is_spray = True

            if is_spray:
                cum_spray.append(cum_spray[-1] + d)
            else:
                cum_spray.append(cum_spray[-1])

            if is_spray:
                if abs(dy) > abs(dx):
                    actions.append("SPRAYING (UP)" if dy > 0 else "SPRAYING (DOWN)")
                else:
                    actions.append("SPRAYING (RIGHT)" if dx > 0 else "SPRAYING (LEFT)")
            else:
                actions.append("TURNING/TRANSIT")

        for i in range(total):
            area_m2 = cum_spray[i] * spray_width
            area_ac = area_m2 / 4046.86
            tooltip_data.append([
                i + 1,
                round(cum_dist[i], 1),
                round(area_m2, 1),
                round(area_ac, 3),
                actions[i]
            ])
    else:
        for i in range(total):
            tooltip_data.append([i + 1, 0.0, 0.0, 0.0, "START"])

    hover_tmpl = (
        "<br>"
        "&nbsp;&nbsp;<b>— WAYPOINT DETAIL —</b>&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Waypoint #:</b> %{customdata[0]}&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Position (X, Y):</b> (%{x:.1f}, %{y:.1f}) m&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Cumulative Distance:</b> %{customdata[1]:.1f} m&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Sprayed Area:</b> %{customdata[2]:.1f} m² (%{customdata[3]:.3f} ac)&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Flight State:</b> %{customdata[4]}&nbsp;&nbsp;"
        "<br>"
        "<extra></extra>"
    )

    fig.add_trace(go.Scatter(
        x=xs, y=ys, mode="lines+markers",
        marker=dict(size=5, color="blue"),
        line=dict(color="blue", width=2),
        name="Flight Path",
        customdata=tooltip_data,
        hovertemplate=hover_tmpl,
        hoverlabel=dict(
            bgcolor="#ffffff",
            bordercolor="#dddddd",
            font=dict(family="sans-serif", size=15, color="#333333"),
            align="left",
        ),
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
        round(baseline.total_acres, 2),
        baseline.total_flights,
        round(baseline.time_breakdown["flying"], 1),
        round(baseline.acres_per_hour, 2),
    ]

    baseline_hover = (
        "<br>"
        "&nbsp;&nbsp;<b>— BASELINE PERFORMANCE —</b>&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Metric Name:</b> %{x}&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Recorded Value:</b> %{y}&nbsp;&nbsp;"
        "<br>"
        "<extra></extra>"
    )

    fig = go.Figure()
    fig.add_trace(go.Bar(
        name="Baseline", x=categories, y=baseline_vals,
        marker_color="lightblue",
        hovertemplate=baseline_hover,
        hoverlabel=dict(
            bgcolor="#ffffff",
            bordercolor="#dddddd",
            font=dict(family="sans-serif", size=15, color="#333333"),
            align="left",
        ),
    ))

    if optimized is not None:
        opt_vals = [
            round(optimized.total_acres, 2),
            optimized.total_flights,
            round(optimized.time_breakdown["flying"], 1),
            round(optimized.acres_per_hour, 2),
        ]
        opt_hover = (
            "<br>"
            "&nbsp;&nbsp;<b>— OPTIMIZED PERFORMANCE —</b>&nbsp;&nbsp;<br>"
            "&nbsp;&nbsp;<b>Metric Name:</b> %{x}&nbsp;&nbsp;<br>"
            "&nbsp;&nbsp;<b>Recorded Value:</b> %{y}&nbsp;&nbsp;"
            "<br>"
            "<extra></extra>"
        )
        fig.add_trace(go.Bar(
            name="Optimized", x=categories, y=opt_vals,
            marker_color="orange",
            hovertemplate=opt_hover,
            hoverlabel=dict(
                bgcolor="#ffffff",
                bordercolor="#dddddd",
                font=dict(family="sans-serif", size=15, color="#333333"),
                align="left",
            ),
        ))

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

    hovertemplate = (
        "<br>"
        "&nbsp;&nbsp;<b>— TIME DETAIL —</b>&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Category:</b> %{label}&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Total Duration:</b> %{value:.1f} min&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Percentage:</b> %{percent}&nbsp;&nbsp;"
        "<br>"
        "<extra></extra>"
    )

    fig = go.Figure(data=[go.Pie(
        labels=labels, values=values,
        marker=dict(colors=colors),
        textinfo="label+percent",
        hovertemplate=hovertemplate,
        hoverlabel=dict(
            bgcolor="#ffffff",
            bordercolor="#dddddd",
            font=dict(family="sans-serif", size=15, color="#333333"),
            align="left",
        ),
    )])
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

    bar_hover = (
        "<br>"
        "&nbsp;&nbsp;<b>— FLIGHT SUMMARY (AREA) —</b>&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Flight Number:</b> %{x}&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Sprayed Area:</b> %{y:.2f} ac&nbsp;&nbsp;"
        "<br>"
        "<extra></extra>"
    )

    scatter_hover = (
        "<br>"
        "&nbsp;&nbsp;<b>— FLIGHT SUMMARY (TIME) —</b>&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Flight Number:</b> %{x}&nbsp;&nbsp;<br>"
        "&nbsp;&nbsp;<b>Flight Time:</b> %{y:.1f} min&nbsp;&nbsp;"
        "<br>"
        "<extra></extra>"
    )

    fig = make_subplots(specs=[[{"secondary_y": True}]])
    fig.add_trace(
        go.Bar(
            x=flights, y=areas, name="Acres", marker_color="forestgreen",
            hovertemplate=bar_hover,
            hoverlabel=dict(
                bgcolor="#ffffff",
                bordercolor="#dddddd",
                font=dict(family="sans-serif", size=15, color="#333333"),
                align="left",
            ),
        ),
        secondary_y=False
    )
    fig.add_trace(
        go.Scatter(
            x=flights, y=times, name="Flight Time (min)",
            mode="lines+markers", line=dict(color="orange", width=2),
            hovertemplate=scatter_hover,
            hoverlabel=dict(
                bgcolor="#ffffff",
                bordercolor="#dddddd",
                font=dict(family="sans-serif", size=15, color="#333333"),
                align="left",
            ),
        ),
        secondary_y=True
    )

    fig.update_xaxes(title_text="Flight #", dtick=1)
    fig.update_yaxes(title_text="Acres Sprayed", secondary_y=False)
    fig.update_yaxes(title_text="Flight Time (min)", secondary_y=True)
    fig.update_layout(title=title, height=300)
    return fig
