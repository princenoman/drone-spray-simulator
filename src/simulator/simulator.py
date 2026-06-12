from typing import List, Tuple
from .field import Field
from .drone import Drone
from .operations import OperationConfig
from .metrics import SimulationMetrics, FlightRecord


class Simulator:
    def __init__(self, field: Field, drone: Drone, ops: OperationConfig):
        self.field = field
        self.drone = drone
        self.ops = ops

    def run(self, waypoints: List[Tuple[float, float]], hours_per_day: float = 24.0, use_lookahead: bool = False) -> SimulationMetrics:
        total_minutes_available = hours_per_day * 60
        elapsed_minutes = 0.0
        metrics = SimulationMetrics()

        self.ref_vec = self._get_sweep_reference_vector(waypoints)

        max_flight_distance = self.drone.speed * self.drone.battery_capacity * 60

        total_path_distance = self._path_distance(waypoints)
        if total_path_distance == 0:
            return metrics

        segments = self._segment_path(waypoints, max_flight_distance, use_lookahead)
        flight_number = 0

        for segment in segments:
            seg_spray_dist = self._spray_distance(segment)
            seg_total_dist = self._path_distance(segment)
            seg_time = seg_total_dist / self.drone.speed / 60
            seg_area = (seg_spray_dist * self.drone.spray_width) / 4046.86

            flight_overhead = self.ops.ground_overhead_per_flight

            if elapsed_minutes + seg_time + flight_overhead > total_minutes_available:
                break

            flight_number += 1
            metrics.total_flights = flight_number
            metrics.total_distance_m += seg_total_dist
            metrics.total_flying_time_min += seg_time
            metrics.total_acres += seg_area
            elapsed_minutes += seg_time + flight_overhead

            metrics.flights.append(FlightRecord(
                flight_number=flight_number,
                distance_m=seg_total_dist,
                flight_time_min=seg_time,
                area_sprayed_acres=seg_area,
                path=segment,
            ))

        metrics.total_idle_time_min = elapsed_minutes - metrics.total_flying_time_min
        metrics.total_operating_hours = elapsed_minutes / 60

        return metrics

    def _path_distance(self, waypoints: List[Tuple[float, float]]) -> float:
        if len(waypoints) < 2:
            return 0.0
        total = 0.0
        for i in range(len(waypoints) - 1):
            dx = waypoints[i+1][0] - waypoints[i][0]
            dy = waypoints[i+1][1] - waypoints[i][1]
            total += (dx*dx + dy*dy) ** 0.5
        return total

    def _get_sweep_reference_vector(self, waypoints: List[Tuple[float, float]]) -> Tuple[float, float]:
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

    def _spray_distance(self, waypoints: List[Tuple[float, float]]) -> float:
        if len(waypoints) < 2:
            return 0.0
        total = 0.0
        ref_vec = getattr(self, 'ref_vec', (1.0, 0.0))
        for i in range(len(waypoints) - 1):
            x1, y1 = waypoints[i]
            x2, y2 = waypoints[i+1]
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            if self.field.contains(mx, my):
                dx = x2 - x1
                dy = y2 - y1
                d = (dx*dx + dy*dy) ** 0.5
                if d > 0:
                    dot_prod = abs((dx / d) * ref_vec[0] + (dy / d) * ref_vec[1])
                    if dot_prod > 0.99:
                        total += d
        return total

    def _dist(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    def _segment_path(self, waypoints: List[Tuple[float, float]], max_dist: float, use_lookahead: bool = False) -> List[List[Tuple[float, float]]]:
        bounds = self.field.bounds
        home = ((bounds[0] + bounds[2]) / 2, bounds[1])

        segments = []
        if not waypoints:
            return segments

        waypoints = list(waypoints)
        current_flight = [home]
        r_dist = max_dist
        w_idx = 0

        while w_idx < len(waypoints):
            p_next = waypoints[w_idx]
            p_curr = current_flight[-1]
            d_step = self._dist(p_curr, p_next)
            d_home_next = self._dist(p_next, home)
            d_total_if_proceed = d_step + d_home_next

            if r_dist >= d_total_if_proceed:
                should_proceed = True
                if use_lookahead and w_idx + 1 < len(waypoints):
                    p_next2 = waypoints[w_idx + 1]
                    d_step2 = self._dist(p_next, p_next2)
                    d_home_next2 = self._dist(p_next2, home)
                    d_total_if_proceed2 = d_step + d_step2 + d_home_next2
                    
                    if r_dist < d_total_if_proceed2:
                        d_home_curr = self._dist(p_curr, home)
                        if d_home_curr < d_home_next:
                            should_proceed = False

                if should_proceed:
                    current_flight.append(p_next)
                    r_dist -= d_step
                    w_idx += 1
                else:
                    current_flight.append(home)
                    segments.append(current_flight)
                    current_flight = [home]
                    r_dist = max_dist
            else:
                if len(current_flight) == 1:
                    clip_dist = max_dist / 2
                    if d_step > 0:
                        ratio = clip_dist / d_step
                        dx = p_next[0] - home[0]
                        dy = p_next[1] - home[1]
                        clip_p = (home[0] + dx * ratio, home[1] + dy * ratio)
                        current_flight.append(clip_p)
                        waypoints[w_idx] = clip_p
                    else:
                        w_idx += 1
                    current_flight.append(home)
                    segments.append(current_flight)
                    current_flight = [home]
                    r_dist = max_dist
                else:
                    current_flight.append(home)
                    segments.append(current_flight)
                    current_flight = [home]
                    r_dist = max_dist

        if len(current_flight) > 1:
            if current_flight[-1] != home:
                current_flight.append(home)
            segments.append(current_flight)

        return segments
