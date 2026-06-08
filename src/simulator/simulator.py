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

    def run(self, waypoints: List[Tuple[float, float]], hours_per_day: float = 24.0) -> SimulationMetrics:
        total_minutes_available = hours_per_day * 60
        elapsed_minutes = 0.0
        metrics = SimulationMetrics()

        max_flight_distance = self.drone.speed * self.drone.battery_capacity * 60

        total_path_distance = self._path_distance(waypoints)
        if total_path_distance == 0:
            return metrics

        segments = self._segment_path(waypoints, max_flight_distance)
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

    def _spray_distance(self, waypoints: List[Tuple[float, float]]) -> float:
        if len(waypoints) < 2:
            return 0.0
        total = 0.0
        for i in range(len(waypoints) - 1):
            x1, y1 = waypoints[i]
            x2, y2 = waypoints[i+1]
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            if self.field.contains(mx, my):
                dx = x2 - x1
                dy = y2 - y1
                total += (dx*dx + dy*dy) ** 0.5
        return total

    def _segment_path(self, waypoints: List[Tuple[float, float]], max_dist: float) -> List[List[Tuple[float, float]]]:
        segments = []
        current_seg = [waypoints[0]]
        running_dist = 0.0

        for i in range(1, len(waypoints)):
            dx = waypoints[i][0] - waypoints[i-1][0]
            dy = waypoints[i][1] - waypoints[i-1][1]
            step_dist = (dx*dx + dy*dy) ** 0.5

            if running_dist + step_dist > max_dist and len(current_seg) >= 2:
                segments.append(current_seg)
                current_seg = [waypoints[i-1], waypoints[i]]
                running_dist = step_dist
            else:
                current_seg.append(waypoints[i])
                running_dist += step_dist

        if len(current_seg) >= 2:
            segments.append(current_seg)

        return segments
