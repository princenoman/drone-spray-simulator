import math
from typing import List, Tuple, Optional
from src.simulator.field import Field
from src.simulator.drone import Drone


class OptimizerPlanner:
    def __init__(self, field: Field, drone: Drone):
        self.field = field
        self.drone = drone

    def plan(self, sweep_angle_deg: Optional[float] = None) -> List[Tuple[float, float]]:
        if sweep_angle_deg is None:
            sweep_angle_deg = self._find_best_sweep_direction()

        if self.field.is_polygon:
            return self._sweep_path_polygon(sweep_angle_deg)
        return self._sweep_path_rectangle(sweep_angle_deg)

    def _sweep_path_polygon(self, sweep_angle_deg: float) -> List[Tuple[float, float]]:
        angle_rad = math.radians(sweep_angle_deg)
        sweep = (math.cos(angle_rad), math.sin(angle_rad))
        perp = (-sweep[1], sweep[0])

        bounds = self.field.bounds
        cx = (bounds[0] + bounds[2]) / 2
        cy = (bounds[1] + bounds[3]) / 2

        verts = self.field.vertices
        projections = [perp[0] * (v[0] - cx) + perp[1] * (v[1] - cy) for v in verts]
        proj_min, proj_max = min(projections), max(projections)

        n_rows = max(1, math.ceil((proj_max - proj_min) / self.drone.spray_width))
        spacing = (proj_max - proj_min) / n_rows

        waypoints = []
        for row in range(n_rows):
            proj_val = proj_min + row * spacing + spacing / 2
            pts = self.field.intersect_sweep_line(proj_val, perp, sweep)
            pts.sort(key=lambda p: sweep[0] * p[0] + sweep[1] * p[1])

            if len(pts) < 2 or len(pts) % 2 != 0:
                continue

            pairs = [(pts[i], pts[i+1]) for i in range(0, len(pts), 2)]

            if row % 2 == 0:
                for a_pt, b_pt in pairs:
                    if not waypoints or self._dist(waypoints[-1], a_pt) > 0.01:
                        waypoints.append(a_pt)
                    waypoints.append(b_pt)
            else:
                for a_pt, b_pt in reversed(pairs):
                    if not waypoints or self._dist(waypoints[-1], b_pt) > 0.01:
                        waypoints.append(b_pt)
                    waypoints.append(a_pt)

        return waypoints

    def _sweep_path_rectangle(self, sweep_angle_deg: float) -> List[Tuple[float, float]]:
        angle_rad = math.radians(sweep_angle_deg)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)

        eff_width = abs(cos_a * self.field.width) + abs(sin_a * self.field.height)
        eff_height = abs(sin_a * self.field.width) + abs(cos_a * self.field.height)

        n_rows = max(1, math.ceil(eff_width / self.drone.spray_width))
        spacing = eff_width / n_rows

        waypoints = []
        for row in range(n_rows):
            x_pos = row * spacing + spacing / 2 - eff_width / 2

            if row % 2 == 0:
                start_local = (x_pos, -eff_height / 2)
                end_local = (x_pos, eff_height / 2)
            else:
                start_local = (x_pos, eff_height / 2)
                end_local = (x_pos, -eff_height / 2)

            rot_start = self._rotate_point(start_local, angle_rad)
            rot_end = self._rotate_point(end_local, angle_rad)

            shifted_start = (rot_start[0] + self.field.width / 2,
                             rot_start[1] + self.field.height / 2)
            shifted_end = (rot_end[0] + self.field.width / 2,
                           rot_end[1] + self.field.height / 2)

            c_start = self._clamp_to_field(shifted_start)
            c_end = self._clamp_to_field(shifted_end)

            if not waypoints or self._dist(waypoints[-1], c_start) > 0.01:
                waypoints.append(c_start)
            waypoints.append(c_end)

        return waypoints

    def _rotate_point(self, pt: Tuple[float, float], angle: float) -> Tuple[float, float]:
        x, y = pt
        return (x * math.cos(angle) - y * math.sin(angle),
                x * math.sin(angle) + y * math.cos(angle))

    def _clamp_to_field(self, pt: Tuple[float, float]) -> Tuple[float, float]:
        x = max(0, min(self.field.width, pt[0]))
        y = max(0, min(self.field.height, pt[1]))
        return (x, y)

    def _dist(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    def _find_best_sweep_direction(self) -> float:
        best_angle = 0.0
        best_cost = float('inf')

        for angle in range(0, 91, 5):
            if self.field.is_polygon:
                wpts = self._sweep_path_polygon(float(angle))
            else:
                wpts = self._sweep_path_rectangle(float(angle))

            if len(wpts) < 2:
                continue

            n_turns = (len(wpts) - 2) // 2
            total_dist = self._path_distance(wpts)
            cost = n_turns * 10 + total_dist * 0.001

            if cost < best_cost:
                best_cost = cost
                best_angle = float(angle)

        return best_angle

    def _path_distance(self, waypoints: List[Tuple[float, float]]) -> float:
        if len(waypoints) < 2:
            return 0.0
        total = 0.0
        for i in range(len(waypoints) - 1):
            dx = waypoints[i+1][0] - waypoints[i][0]
            dy = waypoints[i+1][1] - waypoints[i][1]
            total += (dx*dx + dy*dy) ** 0.5
        return total

    def describe(self) -> str:
        angle = self._find_best_sweep_direction()
        return f"Optimized (sweep={angle:.0f}deg)"
