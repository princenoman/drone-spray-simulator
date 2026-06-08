import math
from typing import List, Tuple
from src.simulator.field import Field
from src.simulator.drone import Drone


class LawnmowerPlanner:
    def __init__(self, field: Field, drone: Drone):
        self.field = field
        self.drone = drone

    def plan(self) -> List[Tuple[float, float]]:
        if self.field.is_polygon:
            return self._sweep_path(0.0)
        return self._plan_rectangle()

    def _plan_rectangle(self) -> List[Tuple[float, float]]:
        n_rows = max(1, math.ceil(self.field.height / self.drone.spray_width))
        spacing = self.field.height / n_rows
        waypoints = []
        for row in range(n_rows):
            y = row * spacing + spacing / 2
            if row % 2 == 0:
                waypoints.append((0, y))
                waypoints.append((self.field.width, y))
            else:
                waypoints.append((self.field.width, y))
                waypoints.append((0, y))
        return waypoints

    def _sweep_path(self, sweep_angle_deg: float) -> List[Tuple[float, float]]:
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

    def _dist(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    def describe(self) -> str:
        return "Lawnmower"
