import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


@dataclass
class Field:
    width: Optional[float] = None
    height: Optional[float] = None
    vertices: Optional[List[Tuple[float, float]]] = None
    name: str = "Field"

    def __post_init__(self):
        if self.vertices is not None and len(self.vertices) < 3:
            self.vertices = None

    @property
    def is_polygon(self) -> bool:
        return self.vertices is not None and len(self.vertices) >= 3

    @property
    def area_sq_m(self) -> float:
        if self.is_polygon:
            return self._polygon_area()
        if self.width is not None and self.height is not None:
            return self.width * self.height
        return 0.0

    @property
    def area_acres(self) -> float:
        return self.area_sq_m / 4046.86

    @property
    def bounds(self) -> Tuple[float, float, float, float]:
        if self.is_polygon:
            xs = [v[0] for v in self.vertices]
            ys = [v[1] for v in self.vertices]
            return (min(xs), min(ys), max(xs), max(ys))
        return (0.0, 0.0, self.width or 0.0, self.height or 0.0)

    def contains(self, x: float, y: float) -> bool:
        if self.is_polygon:
            return self._point_in_polygon(x, y)
        return (0 <= x <= (self.width or 0) and
                0 <= y <= (self.height or 0))

    def rows(self, spray_width: float) -> int:
        if self.is_polygon:
            _, ymin, _, ymax = self.bounds
            return max(1, math.ceil((ymax - ymin) / spray_width))
        return max(1, math.ceil((self.width or 0) / spray_width))

    def _polygon_area(self) -> float:
        verts = self.vertices
        n = len(verts)
        area = 0.0
        for i in range(n):
            x1, y1 = verts[i]
            x2, y2 = verts[(i + 1) % n]
            area += x1 * y2 - x2 * y1
        return abs(area) / 2.0

    def _point_in_polygon(self, x: float, y: float) -> bool:
        verts = self.vertices
        n = len(verts)
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = verts[i]
            xj, yj = verts[j]
            if ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / (yj - yi) + xi
            ):
                inside = not inside
            j = i
        return inside

    def intersect_sweep_line(self, proj_val: float, perp: Tuple[float, float],
                             sweep: Tuple[float, float]) -> List[Tuple[float, float]]:
        pts = []
        verts = self.vertices
        n = len(verts)
        px, py = perp
        cx = (self.bounds[0] + self.bounds[2]) / 2
        cy = (self.bounds[1] + self.bounds[3]) / 2
        rhs = proj_val + px * cx + py * cy
        for i in range(n):
            x1, y1 = verts[i]
            x2, y2 = verts[(i + 1) % n]
            dx, dy = x2 - x1, y2 - y1
            denom = px * dx + py * dy
            if abs(denom) < 1e-12:
                continue
            t = (rhs - px * x1 - py * y1) / denom
            if 0 <= t <= 1:
                pts.append((x1 + t * dx, y1 + t * dy))
        return pts
