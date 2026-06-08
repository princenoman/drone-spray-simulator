from dataclasses import dataclass, field
from typing import List


@dataclass
class FlightRecord:
    flight_number: int
    distance_m: float
    flight_time_min: float
    area_sprayed_acres: float
    path: List[tuple] = field(default_factory=list)


@dataclass
class SimulationMetrics:
    total_acres: float = 0.0
    total_flights: int = 0
    total_flying_time_min: float = 0.0
    total_idle_time_min: float = 0.0
    total_distance_m: float = 0.0
    total_operating_hours: float = 0.0
    flights: List[FlightRecord] = field(default_factory=list)

    @property
    def time_breakdown(self) -> dict:
        total = self.total_flying_time_min + self.total_idle_time_min
        if total == 0:
            return {"flying": 0, "idle": 0}
        return {
            "flying": self.total_flying_time_min / total * 100,
            "idle": self.total_idle_time_min / total * 100,
        }

    @property
    def acres_per_hour(self) -> float:
        total_hours = (self.total_flying_time_min + self.total_idle_time_min) / 60
        if total_hours == 0:
            return 0.0
        return self.total_acres / total_hours
