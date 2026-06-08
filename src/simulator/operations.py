from dataclasses import dataclass


@dataclass
class OperationConfig:
    battery_swap_time: float = 2.0       # minutes
    refill_time: float = 3.0             # minutes
    takeoff_landing_time: float = 1.0    # minutes (total for takeoff + landing)

    @property
    def ground_overhead_per_flight(self) -> float:
        return self.battery_swap_time + self.refill_time + self.takeoff_landing_time
