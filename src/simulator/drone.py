from dataclasses import dataclass


@dataclass
class Drone:
    speed: float              # m/s
    spray_width: float        # meters
    battery_capacity: float   # minutes of flight time
