from src.simulator.field import Field
from src.simulator.drone import Drone
from src.simulator.operations import OperationConfig

DEFAULT_FIELD = Field(width=400.0, height=300.0, name="Rectangle Field")
DEFAULT_POLYGON = Field(
    vertices=[(50, 50), (350, 50), (400, 150), (350, 250), (50, 250), (0, 150)],
    name="Hexagon Field",
)
DEFAULT_DRONE = Drone(speed=5.0, spray_width=10.0, battery_capacity=15.0)
DEFAULT_OPS = OperationConfig(
    battery_swap_time=2.0,
    refill_time=3.0,
    takeoff_landing_time=1.0,
)
DEFAULT_HOURS_PER_DAY = 12.0
