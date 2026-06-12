import sys
sys.path.append(r"c:\Users\USER\Desktop\drone-spray-simulator")

from src.simulator.field import Field
from src.simulator.drone import Drone
from src.simulator.operations import OperationConfig
from src.simulator.simulator import Simulator
from src.planners.lawnmower import LawnmowerPlanner
from src.planners.optimizer import OptimizerPlanner

# Set up field matching user's config: 680x300
field = Field(width=680.0, height=300.0, name="Rectangle Field")
drone = Drone(speed=5.0, spray_width=10.0, battery_capacity=15.0)
ops = OperationConfig(battery_swap_time=2.0, refill_time=3.0, takeoff_landing_time=1.0)

# Simulate segment splitting manually in MockSimulator
class MockSimulator(Simulator):
    def _segment_path(self, waypoints, max_dist, use_lookahead=False):
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
                    # Mid-row split
                    L = self._dist(p_curr, p_next)
                    t_safe = 0.0
                    if L > 0:
                        step_size = 1.0
                        n_steps = int(L / step_size)
                        for step in range(1, n_steps + 1):
                            t = step / n_steps
                            px = p_curr[0] + t * (p_next[0] - p_curr[0])
                            py = p_curr[1] + t * (p_next[1] - p_curr[1])
                            d_travel = t * L
                            d_home = self._dist((px, py), home)
                            if d_travel + d_home <= r_dist:
                                t_safe = t
                            else:
                                break
                    
                    if t_safe > 0.0:
                        p_safe = (p_curr[0] + t_safe * (p_next[0] - p_curr[0]),
                                  p_curr[1] + t_safe * (p_next[1] - p_curr[1]))
                        current_flight.append(p_safe)
                        current_flight.append(home)
                        segments.append(current_flight)
                        current_flight = [home]
                        r_dist = max_dist
                        waypoints.insert(w_idx, p_safe)
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

# Plan paths
base_planner = LawnmowerPlanner(field, drone)
base_waypoints = base_planner.plan(sweep_angle_deg=0.0)

opt_planner = OptimizerPlanner(field, drone)
opt_waypoints = opt_planner.plan(sweep_angle_deg=0.0)

sim = MockSimulator(field, drone, ops)
hours_per_day = 12.0

print("--- Running Baseline (Lawnmower) with mid-row split ---")
base_metrics = sim.run(base_waypoints, hours_per_day, use_lookahead=False)
print(f"Total flights: {base_metrics.total_flights}")
print(f"Total acres sprayed: {base_metrics.total_acres:.4f} acres")
print(f"Acres per hour: {base_metrics.acres_per_hour:.4f} ac/hr")
for fl in base_metrics.flights:
    print(f"  Flight {fl.flight_number}: {fl.area_sprayed_acres:.4f} ac, {fl.flight_time_min:.1f} min")

print("\n--- Running Optimized (Look-Ahead) with mid-row split ---")
opt_metrics = sim.run(opt_waypoints, hours_per_day, use_lookahead=True)
print(f"Total flights: {opt_metrics.total_flights}")
print(f"Total acres sprayed: {opt_metrics.total_acres:.4f} acres")
print(f"Acres per hour: {opt_metrics.acres_per_hour:.4f} ac/hr")
for fl in opt_metrics.flights:
    print(f"  Flight {fl.flight_number}: {fl.area_sprayed_acres:.4f} ac, {fl.flight_time_min:.1f} min")
