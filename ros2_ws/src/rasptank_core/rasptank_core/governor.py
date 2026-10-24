"""Speed governor: limits forward speed with the front ultrasonic sensor.

    anyone --/cmd_vel--> governor --/cmd_vel_safe--> base_node
                            ^ /ultrasonic

It is a FILTER, not a behaviour: it never adds motion, it only reduces the
forward speed it is asked for. Turning and reversing always pass (the
sensor only looks ahead). Fail-closed: without a recent, valid distance,
forward motion is blocked.

Limits of the sensor: a narrow cone straight ahead, and nothing lower than
its height (the 4 cm cube is invisible to it). It protects from walls and
boxes, not from everything.
"""
import math
from dataclasses import dataclass


@dataclass
class GovernorParams:
    stop_distance: float = 0.12     # m from the sensor: no forward motion closer than this
    slow_distance: float = 0.45     # m: full speed allowed beyond this
    min_speed: float = 0.03         # m/s still allowed just outside stop_distance
    max_age: float = 0.5            # s: older distances do not count
    max_speed: float = 0.2          # m/s allowed at slow_distance


def forward_limit(distance, p):
    """Maximum forward speed (m/s) for a measured distance; None = no limit."""
    if distance is None or not math.isfinite(distance):
        return 0.0
    if distance <= p.stop_distance:
        return 0.0
    if distance >= p.slow_distance:
        return None
    k = (distance - p.stop_distance) / (p.slow_distance - p.stop_distance)
    return p.min_speed + k * (p.max_speed - p.min_speed)


def govern(v, w, distance, age, p):
    """(v, w, limited) for a command, the last distance and its age (s)."""
    if v <= 0.0:
        return v, w, False
    if age is None or age > p.max_age:
        distance = None
    limit = forward_limit(distance, p)
    if limit is not None and v > limit:
        return limit, w, True
    return v, w, False
