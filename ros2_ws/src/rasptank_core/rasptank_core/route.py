"""Follow a route of (x, y) waypoints in the odometry frame.

Turn towards the next waypoint, drive to it while correcting the heading,
and move on when it is closer than `tolerance`.
"""
import math
from dataclasses import dataclass

from .kinematics import clamp, normalize_angle


@dataclass
class RouteParams:
    tolerance: float = 0.05        # m
    max_linear: float = 0.15       # m/s
    max_angular: float = 1.0       # rad/s
    k_linear: float = 0.8
    k_angular: float = 2.0
    turn_in_place: float = 0.5     # rad, larger errors: turn without advancing


class RouteFollower:
    def __init__(self, params=None):
        self.p = params or RouteParams()
        self.waypoints = []
        self.index = 0

    def set_route(self, waypoints):
        self.waypoints = [tuple(w) for w in waypoints]
        self.index = 0

    def add(self, x, y):
        self.waypoints.append((x, y))

    def clear(self):
        self.waypoints = []
        self.index = 0

    @property
    def done(self):
        return self.index >= len(self.waypoints)

    def step(self, pose):
        """Twist (v, w) towards the current waypoint for the pose."""
        while not self.done:
            gx, gy = self.waypoints[self.index]
            dist = math.hypot(gx - pose.x, gy - pose.y)
            if dist > self.p.tolerance:
                break
            self.index += 1
        if self.done:
            return 0.0, 0.0
        heading = math.atan2(gy - pose.y, gx - pose.x)
        err = normalize_angle(heading - pose.yaw)
        w = clamp(self.p.k_angular * err, -self.p.max_angular, self.p.max_angular)
        if abs(err) > self.p.turn_in_place:
            return 0.0, w
        v = clamp(self.p.k_linear * dist, 0.0, self.p.max_linear) * math.cos(err)
        return v, w
