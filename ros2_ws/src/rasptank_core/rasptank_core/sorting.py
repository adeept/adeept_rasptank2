"""Target selection for the sorting mission: several cubes, one goal per class.

The pick-and-place state machine (mission.py) handles ONE cube and ONE goal
at a time; this decides which ones:

  * while searching, the nearest cube of any class that is not already on
    a goal (closer than `exclusion` to a goal seen before);
  * from the approach on, it LOCKS that cube and follows it (same class,
    within `track_radius` of where it was), so it does not switch to another
    cube halfway, nor mistake another one for the cube it has just picked up;
  * the goal is the ArUco marker ("meta_<id>") of the locked cube's class.

Positions are kept in the odometry frame, so they survive the robot moving
between detections. Pure Python, no ROS.
"""
import math
from dataclasses import dataclass

from .mission import Target


@dataclass
class Seen:
    cls: str
    x: float          # odometry frame
    y: float
    stamp: float


def to_odom(pose, x, y):
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return pose.x + c * x - s * y, pose.y + s * x + c * y


def to_base(pose, x, y):
    dx, dy = x - pose.x, y - pose.y
    c, s = math.cos(-pose.yaw), math.sin(-pose.yaw)
    return c * dx - s * dy, s * dx + c * dy


class Sorter:
    def __init__(self, goals, exclusion=0.25, track_radius=0.15, max_age=0.7):
        """`goals`: {cube class: ArUco marker id}."""
        self.goals = dict(goals)
        self.exclusion = exclusion
        self.track_radius = track_radius
        self.max_age = max_age
        self.cubes = []               # cubes in the last detection message
        self.goal_seen = {}           # marker id -> Seen (last time seen)
        self.locked = None            # Seen: the cube being picked up
        self.last = None              # the cube target returned last
        self.delivered = []           # classes delivered, in order

    def observe(self, detections, pose, now):
        """Detections of one image: (class, x, y) in the robot frame."""
        cubes = []
        for cls, x, y in detections:
            ox, oy = to_odom(pose, x, y)
            if cls in self.goals:
                cubes.append(Seen(cls, ox, oy, now))
            elif cls.startswith('meta_'):
                try:
                    self.goal_seen[int(cls[5:])] = Seen(cls, ox, oy, now)
                except ValueError:
                    pass
        self.cubes = cubes

    def _on_a_goal(self, c):
        return any(math.hypot(c.x - g.x, c.y - g.y) < self.exclusion
                   for g in self.goal_seen.values())

    def cube_target(self, pose, now):
        fresh = [c for c in self.cubes if now - c.stamp <= self.max_age]
        if self.locked is not None:
            fresh = [c for c in fresh if c.cls == self.locked.cls
                     and math.hypot(c.x - self.locked.x, c.y - self.locked.y) < self.track_radius]
        else:
            fresh = [c for c in fresh if not self._on_a_goal(c)]
        if not fresh:
            return None
        best = min(fresh, key=lambda c: math.hypot(c.x - pose.x, c.y - pose.y))
        self.last = best
        if self.locked is not None:
            self.locked = best                    # follow it as it is approached
        x, y = to_base(pose, best.x, best.y)
        return Target(x, y, best.stamp)

    def goal_target(self, pose, now):
        if self.locked is None:
            return None
        g = self.goal_seen.get(self.goals[self.locked.cls])
        if g is None or now - g.stamp > self.max_age:
            return None
        x, y = to_base(pose, g.x, g.y)
        return Target(x, y, g.stamp)

    def lock(self):
        """Lock on the cube returned last (the mission starts approaching it)."""
        if self.last is not None:
            self.locked = self.last

    def unlock(self):
        self.locked = None

    @property
    def locked_class(self):
        return None if self.locked is None else self.locked.cls

    def deliver(self):
        if self.locked is not None:
            self.delivered.append(self.locked.cls)
        self.locked = None
