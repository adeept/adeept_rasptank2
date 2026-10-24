"""Differential (tracked) drive kinematics and open-loop odometry.

The RaspTank has no wheel encoders, so the odometry integrates the
*commanded* velocity. On the real robot it drifts (the tracks slip); in the
simulator it is the ground truth.
"""
import math
from dataclasses import dataclass


def clamp(value, low, high):
    return max(low, min(high, value))


def limit_twist(v, w, max_v, max_w):
    """Clamp a twist to the robot's limits."""
    return clamp(v, -max_v, max_v), clamp(w, -max_w, max_w)


def twist_to_tracks(v, w, separation):
    """Linear speed of the left and right tracks (m/s) for a twist."""
    half = separation / 2.0
    return v - w * half, v + w * half


def tracks_to_twist(v_left, v_right, separation):
    return (v_left + v_right) / 2.0, (v_right - v_left) / separation


def saturate_tracks(v_left, v_right, max_track_speed):
    """Scale both track speeds by the same factor if one is too fast.

    The robot then keeps the curvature it was asked for.
    """
    peak = max(abs(v_left), abs(v_right))
    if peak > max_track_speed:
        k = max_track_speed / peak
        return v_left * k, v_right * k
    return v_left, v_right


def tracks_to_throttle(v_left, v_right, max_track_speed, min_throttle=0.0):
    """Motor throttles in [-1, 1] for the track speeds (saturated first).

    `min_throttle` compensates the dead band of the DC motors: any non-zero
    throttle is raised to at least that value.
    """
    v_left, v_right = saturate_tracks(v_left, v_right, max_track_speed)
    t_left = v_left / max_track_speed
    t_right = v_right / max_track_speed

    def dead_band(t):
        if t == 0.0 or min_throttle <= 0.0:
            return t
        return math.copysign(min_throttle + (1.0 - min_throttle) * abs(t), t)

    return dead_band(t_left), dead_band(t_right)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class Odometry:
    """Integrates a twist (v, w) into a 2D pose."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.pose = Pose2D(x, y, yaw)
        self.v = 0.0
        self.w = 0.0

    def update(self, v, w, dt):
        p = self.pose
        if abs(w) < 1e-9:
            p.x += v * dt * math.cos(p.yaw)
            p.y += v * dt * math.sin(p.yaw)
        else:
            # exact integration along the arc
            r = v / w
            yaw_new = p.yaw + w * dt
            p.x += r * (math.sin(yaw_new) - math.sin(p.yaw))
            p.y -= r * (math.cos(yaw_new) - math.cos(p.yaw))
            p.yaw = yaw_new
        p.yaw = normalize_angle(p.yaw)
        self.v, self.w = v, w
        return p


class CommandWatchdog:
    """Stops the robot when commands stop arriving.

    `feed(now)` on each command, `expired(now)` in the control loop.
    """

    def __init__(self, timeout):
        self.timeout = timeout
        self.last = None

    def feed(self, now):
        self.last = now

    def expired(self, now):
        return self.last is None or (now - self.last) > self.timeout
