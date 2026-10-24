"""True motion of the simulated tracks, with the errors of a real robot.

base_node integrates what it ASKS for (its odometry); the simulator moves
the robot with what the motors really DO with the throttle it sends
(/base/throttle). The difference is the odometry error that the
calibration (rasptank_calibration) has to find and correct.

World YAML, all optional (the defaults are an ideal robot):

    robot_errors:
      max_track_speed: 0.25   # m/s of a track at throttle 1 (nominal)
      left_scale: 0.93        # this track is 7 % slower
      right_scale: 1.0
      dead_band: 0.15         # throttle below this does not move the track
      track_separation: 0.12  # geometric
      turn_slip: 1.25         # the robot turns as if the tracks were 25 % further apart
      noise: 0.03             # random relative error of each track speed
"""
import math
import random
from dataclasses import dataclass

from rasptank_core.kinematics import Pose2D, normalize_angle


@dataclass
class MotorErrors:
    max_track_speed: float = 0.25
    left_scale: float = 1.0
    right_scale: float = 1.0
    dead_band: float = 0.0
    track_separation: float = 0.12
    turn_slip: float = 1.0
    noise: float = 0.0

    @classmethod
    def from_config(cls, config):
        fields = cls.__dataclass_fields__
        return cls(**{k: float(v) for k, v in (config or {}).items() if k in fields})


def track_speed(throttle, scale, e, rng=random):
    """Real speed (m/s) of a track for a throttle in [-1, 1]."""
    magnitude = abs(throttle)
    if magnitude <= e.dead_band:
        return 0.0
    effective = (magnitude - e.dead_band) / (1.0 - e.dead_band)
    speed = effective * e.max_track_speed * scale
    if e.noise > 0.0:
        speed *= max(0.0, 1.0 + rng.gauss(0.0, e.noise))
    return math.copysign(speed, throttle)


class TrueMotion:
    def __init__(self, errors=None, seed=None):
        self.e = errors or MotorErrors()
        self.pose = Pose2D()
        self.rng = random.Random(seed)

    def step(self, t_left, t_right, dt):
        e = self.e
        v_l = track_speed(t_left, e.left_scale, e, self.rng)
        v_r = track_speed(t_right, e.right_scale, e, self.rng)
        v = (v_l + v_r) / 2.0
        w = (v_r - v_l) / (e.track_separation * e.turn_slip)
        p = self.pose
        p.x += v * dt * math.cos(p.yaw + w * dt / 2.0)
        p.y += v * dt * math.sin(p.yaw + w * dt / 2.0)
        p.yaw = normalize_angle(p.yaw + w * dt)
        return p
