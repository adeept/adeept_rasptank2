"""Odometry calibration of the tracked base: the maths, without ROS.

The RaspTank has no encoders: base_node assumes how fast the tracks go for
a throttle. These tests find the systematic errors of that assumption:

  dead band   throttle where the tracks start to move     -> min_throttle
  straight    distance and sideways drift of a straight   -> max_track_speed,
              run                                            left_gain, right_gain
  rotation    angle really turned for a commanded turn     -> track_separation
                                                              (effective: tracks slip)

The measurements come from a tape measure (manual mode) or from the ArUco
marker seen by the camera (automatic mode); see rasptank_calibration.
Do them in this order (dead band, straight, rotation) and repeat the last
two: each one assumes the previous ones are done.
Random errors (slip that changes every time, other floors, battery) are
not corrected: repeat the tests and average, and calibrate on the floor
where the robot will be used.
"""
import math
from dataclasses import asdict, dataclass

PARAMS = ['max_track_speed', 'track_separation', 'min_throttle', 'left_gain', 'right_gain',
          'battery_reference']


@dataclass
class BaseCalibration:
    max_track_speed: float = 0.25
    track_separation: float = 0.12
    min_throttle: float = 0.0
    left_gain: float = 1.0
    right_gain: float = 1.0
    battery_reference: float = 0.0

    @classmethod
    def from_dict(cls, values):
        return cls(**{k: float(values[k]) for k in PARAMS if k in values})

    def to_dict(self):
        return asdict(self)

    def copy(self, **changes):
        values = self.to_dict()
        values.update(changes)
        return BaseCalibration(**values)


def clamp(value, low, high):
    return max(low, min(high, value))


def dead_band(throttle_when_moving, cal, margin=0.85):
    """min_throttle from the throttle at which the tracks started to move.

    A bit below (`margin`): the detection always comes late (the robot has
    to move a few millimetres before anyone, or the camera, notices it).
    """
    return cal.copy(min_throttle=round(clamp(throttle_when_moving * margin, 0.0, 0.6), 3))


def straight(commanded, measured, lateral, cal):
    """Correct speed scale and track balance from a straight run.

    commanded  distance base_node believed it drove (m)
    measured   real distance along the start direction (m)
    lateral    real sideways deviation at the end (m, + = left)

    A constant speed difference between the tracks makes an arc: the end
    deviates lateral = curvature * d^2 / 2, and curvature = (vr - vl) /
    (separation * v). So the right track is faster by f = 2 * lateral *
    separation / d^2 (relative to the mean), and the gains share the fix.
    """
    if commanded <= 0 or measured <= 0:
        raise ValueError('las distancias tienen que ser positivas')
    scale = measured / commanded
    f = 2.0 * lateral * cal.track_separation / (measured * measured)
    left = clamp(cal.left_gain * (1.0 + f / 2.0), 0.5, 2.0)
    right = clamp(cal.right_gain * (1.0 - f / 2.0), 0.5, 2.0)
    return cal.copy(max_track_speed=round(cal.max_track_speed * scale, 4),
                    left_gain=round(left, 4), right_gain=round(right, 4))


def rotation(commanded_deg, measured_deg, cal):
    """Effective track separation from a commanded and a real rotation.

    Turning in place, w = (vr - vl) / separation: if the robot turns less
    than it believes, it behaves as if the tracks were further apart.
    """
    if commanded_deg == 0 or measured_deg == 0 or (commanded_deg > 0) != (measured_deg > 0):
        raise ValueError(f'el giro medido ({measured_deg:.1f}°) tiene que ir en el mismo sentido '
                         f'que el pedido ({commanded_deg:.1f}°)')
    separation = cal.track_separation * commanded_deg / measured_deg
    return cal.copy(track_separation=round(clamp(separation, 0.05, 0.5), 4))


def battery(voltage, cal):
    """Remember the voltage of the calibration: base_node then scales the
    throttle by reference / current voltage (a discharged battery is slower)."""
    return cal.copy(battery_reference=round(voltage, 2) if voltage and voltage > 0 else 0.0)


def average(calibrations):
    """Field by field mean of several results of the same test."""
    n = len(calibrations)
    return BaseCalibration(**{k: round(sum(getattr(c, k) for c in calibrations) / n, 4)
                              for k in PARAMS})


# ------------------------------------------------------------ automatic mode

def relative_motion(before, after):
    """Robot motion (forward, left, yaw) between two views of a fixed marker.

    `before`/`after` are (x, y, yaw) of the marker in the robot frame
    (x forward, y left, yaw of the marker's normal). Returns where the robot
    ended, in the frame where it started.
    """
    x0, y0, a0 = before
    x1, y1, a1 = after
    theta = normalize(a0 - a1)                     # how much the robot turned
    c, s = math.cos(theta), math.sin(theta)
    # marker = start_pose + R(theta) @ marker_after  ->  solve for the end pose
    dx = x0 - (c * x1 - s * y1)
    dy = y0 - (s * x1 + c * y1)
    return dx, dy, theta


def arc_end(distance, curvature):
    """End pose (x, y, yaw) after driving `distance` along a constant arc."""
    theta = curvature * distance
    if abs(curvature) < 1e-9:
        return distance, 0.0, 0.0
    return math.sin(theta) / curvature, (1.0 - math.cos(theta)) / curvature, theta


def marker_after_arc(before, distance, curvature):
    """Where a fixed marker (x, y in the robot frame) appears after an arc."""
    ex, ey, th = arc_end(distance, curvature)
    dx, dy = before[0] - ex, before[1] - ey
    c, s = math.cos(-th), math.sin(-th)
    return c * dx - s * dy, s * dx + c * dy


def straight_from_marker(before, after):
    """Real (forward, left) of a straight run, from the marker POSITIONS only.

    The marker's orientation from solvePnP is too noisy (several degrees
    with a small marker); its position is precise. A constant imbalance
    between the tracks drives an arc, which has two unknowns (distance and
    curvature) that the two coordinates of the marker determine. Solved by
    Gauss-Newton from a straight-line guess.
    """
    d = before[0] - after[0]
    k = 0.0
    for _ in range(30):
        px, py = marker_after_arc(before, d, k)
        rx, ry = after[0] - px, after[1] - py
        h = 1e-6
        ax, ay = marker_after_arc(before, d + h, k)
        bx, by = marker_after_arc(before, d, k + h)
        j11, j21 = (ax - px) / h, (ay - py) / h
        j12, j22 = (bx - px) / h, (by - py) / h
        det = j11 * j22 - j12 * j21
        if abs(det) < 1e-12:
            break
        d += (j22 * rx - j12 * ry) / det
        k += (-j21 * rx + j11 * ry) / det
        if abs(rx) + abs(ry) < 1e-9:
            break
    ex, ey, _ = arc_end(d, k)
    return ex, ey


def turned_from_bearings(commanded_deg, bearing_before, bearing_after):
    """Real rotation (deg) of an in-place turn, from the marker bearing.

    The bearing (atan2(y, x) of the marker, deg) is far more precise than
    the marker's orientation. Turning left by A moves the marker right by A,
    so the real turn is the commanded one plus the bearing change, wrapped
    to the nearest multiple of 360 of the commanded.
    """
    error = normalize_deg(bearing_before - bearing_after - commanded_deg)
    return commanded_deg + error


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def normalize_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0
