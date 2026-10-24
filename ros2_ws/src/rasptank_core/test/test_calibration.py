import math

import pytest

from rasptank_core.calibration import (BaseCalibration, arc_end, battery, dead_band,
                                       marker_after_arc, relative_motion, rotation, straight,
                                       straight_from_marker, turned_from_bearings)
from rasptank_core.kinematics import Odometry, tracks_to_throttle, twist_to_tracks

CAL = BaseCalibration()


class Robot:
    """A tracked robot with systematic errors, driven like base_node does."""

    def __init__(self, speed=0.22, left=0.92, right=1.0, dead=0.12, slip=1.3, sep=0.12):
        self.speed, self.left, self.right, self.dead, self.slip, self.sep = \
            speed, left, right, dead, slip, sep

    def track(self, t, scale):
        if abs(t) <= self.dead:
            return 0.0
        return math.copysign((abs(t) - self.dead) / (1 - self.dead) * self.speed * scale, t)

    def drive(self, cal, v, w, seconds):
        """True (x, y, yaw) after a command, starting at the origin."""
        pose = Odometry()
        left, right = twist_to_tracks(v, w, cal.track_separation)
        tl, tr = tracks_to_throttle(left * cal.left_gain, right * cal.right_gain,
                                    cal.max_track_speed, cal.min_throttle)
        vl = self.track(tl, self.left)
        vr = self.track(tr, self.right)
        for _ in range(int(seconds / 0.01)):
            pose.update((vl + vr) / 2, (vr - vl) / (self.sep * self.slip), 0.01)
        return pose.pose


def calibrate(robot, rounds=3):
    cal = dead_band(robot.dead / 0.85, CAL)            # detected exactly at the dead band
    for _ in range(rounds):
        p = robot.drive(cal, 0.1, 0.0, 10.0)         # commanded 1 m
        cal = straight(1.0, p.x, p.y, cal)
        p = robot.drive(cal, 0.0, 1.0, 2 * math.pi)  # commanded 360 deg
        cal = rotation(360.0, 360.0 + math.degrees(p.yaw), cal)
    return cal


def test_calibration_recovers_the_errors():
    robot = Robot()
    before = robot.drive(CAL, 0.1, 0.0, 10.0)
    cal = calibrate(robot)
    after = robot.drive(cal, 0.1, 0.0, 10.0)
    assert abs(after.x - 1.0) < 0.01 < abs(before.x - 1.0)
    assert abs(after.y) < 0.01 < abs(before.y)
    turn = robot.drive(cal, 0.0, 1.0, math.pi / 2)
    assert math.degrees(turn.yaw) == pytest.approx(90.0, abs=1.5)
    assert cal.track_separation == pytest.approx(0.12 * 1.3, rel=0.03)
    assert cal.left_gain > 1.0 > cal.right_gain


def test_ideal_robot_needs_nothing():
    cal = calibrate(Robot(speed=0.25, left=1.0, dead=0.0, slip=1.0), rounds=1)
    assert cal.max_track_speed == pytest.approx(0.25, rel=1e-3)
    assert cal.left_gain == pytest.approx(1.0, abs=1e-3)
    assert cal.track_separation == pytest.approx(0.12, rel=1e-3)


def test_input_errors():
    with pytest.raises(ValueError):
        straight(1.0, 0.0, 0.0, CAL)
    with pytest.raises(ValueError):
        rotation(360.0, -300.0, CAL)
    assert battery(7.9, CAL).battery_reference == 7.9
    assert battery(None, CAL).battery_reference == 0.0


def test_relative_motion_from_marker():
    # marker 1 m ahead facing the robot (normal yaw pi); the robot drives
    # 0.3 m forward, 0.05 m to the left and turns 10 deg
    th = math.radians(10)
    end = (0.3, 0.05)
    mx, my = 1.0 - end[0], 0.0 - end[1]
    after = (math.cos(-th) * mx - math.sin(-th) * my, math.sin(-th) * mx + math.cos(-th) * my,
             math.pi - th)
    dx, dy, dth = relative_motion((1.0, 0.0, math.pi), after)
    assert (dx, dy, dth) == pytest.approx((0.3, 0.05, th), abs=1e-9)


def test_turn_from_bearings():
    # commanded 360, real 340: the marker that was dead ahead ends 20 deg to the left
    assert turned_from_bearings(360.0, 0.0, 20.0) == pytest.approx(340.0)
    assert turned_from_bearings(25.0, 0.0, -20.0) == pytest.approx(20.0)
    assert turned_from_bearings(-25.0, 3.0, 25.0) == pytest.approx(-22.0)


def test_reversing_drifts_to_the_same_side():
    # procedure.straight_auto relies on this to use the return leg as is
    r = Robot(dead=0.0, slip=1.0, speed=0.25)
    fwd, back = r.drive(CAL, 0.1, 0.0, 3.0), r.drive(CAL, -0.1, 0.0, 3.0)
    assert fwd.y > 0 and back.y == pytest.approx(fwd.y)


@pytest.mark.parametrize('distance,curvature', [(0.3, 0.0), (0.3, 0.6), (-0.3, 0.6), (0.3, -1.2)])
def test_straight_from_marker_positions(distance, curvature):
    before = (1.0, 0.05)
    after = marker_after_arc(before, distance, curvature)
    ex, ey, _ = arc_end(distance, curvature)
    assert straight_from_marker(before, after) == pytest.approx((ex, ey), abs=1e-6)
