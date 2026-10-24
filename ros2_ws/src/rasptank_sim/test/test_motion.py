import math

import pytest

from rasptank_core.kinematics import Odometry
from rasptank_sim.motion import MotorErrors, TrueMotion, track_speed


def drive(motion, odo, t_left, t_right, seconds, max_speed=0.25, sep=0.12):
    for _ in range(int(seconds / 0.02)):
        motion.step(t_left, t_right, 0.02)
        odo.update((t_left + t_right) / 2 * max_speed, (t_right - t_left) * max_speed / sep, 0.02)


def test_ideal_robot_matches_odometry():
    motion, odo = TrueMotion(), Odometry()
    drive(motion, odo, 0.4, 0.6, 3.0)
    assert motion.pose.x == pytest.approx(odo.pose.x, abs=1e-3)
    assert motion.pose.y == pytest.approx(odo.pose.y, abs=1e-3)
    assert motion.pose.yaw == pytest.approx(odo.pose.yaw, abs=1e-3)


def test_dead_band_and_scale():
    e = MotorErrors(dead_band=0.2, left_scale=0.9)
    assert track_speed(0.15, 1.0, e) == 0.0
    assert track_speed(1.0, 0.9, e) == pytest.approx(0.25 * 0.9)
    assert track_speed(-0.6, 1.0, e) == pytest.approx(-0.25 * 0.5)


def test_errors_make_the_robot_drift_and_underturn():
    motion = TrueMotion(MotorErrors(left_scale=0.9))
    drive(motion, Odometry(), 0.4, 0.4, 4.0)
    assert motion.pose.y > 0.01                     # slower left track: drifts left
    motion = TrueMotion(MotorErrors(turn_slip=1.25))
    odo = Odometry()
    drive(motion, odo, -0.4, 0.4, 1.0)
    assert motion.pose.yaw == pytest.approx(odo.pose.yaw / 1.25, abs=1e-3)


def test_from_config_ignores_unknown_keys():
    e = MotorErrors.from_config({'left_scale': 0.9, 'whatever': 3})
    assert e.left_scale == 0.9 and math.isclose(e.right_scale, 1.0)
