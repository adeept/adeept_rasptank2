import math

import pytest

from rasptank_core.kinematics import (CommandWatchdog, Odometry, limit_twist, tracks_to_throttle,
                                      tracks_to_twist, twist_to_tracks)


def test_tracks_round_trip():
    left, right = twist_to_tracks(0.1, 0.5, 0.12)
    assert left < right   # turning left: the right track goes faster
    assert tracks_to_twist(left, right, 0.12) == pytest.approx((0.1, 0.5))


def test_throttle_keeps_curvature_when_saturated():
    tl, tr = tracks_to_throttle(0.2, 0.4, max_track_speed=0.2)
    assert tr == pytest.approx(1.0)
    assert tl == pytest.approx(0.5)


def test_throttle_dead_band():
    tl, tr = tracks_to_throttle(0.01, -0.01, max_track_speed=0.2, min_throttle=0.3)
    assert tl > 0.3 and tr < -0.3
    assert tracks_to_throttle(0.0, 0.0, 0.2, 0.3) == (0.0, 0.0)


def test_limit_twist():
    assert limit_twist(1.0, -5.0, 0.2, 1.0) == (0.2, -1.0)


def test_odometry_straight_and_circle():
    odo = Odometry()
    for _ in range(100):
        odo.update(0.1, 0.0, 0.1)
    assert odo.pose.x == pytest.approx(1.0)
    odo = Odometry()
    # full circle of radius 0.2 comes back to the start
    w = 0.5
    for _ in range(round(2 * math.pi / w / 0.01)):
        odo.update(0.2 * w, w, 0.01)
    assert odo.pose.x == pytest.approx(0.0, abs=2e-3)
    assert odo.pose.y == pytest.approx(0.0, abs=2e-3)


def test_watchdog():
    wd = CommandWatchdog(0.5)
    assert wd.expired(0.0)
    wd.feed(1.0)
    assert not wd.expired(1.4)
    assert wd.expired(1.6)
