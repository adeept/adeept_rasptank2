"""The calibration procedures against a fake robot with hidden errors."""
# FakeSession is also used by test_flow.py
import math

import pytest

from rasptank_calibration import procedure as proc
from rasptank_calibration.session import Aborted, CalibrationError
from rasptank_core.calibration import BaseCalibration
from rasptank_core.kinematics import Pose2D, normalize_angle, tracks_to_throttle, twist_to_tracks

MARKER = (1.0, 0.0)          # world position; it faces the robot


class FakeSession:
    """Session API (params/apply/move/ramp/marker) over a simulated robot."""

    def __init__(self, speed=0.22, left=0.92, right=1.0, dead=0.12, slip=1.3):
        self.cal = BaseCalibration()
        self.truth = Pose2D()
        self.e = dict(speed=speed, left=left, right=right, dead=dead, slip=slip)
        self.voltage = 8.1
        self.stopped = False

    def params(self):
        return self.cal

    def apply(self, cal):
        self.cal = cal

    def _track(self, t, scale):
        e = self.e
        if abs(t) <= e['dead']:
            return 0.0
        return math.copysign((abs(t) - e['dead']) / (1 - e['dead']) * e['speed'] * scale, t)

    def _drive(self, v, w, seconds):
        c = self.cal
        left, right = twist_to_tracks(v, w, c.track_separation)
        tl, tr = tracks_to_throttle(left * c.left_gain, right * c.right_gain, c.max_track_speed,
                                    c.min_throttle)
        vl, vr = self._track(tl, self.e['left']), self._track(tr, self.e['right'])
        p = self.truth
        for _ in range(int(round(seconds / 0.01))):
            vv, ww = (vl + vr) / 2, (vr - vl) / (0.12 * self.e['slip'])
            p.x += vv * 0.01 * math.cos(p.yaw)
            p.y += vv * 0.01 * math.sin(p.yaw)
            p.yaw += ww * 0.01          # unwrapped, like Session.truth

    def move(self, v, w, seconds):
        if self.stopped:
            raise Aborted('parada de emergencia activa')
        self._drive(v, w, seconds)
        return v * seconds, math.degrees(w * seconds)

    def ramp(self, until, rate=0.015, limit=0.5):
        original = self.cal
        self.cal = original.copy(min_throttle=0.0)
        throttle = 0.0
        start = (self.truth.x, self.truth.y)
        while throttle < limit and not until():
            throttle += rate * 0.05
            self._drive(throttle * original.max_track_speed, 0.0, 0.05)
            self.moved = math.hypot(self.truth.x - start[0], self.truth.y - start[1])
        self.cal = original
        return throttle

    def marker(self, samples=12, timeout=4.0):
        p = self.truth
        dx, dy = MARKER[0] - p.x, MARKER[1] - p.y
        c, s = math.cos(-p.yaw), math.sin(-p.yaw)
        x, y = c * dx - s * dy, s * dx + c * dy
        if x <= 0.1 or abs(math.atan2(y, x)) > math.radians(30):
            return None
        return x, y, normalize_angle(math.pi - p.yaw)

    def log(self, text):
        pass

    def latest_marker(self, max_age=0.5):
        return self.marker()


def calibrate_auto(s, rounds=2):
    proc.dead_band_auto(s)
    for _ in range(rounds):
        proc.straight_auto(s)
        proc.rotation_auto(s)
    return s.cal


def test_auto_finds_the_hidden_errors():
    s = calibrate_auto(FakeSession())
    cal = s
    assert cal.min_throttle == pytest.approx(0.12, abs=0.02)
    assert cal.track_separation == pytest.approx(0.12 * 1.3, rel=0.05)
    assert cal.left_gain / cal.right_gain == pytest.approx(1 / 0.92, rel=0.02)
    s2 = FakeSession()
    s2.cal = cal
    distance, turn, report = proc.verify_auto(s2)
    assert abs(distance) < 0.02 and abs(turn) < 3.0, report


def test_manual_with_exact_measurements():
    s = FakeSession()
    proc.dead_band_manual(s, lambda: getattr(s, 'moved', 0) > 0.005)
    for _ in range(2):
        start = (s.truth.x, s.truth.y, s.truth.yaw)
        commanded = proc.straight_run_manual(s)
        dx, dy = s.truth.x - start[0], s.truth.y - start[1]
        c, sn = math.cos(-start[2]), math.sin(-start[2])
        proc.straight_manual(s, commanded, c * dx - sn * dy, sn * dx + c * dy)
        s.truth = Pose2D()
        commanded = proc.rotation_run_manual(s)
        turned = math.degrees(s.truth.yaw) % 360.0
        error = turned - 360.0 if turned > 180 else turned
        proc.rotation_manual(s, commanded, error)
        s.truth = Pose2D()
    assert s.cal.track_separation == pytest.approx(0.156, rel=0.05)
    proc.verify_run_manual(s)
    assert math.hypot(s.truth.x, s.truth.y) < 0.05


def test_lost_marker_and_emergency_stop_abort():
    s = FakeSession()
    s.truth.yaw = math.radians(90)            # looking away from the marker
    with pytest.raises(CalibrationError):
        proc.straight_auto(s)
    s = FakeSession()
    s.stopped = True
    with pytest.raises(Aborted):
        proc.straight_run_manual(s)


def test_battery_reference():
    s = FakeSession()
    assert proc.battery(s, True).calibration.battery_reference == pytest.approx(8.1)
    assert proc.battery(s, False).calibration.battery_reference == 0.0
