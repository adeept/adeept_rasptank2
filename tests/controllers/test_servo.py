# python
# File: tests/test_servo.py
import unittest
from unittest.mock import MagicMock

from tests import async_helper
from web.controllers import servo as servo_module
from web.controllers.servo import ANTICLOCKWISE


class FakeServo:
    def __init__(self):
        self.angle = None


class TestServoCtrlThread(unittest.TestCase):
    def setUp(self):

        # prepare fake controller with servo(channel) factory
        self.fake_servo = FakeServo()
        controller = MagicMock()
        controller.servo = lambda ch: self.fake_servo

        # instantiate while thread start is patched
        self.svc = servo_module.ServoCtrlThread("test", controller, 0, 90, 1)

    def tearDown(self):
        try:
            if self.svc.is_alive():
                self.svc.stop_thread()
        except Exception:
            pass

    def test_initial_angle_set(self):
        self.svc.reset()
        self.assertEqual(90, int(round(self.svc.angle_current_value)))
        self.assertEqual(90, self.fake_servo.angle)

    def test_reset(self):
        self.assertEqual(90, self.svc.angle_initial_value)
        self.assertEqual(90, int(round(self.svc.angle_current_value)))
        self.assertEqual(90, self.fake_servo.angle)

    def test_set_angle_within_range(self):
        # call the name-mangled private method
        self.svc.move_to(120)
        async_helper.wait_for(lambda: self.assert_angle_reached(120))

    def test_set_angle_clips_to_max(self):
        # set beyond maximum -> clipped to angle_maximum_range (180)
        self.svc.move_to(200)
        async_helper.wait_for(lambda: self.assert_angle_reached(self.svc.angle_maximum_range))

    def test_set_angle_stops(self):
        self.svc.move_to(180)
        self.svc.stop()
        async_helper.wait_for(lambda: self.assert_angle_not_reached(180))

    def test_set_angle_clips_to_min(self):
        self.svc.move_to(-15)
        async_helper.wait_for(lambda: self.assert_angle_reached(self.svc.angle_minimum_range))

    def test_increment_and_decrement_pwm(self):
        # start from 90 by default
        self.svc.incrementPwm()
        self.assertEqual(91, self.fake_servo.angle)
        self.svc.incrementPwm()
        self.assertEqual(92, self.fake_servo.angle)
        self.svc.derementPwm()
        self.assertEqual(91, self.fake_servo.angle)

    def test_clockwise(self):
        self.svc.clockwise()
        async_helper.wait_for(lambda: self.assert_angle_reached(self.svc.angle_maximum_range))

    def test_anticlockwise(self):
        self.svc.anticlockwise()
        async_helper.wait_for(lambda: self.assert_angle_reached(self.svc.angle_minimum_range))

    def test_move_by_number_of_steps(self):
        expected_angle = self.svc.angle_current_value + 10
        self.svc.move(number_of_steps=10)
        async_helper.wait_for(lambda: self.assert_angle_reached(expected_angle))

    def test_clockwise_reversed_direction(self):
        self.svc.servo_direction=ANTICLOCKWISE
        self.svc.clockwise()
        async_helper.wait_for(lambda: self.assert_angle_reached(self.svc.angle_minimum_range))

    def test_anticlockwise_reversed_direction(self):
        # set beyond maximum -> clipped to angle_maximum_range (180)
        self.svc.servo_direction=ANTICLOCKWISE
        self.svc.anticlockwise()
        async_helper.wait_for(lambda: self.assert_angle_reached(self.svc.angle_maximum_range))

    def assert_angle_reached(self, expected):
        try:
            self.assertEqual(False, self.svc._ServoCtrlThread__flag.is_set())
            self.assertEqual(expected, self.svc.angle_current_value)
            self.assertEqual(expected, self.fake_servo.angle)
            return True
        except AssertionError:
            return False

    def assert_angle_not_reached(self, notExpected):
        try:
            self.assertEqual(False, self.svc._ServoCtrlThread__flag.is_set())
            self.assertNotEqual(notExpected, self.svc.angle_current_value)
            self.assertNotEqual(notExpected, self.fake_servo.angle)
            return True
        except AssertionError:
            return False

if __name__ == "__main__":
    unittest.main()
