# File: tests/test_motors.py
import unittest
from unittest.mock import MagicMock

from web.controllers.motors import Movement


class SimpleMotor:
    def __init__(self):
        self.throttle = None

    def __repr__(self):
        return f"<SimpleMotor throttle={self.throttle}>"


class TestMovement(unittest.TestCase):
    def setUp(self):
        # construct controller that returns SimpleMotor instances
        self.m1 = SimpleMotor()
        self.m2 = SimpleMotor()
        controller = MagicMock()
        controller.motor = lambda idx: self.m1 if idx == 1 else self.m2
        # motor1_direction and motor2_direction simulate wiring/polarity
        self.movement = Movement(controller, motor1_direction=1, motor2_direction=1)

    def test_forward_sets_throttles_positive(self):
        self.movement.forward()
        self.assertEqual(100, self.m1.throttle)
        self.assertEqual(100, self.m2.throttle)

    def test_backward_sets_throttles_negative(self):
        self.movement.backward()
        self.assertEqual(-100, self.m1.throttle)
        self.assertEqual(-100, self.m2.throttle)

    def test_left_and_right(self):
        self.movement.left()
        self.assertEqual(100, self.m1.throttle)
        self.assertEqual(-100, self.m2.throttle)

        self.movement.right()
        self.assertEqual(-100, self.m1.throttle)
        self.assertEqual(100, self.m2.throttle)

    def test_stop_sets_zero(self):
        self.movement.stop()
        self.assertEqual(0, self.m1.throttle)
        self.assertEqual(0, self.m2.throttle)

    def test_speed_clamping(self):
        self.movement.speed(150)
        self.movement.forward()
        self.assertEqual(100, self.m1.throttle)
        self.movement.speed(-10)
        self.movement.forward()
        self.assertEqual(0, self.m1.throttle)


if __name__ == "__main__":
    unittest.main()
