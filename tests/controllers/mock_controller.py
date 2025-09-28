class MockMotor:
    def __init__(self):
        self.throttle = 0

    def __repr__(self):
        return f"<MockMotor throttle={self.throttle}>"

class MockServo:
    def __init__(self):
        self.angle = None

    def __repr__(self):
        return f"<MockServo angle={self.angle}>"

class MockController:
    def __init__(self, motors=None, servos=None, dc_motors=None):
        self._motors = dict(motors or {})
        self._servos = dict(servos or {})
        self._dc_motors = dict(dc_motors or {})

    def motor(self, idx):
        return self._motors.setdefault(idx, MockMotor())

    def servo(self, channel):
        return self._servos.setdefault(channel, MockServo())
