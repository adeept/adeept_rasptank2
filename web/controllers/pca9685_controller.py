from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

SERVO_1 = 0
SERVO_2 = 1
SERVO_3 = 2
SERVO_4 = 3
SERVO_5 = 4
SERVO_6 = 5
SERVO_7 = 6
SERVO_8 = 7
MOTOR_4_POSITIVE_POLE =  8
MOTOR_4_NEGATIVE_POLE =  9
MOTOR_3_NEGATIVE_POLE =  10
MOTOR_3_POSITIVE_POLE =  11
MOTOR_2_POSITIVE_POLE =  12
MOTOR_2_NEGATIVE_POLE =  13
MOTOR_1_NEGATIVE_POLE =  14
MOTOR_1_POSITIVE_POLE =  15

FREQ = 50

class PCA9685Controller:

    def __init__(self):
        i2c = busio.I2C(SCL, SDA)
        # Create a simple PCA9685 class instance.
        #  pwm_motor.channels[7].duty_cycle = 0xFFFF
        self.pca_9685 = PCA9685(i2c, address=0x5f)  # default 0x40
        self.pca_9685.frequency = FREQ
        self.__channels = self.pca_9685.__channels

    def motor(self, motor_index):

        motor_instance = None
        match motor_index:
            case 1:
                motor_instance = motor.DCMotor(self.__channels[MOTOR_1_POSITIVE_POLE],
                                               self.__channels[MOTOR_1_NEGATIVE_POLE])
            case 2:
                motor_instance = motor.DCMotor(self.__channels[MOTOR_2_POSITIVE_POLE],
                                               self.__channels[MOTOR_2_NEGATIVE_POLE])
            case _:
                raise ValueError(f"Invalid motor index {motor_index}")

        motor_instance.decay_mode = motor.SLOW_DECAY
        return motor_instance

    def servo(self, servo_index):
        if 0 > servo_index > 7:
            raise ValueError(f"Invalid servos index {servo_index}")
        return servo.Servo(self.__channels[servo_index], min_pulse=500, max_pulse=2400,actuation_range=180)
