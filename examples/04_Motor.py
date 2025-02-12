#!/usr/bin/env/python3
import time
from board import SCL, SDA
import busio
#from adafruit_motor import servo,motor
#from adafruit_motor import motor
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

# motor_EN_A: Pin7  |  motor_EN_B: Pin11
# motor_A:  Pin8,Pin10    |  motor_B: Pin13,Pin12


MOTOR_M1_IN1 =  15      #Define the positive pole of M1
MOTOR_M1_IN2 =  14      #Define the negative pole of M1
MOTOR_M2_IN1 =  12      #Define the positive pole of M2
MOTOR_M2_IN2 =  13      #Define the negative pole of M2
MOTOR_M3_IN1 =  11      #Define the positive pole of M3
MOTOR_M3_IN2 =  10      #Define the negative pole of M3
MOTOR_M4_IN1 =  8       #Define the positive pole of M4
MOTOR_M4_IN2 =  9       #Define the negative pole of M4

Dir_forward   = 0
Dir_backward  = 1

left_forward  = 1
left_backward = 0

right_forward = 0
right_backward= 1

pwn_A = 0
pwm_B = 0
  
def map(x,in_min,in_max,out_min,out_max):
  return (x - in_min)/(in_max - in_min) *(out_max - out_min) +out_min


#def setup():
i2c = busio.I2C(SCL, SDA)
  #i2c = busio.I2C()
  # Create a simple PCA9685 class instance.
#  pwm_motor.channels[7].duty_cycle = 0xFFFF
pwm_motor = PCA9685(i2c, address=0x5f) #default 0x40
pwm_motor.frequency = 50
 # time.sleep(0.1)
#  pwm_motor.channels[7].duty_cycle = 0xFFFF

#  motor11 = motor.DCMotor(pca.channels[5], pca.channels[6])
motor1 = motor.DCMotor(pwm_motor.channels[MOTOR_M1_IN1],pwm_motor.channels[MOTOR_M1_IN2] )
motor1.decay_mode = (motor.SLOW_DECAY)
motor2 = motor.DCMotor(pwm_motor.channels[MOTOR_M2_IN1],pwm_motor.channels[MOTOR_M2_IN2] )
motor2.decay_mode = (motor.SLOW_DECAY)
motor3 = motor.DCMotor(pwm_motor.channels[MOTOR_M3_IN1],pwm_motor.channels[MOTOR_M3_IN2] )
motor3.decay_mode = (motor.SLOW_DECAY)
motor4 = motor.DCMotor(pwm_motor.channels[MOTOR_M4_IN1],pwm_motor.channels[MOTOR_M4_IN2] )
motor4.decay_mode = (motor.SLOW_DECAY)
#  motorStop()

def Motor(channel,direction,motor_speed):
  if motor_speed > 100:
    motor_speed = 100
  elif motor_speed < 0:
    motor_speed = 0
  speed = map(motor_speed, 0, 100, 0, 1.0)
  if direction == -1:
    speed = -speed

  if channel == 1:
    motor1.throttle = speed
    # print("1111")
  elif channel == 2:
    motor2.throttle = speed
    # print("2222")
  elif channel == 3:
    motor3.throttle = speed
    # print("3333")
  elif channel == 4:
    motor4.throttle = speed
    # print("4444")


def motorStop():#Motor stops
    motor1.throttle = 0
    motor2.throttle = 0
    motor3.throttle = 0
    motor4.throttle = 0

def destroy():
  motorStop()
  pwm_motor.deinit()


if __name__ == '__main__':
  try:
    chann =  1
    # while True:
     
    for i in range(10):
      speed_set = 50
      Motor(chann, -1, speed_set)
      Motor(2, -1, speed_set)
      # Motor(3, -1, speed_set)
      # Motor(4, -1, speed_set)

      print("Forward")
      time.sleep(2)
      Motor(chann, 1 ,speed_set)
      Motor(2, 1 ,speed_set)
      # Motor(3, 1 ,speed_set)
      # Motor(4, 1 ,speed_set)
      print("Backward")
      time.sleep(2)
    destroy()
  except KeyboardInterrupt:
    destroy()

