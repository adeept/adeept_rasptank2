#!/usr/bin/env/python3
'''
 SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
# sudo pip3 install adafruit-circuitpython-motor
# sudo pip3 install adafruit-circuitpython-pca9685
'''
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)
# Create a simple PCA9685 class instance.
pca = PCA9685(i2c, address=0x5f) #default 0x40

pca.frequency = 50

# servo7 = servo.Servo(pca.channels[7], min_pulse=580, max_pulse=2350)
# servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2600)
# servo7 = servo.Servo(pca.channels[7], min_pulse=400, max_pulse=2400)
# servo7 = servo.Servo(pca.channels[7], min_pulse=600, max_pulse=2500)
# servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2400)
# The pulse range is 750 - 2250 by default. This range typically gives 135 degrees of
# range, but the default is to use 180 degrees. You can specify the expected range if you wish:
# servo7 = servo.Servo(pca.channels[7], actuation_range=135)
def set_angle(ID, angle):
    servo_angle = servo.Servo(pca.channels[ID], min_pulse=500, max_pulse=2400,actuation_range=180)
    servo_angle.angle = angle
'''
# You can also specify the movement fractionally.
fraction = 0.0
while fraction < 1.0:
    servo7.fraction = fraction
    fraction += 0.01
    time.sleep(0.03)
pca.deinit()
'''

def test(channel):
    for i in range(180): # The servo turns from 0 to 180 degrees.
        for channel in range(0,16) :
            set_angle(channel, i)
        # set_angle(channel, i)
        time.sleep(0.01)
    time.sleep(0.5)
    for i in range(180): # The servo turns from 180 to 0 degrees.
        for channel in range(0,16) :
            set_angle(channel, i)
        # set_angle(channel, 180-i)
        time.sleep(0.01)
    time.sleep(0.5)

if __name__ == "__main__":
    channel = 0
    while True:
        test(channel)

'''

'''

#servo7.angle = 90
