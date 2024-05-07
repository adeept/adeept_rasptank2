#!/usr/bin/env python3
# File name   : functions.py
# Description : Control Functions
# Author	  : Devin
# Date		: 2024/03/10
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

import threading
# from mpu6050 import mpu6050
import os
import json
import ultra
import Kalman_filter
import move
# import speech
import RPIservo
from gpiozero import InputDevice

scGear = RPIservo.ServoCtrl()
# scGear.setup()
scGear.start()
# i2c = busio.I2C(SCL, SDA)
# # Create a simple PCA9685 class instance.
# pwm_fuc_servo = PCA9685(i2c, address=0x5f) #default 0x40
# pwm_fuc_servo.frequency = 50


move.setup()
kalman_filter_X =  Kalman_filter.Kalman_filter(0.01,0.1)

# MPU_connection = 1
# try:
# 	sensor = mpu6050(0x68)
# 	print('mpu6050 connected, PT MODE ON')
# except:
# 	MPU_connection = 0
# 	print('mpu6050 disconnected, ARM MODE ON')

curpath = os.path.realpath(__file__)
thisPath = "/" + os.path.dirname(curpath)

def num_import_int(initial):        #Call this function to import data from '.txt' file
	global r
	with open(thisPath+"/RPIservo.py") as f:
		for line in f.readlines():
			if(line.find(initial) == 0):
				r=line
	begin=len(list(initial))
	snum=r[begin:]
	n=int(snum)
	return n

pwm0_direction = 1
pwm0_init = num_import_int('init_pwm0 = ')
pwm0_max  = 180
pwm0_min  = 0
pwm0_pos  = pwm0_init

pwm1_direction = 1
pwm1_init = num_import_int('init_pwm1 = ')
pwm1_max  = 180
pwm1_min  = 0
pwm1_pos  = pwm1_init

pwm2_direction = 1
pwm2_init = num_import_int('init_pwm2 = ')
pwm2_max  = 180
pwm2_min  = 0
pwm2_pos  = pwm2_init

line_pin_left = 22
line_pin_middle = 27
line_pin_right = 17




# def setup():
# 	global track_line_left, track_line_middle,track_line_right
# 	track_line_left = InputDevice(pin=line_pin_right)
# 	track_line_middle = InputDevice(pin=line_pin_middle)
# 	track_line_right = InputDevice(pin=line_pin_left)


class Functions(threading.Thread):
	def __init__(self, *args, **kwargs):
		self.functionMode = 'none'
		self.steadyGoal = 0

		self.scanNum = 3
		self.scanList = [0,0,0]
		self.scanPos = 1
		self.scanDir = 1
		self.rangeKeep = 0.7
		self.scanRange = 100
		self.scanServo = 1
		self.turnServo = 2
		self.turnWiggle = 200

		# setup()

		super(Functions, self).__init__(*args, **kwargs)
		self.__flag = threading.Event()
		self.__flag.clear()

	# def set_angle(self, ID, angle):
	# 	servo_angle = servo.Servo(pwm_fuc_servo.channels[ID], min_pulse=500, max_pulse=2400,actuation_range=180)
	# 	# print(angle)
	# 	servo_angle.angle = angle

	def pwmGenOut(self, angleInput):
		# return int(round(23/9*angleInput))
		return int(angleInput)

	def setup(self):
		global track_line_left, track_line_middle,track_line_right
		track_line_left = InputDevice(pin=line_pin_right)
		track_line_middle = InputDevice(pin=line_pin_middle)
		track_line_right = InputDevice(pin=line_pin_left)

	def radarScan(self):
		# global pwm0_pos,pwm0_max
		pwm0_min = -90
		pwm0_max =  90

		scan_speed = 1
		result = []

		pwm0_pos = pwm0_max
		# pwm0_pos = 90
		# print(pwm0_pos)
		# self.set_angle(1, pwm0_pos-1)
		# scGear.moveAngle(1, pwm0_pos)
		scGear.moveAngle(1, 0)
		scGear.moveAngle(0, 0)
		# print("4444")
		time.sleep(0.8)
		# time.sleep(1)

		while pwm0_pos>pwm0_min:
			pwm0_pos-=scan_speed
			scGear.moveAngle(1, pwm0_pos)
			scGear.moveAngle(0, pwm0_pos)
			# print("11122")
			dist = ultra.checkdist()
			# print("22233")
			if dist > 200:
				continue
			theta = 90 + pwm0_pos 
			result.append([dist, theta])
			time.sleep(0.02)
	
		scGear.set_angle(1, 0)
		return result


	def pause(self):
		self.functionMode = 'none'
		move.motorStop()
		self.__flag.clear()


	def resume(self):
		self.__flag.set()


	def automatic(self):
		# print("aaa")
		self.functionMode = 'Automatic'
		self.resume()


	def trackLine(self):
		self.functionMode = 'trackLine'
		self.resume()


	def keepDistance(self):
		self.functionMode = 'keepDistance'
		self.resume()


	def steady(self,goalPos):
		self.functionMode = 'Steady'
		self.steadyGoal = goalPos
		self.resume()


	# def speech(self):
	# 	self.functionMode = 'speechRecProcessing'
	# 	self.resume()


	def trackLineProcessing(self):
		status_right = track_line_right.value
		status_middle = track_line_middle.value
		status_left = track_line_left.value
		#print('R%d   M%d   L%d'%(status_right,status_middle,status_left))
		if status_middle == 0:
			scGear.moveAngle(2, 0)
			move.move(80,1,"mid")
		elif status_left == 0:
			scGear.moveAngle(2, 30)
			move.move(80,1,"left")
		elif status_right == 0:
			scGear.moveAngle(2,-30)
			move.move(80,1,"left")
		else:
			move.move(0,1,"no")
		print(status_left,status_middle,status_right)
		time.sleep(0.1)


	# def automaticProcessing(self):
	# 	print('automaticProcessing')

	# 	scGear.moveAngle(2, 0)
	# 	if self.scanPos == 1:
	#		scGear.moveAngle(self.scanServo, pwm1_init+self.scanRange)
	# 		time.sleep(0.3)
	# 		self.scanList[0] = ultra.checkdist()
	# 	elif self.scanPos == 2:
	#		scGear.moveAngle(self.scanServo, pwm1_init)
	# 		time.sleep(0.3)
	# 		self.scanList[1] = ultra.checkdist()
	# 	elif self.scanPos == 3:
	#		scGear.moveAngle(self.scanServo, pwm1_init-self.scanRange)
	# 		time.sleep(0.3)
	# 		self.scanList[2] = ultra.checkdist()

	# 	self.scanPos = self.scanPos + self.scanDir

	# 	if self.scanPos > self.scanNum or self.scanPos < 1:
	# 		if self.scanDir == 1:self.scanDir = -1
	# 		elif self.scanDir == -1:self.scanDir = 1
	# 		self.scanPos = self.scanPos + self.scanDir*2
	# 	print(self.scanList)

	# 	if min(self.scanList) < self.rangeKeep:
	# 		if self.scanList.index(min(self.scanList)) == 0:
	# 			scGear.moveAngle(2, -30)
	# 		elif self.scanList.index(min(self.scanList)) == 1:
	# 			if self.scanList[0] < self.scanList[2]:
	# 				scGear.moveAngle(2, -45)
	# 			else:
	# 				scGear.moveAngle(2, 45)
	# 		elif self.scanList.index(min(self.scanList)) == 2:
	# 			scGear.moveAngle(2, 30)
	# 		if max(self.scanList) < self.rangeKeep or min(self.scanList) < self.rangeKeep/3:
	# 			move.motor_left(1, 1, 80)
	# 			move.motor_right(1, 1, 80)
	# 	else:
	# 		#move along
	# 		move.motor_left(1, 0, 80)
	# 		move.motor_right(1, 0, 80)
	# 		pass

	
# Filter out occasional incorrect distance data.
	def distRedress(self): 
		mark = 0
		distValue = ultra.checkdist()
		while True:
			distValue = ultra.checkdist()
			if distValue > 900:
				mark +=  1
			elif mark > 5 or distValue < 900:
					break
			print(distValue)
		return round(distValue,2)

	def automaticProcessing(self):
		print('automaticProcessing')
		scGear.moveAngle(1, 0)
		dist = self.distRedress()
		print(dist, "cm")
		time.sleep(0.2)
		if dist >= 40:			# More than 40CM, go straight.
			scGear.moveAngle(2, 0)
			move.move(80, 1, "mid")
			print("Forward")
		# More than 20cm and less than 40cm, detect the distance between the left and right sides.
		elif dist > 20 and dist < 50:	
			scGear.moveAngle(1, 30)
			move.move(0, 1, "mid")
			time.sleep(0.3)
			distLeft = self.distRedress()
			self.scanList[0] = distLeft

			# Go in the direction where the detection distance is greater.
			scGear.moveAngle(1, -30)
			time.sleep(0.3)
			distRight = self.distRedress()
			self.scanList[1] = distRight
			print(self.scanList)
			scGear.moveAngle(1, 0)
			if self.scanList[0] >= self.scanList[1]:
				scGear.moveAngle(2, 30)
				time.sleep(0.3)
				move.move(80,1,"left")
				print("Left")
				time.sleep(1)
			else:
				scGear.moveAngle(2, -30)
				time.sleep(0.3)
				move.move(80, 1, "right")
				print("Right")
				time.sleep(1)
		else:		# The distance is less than 20cm, back.
			move.move(80, -1, "mid")
			print("Back")
			time.sleep(1)


	# def steadyProcessing(self):
	# 	print('steadyProcessing')
	# 	xGet = sensor.get_accel_data()
	# 	xGet = xGet['x']
	# 	xOut = kalman_filter_X.kalman(xGet)
	#   scGear.moveAngle(2, self.steadyGoal+pwmGenOut(xGet*9))
	
	# 	time.sleep(0.05)


	# def speechRecProcessing(self):
	# 	print('speechRecProcessing')
	# 	speech.run()


	def keepDisProcessing(self):
		# print('keepDistanceProcessing')
		distanceGet = ultra.checkdist()
		if distanceGet > (self.rangeKeep/2+0.1):
			move.move(80, 1, "mid")
		elif distanceGet < (self.rangeKeep/2-0.1):
			move.move(80, -1, "mid")
		else:
			move.motorStop()


	def functionGoing(self):
		if self.functionMode == 'none':
			self.pause()
		elif self.functionMode == 'Automatic':
			self.automaticProcessing()
			# print("aaa")
		elif self.functionMode == 'Steady':
			self.steadyProcessing()
		elif self.functionMode == 'trackLine':
			self.trackLineProcessing()
		# elif self.functionMode == 'speechRecProcessing':
		# 	self.speechRecProcessing()
		elif self.functionMode == 'keepDistance':
			self.keepDisProcessing()


	def run(self):
		while 1:
			self.__flag.wait()
			self.functionGoing()
			pass


if __name__ == '__main__':
	pass
	try:
		fuc=Functions()
		fuc.setup()
		while True:
			# fuc.radarScan()
			# fuc.start()
			# print("qqq")
			# fuc.automaticProcessing()
			# print("www")
			# fuc.trackLineProcessing()
			fuc.keepDisProcessing()
			# # fuc.steady(300)
			# time.sleep(30)
			# fuc.pause()
			# time.sleep(1)
			# move.move(80, 'no', 'no', 0.5)
	except KeyboardInterrupt:

			move.motorStop()
