#!/usr/bin/env python3
# File name   : functions.py
# Description : Control Functions
# Author	  : Adeept
# Date		: 2024/03/10
import time
import threading
import ultra
import move
import RPIservo
from gpiozero import InputDevice

scGear = RPIservo.ServoCtrl()
scGear.start()

TL_Speed = 50

AutoMatic_Speed = 50
KD_Speed = 50

move.setup()


line_pin_left = 22
line_pin_middle = 27
line_pin_right = 17



class Functions(threading.Thread):
	def __init__(self, *args, **kwargs):
		self.functionMode = 'none'

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

	def pwmGenOut(self, angleInput):
		# return int(round(23/9*angleInput))
		return int(angleInput)

	def setup(self):
		global track_line_left, track_line_middle,track_line_right
		track_line_left = InputDevice(pin=line_pin_right)
		track_line_middle = InputDevice(pin=line_pin_middle)
		track_line_right = InputDevice(pin=line_pin_left)


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



	def trackLineProcessing(self):
		status_right = track_line_right.value
		status_middle = track_line_middle.value
		status_left = track_line_left.value
		print('R%d   M%d   L%d'%(status_right,status_middle,status_left))
		if status_middle == 0:
			move.trackingMove(TL_Speed,1,"mid")
		elif status_left == 0:
			move.trackingMove(TL_Speed,1,"left")
		elif status_right == 0:
			move.trackingMove(TL_Speed,1,"right")
		else:
			move.trackingMove(TL_Speed,1,"no")
		time.sleep(0.1)


	
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
		# scGear.moveAngle(1, 0)
		dist = self.distRedress()
		print(dist, "cm")
		time.sleep(0.2)
		if dist >= 40:			# More than 40CM, go straight.
			move.move(AutoMatic_Speed, 1, "mid")
			print("Forward")
		# More than 20cm and less than 40cm, speed reduced.
		elif dist > 20 and dist < 40:	
			move.move(AutoMatic_Speed, 1, "left")
			time.sleep(0.2)
			distLeft = self.distRedress()
			self.scanList[0] = distLeft
			move.move(AutoMatic_Speed, 1, "right")
			time.sleep(0.4)
			distRight = self.distRedress()
			self.scanList[1] = distRight
			print(self.scanList)
			if self.scanList[0] >= self.scanList[1]:
				move.move(AutoMatic_Speed,1,"left")
				print("Left")
				time.sleep(0.5)
			else:
				move.move(AutoMatic_Speed, 1, "right")
				print("Right")
				time.sleep(0.2)
		else:		# The distance is less than 20cm, back.
			move.move(AutoMatic_Speed, -1, "mid")
			print("Back")
			time.sleep(0.8)


	def keepDisProcessing(self):
		distanceGet = ultra.checkdist()
		if distanceGet > (self.rangeKeep/2+0.1):
			move.move(KD_Speed, 1, "mid")
		elif distanceGet < (self.rangeKeep/2-0.1):
			move.move(KD_Speed, -1, "mid")
		else:
			move.motorStop()


	def functionGoing(self):
		if self.functionMode == 'none':
			self.pause()
		elif self.functionMode == 'Automatic':
			self.automaticProcessing()
		elif self.functionMode == 'trackLine':
			self.trackLineProcessing()
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
		
	except KeyboardInterrupt:

			move.motorStop()
