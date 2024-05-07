#!/usr/bin/env python3
# File name   : RPiservo.py
# Description : Multi-threaded Control Servos
# Author	  : devin
from __future__ import division
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import threading
import random




# When the motor occupies pins 9-15 on the PCA9685, 
# the servo can only use pins 0-8.


# pwm_servo = Adafruit_PCA9685.PCA9685()
# pwm_servo.set_pwm_freq(50)

init_pwm0 = 90
init_pwm1 = 90
init_pwm2 = 90
init_pwm3 = 90

init_pwm4 = 90
init_pwm5 = 90
init_pwm6 = 90
init_pwm7 = 90

servo_num = 8
i2c = None
pwm_servo = None

class ServoCtrl(threading.Thread):

    def __init__(self, *args, **kwargs):
        self.sc_direction = [1,1,1,1, 1,1,1,1]
        # # If motors are not used, 16 servos need to be controlled at the same time.
        # self.sc_direction = [1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1] 
        self.initPos = [init_pwm0,init_pwm1,init_pwm2,init_pwm3,
                        init_pwm4,init_pwm5,init_pwm6,init_pwm7]
        # self.goalPos = [300,300,300,300, 300,300,300,300]
        # self.nowPos  = [300,300,300,300, 300,300,300,300]
        self.goalPos = [90,90,90,90, 90,90,90,90]
        self.nowPos  = [90,90,90,90, 90,90,90,90]
        self.bufferPos  = [90.0,90.0,90.0,90.0, 90.0,90.0,90.0,90.0]
        self.lastPos = [90,90,90,90, 90,90,90,90]
        self.ingGoal = [90,90,90,90, 90,90,90,90]
        self.maxPos  = [180,180,180,180, 180,180,180,180]
        self.minPos  = [0,0,0,0, 0,0,0,0]
        self.scSpeed = [0,0,0,0, 0,0,0,0]

        # self.ctrlRangeMax = 560
        # self.ctrlRangeMin = 100
        self.ctrlRangeMax = 180
        self.ctrlRangeMin = 0
        self.angleRange = 180

        '''
        scMode: 'init' 'auto' 'certain' 'quick' 'wiggle'
        '''
        self.scMode = 'auto'
        self.scTime = 2.0
        self.scSteps = 30
        # self.scTime = 0.8
        # self.scSteps = 12
        
        # self.scDelay = 0.037
        # self.scMoveTime = 0.037
        self.scDelay = 0.09
        self.scMoveTime = 0.09
        

        self.goalUpdate = 0
        self.wiggleID = 0
        self.wiggleDirection = 1

        super(ServoCtrl, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.clear()

    # def setup(self):
    # 	global i2c,pwm_servo
    # 	i2c = busio.I2C(SCL, SDA)
    # 	# Create a simple PCA9685 class instance.
    # 	pwm_servo = PCA9685(i2c, address=0x5f) #default 0x40

    # 	pwm_servo.frequency = 50
    # 	print("setup")

    def set_angle(self,ID, angle):
        # global i2c,pwm_servo
        i2c = busio.I2C(SCL, SDA)
        # Create a simple PCA9685 class instance.
        pwm_servo = PCA9685(i2c, address=0x5f) #default 0x40

        pwm_servo.frequency = 50
        servo_angle = servo.Servo(pwm_servo.channels[ID], min_pulse=500, max_pulse=2400,actuation_range=180)
        servo_angle.angle = angle


    # def set_pwm(self):
        
    # 	pwm_servo.set_angle(channel, angle)
    def pause(self):
        print('......................pause..........................')
        self.__flag.clear()


    def resume(self):
        print('resume')
        self.__flag.set()


    def moveInit(self):
        self.scMode = 'init'
        for i in range(0,servo_num):
            self.set_angle(i,self.initPos[i])
            self.lastPos[i] = self.initPos[i]
            self.nowPos[i] = self.initPos[i]
            self.bufferPos[i] = float(self.initPos[i])
            self.goalPos[i] = self.initPos[i]
        self.pause()


    def initConfig(self, ID, initInput, moveTo):
        if initInput > self.minPos[ID] and initInput < self.maxPos[ID]:
            self.initPos[ID] = initInput
            if moveTo:
                self.set_angle(ID,self.initPos[ID])
        else:
            print('initPos Value Error.')


    def moveServoInit(self, ID):
        self.scMode = 'init'
        for i in range(0,len(ID)):
            self.set_angle(ID[i], self.initPos[ID[i]])
            self.lastPos[ID[i]] = self.initPos[ID[i]]
            self.nowPos[ID[i]] = self.initPos[ID[i]]
            self.bufferPos[ID[i]] = float(self.initPos[ID[i]])
            self.goalPos[ID[i]] = self.initPos[ID[i]]
        self.pause()


    def posUpdate(self):
        self.goalUpdate = 1
        for i in range(0,servo_num):
            self.lastPos[i] = self.nowPos[i]
        self.goalUpdate = 0

    def returnServoAngle(self, ID):
        # # self.nowPos[ID] = int(self.initPos[ID] + self.sc_direction[ID]*self.pwmGenOut(angleInput))
        # pwmGenOut_value = int((self.nowPos[ID] - self.initPos[ID] )/self.sc_direction[ID])
        # # self.pwmGenOut(angleInput) = int(round(((self.ctrlRangeMax-self.ctrlRangeMin)/self.angleRange*angleInput),0))
        # angle_value = pwmGenOut_value*self.angleRange/((self.ctrlRangeMax-self.ctrlRangeMin))
        return self.nowPos[ID]


    def speedUpdate(self, IDinput, speedInput):
        for i in range(0,len(IDinput)):
            self.scSpeed[IDinput[i]] = speedInput[i]


    def moveAuto(self):
        for i in range(0,servo_num):
            self.ingGoal[i] = self.goalPos[i]

        for i in range(0, self.scSteps):
            for dc in range(0,servo_num):
                if not self.goalUpdate:
                    self.nowPos[dc] = int(round((self.lastPos[dc] + (((self.goalPos[dc] - self.lastPos[dc])/self.scSteps)*(i+1))),0))
                    self.set_angle(dc, self.nowPos[dc])

                if self.ingGoal != self.goalPos:
                    self.posUpdate()
                    time.sleep(self.scTime/self.scSteps)
                    return 1
            time.sleep((self.scTime/self.scSteps - self.scMoveTime))

        self.posUpdate()
        self.pause()
        return 0


    def moveCert(self):
        for i in range(0,servo_num):
            self.ingGoal[i] = self.goalPos[i]
            self.bufferPos[i] = self.lastPos[i]

        while self.nowPos != self.goalPos:
            for i in range(0,servo_num):
                if self.lastPos[i] < self.goalPos[i]:
                    self.bufferPos[i] += self.pwmGenOut(self.scSpeed[i])/(1/self.scDelay)
                    newNow = int(round(self.bufferPos[i], 0))
                    if newNow > self.goalPos[i]:newNow = self.goalPos[i]
                    self.nowPos[i] = newNow
                elif self.lastPos[i] > self.goalPos[i]:
                    self.bufferPos[i] -= self.pwmGenOut(self.scSpeed[i])/(1/self.scDelay)
                    newNow = int(round(self.bufferPos[i], 0))
                    if newNow < self.goalPos[i]:newNow = self.goalPos[i]
                    self.nowPos[i] = newNow

                if not self.goalUpdate:
                    self.set_angle(i, self.nowPos[i])

                if self.ingGoal != self.goalPos:
                    self.posUpdate()
                    return 1
            self.posUpdate()
            time.sleep(self.scDelay-self.scMoveTime)

        else:
            self.pause()
            return 0


    def pwmGenOut(self, angleInput):
        # print(int(round(((self.ctrlRangeMax-self.ctrlRangeMin)/self.angleRange*angleInput),0)))
        return int(round(((self.ctrlRangeMax-self.ctrlRangeMin)/self.angleRange*angleInput),0))


    def setAutoTime(self, autoSpeedSet):
        self.scTime = autoSpeedSet


    def setDelay(self, delaySet):
        self.scDelay = delaySet


    def autoSpeed(self, ID, angleInput):
        self.scMode = 'auto'
        self.goalUpdate = 1
        for i in range(0,len(ID)):
            newGoal = self.initPos[ID[i]] + self.pwmGenOut(angleInput[i])*self.sc_direction[ID[i]]
            if newGoal>self.maxPos[ID[i]]:newGoal=self.maxPos[ID[i]]
            elif newGoal<self.minPos[ID[i]]:newGoal=self.minPos[ID[i]]
            self.goalPos[ID[i]] = newGoal
        self.goalUpdate = 0
        self.resume()


    def certSpeed(self, ID, angleInput, speedSet):
        self.scMode = 'certain'
        self.goalUpdate = 1
        for i in range(0,len(ID)):
            newGoal = self.initPos[ID[i]] + self.pwmGenOut(angleInput[i])*self.sc_direction[ID[i]]
            if newGoal>self.maxPos[ID[i]]:newGoal=self.maxPos[ID[i]]
            elif newGoal<self.minPos[ID[i]]:newGoal=self.minPos[ID[i]]
            self.goalPos[ID[i]] = newGoal
        self.speedUpdate(ID, speedSet)
        self.goalUpdate = 0
        self.resume()


    def moveWiggle(self):
        self.bufferPos[self.wiggleID] += self.wiggleDirection*self.sc_direction[self.wiggleID]*self.pwmGenOut(self.scSpeed[self.wiggleID])/(1/self.scDelay)
        newNow = int(round(self.bufferPos[self.wiggleID], 0))
        if self.bufferPos[self.wiggleID] > self.maxPos[self.wiggleID]:self.bufferPos[self.wiggleID] = self.maxPos[self.wiggleID]
        elif self.bufferPos[self.wiggleID] < self.minPos[self.wiggleID]:self.bufferPos[self.wiggleID] = self.minPos[self.wiggleID]
        self.nowPos[self.wiggleID] = newNow
        self.lastPos[self.wiggleID] = newNow
        if self.bufferPos[self.wiggleID] < self.maxPos[self.wiggleID] and self.bufferPos[self.wiggleID] > self.minPos[self.wiggleID]:
            self.set_angle(self.wiggleID, self.nowPos[self.wiggleID])
        else:
            self.stopWiggle()
        time.sleep(self.scDelay-self.scMoveTime)


    def stopWiggle(self):
        self.pause()
        self.posUpdate()


    def singleServo(self, ID, direcInput, speedSet):
        self.wiggleID = ID
        self.wiggleDirection = direcInput
        self.scSpeed[ID] = speedSet
        self.scMode = 'wiggle'
        self.posUpdate()
        self.resume()


    def moveAngle(self, ID, angleInput):
        # print(angleInput)
        self.nowPos[ID] = int(self.initPos[ID] + self.sc_direction[ID]*self.pwmGenOut(angleInput))
        # print(self.nowPos[ID])
        if self.nowPos[ID] > self.maxPos[ID]:self.nowPos[ID] = self.maxPos[ID]
        elif self.nowPos[ID] < self.minPos[ID]:self.nowPos[ID] = self.minPos[ID]
        self.lastPos[ID] = self.nowPos[ID]
        # print(self.nowPos[ID])
        self.set_angle(ID, self.nowPos[ID])


    def scMove(self):
        if self.scMode == 'init':
            self.moveInit()
        elif self.scMode == 'auto':
            self.moveAuto()
        elif self.scMode == 'certain':
            self.moveCert()
        elif self.scMode == 'wiggle':
            self.moveWiggle()


    def setPWM(self, ID, PWM_input):
        self.lastPos[ID] = PWM_input
        self.nowPos[ID] = PWM_input
        self.bufferPos[ID] = float(PWM_input)
        self.goalPos[ID] = PWM_input
        self.set_angle(ID, PWM_input)
        self.pause()


    def run(self):
        while 1:
            self.__flag.wait()
            self.scMove()
            pass


if __name__ == '__main__':
    
    scGear = ServoCtrl()
    scGear.moveInit()
    # scGear.start()
    sc = ServoCtrl()
    # sc.setup()
    sc.start()
    
    value = 0
    dir = 1
    while 1:
        # sc.set_angle(0,0)
        # sc.set_angle(0,1)
        # sc.set_angle(0,2)
        # sc.set_angle(0,3)
        # time.sleep(1)
        # sc.set_angle(0,90)
        # time.sleep(1)
        # sc.set_angle(0,180)
        # time.sleep(1)

        # sc.singleServo(7, 1, 1)
        # time.sleep(3)
        # sc.singleServo(7, -1, 3)
        # time.sleep(1)
        # value = 0
        # if dir == 1:
        # 	# sc.set_angle(0,value)
        # 	sc.moveAngle(1,value)
        # 	value = value + 1
        # 	# print("1111")
        # else:
        # 	# sc.set_angle(0,value)
        # 	sc.moveAngle(0,value)
        # 	value = value - 1

        # if value == 180:
        # 	dir = -1
        # elif value == 0:
        # 	dir = 1
        # time.sleep(0.01)		
        # time.sleep(1)
        # sc.singleServo(0, 1, 10)
        scGear.moveAngle(1, -50)
        time.sleep(1)
        # sc.singleServo(0, -1, 10)
        scGear.moveAngle(1, 50)
        time.sleep(1)
        # sc.stopWiggle()
        # scGear.stopWiggle()
        # time.sleep(1)
        # sc.singleServo(1, 1, 1)
        # sc.singleServo(1, 1, 1)
        # time.sleep(1)

        # moveAngle stopWiggle singleServo

