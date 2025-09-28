#!/usr/bin/env python3
# File name   : RPiservo.py
# Description : Multi-threaded Control Servos
# Author	  : Adeept
from __future__ import division
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import threading

# When the motor occupies pins 9-15 on the PCA9685, 
# the servo can only use pins 0-8.
i2c = busio.I2C(SCL, SDA)
# Create a simple PCA9685 class instance.
pwm_servo = PCA9685(i2c, address=0x5f) #default 0x40
pwm_servo.frequency = 50

servo_num = 8
i2c = None
pwm_servo = None

class ServoCtrlThread(threading.Thread):
    
    def __init__(self, name, controler, channel_number, initPos, direction):
        self.__name = name
        self.__channel = controler.pwm_servo.__channels[channel_number]
        self.sc_direction = direction
        self.initPos = initPos
        self.ingGoal = initPos
        self.maxPos = 180
        self.minPos = 0
        self.scSpeed = 0
        self.ctrl = controler
        self.ctrlRangeMax = 180
        self.ctrlRangeMin = 0
        self.angleRange = 180
        self.scMode = 'auto'
        self.scTime = 2.0
        self.scSteps = 30
        self.scDelay = 0.09
        self.scMoveTime = 0.09
        self.goalUpdate = 0
        self.wiggleDirection = 1
        self.setPWM(initPos)

        super().__init__()
        self.__flag = threading.Event()
        self.__flag.clear()
        self.start()

    ##################################
    ########## SERVO Action ##########
    ##################################            
    def clockwise(self):
        self.__move(self.sc_direction, 1)
        
    def anticlockwise(self):
        self.__move(-self.sc_direction, 1)
        
    def stopWiggle(self):
        self.pause()
        self.goalUpdate = 1
        self.lastPos = self.nowPos
        self.goalUpdate = 0
        
    def __move(self, direcInput, speedSet):
        self.wiggleDirection = direcInput
        self.scSpeed = speedSet
        self.scMode = 'wiggle'
        self.goalUpdate = 1
        self.lastPos = self.nowPos
        self.goalUpdate = 0
        self.resume()
        
    def set_angle(self, angle):
        servo_angle = servo.Servo(self.__channel, min_pulse=500, max_pulse=2400,actuation_range=180)
        servo_angle.angle = angle
        #self.ctrl.set_angle(self.__id, angle)
        
    def scMove(self):
        if self.scMode == 'init':
            self.reset()
        elif self.scMode == 'auto':
            self.moveAuto()
        elif self.scMode == 'certain':
            self.moveCert()
        elif self.scMode == 'wiggle':
            self.moveWiggle()
        
    def moveAuto(self):
        self.ingGoal = self.goalPos

        for step in range(1, self.scSteps + 1):
            if not self.goalUpdate:
                self.nowPos = int(round((self.lastPos + (((self.goalPos - self.lastPos)/self.scSteps)*step)),0))
                self.set_angle(self.nowPos)

                if self.ingGoal != self.goalPos:
                    self.goalUpdate = 1
                    self.lastPos = self.nowPos
                    self.goalUpdate = 0
                    time.sleep(self.scTime/self.scSteps)
                    return 1
            time.sleep((self.scTime/self.scSteps - self.scMoveTime))

        self.goalUpdate = 1
        self.lastPos = self.nowPos
        self.goalUpdate = 0
        self.pause()
        return 0

    def moveCert(self):
        self.ingGoal = self.goalPos
        self.bufferPos = self.lastPos

        while self.nowPos != self.goalPos:
            if self.lastPos < self.goalPos:
                self.bufferPos += self.pwmGenOut(self.scSpeed)/(1/self.scDelay)
                self.nowPos = int(round(self.bufferPos, 0))
                if self.nowPos > self.goalPos: 
                    self.nowPos = self.goalPos
                    
            elif self.lastPos > self.goalPos:
                self.bufferPos -= self.pwmGenOut(self.scSpeed)/(1/self.scDelay)
                self.nowPos = int(round(self.bufferPos, 0))
                if self.nowPos < self.goalPos:
                    self.nowPos = self.goalPos

            if not self.goalUpdate:
                self.set_angle(i, self.nowPos)

            if self.ingGoal != self.goalPos:
                self.goalUpdate = 1
                self.lastPos = self.nowPos
                self.goalUpdate = 0
                return 1
            
            self.goalUpdate = 1
            self.lastPos = self.nowPos
            self.goalUpdate = 0
            time.sleep(self.scDelay-self.scMoveTime)

        else:
            self.pause()
            return 0   

    def moveWiggle(self):
        self.bufferPos += self.wiggleDirection*self.pwmGenOut(self.scSpeed)/(1/self.scDelay)
        newNow = int(round(self.bufferPos, 0))
        if self.bufferPos > self.maxPos:
            self.bufferPos = self.maxPos
        elif self.bufferPos < self.minPos:
            self.bufferPos = self.minPos
        self.nowPos = newNow
        self.lastPos = newNow
        if self.bufferPos < self.maxPos and self.bufferPos > self.minPos:
            self.set_angle(self.nowPos)
        else:
            self.stopWiggle()
        time.sleep(self.scDelay-self.scMoveTime)

    def pwmGenOut(self, angleInput):
        return int(round(((self.ctrlRangeMax-self.ctrlRangeMin)/self.angleRange*angleInput),0))
        
    ##################################
    ########## SERVO PWM    ##########
    ##################################
    def incrementPwm(self):
        self.setPWM(self.nowPos + 1)
        
    def derementPwm(self):
        self.setPWM(self.nowPos - 1)

    def reset(self):
        self.setPWM(self.initPos)
        self.pause()
        
    def setPWM(self, PWM_input):
        if PWM_input > self.minPos and PWM_input < self.maxPos:
            self.set_angle(PWM_input)
            self.lastPos = PWM_input
            self.nowPos = PWM_input
            self.bufferPos = float(PWM_input)
            self.goalPos = PWM_input
        else:
            print('initPos Value Error.')
   
    ##################################
    ####### Thread Management ########
    ##################################
    def run(self):
        while 1:
            self.__flag.wait()
            self.scMove()
            pass
            
    def pause(self):
        print(f'-> pause {self.__name}')
        self.__flag.clear()

    def resume(self):
        print(f'-> resume {self.__name}')
        self.__flag.set()
        

class ServoCtrl(threading.Thread):

    def __init__(self, *args, **kwargs):
        # global i2c,pwm_servo
        self.i2c = busio.I2C(SCL, SDA)
        # Create a simple PCA9685 class instance.
        self.pwm_servo = PCA9685(self.i2c, address=0x5f) #default 0x40
        
        self.pwm_servo.frequency = 50

        
        self.sc_direction = [1,1,1,1, 1,1,1,1]
        # # If motors are not used, 16 servos need to be controlled at the same time.
        # self.sc_direction = [1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1] 
        self.initPos = [90,90,90,90, 90,90,90,90]
        self.goalPos = [90,90,90,90, 90,90,90,90]
        self.nowPos  = [90,90,90,90, 90,90,90,90]
        self.bufferPos  = [90.0,90.0,90.0,90.0, 90.0,90.0,90.0,90.0]
        self.lastPos = [90,90,90,90, 90,90,90,90]
        self.ingGoal = [90,90,90,90, 90,90,90,90]
        self.maxPos  = [180,180,180,180, 180,180,180,180]
        self.minPos  = [0,0,0,0, 0,0,0,0]
        self.scSpeed = [0,0,0,0, 0,0,0,0]
        self.ctrlRangeMax = 180
        self.ctrlRangeMin = 0
        self.angleRange = 180

        '''
        scMode: 'init' 'auto' 'certain' 'quick' 'wiggle'
        '''
        self.scMode = 'auto'
        self.scTime = 2.0
        self.scSteps = 30

        self.scDelay = 0.09
        self.scMoveTime = 0.09
        

        self.goalUpdate = 0
        self.wiggleID = 0
        self.wiggleDirection = 1

        super(ServoCtrl, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.clear()

    def set_angle(self,ID, angle):
        servo_angle = servo.Servo(self.pwm_servo.__channels[ID], min_pulse=500, max_pulse=2400, actuation_range=180)
        servo_angle.angle = angle

  
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
        time.sleep(self.scDelay - self.scMoveTime)


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
    
   

