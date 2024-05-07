#!/usr/bin/env python3
# coding: utf-8
import os
import cv2
from base_camera import BaseCamera
import RPIservo
import numpy as np
import move
import switch
import datetime
import Kalman_filter
import PID
import time
import threading
import imutils

import picamera2
import libcamera

from picamera2 import Picamera2, Preview
import io
# from threading import Condition, Thread, Event
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput
# import robotLight

# led = robotLight.RobotLight()
pid = PID.PID()
pid.SetKp(0.5)
pid.SetKd(0)
pid.SetKi(0)

Threshold = 80 # 
findLineMove = 1
tracking_servo_status = 0
FLCV_Status = 0

CVRun = 1
linePos_1 = 440
linePos_2 = 380
lineColorSet = 255
frameRender = 1
findLineError = 20
# left_forward  = 0
# left_backward = 1

# right_forward = 0
# right_backward= 1
# When turning, only one wheel pushes the car, so a value higher than forward_speed is required.
turn_speed = 35 # Range of values: 0-100
forward_speed = 20 # Avoid too fast, the video screen does not respond in time. Range of values: 0-100.


hflip = 0 # Video flip horizontally: 0 or 1 （视频是否需要水平翻转）
vflip = 0 # Video vertical flip: 0/1 （视频是否需要垂直翻转）
ImgIsNone = 0

colorUpper = np.array([44, 255, 255])
colorLower = np.array([24, 100, 100])

def map(input, in_min,in_max,out_min,out_max):
    return (input-in_min)/(in_max-out_min)*(out_max-out_min)+out_min

class CVThread(threading.Thread):
# class CVThread(Thread):
    font = cv2.FONT_HERSHEY_SIMPLEX

    kalman_filter_X =  Kalman_filter.Kalman_filter(0.01,0.1)
    kalman_filter_Y =  Kalman_filter.Kalman_filter(0.01,0.1)
    P_direction = -1
    T_direction = -1
    P_servo = 1 # Horizontal servo
    T_servo = 2 # Vertical servo
    P_anglePos = 0
    T_anglePos = 0
    cameraDiagonalW = 64
    cameraDiagonalH = 48
    videoW = 640
    videoH = 480
    Y_lock = 0
    X_lock = 0
    tor = 17

    scGear = RPIservo.ServoCtrl()
    scGear.moveInit()
    Tracking_sc = RPIservo.ServoCtrl()
    Tracking_sc.start()
    move.setup()
    # switch.switchSetup()

    def __init__(self, *args, **kwargs):
        self.CVThreading = 0
        self.CVMode = 'none'
        self.imgCV = None

        self.mov_x = None
        self.mov_y = None
        self.mov_w = None
        self.mov_h = None

        self.radius = 0
        self.box_x = None
        self.box_y = None
        self.drawing = 0

        self.findColorDetection = 0

        self.left_Pos1 = None
        self.right_Pos1 = None
        self.center_Pos1 = None

        self.left_Pos2 = None
        self.right_Pos2 = None
        self.center_Pos2 = None

        self.center = None
        
        self.tracking_servo_left = None
        self.tracking_servo_left_mark = 0
        self.tracking_servo_right_mark = 0
        self.servo_left_stop = 0
        self.servo_right_stop = 0

        super(CVThread, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        # self.__flag = Event()
        self.__flag.clear()

        self.avg = None
        self.motionCounter = 0
        self.lastMovtionCaptured = datetime.datetime.now()
        self.frameDelta = None
        self.thresh = None
        self.cnts = None

    def mode(self, invar, imgInput):
        self.CVMode = invar
        self.imgCV = imgInput
        self.resume()

    def elementDraw(self,imgInput):
        if self.CVMode == 'none':
            pass

        elif self.CVMode == 'findColor':
            if self.findColorDetection:
                cv2.putText(imgInput,'Target Detected',(40,60), CVThread.font, 0.5,(255,255,255),1,cv2.LINE_AA)
                self.drawing = 1
            else:
                cv2.putText(imgInput,'Target Detecting',(40,60), CVThread.font, 0.5,(255,255,255),1,cv2.LINE_AA)
                self.drawing = 0

            if self.radius > 10 and self.drawing:
                cv2.rectangle(imgInput,(int(self.box_x-self.radius),int(self.box_y+self.radius)),(int(self.box_x+self.radius),int(self.box_y-self.radius)),(255,255,255),1)

        elif self.CVMode == 'findlineCV':
            CVThread.scGear.moveAngle(2, -45) # The camera looks down.

            if frameRender:
                imgInput = cv2.cvtColor(imgInput, cv2.COLOR_BGR2GRAY)
                '''
                Image binarization, the method of processing functions can be searched for "threshold" in the link: http://docs.opencv.org/3.0.0/examples.html
                '''
                # retval_bw, imgInput =  cv2.threshold(imgInput, 0, 255, cv2.THRESH_OTSU) # THRESH_OTSU:Adaptive Threshold (Dynamic Threshold).flag, use Otsu algorithm to choose the threshold value.
                retval_bw, imgInput =  cv2.threshold(imgInput, Threshold, 255, cv2.THRESH_BINARY) # Set the threshold manually and set it to 80.
                # imgInput = cv2.erode(imgInput, None, iterations=6)
                imgInput = cv2.erode(imgInput, None, iterations=2) #  erode
                imgInput = cv2.dilate(imgInput, None, iterations=2) # dilate

            try:
                if lineColorSet == 255:
                    cv2.putText(imgInput,('Following White Line'),(30,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(128,255,128),1,cv2.LINE_AA)
                else:
                    cv2.putText(imgInput,('Following Black Line'),(30,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(128,255,128),1,cv2.LINE_AA)

                # cv2.line(imgInput,(self.left_Pos1,(linePos_1+30)),(self.left_Pos1,(linePos_1-30)),(255,128,64),1)
                # cv2.line(imgInput,(self.right_Pos1,(linePos_1+30)),(self.right_Pos1,(linePos_1-30)),(64,128,255),)
                # cv2.line(imgInput,(0,linePos_1),(640,linePos_1),(255,255,64),1)

                # cv2.line(imgInput,(self.left_Pos2,(linePos_2+30)),(self.left_Pos2,(linePos_2-30)),(255,128,64),1)
                # cv2.line(imgInput,(self.right_Pos2,(linePos_2+30)),(self.right_Pos2,(linePos_2-30)),(64,128,255),1)
                # cv2.line(imgInput,(0,linePos_2),(640,linePos_2),(255,255,64),1)

                # cv2.line(imgInput,((self.center-20),int((linePos_1+linePos_2)/2)),((self.center+20),int((linePos_1+linePos_2)/2)),(0,0,0),1)
                # cv2.line(imgInput,((self.center),int((linePos_1+linePos_2)/2+20)),((self.center),int((linePos_1+linePos_2)/2-20)),(0,0,0),1)
                
                imgInput=cv2.merge((imgInput.copy(),imgInput.copy(),imgInput.copy()))
                cv2.line(imgInput,(self.left_Pos1,(linePos_1+30)),(self.left_Pos1,(linePos_1-30)),(255,128,64),2)
                cv2.line(imgInput,(self.right_Pos1,(linePos_1+30)),(self.right_Pos1,(linePos_1-30)),(64,128,255),2)
                cv2.line(imgInput,(0,linePos_1),(640,linePos_1),(255,128,64),1)

                cv2.line(imgInput,(self.left_Pos2,(linePos_2+30)),(self.left_Pos2,(linePos_2-30)),(64,128,255),2)
                cv2.line(imgInput,(self.right_Pos2,(linePos_2+30)),(self.right_Pos2,(linePos_2-30)),(64,128,255),2)
                cv2.line(imgInput,(0,linePos_2),(640,linePos_2),(64,128,255),1)

                cv2.line(imgInput,((self.center-20),int((linePos_1+linePos_2)/2)),((self.center+20),int((linePos_1+linePos_2)/2)),(0,0,0),1)
                cv2.line(imgInput,((self.center),int((linePos_1+linePos_2)/2+20)),((self.center),int((linePos_1+linePos_2)/2-20)),(0,0,0),1)

            except:
                pass

        elif self.CVMode == 'watchDog':
            if self.drawing:
                cv2.rectangle(imgInput, (self.mov_x, self.mov_y), (self.mov_x + self.mov_w, self.mov_y + self.mov_h), (128, 255, 0), 1)

        return imgInput


    def watchDog(self, imgInput):
        timestamp = datetime.datetime.now()
        gray = cv2.cvtColor(imgInput, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        if self.avg is None:
            print("[INFO] starting background model...")
            self.avg = gray.copy().astype("float")
            return 'background model'

        cv2.accumulateWeighted(gray, self.avg, 0.5)
        self.frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(self.avg))

        # threshold the delta image, dilate the thresholded image to fill
        # in holes, then find contours on thresholded image
        self.thresh = cv2.threshold(self.frameDelta, 5, 255,
            cv2.THRESH_BINARY)[1]
        self.thresh = cv2.dilate(self.thresh, None, iterations=2)
        self.cnts = cv2.findContours(self.thresh.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        self.cnts = imutils.grab_contours(self.cnts)
        # print('x')
        # loop over the contours
        for c in self.cnts:
            # if the contour is too small, ignore it
            if cv2.contourArea(c) < 5000:
                continue
     
            # compute the bounding box for the contour, draw it on the frame,
            # and update the text
            (self.mov_x, self.mov_y, self.mov_w, self.mov_h) = cv2.boundingRect(c)
            self.drawing = 1
            
            self.motionCounter += 1
            #print(motionCounter)
            #print(text)
            self.lastMovtionCaptured = timestamp

        if (timestamp - self.lastMovtionCaptured).seconds >= 0.5:
            self.drawing = 0
        self.pause()


    # def findLineCtrl(self, posInput, setCenter):
    def findLineCtrl(self, posInput):
        global findLineMove,tracking_servo_status,FLCV_Status
        # # if posInput:
        
        if FLCV_Status == 0:    
            CVThread.scGear.moveAngle(0, 0) # car center. (servo_num, deflection angle)
            CVThread.scGear.moveAngle(1, 0) # camera centered. (servo_num, deflection angle)

            FLCV_Status = 1
        if posInput != None and findLineMove == 1:
            # if posInput < 400 and posInput > 250:
            #     FLCV_Status = 1
            # elif FLCV_Status != 1:
            #     pass
            if FLCV_Status == -1:
                CVThread.Tracking_sc.stopWiggle()
                self.tracking_servo_left_mark = 0
                self.tracking_servo_right_mark = 0
                FLCV_Status = 1
            if posInput > 480: # The position of the center of the black line in the screen (value range: 0-640)
                tracking_servo_status = 1 #  right. -1/0/1: left/mid/right. In which direction the track may be offset out of the tracking area.
                #turnRight
                if CVRun:
                    CVThread.scGear.moveAngle(0, -30) # car center. (servo_num, deflection angle)
                    move.video_Tracking_Move(turn_speed, 1) # 'no'/'right':turn Right, turn_speed锛歭eft wheel speed, 0.2:turn_speed*0.2 = right wheel speed
                else:
                    CVThread.scGear.moveAngle(0, 0)
                    move.motorStop() # stop

            elif posInput < 180: # turnLeft.
                tracking_servo_status = -1 # left
                if CVRun:
                    CVThread.scGear.moveAngle(0, 30) # car center. (servo_num, deflection angle)
                    move.video_Tracking_Move(turn_speed, 1) # 'no'/'right':turn Right, turn_speed锛歭eft wheel speed, 0.2:turn_speed*0.2 = right wheel speed
                
                else:
                    CVThread.scGear.moveAngle(0, 0)
                    move.motorStop() # stop.
                        
            else:
                tracking_servo_status = 0 # mid
                if CVRun:
                    # In the range of 180-480, adjust the motor speed difference according to the offset.
                    error = 320-posInput
                    outv = int(round((pid.GenOut(error)),0))
                    coef = map(abs(outv), -160, 160, -30, 30) # 
                    # print(outv)
                    # if outv >0: 

                    CVThread.scGear.moveAngle(0, coef) # car center. (servo_num, deflection angle)
                    move.video_Tracking_Move(turn_speed, 1) # 'no'/'right':turn Right, turn_speed锛歭eft wheel speed, 0.2:turn_speed*0.2 = right wheel speed

                    # elif outv <=0: 
                    #     move.video_Tracking_Move(1, left_forward, forward_speed)
                    #     move.video_Tracking_Move(forward_speed, 1, forward_speed)
                    #     move.motor_right(1, right_forward, int(forward_speed*coef))
                else: 
                    move.motorStop() # stop
                pass
        
        else: # Tracking color not found.
            move.motorStop() # stop.
            FLCV_Status = -1
            if tracking_servo_status == -1 : # -1/0/1: left/mid/right. rotation left.
                angle_Limit = CVThread.Tracking_sc.returnServoAngle(0) # Servo deflection angle.
                print(angle_Limit)
                if angle_Limit > 20: # The deflection angle is limited to within 30 degrees.
                    CVThread.scGear.moveAngle(0, -30)
                    move.video_Tracking_Move(turn_speed, 1)
                    # move.motor_left(1, left_forward, turn_speed)  # the car turn left.
                    # move.motor_right(1, right_forward, int(turn_speed*0.1))
                    if self.tracking_servo_left_mark == 0 or self.servo_left_stop == 0: # stop servo rotation left.
                        CVThread.Tracking_sc.stopWiggle() 
                        self.tracking_servo_left_mark = 1
                        self.servo_left_stop = 1

                if self.tracking_servo_left_mark == 0: # servo rotation left.
                    CVThread.Tracking_sc.singleServo(1, 1, 10) # (servo_num, direction,rotation_speed),direction: 1:left, -1:right
                    self.tracking_servo_left_mark = 1 # The servos will turn continuously and only need to be run once.
                    self.servo_left_stop = 0

            elif tracking_servo_status == 1 : # rotation right
                angle_Limit = CVThread.Tracking_sc.returnServoAngle(0)
                # print(angle_Limit)
                if angle_Limit < -20: # The deflection angle is limited to within 30 degrees.
                    CVThread.scGear.moveAngle(0, 30)
                    move.video_Tracking_Move(turn_speed, 1)
                    # move.motor_left(1, left_forward, int(turn_speed*0.1)) # the car turn right.
                    # move.motor_right(1, right_forward, turn_speed)
                    if self.tracking_servo_right_mark == 0 or self.servo_right_stop == 0: # stop servo rotation right.
                        CVThread.Tracking_sc.stopWiggle() 
                        self.tracking_servo_right_mark = 1
                        self.servo_right_stop = 1

                if self.tracking_servo_right_mark == 0: # servo rotation right.
                    CVThread.Tracking_sc.singleServo(0, -1, 1) # (servo_num, direction,rotation_speed),direction: 1:left, -1:right
                    self.tracking_servo_right_mark = 1 # The servos will turn continuously and only need to be run once.
                    self.servo_right_stop = 0

            else:  # no track ahead. tracking_servo_status==0
                pass
        # if posInput > (setCenter + findLineError):
        #     # move.motorStop()
        #     #turnRight
        #     error = (posInput-320)/3
        #     outv = int(round((pid.GenOut(error)),0))
        #     CVThread.scGear.moveAngle(0,-outv)
        #     if CVRun:
        #         # move.move(80, 'no', 'right', 0.5)
        #         pass
        #     else:
        #         pass
        #         # move.move(80, 'no', 'no', 0.5)
        #     pass
        # elif posInput < (setCenter - findLineError):
        #     # move.motorStop()
        #     #turnLeft
        #     error = (320-posInput)/3
        #     outv = int(round((pid.GenOut(error)),0))
        #     CVThread.scGear.moveAngle(0,outv)
        #     if CVRun:
        #         pass
        #         # move.move(80, 'no', 'left', 0.5)
        #     else:
        #         pass
        #         # move.move(80, 'no', 'no', 0.5)
        #     pass
        # else:
        #     if CVRun:
        #         pass
        #         # move.move(80, 'forward', 'no', 0.5)
        #     else:
        #         pass
        #         # move.move(80, 'no', 'no', 0.5)
        #     #forward
        #     pass
        # # else:
        # #     pass



    def findlineCV(self, frame_image):
        frame_findline = cv2.cvtColor(frame_image, cv2.COLOR_BGR2GRAY)
        # retval, frame_findline =  cv2.threshold(frame_findline, 0, 255, cv2.THRESH_OTSU)
        retval, frame_findline =  cv2.threshold(frame_findline, Threshold, 255, cv2.THRESH_BINARY) # Set the threshold manually and set it to 80.
        frame_findline = cv2.erode(frame_findline, None, iterations=2)
        frame_findline = cv2.dilate(frame_findline, None, iterations=2)
        colorPos_1 = frame_findline[linePos_1]
        colorPos_2 = frame_findline[linePos_2]
        
        try:
            lineColorCount_Pos1 = np.sum(colorPos_1 == lineColorSet)
            lineColorCount_Pos2 = np.sum(colorPos_2 == lineColorSet)

            lineIndex_Pos1 = np.where(colorPos_1 == lineColorSet)
            lineIndex_Pos2 = np.where(colorPos_2 == lineColorSet)

            # Roughly judge whether there is a color to track.
            if lineIndex_Pos1 !=[]:
                if abs(lineIndex_Pos1[0][-1] - lineIndex_Pos1[0][0]) > 500:
                    print("Tracking color not found")
                    findLineMove = 0    # No tracking color, stop moving
                else:
                    findLineMove = 1
            elif lineIndex_Pos2!= []:
                if abs(lineIndex_Pos2[0][-1] - lineIndex_Pos2[0][0]) > 500:
                    print("Tracking color not found")
                    findLineMove = 0
                else:
                    findLineMove = 1

            if lineColorCount_Pos1 == 0:
                lineColorCount_Pos1 = 1
            if lineColorCount_Pos2 == 0:
                lineColorCount_Pos2 = 1

            self.left_Pos1 = lineIndex_Pos1[0][1] # Is [1] instead of [0], in order to remove black/white edges that may appear on the far left
            self.right_Pos1 = lineIndex_Pos1[0][lineColorCount_Pos1-2]   # 

            self.center_Pos1 = int((self.left_Pos1+self.right_Pos1)/2)

            self.left_Pos2 =  lineIndex_Pos2[0][1]
            self.right_Pos2 = lineIndex_Pos2[0][lineColorCount_Pos2-2]
            self.center_Pos2 = int((self.left_Pos2+self.right_Pos2)/2)
            # print("2L/C/R: %s/%s/%s" %(self.left_Pos2, self.center_Pos2, self.right_Pos2))

            self.center = int((self.center_Pos1+self.center_Pos2)/2)
        except:
            self.center = None
            pass

        self.findLineCtrl(self.center)
        self.pause()


    def servoMove(ID, Dir, errorInput):
        if ID == 1:
            errorGenOut = CVThread.kalman_filter_X.kalman(errorInput)
            CVThread.P_anglePos += 0.15*(errorGenOut*Dir)*CVThread.cameraDiagonalW/CVThread.videoW

            if abs(errorInput) > CVThread.tor:
                CVThread.scGear.moveAngle(ID,CVThread.P_anglePos)
                CVThread.X_lock = 0
            else:
                CVThread.X_lock = 1
        elif ID == 2:
            errorGenOut = CVThread.kalman_filter_Y.kalman(errorInput)
            CVThread.T_anglePos += 0.1*(errorGenOut*Dir)*CVThread.cameraDiagonalH/CVThread.videoH

            if abs(errorInput) > CVThread.tor:
                CVThread.scGear.moveAngle(ID,CVThread.T_anglePos)
                CVThread.Y_lock = 0
            else:
                CVThread.Y_lock = 1
        else:
            print('No servoPort %d assigned.'%ID)

    def findColor(self, frame_image):
        hsv = cv2.cvtColor(frame_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, colorLower, colorUpper)#1
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        if len(cnts) > 0:
            self.findColorDetection = 1
            c = max(cnts, key=cv2.contourArea)
            ((self.box_x, self.box_y), self.radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            X = int(self.box_x)
            Y = int(self.box_y)
            error_Y = 240 - Y
            error_X = 320 - X
            # CVThread.servoMove(CVThread.P_servo, CVThread.P_direction, error_X)
            CVThread.servoMove(CVThread.T_servo, CVThread.T_direction, error_Y)

            # if CVThread.X_lock == 1 and CVThread.Y_lock == 1:
            # if CVThread.Y_lock == 1:
            #     led.setColor(255,78,0)
            #     # switch.switch(1,1)
            #     # switch.switch(2,1)
            #     # switch.switch(3,1)
            # else:
            #     led.setColor(0,78,255)
            #     # switch.switch(1,0)
            #     # switch.switch(2,0)
            #     # switch.switch(3,0)
        else:
            self.findColorDetection = 0
            # move.motorStop()
        self.pause()


    def pause(self):
        self.__flag.clear()

    def resume(self):
        self.__flag.set()

    def run(self):
        while 1:
            self.__flag.wait()
            if self.CVMode == 'none':
                continue
            
            elif self.CVMode == 'findColor':
                self.CVThreading = 1
                self.findColor(self.imgCV)
                self.CVThreading = 0
            elif self.CVMode == 'findlineCV':
                self.CVThreading = 1
                # Camera.CVRunSet(1)
                self.findlineCV(self.imgCV)
                self.CVThreading = 0
            elif self.CVMode == 'watchDog':
                self.CVThreading = 1
                self.watchDog(self.imgCV)
                self.CVThreading = 0
            else:
                pass



# class StreamingOutput(io.BufferedIOBase):
#     def __init__(self):
#         self.frame = None
#         self.condition = Condition()

#     def write(self, buf):
#         with self.condition:
#             self.frame = buf
#             self.condition.notify_all()

class Camera(BaseCamera):
    video_source = 0
    modeSelect = 'none'
    # modeSelect = 'findlineCV'
    # modeSelect = 'findColor'
    # modeSelect = 'watchDog'


    # def __init__(self):
        # if os.environ.get('OPENCV_CAMERA_SOURCE'):
        #     Camera.set_video_source(int(os.environ['OPENCV_CAMERA_SOURCE']))
        # super(Camera, self).__init__()


    def colorFindSet(self, invarH, invarS, invarV):
        global colorUpper, colorLower
        HUE_1 = invarH+15
        HUE_2 = invarH-15
        if HUE_1>180:HUE_1=180
        if HUE_2<0:HUE_2=0

        SAT_1 = invarS+150
        SAT_2 = invarS-150
        if SAT_1>255:SAT_1=255
        if SAT_2<0:SAT_2=0

        VAL_1 = invarV+150
        VAL_2 = invarV-150
        if VAL_1>255:VAL_1=255
        if VAL_2<0:VAL_2=0

        colorUpper = np.array([HUE_1, SAT_1, VAL_1])
        colorLower = np.array([HUE_2, SAT_2, VAL_2])
        print('HSV_1:%d %d %d'%(HUE_1, SAT_1, VAL_1))
        print('HSV_2:%d %d %d'%(HUE_2, SAT_2, VAL_2))
        print(colorUpper)
        print(colorLower)

    def modeSet(self, invar):
        Camera.modeSelect = invar

    def CVRunSet(self, invar):
        global CVRun
        CVRun = invar

    def linePosSet_1(self, invar):
        global linePos_1
        linePos_1 = invar

    def linePosSet_2(self, invar):
        global linePos_2
        linePos_2 = invar

    def colorSet(self, invar):
        global lineColorSet
        lineColorSet = invar

    def randerSet(self, invar):
        global frameRender
        frameRender = invar

    def errorSet(self, invar):
        global findLineError
        findLineError = invar

    def Threshold(self, value):
        global Threshold
        Threshold = value
        
    def ThresholdOK(self):
        global Threshold
        return Threshold

    @staticmethod
    def set_video_source(source):
        Camera.video_source = source


    # @staticmethod
    # def frames():
    #     imshow_flag = False

    #     picam2 = Picamera2()
    #     preview_config = picam2.preview_configuration
    #     # preview_config.size = (800, 600)
    #     preview_config.size = (640, 480)
    #     preview_config.format = 'RGB888'  # 'XRGB8888', 'XBGR8888', 'RGB888', 'BGR888', 'YUV420'
    #     hflip = 0
    #     vflip = 0
    #     preview_config.transform = libcamera.Transform(hflip=hflip, vflip=vflip)
    #     preview_config.colour_space = libcamera.ColorSpace.Sycc()
    #     preview_config.buffer_count = 4
    #     preview_config.queue = True

    #     try:
    #         picam2.start()
    #     except Exception as e:
    #         print(f"\033[38;5;1mError:\033[0m\n{e}")
    #         print("\nPlease check whether the camera is connected well,  \
    #         and disable the \"legacy camera driver\" on raspi-config")

    #     while True:
    #         start_time = time.time()
    #         # read current frame
    #         img = picam2.capture_array()

    #         yield cv2.imencode('.jpg', img)[1].tobytes()


# _______________________________________________________
    @staticmethod
    def frames():
        global ImgIsNone,hflip,vflip
        picam2 = Picamera2() 
        
        preview_config = picam2.preview_configuration
        # preview_config.size = (800, 600)
        preview_config.size = (640, 480)
        preview_config.format = 'RGB888'  # 'XRGB8888', 'XBGR8888', 'RGB888', 'BGR888', 'YUV420'
        # hflip = 0
        # vflip = 0
        preview_config.transform = libcamera.Transform(hflip=hflip, vflip=vflip)
        preview_config.colour_space = libcamera.ColorSpace.Sycc()
        preview_config.buffer_count = 4
        preview_config.queue = True
        # preview_config.framerate = 20

        # if not camera.isOpened():
        if not picam2.is_open:
            raise RuntimeError('Could not start camera.')

        try:
            picam2.start()
        except Exception as e:
            print(f"\033[38;5;1mError:\033[0m\n{e}")
            print("\nPlease check whether the camera is connected well,  \
                  and disable the \"legacy camera driver\" on raspi-config")

        cvt = CVThread()
        cvt.start()

        while True:
            start_time = time.time()
            # read current frame
            img = picam2.capture_array()

            if img is None:
                if ImgIsNone == 0:
                    print("--------------------")
                    print("\033[31merror: Unable to read camera data.\033[0m")
                    print("\033[33mIt may be that the Legacy camera is not turned on or the camera is not connected correctly.\033[0m")
                    print("Open the Legacy camera: Enter in Raspberry Pi\033[34m'sudo raspi-config'\033[0m -->Select\033[34m'3 Interface Options'\033[0m -->\033[34m'I1 Legacy Camera'\033[0m.")
                    print("Use the command: \033[34m'sudo killall python3'\033[0m. Close the self-starting program webServer.py")
                    print("Use the command: \033[34m'raspistill -t 1000 -o image.jpg'\033[0m to check whether the camera can be used correctly.")
                    print("Press the keyboard keys \033[34m'Ctrl + C'\033[0m multiple times to exit the current program.")
                    print("--------Ctrl+C quit-----------")
                    ImgIsNone = 1
                continue
            
            # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # print("img1:%d", type(img))
            if Camera.modeSelect == 'none':
                # switch.switch(1,0)
                cvt.pause()
            else:
                if cvt.CVThreading:
                    pass
                else:
                    pass
                    cvt.mode(Camera.modeSelect, img)
                    cvt.resume()
                try:
                    pass
                    img = cvt.elementDraw(img)
                except:
                    pass
            


            # img = cv2.imdecode(img)
            # encode as a jpeg image and return it
            if cv2.imencode('.jpg', img)[0]:
                yield cv2.imencode('.jpg', img)[1].tobytes()
            
