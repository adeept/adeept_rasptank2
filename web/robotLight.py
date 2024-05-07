#!/usr/bin/env python3
# File name   : servo.py
# Description : Control lights
# Author	  : William
# Date		: 2019/02/23
import time
import sys
from gpiozero import PWMOutputDevice as PWM
from rpi_ws281x import *
import threading

def check_rpi_model():
    _, result = run_command("cat /proc/device-tree/model |awk '{print $3}'")
    result = result.strip()
    if result == '3':
        return 3
    elif result == '4':
        return 4
    elif result == '5':
        return 5
    else:
        return None

def run_command(cmd=""):
    import subprocess
    p = subprocess.Popen(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    result = p.stdout.read().decode('utf-8')
    status = p.poll()
    return status, result


def map(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class RobotWS2812(threading.Thread):
    def __init__(self, *args, **kwargs):
        self.LED_COUNT	  	= 16	  # Number of LED pixels.
        self.LED_PIN		= 12	  # GPIO pin connected to the pixels (18 uses PWM!).
        self.LED_FREQ_HZ	= 800000  # LED signal frequency in hertz (usually 800khz)
        self.LED_DMA		= 10	  # DMA channel to use for generating signal (try 10)
        self.LED_BRIGHTNESS = 255	 # Set to 0 for darkest and 255 for brightest
        self.LED_INVERT	 = False   # True to invert the signal (when using NPN transistor level shift)
        self.LED_CHANNEL	= 0	   # set to '1' for GPIOs 13, 19, 41, 45 or 53

        self.colorBreathR = 0
        self.colorBreathG = 0
        self.colorBreathB = 0
        self.breathSteps = 10


        self.lightMode = 'none'		#'none' 'police' 'breath'


        # Create NeoPixel object with appropriate configuration.
        self.strip = Adafruit_NeoPixel(self.LED_COUNT, self.LED_PIN, self.LED_FREQ_HZ, self.LED_DMA, self.LED_INVERT, self.LED_BRIGHTNESS, self.LED_CHANNEL)
        # Intialize the library (must be called once before other functions).
        self.strip.begin()

        super(RobotWS2812, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.clear()

    # Define functions which animate LEDs in various ways.
    def setColor(self, R, G, B):
        """Wipe color across display a pixel at a time."""
        color = Color(int(R),int(G),int(B))
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            self.strip.show()

    def setSomeColor(self, R, G, B, ID):
        color = Color(int(R),int(G),int(B))
        #print(int(R),'  ',int(G),'  ',int(B))
        for i in ID:
            self.strip.setPixelColor(i, color)
            self.strip.show()

    def pause(self):
        self.lightMode = 'none'
        self.setColor(0,0,0)
        self.__flag.clear()

    def resume(self):
        self.__flag.set()


    def police(self):
        self.lightMode = 'police'
        self.resume()


    def policeProcessing(self):
        while self.lightMode == 'police':
            for i in range(0,3):
                self.setSomeColor(0,0,255,[0,1,2,3,4,5,6,7,8,9,10,11])
                self.blue()
                time.sleep(0.05)
                self.setSomeColor(0,0,0,[0,1,2,3,4,5,6,7,8,9,10,11])
                self.both_off()
                time.sleep(0.05)
            if self.lightMode != 'police':
                break
            time.sleep(0.1)
            for i in range(0,3):
                self.setSomeColor(255,0,0,[0,1,2,3,4,5,6,7,8,9,10,11])
                self.red()
                time.sleep(0.05)
                self.setSomeColor(0,0,0,[0,1,2,3,4,5,6,7,8,9,10,11])
                self.both_off()
                time.sleep(0.05)
            time.sleep(0.1)


    def breath(self, R_input, G_input, B_input):
        self.lightMode = 'breath'
        self.colorBreathR = R_input
        self.colorBreathG = G_input
        self.colorBreathB = B_input
        self.resume()


    def breathProcessing(self):
        while self.lightMode == 'breath':
            for i in range(0,self.breathSteps):
                if self.lightMode != 'breath':
                    break
                self.setColor(self.colorBreathR*i/self.breathSteps, self.colorBreathG*i/self.breathSteps, self.colorBreathB*i/self.breathSteps)
                time.sleep(0.03)
            for i in range(0,self.breathSteps):
                if self.lightMode != 'breath':
                    break
                self.setColor(self.colorBreathR-(self.colorBreathR*i/self.breathSteps), self.colorBreathG-(self.colorBreathG*i/self.breathSteps), self.colorBreathB-(self.colorBreathB*i/self.breathSteps))
                time.sleep(0.03)

    def lightChange(self):
        if self.lightMode == 'none':
            self.pause()
        elif self.lightMode == 'police':
            self.policeProcessing()
        elif self.lightMode == 'breath':
            self.breathProcessing()


    def run(self):
        while 1:
            self.__flag.wait()
            self.lightChange()
            pass



class RobotLight(threading.Thread):
    def __init__(self, *args, **kwargs):

        self.left_R = 7
        self.left_G = 0
        self.left_B = 8

        self.right_R = 5
        self.right_G = 6
        self.right_B = 1
        
        self.Left_G = PWM(pin=self.left_R,initial_value=1.0, frequency=2000)
        self.Left_B = PWM(pin=self.left_G,initial_value=1.0, frequency=2000)
        self.Left_R = PWM(pin=self.left_B,initial_value=1.0, frequency=2000)
        
        self.Right_G = PWM(pin=self.right_R,initial_value=1.0, frequency=2000)
        self.Right_B = PWM(pin=self.right_G,initial_value=1.0, frequency=2000)
        self.Right_R = PWM(pin=self.right_B,initial_value=1.0, frequency=2000)

        # super(RobotLight, self).__init__(*args, **kwargs)
        # self.__flag = threading.Event()
        # self.__flag.clear()

    # def setColorLED(self,LED_num, col):   # For example : col = 0x112233
    #     if LED_num ==1 :
    #         R_val = (col & 0xff0000) >> 16
    #         G_val = (col & 0x00ff00) >> 8
    #         B_val = (col & 0x0000ff) >> 0
            
    #         R_val = map(R_val, 0, 255, 0, 1.00)
    #         G_val = map(G_val, 0, 255, 0, 1.00)
    #         B_val = map(B_val, 0, 255, 0, 1.00)

    #         self.Left_R.value = 1.0-R_val
    #         self.Left_G.value = 1.0-G_val
    #         self.Left_B.value = 1.0-B_val
    #     elif LED_num == 2:
    #         R_val = (col & 0xff0000) >> 16
    #         G_val = (col & 0x00ff00) >> 8
    #         B_val = (col & 0x0000ff) >> 0
            
    #         R_val = map(R_val, 0, 255, 0, 1.00)
    #         G_val = map(G_val, 0, 255, 0, 1.00)
    #         B_val = map(B_val, 0, 255, 0, 1.00)

    #         self.Right_R.value = 1.0-R_val
    #         self.Right_G.value = 1.0-G_val
    #         self.Right_B.value = 1.0-B_val

    def setRGBColor(self,LED_num, R,G,B):   # For example : (1,  255,0,0)
        if LED_num ==1 :
            R_val = map(R, 0, 255, 0, 1.00)
            G_val = map(G, 0, 255, 0, 1.00)
            B_val = map(B, 0, 255, 0, 1.00)
            self.Left_R.value = 1.0-R_val
            self.Left_G.value = 1.0-G_val
            self.Left_B.value = 1.0-B_val

        elif LED_num == 2:
            R_val = map(R, 0, 255, 0, 1.00)
            G_val = map(G, 0, 255, 0, 1.00)
            B_val = map(B, 0, 255, 0, 1.00)
            self.Right_R.value = 1.0-R_val
            self.Right_G.value = 1.0-G_val
            self.Right_B.value = 1.0-B_val

    def both_on(self,R,G,B):
        self.setRGBColor(1, R,G,B)
        self.setRGBColor(2, R,G,B)

    def RGB_left_on(self,R,G,B):
        self.setRGBColor(1, R,G,B)
        self.setRGBColor(2, 0,0,0)

    def RGB_right_on(self,R,G,B):
        self.setRGBColor(1, 0,0,0)
        self.setRGBColor(2, R,G,B)

    def both_off(self):
        self.setRGBColor(1, 0,0,0)
        self.setRGBColor(2, 0,0,0)

    # def pause(self):
    #     self.lightMode = 'none'
    #     self.setColor(0,0,0)
    #     self.__flag.clear()

    # def resume(self):
    #     self.__flag.set()



    # def frontLight(self, switch):
    #     if switch == 'on':
    #         GPIO.output(6, GPIO.HIGH)
    #         GPIO.output(13, GPIO.HIGH)
    #     elif switch == 'off':
    #         GPIO.output(5,GPIO.LOW)
    #         GPIO.output(13,GPIO.LOW)


    # def switch(self, port, status):
    #     if port == 1:
    #         if status == 1:
    #             GPIO.output(5, GPIO.HIGH)
    #         elif status == 0:
    #             GPIO.output(5,GPIO.LOW)
    #         else:
    #             pass
    #     elif port == 2:
    #         if status == 1:
    #             GPIO.output(6, GPIO.HIGH)
    #         elif status == 0:
    #             GPIO.output(6,GPIO.LOW)
    #         else:
    #             pass
    #     elif port == 3:
    #         if status == 1:
    #             GPIO.output(13, GPIO.HIGH)
    #         elif status == 0:
    #             GPIO.output(13,GPIO.LOW)
    #         else:
    #             pass
    #     else:
    #         print('Wrong Command: Example--switch(3, 1)->to switch on port3')


    # def set_all_switch_off(self):
    #     self.switch(1,0)
    #     self.switch(2,0)
    #     self.switch(3,0)


    # def headLight(self, switch):
    #     if switch == 'on':
    #         GPIO.output(5, GPIO.HIGH)
    #     elif switch == 'off':
    #         GPIO.output(5,GPIO.LOW)



if __name__ == '__main__':
    # try:
    #     value = 1
    #     rpi_model = check_rpi_model()
    #     if rpi_model == 5:
    #         print("\033[1;33m WS2812 officially does not support Raspberry Pi 5 for the time being, and the WS2812 LED cannot be used on Raspberry Pi 5.\033[0m")
    #         value = 0
    #     else:
    #         WS2812=RobotWS2812()
    #         WS2812.start()
    #     while value!= 0:
    #         WS2812.run()
    # except KeyboardInterrupt:
    #   WS2812.setColor(0, 0, 0)

    # try:
    RL = RobotLight()
    RL.both_on(255,0,0)
    # except Exception as e:
    #     print(e)

    # RL.breath(70,70,255)
    # time.sleep(15)
    # RL.pause()
    # # RL.frontLight('off')
    # time.sleep(2)
    # RL.police()