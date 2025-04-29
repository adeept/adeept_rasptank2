#!/usr/bin/env/python
# File name   : GUIServer.py
# Website     : www.Adeept.com
# Author      : Adeept
# Date		  : 2025/04/28

import time
import threading
import move
import os
import info
import RPIservo

import functions
import robotLight
import switch
import socket
import ast
import FPV
import json
import Voltage
mark_test = 0

speed_set = 25
turnWiggle = 60

direction_command = 'no'
turn_command = 'no'


scGear = RPIservo.ServoCtrl()
scGear.moveInit()
scGear.start()
batteryMonitor = Voltage.BatteryLevelMonitor()
batteryMonitor.start()


init_pwm = []
for i in range(8):
    init_pwm.append(scGear.initPos[i])
init_pwm0 = scGear.initPos[0]
init_pwm1 = scGear.initPos[1]
init_pwm2 = scGear.initPos[2]
init_pwm3 = scGear.initPos[3]
init_pwm4 = scGear.initPos[4]

fuc = functions.Functions()
fuc.setup()
fuc.start()

curpath = os.path.realpath(__file__)
thisPath = "/" + os.path.dirname(curpath)

def servoPosInit():
    scGear.initConfig(0,init_pwm[0],1)
    scGear.initConfig(1,init_pwm[1],1)
    scGear.initConfig(2,init_pwm[2],1)
    scGear.initConfig(3,init_pwm[3],1)
    scGear.initConfig(4,init_pwm[4],1)


def replace_num(initial,new_num):   #Call this function to replace data in '.txt' file
    global r
    newline=""
    str_num=str(new_num)
    with open(thisPath+"/RPIservo.py","r") as f:
        for line in f.readlines():
            if(line.find(initial) == 0):
                line = initial+"%s" %(str_num+"\n")
            newline += line
    with open(thisPath+"/RPIservo.py","w") as f:
        f.writelines(newline)


def FPV_thread():
    global fpv
    fpv=FPV.FPV()
    fpv.capture_thread(addr[0])


def ap_thread():
    os.system("sudo create_ap wlan0 eth0 Adeept_Robot 12345678")


def functionSelect(command_input, response):
    if 'findColor' == command_input:
        fpv.FindColor(1)
        tcpCliSock.send(('FindColor').encode())

    elif 'motionGet' == command_input:
        fpv.WatchDog(1)
        tcpCliSock.send(('WatchDog').encode())

    elif 'stopCV' == command_input:
        fpv.FindColor(0)
        fpv.WatchDog(0)
        FPV.FindLineMode = 0
        time.sleep(0.5)
        move.motorStop()
        switch.switch(1,0)
        switch.switch(2,0)
        switch.switch(3,0)

    elif 'police' == command_input:
        ws2812.police()

    elif 'policeOff' == command_input:
        ws2812.breath(70,70,255)

    elif 'automatic' == command_input:
        fuc.automatic()

    elif 'automaticOff' == command_input:
        fuc.pause()
        time.sleep(0.5)
        move.motorStop()

    elif 'trackLine' == command_input:
        servoPosInit()
        fuc.trackLine()

    elif 'trackLineOff' == command_input:
        fuc.pause()


def switchCtrl(command_input):
    if 'Switch_1_on' in command_input:
        switch.switch(1,1)

    elif 'Switch_1_off' in command_input:
        switch.switch(1,0)

    elif 'Switch_2_on' in command_input:
        switch.switch(2,1)

    elif 'Switch_2_off' in command_input:
        switch.switch(2,0)

    elif 'Switch_3_on' in command_input:
        switch.switch(3,1)

    elif 'Switch_3_off' in command_input:
        switch.switch(3,0) 


def robotCtrl(command_input):
    global direction_command, turn_command
    if 'forward' == command_input:
        direction_command = 'forward'
        move.move(speed_set, 1, "mid")
    
    elif 'backward' == command_input:
        direction_command = 'backward'
        move.move(speed_set, -1, "mid")

    elif 'DS' in command_input:
        direction_command = 'no'
        move.motorStop()

    elif 'left' == command_input:
        turn_command = 'left'
        move.move(speed_set, 1, "left")

    elif 'right' == command_input:
        turn_command = 'right'
        move.move(speed_set, 1, "right")

    elif 'TS' in command_input:
        turn_command = 'no'
        move.motorStop()

    elif 'armUp' == command_input:
        scGear.singleServo(0,  1, 2)

    elif 'armDown' == command_input:
        scGear.singleServo(0, -1, 2)

    elif 'armStop' in command_input:
        scGear.stopWiggle()

    elif 'handUp' == command_input:
        scGear.singleServo(1, -1, 2)

    elif 'handDown' == command_input:
        scGear.singleServo(1,  1, 2)

    elif 'handStop' in command_input:
        scGear.stopWiggle()

    elif 'lookleft' == command_input:
        scGear.singleServo(2,  1, 2)

    elif 'lookright' == command_input:
        scGear.singleServo(2, -1, 2)

    elif 'LRstop' in command_input:
        scGear.stopWiggle()

    elif 'grab' == command_input:
        scGear.singleServo(3,  1, 2)

    elif 'loose' == command_input:
        scGear.singleServo(3, -1, 2)

    elif 'GLstop' == command_input:
        scGear.stopWiggle()

    elif 'up' == command_input: # camera
        scGear.singleServo(4, -1, 1)
    elif 'down' == command_input:
        scGear.singleServo(4,  1, 1)
    elif 'UDstop' in command_input:
        scGear.stopWiggle()


    elif 'home' == command_input:
        scGear.moveServoInit([0])
        scGear.moveServoInit([1])
        scGear.moveServoInit([2])
        scGear.moveServoInit([3])
        scGear.moveServoInit([4])


def configPWM(command_input):
    global init_pwm0, init_pwm1, init_pwm2, init_pwm3, init_pwm4

    if 'SiLeft' in command_input:
        numServo = int(command_input[7:])
        if numServo == 0:
            init_pwm0 -= 1
            scGear.setPWM(0,init_pwm0)
        elif numServo == 1:
            init_pwm1 -= 1
            scGear.setPWM(1,init_pwm1)
        elif numServo == 2:
            init_pwm2 -= 1
            scGear.setPWM(2,init_pwm2)
        elif numServo == 3:
            init_pwm3 -= 1
            scGear.setPWM(3,init_pwm3)
        elif numServo == 4:
            init_pwm4 -= 1
            scGear.setPWM(4,init_pwm4)

    if 'SiRight' in command_input:
        numServo = int(command_input[8:])
        if numServo == 0:
            print(numServo)
            init_pwm0 += 1
            scGear.setPWM(0,init_pwm0)
        elif numServo == 1:
            init_pwm1 += 1
            scGear.setPWM(1,init_pwm1)
        elif numServo == 2:
            init_pwm2 += 1
            scGear.setPWM(2,init_pwm2)
        elif numServo == 3:
            init_pwm3 += 1
            scGear.setPWM(3,init_pwm3)
        elif numServo == 4:
            init_pwm4 += 1
            scGear.setPWM(4,init_pwm4)

    if 'PWMMS' in command_input:
        init_pwm0 = 90
        for i in range(5):
            scGear.moveAngle(i, 0)
            scGear.nowPos[i] = 90

    if 'PWMINIT' == command_input:
        servoPosInit()
    elif 'PWMD' in command_input:
        init_pwm0 = 90
        for i in range(5):
            scGear.moveAngle(i, 0)
            scGear.nowPos[i] = 90



def wifi_check():
    global mark_test
    try:
        time.sleep(3)
        s =socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        s.connect(("1.1.1.1",80))
        ipaddr_check=s.getsockname()[0]
        s.close()
        print(ipaddr_check)
        mark_test = 1  
    except:
        if mark_test == 1:
            mark_test = 0
            move.destroy()      # motor stop.
            scGear.moveInit()   # servo  back initial position.

        ap_threading=threading.Thread(target=ap_thread)   #Define a thread for data receiving
        ap_threading.setDaemon(True)                          #'True' means it is a front thread,it would close when the mainloop() closes
        ap_threading.start()                                  #Thread starts
        ws2812.set_all_led_color_data(35,255,35)
        ws2812.show()



def recv_msg(tcpCliSock):
    global speed_set
    move.setup()

    while True: 
        response = {
            'status' : 'ok',
            'title' : '',
            'data' : None
        }


        data = tcpCliSock.recv(BUFSIZ).decode()
        print(data)

        if not data:
            continue


        if isinstance(data,str):
            robotCtrl(data)

            switchCtrl(data)

            functionSelect(data, response)

            configPWM(data)

            if 'get_info' == data:
                response['title'] = 'get_info'
                response['data'] = [info.get_cpu_tempfunc(), info.get_cpu_use(), info.get_ram_info()]
            if 'wsB' in data:
                try:
                    set_B=data.split()
                    speed_set = int(set_B[1])
                except:
                    pass
            elif 'CVFL' == data:
                FPV.FindLineMode = 1
                tcpCliSock.send(('CVFL_on').encode())


            elif 'CVFLColorSet' in data:
                color = int(data.split()[1])
                FPV.lineColorSet = color

            elif 'CVFLL1' in data:
                try:
                    set_lip1=data.split()
                    lip1_set = int(set_lip1[1])
                    FPV.linePos_1 = lip1_set
                except:
                    pass

            elif 'CVFLL2' in data:
                try:
                    set_lip2=data.split()
                    lip2_set = int(set_lip2[1])
                    FPV.linePos_2 = lip2_set
                except:
                    pass

            elif 'CVFLSP' in data:
                try:
                    set_err=data.split()
                    err_set = int(set_lip1[1])
                    FPV.findLineError = err_set
                except:
                    pass

            elif 'findColorSet' in data:
                try:
                    command_dict = ast.literal_eval(data)
                    if 'data' in command_dict and len(command_dict['data']) == 3:
                        r, g, b = command_dict['data']
                        fpv.colorFindSet(b, g, r)
                        print(f"color: r={r}, g={g}, b={b}")
                except (SyntaxError, ValueError):
                    print("The received string format is incorrect and cannot be parsed.")
        else:
            pass
        response = json.dumps(response)
        tcpCliSock.sendall(response.encode())

def test_Network_Connection():
    while True:
        try:
            s =socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            s.connect(("1.1.1.1",80))
            s.close()
        except:
            move.destroy()
        
        time.sleep(0.5)

if __name__ == '__main__':
    switch.switchSetup()
    switch.set_all_switch_off()                                  

    ws2812=robotLight.Adeept_SPI_LedPixel(16, 255)
    try:
        if ws2812.check_spi_state() != 0:
            ws2812.start()
            ws2812.breath(70,70,255)                       # Set the brightness of lights.
    except:
        ws2812.led_close()
        pass

    HOST = ''
    PORT = 10223                              #Define port serial 
    BUFSIZ = 1024                             #Define buffer size
    ADDR = (HOST, PORT)

   
    wifi_check()
    try:                  #Start server,waiting for client
        tcpSerSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcpSerSock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        tcpSerSock.bind(ADDR)
        tcpSerSock.listen(5)                   
        print("Waiting for client connection")
        tcpCliSock, addr = tcpSerSock.accept()
        print("Connected to the client :" + str(addr))
        fps_threading=threading.Thread(target=FPV_thread)         #Define a thread for FPV and OpenCV
        fps_threading.setDaemon(True)                             #'True' means it is a front thread,it would close when the mainloop() closes
        fps_threading.start()   
        recv_msg(tcpCliSock)  
    except Exception as e:
        print(e)
        ws2812.set_all_led_color_data(0,0,0)
        ws2812.show()



