#!/usr/bin/env/python
# File name   : server.py
# Production  : picar-b
# Website     : www.adeept.com
# Author      : devin

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

#websocket
import asyncio
import websockets

import json
import app


# List of rainbow colors (ROYGBIV)
COLORS = {
    'black': (0, 0, 0),
    'red': (255, 0, 0),
    'orange': (255, 127, 0),
    'yellow': (255, 255, 0),
    'green': (0, 255, 0),
    'blue': (0, 0, 255),
    'indigo': (75, 0, 130),
    'violet': (148, 0, 211),
    'white': (255, 255, 255),
}


OLED_connection = 0

functionMode = 0
speed_set = 100
rad = 0.5
turnWiggle = 60

##################################
####### Servo Controlers  ########
##################################
SERVO_CTRL = RPIservo.ServoCtrl()
ARM = RPIservo.ServoCtrlThread("ARM", SERVO_CTRL, 0, 90, 1)
HAND = RPIservo.ServoCtrlThread("HAND", SERVO_CTRL, 1, 90, -1)
WRIST = RPIservo.ServoCtrlThread("WRIST", SERVO_CTRL, 2, 90, 1)
# 3 is detroyed using 5 instead
CLAW = RPIservo.ServoCtrlThread("CLAW", SERVO_CTRL, 5, 90, 1)
CAMERA = RPIservo.ServoCtrlThread("CAMERA", SERVO_CTRL, 4, 90, -1)
SERVOS = [ARM, HAND, WRIST, CLAW, CAMERA]

##################################
######## Motor Controler #########
##################################
MOVEMENT = move.MovementCtrlThread(-1, -1)

##################################
######### Led Controler ##########
##################################
LED_CTRL = robotLight.LedCtrl(count = 14, bright = 20, color = COLORS['yellow'])

controls = {
  # Servos
  'armUp'    : ARM.clockwise,
  'armDown'  : ARM.anticlockwise,
  'armStop'  : ARM.stopWiggle,
  'handUp'   : HAND.clockwise,
  'handDown' : HAND.anticlockwise,
  'handStop' : HAND.stopWiggle,
  'lookleft' : WRIST.clockwise,
  'lookright': WRIST.anticlockwise,
  'LRstop'   : WRIST.stopWiggle,
  'grab'     : CLAW.clockwise,
  'loose'    : CLAW.anticlockwise,
  'GLstop'   : CLAW.stopWiggle,
  'up'       : CAMERA.clockwise,
  'down'     : CAMERA.anticlockwise,
  'UDstop'   : CAMERA.stopWiggle,
  'home'     : lambda: servoPosInit(),
  # Motors
  'forward'  : MOVEMENT.forward,
  'backward' : MOVEMENT.backward,
  'left'     : MOVEMENT.left,
  'right'    : MOVEMENT.right,
  'DS'       : MOVEMENT.stop,
  'TS'       : MOVEMENT.stop
}

# modeSelect = 'none'
modeSelect = 'PT'

fuc = functions.Functions()
fuc.setup()
fuc.start()

curpath = os.path.realpath(__file__)
thisPath = "/" + os.path.dirname(curpath)

direction_command = 'no'
turn_command = 'no'

def servoPosInit():
    ARM.reset()
    HAND.reset()
    WRIST.reset()
    CAMERA.reset()
    CLAW.reset()


# def replace_num(initial,new_num):   #Call this function to replace data in '.txt' file
    # global r
    # newline=""
    # str_num=str(new_num)
    # with open(thisPath+"/RPIservo.py","r") as f:
        # for line in f.readlines():
            # if(line.find(initial) == 0):
                # line = initial+"%s" %(str_num+"\n")
            # newline += line
    # with open(thisPath+"/RPIservo.py","w") as f:
        # f.writelines(newline)


# def FPV_thread():
#     global fpv
#     fpv=FPV.FPV()
#     fpv.capture_thread(addr[0])


def ap_thread():
    os.system("sudo create_ap wlan0 eth0 Adeept_Robot 12345678")


def functionSelect(command_input, response):
    global functionMode

    if 'findColor' == command_input:
        flask_app.modeselect('findColor')

    if 'motionGet' == command_input:
        flask_app.modeselect('watchDog')

    elif 'stopCV' == command_input:
        flask_app.modeselect('none')
        switch.switch(1,0)
        switch.switch(2,0)
        switch.switch(3,0)
        move.motorStop()

    elif 'KD' == command_input:
        servoPosInit()
        fuc.keepDistance()
        if WS2812_mark:
            WS2812.police()

    elif 'police' == command_input:
        WS2812.police()

    elif 'policeOff' == command_input:
        WS2812.breath(70,70,255)

    elif 'automatic' == command_input:
        if modeSelect == 'PT':
            fuc.automatic()
        else:
            fuc.pause()

    elif 'automaticOff' == command_input:
        if WS2812_mark:
            WS2812.pause()
        fuc.pause()
        move.motorStop()
        time.sleep(0.3)
        move.motorStop()

    elif 'trackLine' == command_input:
        servoPosInit()
        fuc.trackLine()

    elif 'trackLineOff' == command_input:
        fuc.pause()
        move.motorStop()

    elif 'steadyCamera' == command_input:
        fuc.steady(CAMERA.lastPos[2])

    elif 'steadyCameraOff' == command_input:
        fuc.pause()
        move.motorStop()



def switchCtrl(command_input, response):
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

def configPWM(command_input, response):
    
    if 'SiLeft' in command_input:
        numServo = int(command_input[7:])
        SERVOS[numServo].derementPwm()

    if 'SiRight' in command_input:
        numServo = int(command_input[8:])
        SERVOS[numServo].incrementPwm()

    if 'PWMMS' in command_input:
        numServo = int(command_input[6:])
        SERVOS[numServo].initialize()

    if 'PWMINIT' == command_input:
        print(init_pwm1)
        servoPosInit()

    elif 'PWMD' == command_input:
        servoPosInit()

def update_code():
    # Update local to be consistent with remote
    projectPath = thisPath[:-7]
    with open(f'{projectPath}/config.json', 'r') as f1:
        config = json.load(f1)
        if not config['production']:
            print('Update code')
            # Force overwriting local code
            if os.system(f'cd {projectPath} && sudo git fetch --all && sudo git reset --hard origin/master && sudo git pull') == 0:
                print('Update successfully')
                print('Restarting...')
                os.system('sudo reboot')
            
def wifi_check():
    try:
        s =socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        s.connect(("1.1.1.1",80))
        ipaddr_check=s.getsockname()[0]
        s.close()
        print(ipaddr_check)
    except:
        ap_threading=threading.Thread(target=ap_thread)   #Define a thread for data receiving
        ap_threading.setDaemon(True)                          #'True' means it is a front thread,it would close when the mainloop() closes
        ap_threading.start()                                  #Thread starts
        if WS2812_mark:
            WS2812.set_all_led_color_data(0,16,50)
            WS2812.show()
        time.sleep(1)

async def check_permit(websocket):
    while True:
        recv_str = await websocket.recv()
        cred_dict = recv_str.split(":")
        if cred_dict[0] == "admin" and cred_dict[1] == "123456":
            response_str = "congratulation, you have connect with server\r\nnow, you can do something else"
            await websocket.send(response_str)
            return True
        else:
            response_str = "sorry, the username or password is wrong, please submit again"
            await websocket.send(response_str)

async def recv_msg(websocket):
    global speed_set, modeSelect

    while True: 
        response = {
            'status' : 'ok',
            'title' : '',
            'data' : None
        }

        data = ''
        data = await websocket.recv()
        try:
            data = json.loads(data)
        except Exception as e:
            print('not A JSON')

        if not data:
            continue

        if isinstance(data,str):
            if data in controls:
                controls[data]()

            switchCtrl(data, response)

            functionSelect(data, response)

            configPWM(data, response)

            if 'get_info' == data:
                response['title'] = 'get_info'
                response['data'] = [info.get_cpu_tempfunc(), info.get_cpu_use(), info.get_ram_info()]

            if 'wsB' in data:
                try:
                    MOVEMENT.setSpeed(int(data.split()[1]))
                except:
                    pass

            #CVFL
            elif 'CVFL' == data:
                flask_app.modeselect('findlineCV')

            elif 'CVFLColorSet' in data:
                color = int(data.split()[1])
                flask_app.camera.colorSet(color)

            elif 'CVFLL1' in data:
                pos = int(data.split()[1])
                flask_app.camera.linePosSet_1(pos)

            elif 'CVFLL2' in data:
                pos = int(data.split()[1])
                flask_app.camera.linePosSet_2(pos)

            elif 'CVFLSP' in data:
                err = int(data.split()[1])
                flask_app.camera.errorSet(err)

        elif(isinstance(data,dict)):
            if data['title'] == "findColorSet":
                color = data['data']
                flask_app.colorFindSet(color[0],color[1],color[2])

        print(data)
        response = json.dumps(response)
        await websocket.send(response)

async def main_logic(websocket, path):
    await check_permit(websocket)
    await recv_msg(websocket)

if __name__ == '__main__':
    switch.switchSetup()
    switch.set_all_switch_off()
    
    #move.setup()

    HOST = ''
    PORT = 10223                              #Define port serial 
    BUFSIZ = 1024                             #Define buffer size
    ADDR = (HOST, PORT)

    global flask_app
    flask_app = app.webapp()
    flask_app.startthread()
    try:
        LED_CTRL.set_all_led_rgb(COLORS['orange'])
        while  1:
            wifi_check()
            try:                  #Start server,waiting for client
                start_server = websockets.serve(main_logic, '0.0.0.0', 8888)
                asyncio.get_event_loop().run_until_complete(start_server)
                print('waiting for connection...')
                break
            except Exception as e:
                LED_CTRL.set_all_led_rgb(COLORS['red'])
                print('connection error...')
                print(e)
                
                
        LED_CTRL.set_all_led_rgb(COLORS['green'])
        print('start main loop...')
        try:
            asyncio.get_event_loop().run_forever()
        except Exception as e:
            print(e)
            LED_CTRL.set_all_led_rgb(COLORS['red'])
            move.destroy()
    except KeyboardInterrupt:
        print('program stopped...')
        move.destroy()
        LED_CTRL.stop()
        exit(0);
