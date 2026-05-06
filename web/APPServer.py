#!/usr/bin/env/python
# File name   : APPServer.py
# Production  : Adeept_RaspTank_Metal
# Website     : www.adeept.com

import time
import move
import os
import info
import RPIservo
import functions
import robotLight
import switch
import Voltage
import Buzzer
import asyncio
import websockets
import json
import app
import subprocess

functionMode = 0
speed_set = 50
rad = 0.5
turnWiggle = 60

scGear = RPIservo.ServoCtrl()
scGear.moveInit()

P_sc = RPIservo.ServoCtrl()
P_sc.start()

T_sc = RPIservo.ServoCtrl()
T_sc.start()

H1_sc = RPIservo.ServoCtrl()
H1_sc.start()

H2_sc = RPIservo.ServoCtrl()
H2_sc.start()

G_sc = RPIservo.ServoCtrl()
G_sc.start()

modeSelect = 'PT'

init_pwm0 = scGear.initPos[0]
init_pwm1 = scGear.initPos[1]
init_pwm2 = scGear.initPos[2]
init_pwm3 = scGear.initPos[3]
init_pwm4 = scGear.initPos[4]

fuc = functions.Functions()
fuc.setup()
fuc.start()

player = Buzzer.Player()
player.start()

batteryMonitor = Voltage.BatteryLevelMonitor()
batteryMonitor.start()

curpath = os.path.realpath(__file__)
thisPath = "/" + os.path.dirname(curpath)

direction_command = 'no'
turn_command = 'no'

def servoPosInit():
    scGear.initConfig(0,init_pwm0,1)
    scGear.initConfig(1,init_pwm1,1)
    scGear.initConfig(2,init_pwm2,1)
    scGear.initConfig(3,init_pwm3,1)
    scGear.initConfig(4,init_pwm4,1)

def functionSelect(command_input, response):
    global functionMode
    if 'findColor' == command_input:
        flask_app.modeselect('findColor')
        flask_app.modeselectApp('APP')

    if 'motionGet' == command_input:
        flask_app.modeselect('watchDog')

    elif 'stopCV' == command_input:
        flask_app.modeselect('none')
        switch.switch(1,0)
        switch.switch(2,0)
        switch.switch(3,0)
        time.sleep(0.3)
        move.motorStop()

    elif 'KD' == command_input:
        servoPosInit()
        fuc.keepDistance()
    elif 'keepDistanceOff' == command_input:
        fuc.pause()
        move.motorStop()
        time.sleep(0.3)
        move.motorStop()

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

    elif 'Buzzer_Music' == command_input:
        player.start_playing()

    elif 'Buzzer_Music_Off' == command_input:
        player.pause()

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


def robotCtrl(command_input, response):
    global direction_command, turn_command
    clen = len(command_input.split())
    if 'forward' in command_input and clen == 2:
        direction_command = 'forward'
        move.move(speed_set, -1, "mid")
    
    elif 'backward' in command_input and clen == 2:
        direction_command = 'backward'
        move.move(speed_set, 1, "no")

    elif 'left' in command_input and clen == 2:
        turn_command = 'left'
        move.move(speed_set, 1, "left")


    elif 'right' in command_input and clen == 2:
        turn_command = 'right'
        move.move(speed_set, 1, "right")
        
    elif 'DTS' in command_input:
        direction_command = 'no'
        turn_command = 'no'
        move.motorStop()

    elif 'armUp' == command_input: #servo A
        H1_sc.singleServo(0, 1, 2)
    elif 'armDown' == command_input:
        H1_sc.singleServo(0,-1, 2)
    elif 'armStop' in command_input:
        H1_sc.stopWiggle()

    elif 'handUp' == command_input: # servo B
        H2_sc.singleServo(1, -1, 2)
    elif 'handDown' == command_input:
        H2_sc.singleServo(1, 1, 2)
    elif 'handStop' in command_input:
        H2_sc.stopWiggle()

    elif 'lookleft' == command_input.lower(): # servo C
        P_sc.singleServo(2, -1, 2)
    elif 'lookright' == command_input.lower():
        P_sc.singleServo(2, 1, 2)
    elif 'LRstop' in command_input:
        P_sc.stopWiggle()

    elif 'grab' == command_input: # servo D
        G_sc.singleServo(3, 1, 2)
    elif 'loose' == command_input:
        G_sc.singleServo(3, -1, 2)
    elif 'GLstop' in command_input:
        G_sc.stopWiggle()

    elif 'up' == command_input: # camera
        T_sc.singleServo(4, -1, 1)
    elif 'down' == command_input:
        T_sc.singleServo(4, 1, 1)
    elif 'UDStop' in command_input:
        T_sc.stopWiggle()

    elif 'home' == command_input.lower():
        H1_sc.moveServoInit([0])
        H2_sc.moveServoInit([1])
        P_sc.moveServoInit([2])
        G_sc.moveServoInit([3])
        T_sc.moveServoInit([4])

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
            # print('not A JSON')
            pass

        if not data:
            continue

        if isinstance(data,str):
            robotCtrl(data, response)

            switchCtrl(data, response)

            functionSelect(data, response)

            if 'get_info' == data:
                response['title'] = 'get_info'
                response['data'] = [info.get_cpu_tempfunc(), info.get_cpu_use(), info.get_ram_info(), batteryMonitor.get_battery_percentage()]

            if 'wsB' in data:
                try:
                    set_B=data.split()
                    speed_set = int(set_B[1])*10
                except:
                    pass

            #CVFL
            elif 'CVFL' == data:
                flask_app.modeselect('findlineCV')
                T_sc.singleServo(4, 1, 20)

            elif 'CVFLColorSet' in data:
                color = int(data.split()[1])
                flask_app.camera.colorSet(color)

            elif 'CVFLL1' in data:
                pos = int(int(data.split()[1]) / 100 * 480)
                flask_app.camera.linePosSet_1(pos)

            elif 'CVFLL2' in data:
                pos = int(int(data.split()[1]) / 100 *480)
                flask_app.camera.linePosSet_2(pos)


        elif(isinstance(data,dict)):
            color = data['data']
            if "title" in data and data['title'] == "findColorSet":
                flask_app.colorFindSetApp(color[0],color[1],color[2])
            elif data['lightMode'] == "breath":  
                WS2812.breath(color[0],color[1],color[2])
            elif data['lightMode'] == "flowing":
                WS2812.flowing(color[0],color[1],color[2])
            elif data['lightMode'] == "rainbow":
                WS2812.rainbow(color[0],color[1],color[2])
            elif data['lightMode'] == "police":
                WS2812.police()


        print(data)
        response = json.dumps(response)
        await websocket.send(response)
            
async def main_logic(websocket, path):
    await recv_msg(websocket)


if __name__ == '__main__':
    switch.switchSetup()
    switch.set_all_switch_off()

    global flask_app
    flask_app = app.webapp()
    flask_app.startthread()

    try:
        WS2812=robotLight.Adeept_SPI_LedPixel(14, 255)
        if WS2812.check_spi_state() != 0:
            WS2812.start()
            WS2812.breath(70,70,255)
        else:
            WS2812.led_close()
    except KeyboardInterrupt:
        WS2812.led_close()
        pass


    while 1:
        try:                  #Start server,waiting for client
            start_server = websockets.serve(main_logic, '0.0.0.0', 8888)
            asyncio.get_event_loop().run_until_complete(start_server)
            print('waiting for connection...')
            break
        except Exception as e:
            print(e)
            WS2812.set_all_led_color_data(0,0,0)
            WS2812.show()

    try:
        asyncio.get_event_loop().run_forever()
    except Exception as e:
        print(e)
        WS2812.set_all_led_color_data(0,0,0)
        WS2812.show()
        move.destroy()
