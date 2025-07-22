#!/usr/bin/python3
# coding=utf-8
# File name   : setup.py
# Author      : Devin

import os
import time

username = os.popen("echo ${SUDO_USER:-$(who -m | awk '{ print $1 }')}").readline().strip() # pi
user_home = os.popen('getent passwd %s | cut -d: -f 6'%username).readline().strip()         # home
 
curpath = os.path.realpath(__file__)
thisPath = "/" + os.path.dirname(curpath)

print(thisPath)

def replace_num(file,initial,new_num):
    newline=""
    str_num=str(new_num)
    with open(file,"r") as f:
        for line in f.readlines():
            if(line.find(initial) == 0):
                line = (str_num+'\n')
            newline += line
    with open(file,"w") as f:
        f.writelines(newline)


def run_command(cmd=""):
    import subprocess
    p = subprocess.Popen(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    result = p.stdout.read().decode('utf-8')
    status = p.poll()
    return status, result

def check_rpi_model():
    _, result = run_command("cat /proc/device-tree/model |awk '{print $3}'")
    result = result.strip()
    if result == '3':
        return int(3)
    elif result == '4':
        return int(4)
    else:
        return None

def check_raspbain_version():
    _, result = run_command("cat /etc/debian_version|awk -F. '{print $1}'")
    return int(result.strip())

def check_python_version():
    import sys
    major = int(sys.version_info.major)
    minor = int(sys.version_info.minor)
    micro = int(sys.version_info.micro)
    return major, minor, micro

def check_os_bit():
    '''
    # import platform
    # machine_type = platform.machine() 
    latest bullseye uses a 64-bit kernel
    This method is no longer applicable, the latest raspbian will uses 64-bit kernel 
    (kernel 6.1.x) by default, "uname -m" shows "aarch64", 
    but the system is still 32-bit.
    '''
    _ , os_bit = run_command("getconf LONG_BIT")
    return int(os_bit)


commands_apt = [
"sudo apt-get update",
"sudo apt-get install python3-gpiozero python3-pigpio",
"sudo apt-get install  -y python3-opencv",
"sudo apt-get install -y python3-pyqt5 python3-opengl",
"sudo apt-get install -y python3-picamera2",
"sudo apt-get install -y python3-picamera2 --no-install-recommends",
"sudo apt-get install -y python3-opencv",
"sudo apt-get install -y opencv-data",
"sudo apt-get install -y python3-pyaudio"
]
mark_apt = 0
for x in range(3):
    for command in commands_apt:
        if os.system(command) != 0:
            print("Error running installation step apt")
            mark_apt = 1
    if mark_apt == 0:
        break

commands_pip_1 = [
"sudo pip3 install adafruit-circuitpython-motor",
"sudo pip3 install adafruit-circuitpython-pca9685",
"sudo pip3 install flask",
"sudo pip3 install flask_cors",
"sudo pip3 install numpy",
"sudo pip3 install pyzmq",
"sudo pip3 install imutils zmq pybase64 psutil",
"sudo pip3 install websockets==13.0",
"sudo pip3 install rpi_ws281x",
"sudo pip3 install adafruit-circuitpython-ads7830"
]
commands_pip_2 = [
"sudo pip3 install adafruit-circuitpython-motor --break-system-packages",
"sudo pip3 install adafruit-circuitpython-pca9685 --break-system-packages",
"sudo pip3 install flask --break-system-packages",
"sudo pip3 install flask_cors --break-system-packages",
"sudo pip3 install numpy --break-system-packages",
"sudo pip3 install pyzmq --break-system-packages",
"sudo pip3 install imutils zmq pybase64 psutil --break-system-packages",
"sudo pip3 install websockets==13.0 --break-system-packages",
"sudo pip3 install rpi_ws281x --break-system-packages",
"sudo pip3 install adafruit-circuitpython-ads7830 --break-system-packages"
]
mark_pip = 0
OS_version = check_raspbain_version()
if OS_version <= 11:
    for x in range(3):
        for command in commands_pip_1:
            if os.system(command) != 0:
                print("Error running installation step pip")
                mark_pip = 1
        if mark_pip == 0:
            break
else:
    for x in range(3):
        for command in commands_pip_2:
            if os.system(command) != 0:
                print("Error running installation step pip")
                mark_pip = 1
        if mark_pip == 0:
            break

commands_3 = [
    "cd ~",
    "sudo git clone https://github.com/oblique/create_ap",
    "cd create_ap && sudo make install",
    # "cd //home/pi/create_ap && sudo make install",
    "sudo apt-get install -y util-linux procps hostapd iproute2 iw haveged dnsmasq"
]

mark_3 = 0
for x in range(3):
    for command in commands_3:
        if os.system(command) != 0:
            print("Error running installation step 3")
            mark_2 = 1
    if mark_3 == 0:
        break



try:
    replace_num("/boot/config.txt", '#dtparam=i2c_arm=on','dtparam=i2c_arm=on\nstart_x=1\n')
except:
    print('Error updating boot config to enable i2c. Please try again.')



try:
    os.system("sudo touch /"+ user_home +"/startup.sh")
    with open("/"+ user_home +"/startup.sh",'w') as file_to_write:
        #you can choose how to control the robot
        file_to_write.write("#!/bin/sh\nsleep 5\nsudo python3 " + thisPath + "/web/webServer_HAT_V3.1.py")
except:
    pass

os.system("sudo chmod 777 /"+ user_home +"/startup.sh")


if not os.path.exists("/etc/rc.local"):
    print('/etc/rc.local does not exist. It is required to run the program when the raspberry pi starts. \nHowever it is not required for function. \nWould you like to create a /etc/rc.local/ file (recommended)?')
    if input('(y/n): ').strip().lower() == "y":
        os.system("sudo touch /etc/rc.local")
        os.system("sudo chown root:root /etc/rc.local")
        os.system("sudo chmod 755 /etc/rc.local")
        try:
            with open("/etc/rc.local", 'w') as file_to_write:
                file_to_write.write('#!/bin/sh -e\n/' + user_home + '/startup.sh start\nexit 0')
        except:
            print('Error: writing /etc/rc.local/ failed.')
    else:
        print("Program setup without /etc/rc.local complete. \nNote: you will have to run startup.sh manually to set servos for mechanical assembly.")
else: #there is /etc/rc.local
    try:
        replace_num('/etc/rc.local','fi','fi\n/'+ user_home +'/startup.sh start')
        print('/etc/rc.local setup complete. After turning the Raspberry Pi on again, the Raspberry Pi will automatically run the program to set the servos port signal to turn the servos to the middle position, which is convenient for mechanical assembly.')
    except:
        print('Error adding to /startup.sh')

print('The program in Raspberry Pi has been installed, disconnected and restarted. \nYou can now power off the Raspberry Pi to install the camera and driver board (Robot HAT). \nAfter turning on again, the Raspberry Pi will automatically run the program to set the servos port signal to turn the servos to the middle position, which is convenient for mechanical assembly.')
print('restarting...')
os.system("sudo reboot")
