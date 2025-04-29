#!/usr/bin/env/python
# File name   : GUI.py
# Website     : www.Adeept.com
# Author      : Adeept
# Date		  : 2025/04/28

from socket import *
import sys
import time
import threading as thread
import tkinter as tk
import math
import json
import subprocess
try:
	import cv2
	import zmq
	import base64
	import numpy as np
except:
	print("Couldn't import OpenCV, you need to install it first.")

OSD_X = 0#1
OSD_Y = 0
advanced_OSD = 0

PT_stu = 0
UD_stu = 0
HA_stu = 0
GA_stu = 0

def global_init():
	global DS_stu, TS_stu, color_bg, color_text, color_btn, color_line, color_can, color_oval, target_color
	global speed, ip_stu, Switch_3, Switch_2, Switch_1, servo_stu, function_stu
	DS_stu = 0
	TS_stu = 0

	color_bg='#000000'		#Set background color
	color_text='#E1F5FE'	  #Set text color
	color_btn='#0277BD'	   #Set button color
	color_line='#01579B'	  #Set line color
	color_can='#212121'	   #Set canvas color
	color_oval='#2196F3'	  #Set oval color
	target_color='#FF6D00'
	speed = 1
	ip_stu=1

	Switch_3 = 0
	Switch_2 = 0
	Switch_1 = 0

	servo_stu = 0
	function_stu = 0


global_init()


########>>>>>VIDEO<<<<<########
def RGB_to_Hex(r, g, b):
	return ('#'+str(hex(r))[-2:]+str(hex(g))[-2:]+str(hex(b))[-2:]).replace('x','0').upper()


def run_open():
    script_path = 'Footage-GUI.py'
    result = subprocess.run(['python', script_path], capture_output=True, text=True)
    print('stdout:', result.stdout)
    print('stderr:', result.stderr)

def getposHsv(event, x, y, flags, param):
	if event==cv2.EVENT_LBUTTONDOWN:
		print("HSV is", HSVimg[y, x])
		# findColorSet 172 21 69
		tcpClicSock.send(('findColorSet %s %s %s'%(HSVimg[y, x][0], HSVimg[y, x][1], HSVimg[y, x][2])).encode())
		getBGR = source[y, x]
		var_R.set(getBGR[2])
		var_G.set(getBGR[1])
		var_B.set(getBGR[0])
		canvas_show.config(bg = RGB_to_Hex(int(var_R.get()), int(var_G.get()), int(var_B.get())))


def get_FPS():
	global frame_num, fps
	while 1:
		try:
			time.sleep(1)
			fps = frame_num
			frame_num = 0
		except:
			time.sleep(1)


def advanced_OSD_add(draw_on, X, Y):#1
	error_X = X*10
	error_Y = Y*6-2
	#if error_Y > 0:
	X_s = int(200+120-120*math.cos(math.radians(error_Y)))
	Y_s = int(240+120*math.sin(math.radians(error_Y))-error_X*3)

	X_e = int(320+120*math.cos(math.radians(error_Y)))
	Y_e = int(240-120*math.sin(math.radians(error_Y))-error_X*3)
	cv2.line(draw_on,(X_s,Y_s),(X_e,Y_e),(0,255,0),2)
	cv2.putText(draw_on,('horizontal line'),(X_e+10,Y_e), font, 0.5,(0,255,0),1,cv2.LINE_AA)

	cv2.line(draw_on,(X_s,Y_s+270),(X_e,Y_e+270),(0,255,0),2)
	cv2.putText(draw_on,('A_Down'),(X_e+10,Y_e+270), font, 0.5,(0,255,0),1,cv2.LINE_AA)

	cv2.line(draw_on,(X_s,Y_s-270),(X_e,Y_e-270),(0,255,0),2)
	cv2.putText(draw_on,('A_Up'),(X_e+10,Y_e-270), font, 0.5,(0,255,0),1,cv2.LINE_AA)

	X_s_short = int(260+60-60*math.cos(math.radians(error_Y)))
	Y_s_short = int(240+60*math.sin(math.radians(error_Y))-error_X*3)

	X_e_short = int(320+60*math.cos(math.radians(error_Y)))
	Y_e_short = int(240-60*math.sin(math.radians(error_Y))-error_X*3)

	cv2.line(draw_on,(X_s_short,Y_s_short+90),(X_e_short,Y_e_short+90),(0,255,0))
	cv2.line(draw_on,(X_s_short,Y_s_short+180),(X_e_short,Y_e_short+180),(0,255,0))
	cv2.line(draw_on,(X_s_short,Y_s_short+360),(X_e_short,Y_e_short+360),(0,255,0))
	cv2.line(draw_on,(X_s_short,Y_s_short+450),(X_e_short,Y_e_short+450),(0,255,0))

	cv2.line(draw_on,(X_s_short,Y_s_short-90),(X_e_short,Y_e_short-90),(0,255,0))
	cv2.line(draw_on,(X_s_short,Y_s_short-180),(X_e_short,Y_e_short-180),(0,255,0))
	cv2.line(draw_on,(X_s_short,Y_s_short-360),(X_e_short,Y_e_short-360),(0,255,0))
	cv2.line(draw_on,(X_s_short,Y_s_short-450),(X_e_short,Y_e_short-450),(0,255,0))


def opencv_r():
	global frame_num, source, HSVimg
	while True:
		try:
			frame = footage_socket.recv_string()
			img = base64.b64decode(frame)
			npimg = np.frombuffer(img, dtype=np.uint8)
			source = cv2.imdecode(npimg, 1)
			cv2.putText(source,('PC FPS: %s'%fps),(40,20), font, 0.5,(255,255,255),1,cv2.LINE_AA)

			
			try:
				cv2.putText(source,('CPU Temperature: %s'%CPU_TEP),(370,350), font, 0.5,(128,255,128),1,cv2.LINE_AA)
				cv2.putText(source,('CPU Usage: %s'%CPU_USE),(370,380), font, 0.5,(128,255,128),1,cv2.LINE_AA)
				cv2.putText(source,('RAM Usage: %s'%RAM_USE),(370,410), font, 0.5,(128,255,128),1,cv2.LINE_AA)

				cv2.rectangle(source, (167, 320), (473, 330), (255,255,255))

				DIR_show = int(CAR_DIR)
				if DIR_show > 0:
					cv2.rectangle(source, ((320-DIR_show), 323), (320, 327), (255,255,255))
				elif DIR_show < 0:
					cv2.rectangle(source, (320, 323), ((320-DIR_show), 327), (255,255,255))
			except:
				pass

			if advanced_OSD:#1
				advanced_OSD_add(source, OSD_X, OSD_Y)
			
			#cv2.putText(source,('%sm'%ultra_data),(210,290), font, 0.5,(255,255,255),1,cv2.LINE_AA)
			cv2.imshow("Stream", source)
			cv2.setMouseCallback("Stream", getposBgr)

			HSVimg = cv2.cvtColor(source, cv2.COLOR_BGR2Lab)
			cv2.imshow("StreamHSV", HSVimg)
			cv2.setMouseCallback("StreamHSV", getposHsv)

			frame_num += 1
			cv2.waitKey(1)

		except:
			time.sleep(0.5)
			break

fps_threading=thread.Thread(target=get_FPS)		 #Define a thread for FPV and OpenCV
fps_threading.setDaemon(True)							 #'True' means it is a front thread,it would close when the mainloop() closes
fps_threading.start()									 #Thread starts

########>>>>>VIDEO<<<<<########


def replace_num(initial,new_num):   #Call this function to replace data in '.txt' file
	newline=""
	str_num=str(new_num)


def num_import(initial):			#Call this function to import data from '.txt' file
	x = 1


def connection_thread():
	global Switch_3, Switch_2, Switch_1, function_stu, OSD_X, OSD_Y, OSD_info, advanced_OSD, car_info
	while 1:
		car_info = (tcpClicSock.recv(BUFSIZ)).decode()
		print("car_info:  " + car_info)
		if not car_info:
			continue

		elif "get_info" in car_info:
			try:
				cpu_info = json.loads(car_info)['data']
				CPU_TEP_lab.config(text='CPU Temp: %s℃'%cpu_info[0])
				CPU_USE_lab.config(text='CPU Usage: %s'%cpu_info[1])
				RAM_lab.config(text='RAM Usage: %s'%cpu_info[2])
			except Exception as e:
				print('get_info error: not A JSON ' + str(e))
				
		elif 'Switch_3_on' in car_info:
			Switch_3 = 1
			Btn_Switch_3.config(bg='#4CAF50')

		elif 'Switch_2_on' in car_info:
			Switch_2 = 1
			Btn_Switch_2.config(bg='#4CAF50')

		elif 'Switch_1_on' in car_info:
			Switch_1 = 1
			Btn_Switch_1.config(bg='#4CAF50')

		elif 'Switch_3_off' in car_info:
			Switch_3 = 0
			Btn_Switch_3.config(bg=color_btn)

		elif 'Switch_2_off' in car_info:
			Switch_2 = 0
			Btn_Switch_2.config(bg=color_btn)

		elif 'Switch_1_off' in car_info:
			Switch_1 = 0
			Btn_Switch_1.config(bg=color_btn)

		elif 'findColor' in car_info:
			function_stu = 1
			Btn_function_2.config(bg='#4CAF50')

		elif 'motionGet' in car_info:
			function_stu = 1
			Btn_function_3.config(bg='#4CAF50')

		elif 'police' in car_info:
			function_stu = 1
			Btn_function_4.config(bg='#4CAF50')

		elif 'automatic' in car_info:
			function_stu = 1
			Btn_function_5.config(bg='#4CAF50')

		elif 'trackLine' in car_info:
			function_stu = 1
			Btn_function_6.config(bg='#4CAF50')
		elif 'Speech' in car_info:
			function_stu = 1
			Btn_function_6.config(bg='#4CAF50')

		elif 'stopCV' in car_info:
			function_stu = 0
			Btn_function_1.config(bg=color_btn)
			Btn_function_2.config(bg=color_btn)
			Btn_function_3.config(bg=color_btn)
			Btn_function_4.config(bg=color_btn)
			Btn_function_5.config(bg=color_btn)



		elif 'CVFL_on' in car_info:
			function_stu = 1

		elif 'CVFL_off' in car_info:
			function_stu = 0

		elif 'OSD' in car_info:
			OSD_info = car_info.split()
			try:
				OSD_X = float(OSD_info[1])
				OSD_Y = float(OSD_info[2])
			except:
				pass


def Info_receive():	
	while 1:
		try:
			tcpClicSock.send('get_info'.encode())
			time.sleep(3)
		except Exception as e:
			print("get_info error: " + str(e))
			break
			


def socket_connect():	 #Call this function to connect with the server
	global ADDR,tcpClicSock,BUFSIZ,ip_stu,ipaddr
	ip_adr=E1.get()	   #Get the IP address from Entry

	if ip_adr == '':	  #If no input IP address in Entry,import a default IP
		ip_adr=num_import('IP:')
		l_ip_4.config(text='Connecting')
		l_ip_4.config(bg='#FF8F00')
		l_ip_5.config(text='Default:%s'%ip_adr)
		pass
	
	SERVER_IP = ip_adr
	SERVER_PORT = 10223   #Define port serial 
	BUFSIZ = 1024		 #Define buffer size
	ADDR = (SERVER_IP, SERVER_PORT)
	tcpClicSock = socket(AF_INET, SOCK_STREAM) #Set connection value for socket

	for i in range (1,6): #Try 5 times if disconnected
		#try:
		if ip_stu == 1:
			print("Connecting to server @ %s:%d..." %(SERVER_IP, SERVER_PORT))
			print("Connecting")
			tcpClicSock.connect(ADDR)		#Connection with the server
		
			print("Connected")
		
			l_ip_5.config(text='IP:%s'%ip_adr)
			l_ip_4.config(text='Connected')
			l_ip_4.config(bg='#558B2F')

			replace_num('IP:',ip_adr)
			E1.config(state='disabled')	  #Disable the Entry
			Btn14.config(state='disabled')   #Disable the Entry
			
			ip_stu=0						 #'0' means connected

			connection_threading=thread.Thread(target=connection_thread)		 #Define a thread for FPV and OpenCV
			connection_threading.setDaemon(True)							 
			connection_threading.start()									 

			info_threading=thread.Thread(target=Info_receive)		 #get CPU info 
			info_threading.setDaemon(True)							 
			info_threading.start()									 

			video_threading=thread.Thread(target=run_open)		 #Define a thread for FPV and OpenCV
			video_threading.daemon = True					 
			video_threading.start()									 

			break
		else:
			print("Cannot connecting to server,try it latter!")
			l_ip_4.config(text='Try %d/5 time(s)'%i)
			l_ip_4.config(bg='#EF6C00')
			print('Try %d/5 time(s)'%i)
			ip_stu=1
			time.sleep(1)
			continue

	if ip_stu == 1:
		l_ip_4.config(text='Disconnected')
		l_ip_4.config(bg='#F44336')


def connect(event):	   #Call this function to connect with the server
	if ip_stu == 1:
		sc=thread.Thread(target=socket_connect) #Define a thread for connection
		sc.setDaemon(True)					  #'True' means it is a front thread,it would close when the mainloop() closes
		sc.start()							  #Thread starts


def scale_send(event):
	time.sleep(0.03)
	tcpClicSock.send(('wsB %s'%var_Speed.get()).encode())


def servo_buttons(x,y):
	def call_A_up(event):
		global UD_stu
		if UD_stu == 0:
			tcpClicSock.send(('armUp').encode())
			UD_stu = 1

	def call_A_down(event):
		global UD_stu
		if UD_stu == 0:
			tcpClicSock.send(('armDown').encode())
			UD_stu = 1

	def call_A_stop(event):
		global UD_stu
		tcpClicSock.send(('armStop').encode())
		UD_stu = 0


	def call_lookleft(event):
		global PT_stu
		if PT_stu == 0:
			tcpClicSock.send(('lookleft').encode())
			PT_stu = 1

	def call_lookright(event):
		global PT_stu
		if PT_stu == 0:
			tcpClicSock.send(('lookright').encode())
			PT_stu = 1

	def call_LRstop(event):
		global PT_stu
		tcpClicSock.send(('LRstop').encode())
		PT_stu = 0


	def call_handup(event):
		global HA_stu
		if HA_stu == 0:
			tcpClicSock.send(('handUp').encode())
			HA_stu = 1

	def call_handdown(event):
		global HA_stu
		if HA_stu == 0:
			tcpClicSock.send(('handDown').encode())
			HA_stu = 1

	def call_HAstop(event):
		global HA_stu
		tcpClicSock.send(('handStop').encode())
		HA_stu = 0


	def call_grab(event):
		global GA_stu
		if GA_stu == 0:
			tcpClicSock.send(('grab').encode())
			GA_stu = 1

	def call_loose(event):
		global GA_stu
		if GA_stu == 0:
			tcpClicSock.send(('loose').encode())
			GA_stu = 1

	def call_stop(event):
		global GA_stu
		tcpClicSock.send(('GLstop').encode())
		GA_stu = 0

	def call_up(event):
		global GA_stu
		if GA_stu == 0:
			tcpClicSock.send(('up').encode())
			GA_stu = 1

	def call_down(event):
		global GA_stu
		if GA_stu == 0:
			tcpClicSock.send(('down').encode())
			GA_stu = 1

	def call_UDstop(event):
		global GA_stu
		tcpClicSock.send(('UDstop').encode())
		GA_stu = 0

	Btn_0 = tk.Button(root, width=8, text='A_Left',fg=color_text,bg=color_btn,relief='ridge')
	Btn_0.place(x=x,y=y+35)
	Btn_0.bind('<ButtonPress-1>', call_lookleft)
	Btn_0.bind('<ButtonRelease-1>', call_LRstop)
	root.bind('<KeyPress-j>', call_lookleft)
	root.bind('<KeyRelease-j>', call_LRstop)

	Btn_1 = tk.Button(root, width=8, text='A_Up',fg=color_text,bg=color_btn,relief='ridge')
	Btn_1.place(x=x+70,y=y)
	Btn_1.bind('<ButtonPress-1>', call_A_up)
	Btn_1.bind('<ButtonRelease-1>', call_A_stop)
	root.bind('<KeyPress-r>', call_A_up)
	root.bind('<KeyRelease-r>', call_A_stop) 

	Btn_1 = tk.Button(root, width=8, text='A_Down',fg=color_text,bg=color_btn,relief='ridge')
	Btn_1.place(x=x+70,y=y+35)
	Btn_1.bind('<ButtonPress-1>', call_A_down)
	Btn_1.bind('<ButtonRelease-1>', call_A_stop)
	root.bind('<KeyPress-f>', call_A_down)
	root.bind('<KeyRelease-f>', call_A_stop)

	Btn_2 = tk.Button(root, width=8, text='A_Right',fg=color_text,bg=color_btn,relief='ridge')
	Btn_2.place(x=x+140,y=y+35)
	Btn_2.bind('<ButtonPress-1>', call_lookright)
	Btn_2.bind('<ButtonRelease-1>', call_LRstop)
	root.bind('<KeyPress-l>', call_lookright) 
	root.bind('<KeyRelease-l>', call_LRstop)

	Btn_3 = tk.Button(root, width=8, text='Grab',fg=color_text,bg=color_btn,relief='ridge')
	Btn_3.place(x=x,y=y)
	Btn_3.bind('<ButtonPress-1>', call_grab)
	Btn_3.bind('<ButtonRelease-1>', call_stop)
	root.bind('<KeyPress-u>', call_grab) 
	root.bind('<KeyRelease-u>', call_stop) 

	Btn_4 = tk.Button(root, width=8, text='Loose',fg=color_text,bg=color_btn,relief='ridge')
	Btn_4.place(x=x+140,y=y)
	Btn_4.bind('<ButtonPress-1>', call_loose)
	Btn_4.bind('<ButtonRelease-1>', call_stop)
	root.bind('<KeyPress-o>', call_loose) 
	root.bind('<KeyRelease-o>', call_stop)

	Btn_5 = tk.Button(root, width=8, text='H_Down',fg=color_text,bg=color_btn,relief='ridge')
	Btn_5.place(x=x,y=y-55)
	Btn_5.bind('<ButtonPress-1>', call_handdown)
	Btn_5.bind('<ButtonRelease-1>', call_HAstop)
	root.bind('<KeyPress-;>', call_handdown) 
	root.bind('<KeyRelease-;>', call_HAstop)

	Btn_6 = tk.Button(root, width=8, text='H_Up',fg=color_text,bg=color_btn,relief='ridge')
	Btn_6.place(x=x,y=y-55-35)
	Btn_6.bind('<ButtonPress-1>', call_handup)
	Btn_6.bind('<ButtonRelease-1>', call_HAstop)
	root.bind('<KeyPress-p>', call_handup)
	root.bind('<KeyRelease-p>', call_HAstop)



def motor_buttons(x,y):
	def call_left(event):
		global TS_stu
		if TS_stu == 0:
			tcpClicSock.send(('left').encode())
			TS_stu = 1

	def call_right(event):
		global TS_stu
		if TS_stu == 0:
			tcpClicSock.send(('right').encode())
			TS_stu = 1

	def call_forward(event):
		global DS_stu
		if DS_stu == 0:
			tcpClicSock.send(('forward').encode())
			DS_stu = 1

	def call_backward(event):
		global DS_stu
		if DS_stu == 0:
			tcpClicSock.send(('backward').encode())
			DS_stu = 1

	def call_DS(event):
		global DS_stu
		tcpClicSock.send(('DS').encode())
		DS_stu = 0

	def call_TS(event):
		global TS_stu
		tcpClicSock.send(('TS').encode())
		TS_stu = 0

	def call_UP(event):
		tcpClicSock.send(('up').encode())

	def call_DOWN(event):
		tcpClicSock.send(('down').encode())
	def call_UDstop(event):
		tcpClicSock.send(('UDstop').encode())
	Btn_UP = tk.Button(root, width=8, text='UP',fg=color_text,bg=color_btn,relief='ridge')
	Btn_UP.place(x=x,y=y)
	Btn_UP.bind('<ButtonPress-1>', call_UP)
	Btn_UP.bind('<ButtonRelease-1>', call_UDstop)
	root.bind('<KeyPress-i>', call_UP)
	root.bind('<KeyRelease-i>', call_UDstop) 
 
	Btn_DOWN = tk.Button(root, width=8, text='DOWN',fg=color_text,bg=color_btn,relief='ridge')
	Btn_DOWN.place(x=x+140,y=y)
	Btn_DOWN.bind('<ButtonPress-1>', call_DOWN)
	Btn_DOWN.bind('<ButtonRelease-1>', call_UDstop)
	root.bind('<KeyPress-k>', call_DOWN)
	root.bind('<KeyRelease-k>', call_UDstop) 
 
	Btn_0 = tk.Button(root, width=8, text='Left',fg=color_text,bg=color_btn,relief='ridge')
	Btn_0.place(x=x,y=y+35)
	Btn_0.bind('<ButtonPress-1>', call_left)
	Btn_0.bind('<ButtonRelease-1>', call_TS)
	root.bind('<KeyPress-a>', call_left)
	root.bind('<KeyRelease-a>', call_TS)

	Btn_1 = tk.Button(root, width=8, text='Forward',fg=color_text,bg=color_btn,relief='ridge')
	Btn_1.place(x=x+70,y=y)
	Btn_1.bind('<ButtonPress-1>', call_forward)
	Btn_1.bind('<ButtonRelease-1>', call_DS)
	root.bind('<KeyPress-w>', call_forward)
	root.bind('<KeyRelease-w>', call_DS) 

	Btn_1 = tk.Button(root, width=8, text='Backward',fg=color_text,bg=color_btn,relief='ridge')
	Btn_1.place(x=x+70,y=y+35)
	Btn_1.bind('<ButtonPress-1>', call_backward)
	Btn_1.bind('<ButtonRelease-1>', call_DS)
	root.bind('<KeyPress-s>', call_backward)
	root.bind('<KeyRelease-s>', call_DS)

	Btn_2 = tk.Button(root, width=8, text='Right',fg=color_text,bg=color_btn,relief='ridge')
	Btn_2.place(x=x+140,y=y+35)
	Btn_2.bind('<ButtonPress-1>', call_right)
	Btn_2.bind('<ButtonRelease-1>', call_TS)
	root.bind('<KeyPress-d>', call_right) 
	root.bind('<KeyRelease-d>', call_TS) 


def information_screen(x,y):
	global CPU_TEP_lab, CPU_USE_lab, RAM_lab, l_ip_4, l_ip_5
	CPU_TEP_lab=tk.Label(root,width=18,text='CPU Temp:',fg=color_text,bg='#212121')
	CPU_TEP_lab.place(x=x,y=y)						 #Define a Label and put it in position

	CPU_USE_lab=tk.Label(root,width=18,text='CPU Usage:',fg=color_text,bg='#212121')
	CPU_USE_lab.place(x=x,y=y+30)						 #Define a Label and put it in position

	RAM_lab=tk.Label(root,width=18,text='RAM Usage:',fg=color_text,bg='#212121')
	RAM_lab.place(x=x,y=y+60)						 #Define a Label and put it in position

	l_ip_4=tk.Label(root,width=18,text='Disconnected',fg=color_text,bg='#F44336')
	l_ip_4.place(x=x,y=y+95)						 #Define a Label and put it in position

	l_ip_5=tk.Label(root,width=18,text='Use default IP',fg=color_text,bg=color_btn)
	l_ip_5.place(x=x,y=y+130)						 #Define a Label and put it in position


def connent_input(x,y):
	global E1, Btn14
	E1 = tk.Entry(root,show=None,width=16,bg="#37474F",fg='#eceff1', textvariable='')
	# test ip
	E1.insert(0, "")
	E1.place(x=x+5,y=y+25)							 #Define a Entry and put it in position

	l_ip_3=tk.Label(root,width=10,text='IP Address:',fg=color_text,bg='#000000')
	l_ip_3.place(x=x,y=y)						 #Define a Label and put it in position

	Btn14= tk.Button(root, width=8,height=2, text='Connect',fg=color_text,bg=color_btn,relief='ridge')
	Btn14.place(x=x+130,y=y)						  #Define a Button and put it in position

	root.bind('<Return>', connect)
	Btn14.bind('<ButtonPress-1>', connect)


def switch_button(x,y):
	global Btn_Switch_1, Btn_Switch_2, Btn_Switch_3,function_stu
	def call_Switch_1(event):
		global Btn_Switch_1
		if Btn_Switch_1 == 0:
			tcpClicSock.send(('Switch_1_on').encode())
			Btn_Switch_1 = 1
		else:
			tcpClicSock.send(('Switch_1_off').encode())
			Btn_Switch_1 = 0


	def call_Switch_2(event):
		global Btn_Switch_2
		if Btn_Switch_2 == 0:
			tcpClicSock.send(('Switch_2_on').encode())
			Btn_Switch_2 = 1
		else:
			tcpClicSock.send(('Switch_2_off').encode())
			Btn_Switch_2 = 0


	def call_Switch_3(event):
		global Btn_Switch_3
		if Btn_Switch_3 == 0:
			tcpClicSock.send(('Switch_3_on').encode())
			Btn_Switch_3 = 1
		else:
			tcpClicSock.send(('Switch_3_off').encode())
			Btn_Switch_3 = 0

	Btn_Switch_1 = tk.Button(root, width=8, text='Port 1',fg=color_text,bg=color_btn,relief='ridge')
	Btn_Switch_2 = tk.Button(root, width=8, text='Port 2',fg=color_text,bg=color_btn,relief='ridge')
	Btn_Switch_3 = tk.Button(root, width=8, text='Port 3',fg=color_text,bg=color_btn,relief='ridge')

	Btn_Switch_1.place(x=x,y=y)
	Btn_Switch_2.place(x=x+70,y=y)
	Btn_Switch_3.place(x=x+140,y=y)

	Btn_Switch_1.bind('<ButtonPress-1>', call_Switch_1)
	Btn_Switch_2.bind('<ButtonPress-1>', call_Switch_2)
	Btn_Switch_3.bind('<ButtonPress-1>', call_Switch_3)


def scale(x,y,w):
	global var_Speed
	var_Speed = tk.StringVar()
	var_Speed.set(100)

	Scale_B = tk.Scale(root,label=None,
	from_=60,to=100,orient=tk.HORIZONTAL,length=w,
	showvalue=1,tickinterval=None,resolution=10,variable=var_Speed,troughcolor='#448AFF',command=scale_send,fg=color_text,bg=color_bg,highlightthickness=0)
	Scale_B.place(x=x,y=y)							#Define a Scale and put it in position

	canvas_cover=tk.Canvas(root,bg=color_bg,height=30,width=510,highlightthickness=0)
	canvas_cover.place(x=x,y=y+30)




def scale_FL(x,y,w):
	global Btn_CVFL
	def lip1_send(event):
		time.sleep(0.03)
		tcpClicSock.send(('CVFLL1 %s'%var_lip1.get()).encode())

	def lip2_send(event):
		time.sleep(0.03)
		tcpClicSock.send(('CVFLL2 %s'%var_lip2.get()).encode())

	def err_send(event):
		time.sleep(0.03)
		tcpClicSock.send(('CVFLSP %s'%var_err.get()).encode())


	def call_CVFL(event):
		global function_stu
		if function_stu == 0:
			tcpClicSock.send(('CVFL').encode())
			function_stu = 1
		else:
			tcpClicSock.send(('stopCV').encode())
			function_stu = 0
	def call_WB(event):
		global function_stu
		if function_stu == 0:
			tcpClicSock.send(('CVFLColorSet 0').encode())
			function_stu = 1
		else:
			tcpClicSock.send(('CVFLColorSet 255').encode())
			function_stu = 0

	Scale_lip1 = tk.Scale(root,label=None,
	from_=0,to=479,orient=tk.HORIZONTAL,length=w,
	showvalue=1,tickinterval=None,resolution=1,variable=var_lip1,troughcolor='#212121',command=lip1_send,fg=color_text,bg=color_bg,highlightthickness=0)
	Scale_lip1.place(x=x,y=y)							#Define a Scale and put it in position

	Scale_lip2 = tk.Scale(root,label=None,
	from_=0,to=479,orient=tk.HORIZONTAL,length=w,
	showvalue=1,tickinterval=None,resolution=1,variable=var_lip2,troughcolor='#212121',command=lip2_send,fg=color_text,bg=color_bg,highlightthickness=0)
	Scale_lip2.place(x=x,y=y+30)							#Define a Scale and put it in position

	Scale_err = tk.Scale(root,label=None,
	from_=0,to=200,orient=tk.HORIZONTAL,length=w,
	showvalue=1,tickinterval=None,resolution=1,variable=var_err,troughcolor='#212121',command=err_send,fg=color_text,bg=color_bg,highlightthickness=0)
	Scale_err.place(x=x,y=y+60)							#Define a Scale and put it in position

	canvas_cover=tk.Canvas(root,bg=color_bg,height=30,width=510,highlightthickness=0)
	canvas_cover.place(x=x,y=y+90)


	Btn_CVFL = tk.Button(root, width=23, text='CV FL',fg=color_text,bg='#212121',relief='ridge')
	Btn_CVFL.place(x=x+w+21,y=y+20)
	Btn_CVFL.bind('<ButtonPress-1>', call_CVFL)

	Btn_WB = tk.Button(root, width=23, text='LineColorSwitch',fg=color_text,bg='#212121',relief='ridge')
	Btn_WB.place(x=x+w+21,y=y+60)
	Btn_WB.bind('<ButtonPress-1>', call_WB)


def scale_FC(x,y,w):
	global canvas_show
	def R_send(event):
		canvas_show.config(bg = RGB_to_Hex(int(var_R.get()), int(var_G.get()), int(var_B.get())))
		time.sleep(0.03)


	def G_send(event):
		canvas_show.config(bg = RGB_to_Hex(int(var_R.get()), int(var_G.get()), int(var_B.get())))
		time.sleep(0.03)

	def B_send(event):
		canvas_show.config(bg = RGB_to_Hex(int(var_R.get()), int(var_G.get()), int(var_B.get())))
		time.sleep(0.03)


	def call_SET(event):
		r = int(var_R.get())
		g = int(var_G.get())
		b = int(var_B.get())

		data_str = f"{r}, {g}, {b}"
		message = f"{{'title': 'findColorSet', 'data': [{data_str}]}}"
		tcpClicSock.send(message.encode())
	Scale_R = tk.Scale(root,label=None,
	from_=0,to=255,orient=tk.HORIZONTAL,length=w,
	showvalue=1,tickinterval=None,resolution=1,variable=var_R,troughcolor='#FF1744',command=R_send,fg=color_text,bg=color_bg,highlightthickness=0)
	Scale_R.place(x=x,y=y)							#Define a Scale and put it in position

	Scale_G = tk.Scale(root,label=None,
	from_=0,to=255,orient=tk.HORIZONTAL,length=w,
	showvalue=1,tickinterval=None,resolution=1,variable=var_G,troughcolor='#00E676',command=G_send,fg=color_text,bg=color_bg,highlightthickness=0)
	Scale_G.place(x=x,y=y+30)							#Define a Scale and put it in position

	Scale_B = tk.Scale(root,label=None,
	from_=0,to=255,orient=tk.HORIZONTAL,length=w,
	showvalue=1,tickinterval=None,resolution=1,variable=var_B,troughcolor='#2979FF',command=B_send,fg=color_text,bg=color_bg,highlightthickness=0)
	Scale_B.place(x=x,y=y+60)							#Define a Scale and put it in position

	canvas_cover=tk.Canvas(root,bg=color_bg,height=30,width=510,highlightthickness=0)
	canvas_cover.place(x=x,y=y+90)

	canvas_show=tk.Canvas(root,bg=RGB_to_Hex(int(var_R.get()), int(var_G.get()), int(var_B.get())),height=35,width=170,highlightthickness=0)
	canvas_show.place(x=w+x+21,y=y+15)

	Btn_WB = tk.Button(root, width=23, text='Color Set',fg=color_text,bg='#212121',relief='ridge')
	Btn_WB.place(x=x+w+21,y=y+60)
	Btn_WB.bind('<ButtonPress-1>', call_SET)





def function_buttons(x,y):
	global function_stu, Btn_function_1, Btn_function_2, Btn_function_3, Btn_function_4, Btn_function_5

	def call_function_2(event):
		global function_stu
		if function_stu == 0:
			tcpClicSock.send(('findColor').encode())
			function_stu = 1
		else:
			tcpClicSock.send(('stopCV').encode())
			function_stu = 0

	def call_function_3(event):
		global function_stu
		if function_stu == 0:
			tcpClicSock.send(('motionGet').encode())
			function_stu = 1
		else:
			tcpClicSock.send(('stopCV').encode())
			function_stu = 0

	def call_function_4(event):
		global function_stu
		if function_stu == 0:
			tcpClicSock.send(('police').encode())
			function_stu = 1
		else:
			tcpClicSock.send(('policeOff').encode())
			function_stu = 0

	def call_function_5(event):
		global function_stu
		if function_stu == 0:
			tcpClicSock.send(('automatic').encode())
			function_stu = 1
		else:
			tcpClicSock.send(('automaticOff').encode())
			function_stu = 0

	def call_function_1(event):
		global function_stu
		if function_stu == 0:
			tcpClicSock.send(('trackLine').encode())
			function_stu = 1
		else:
			tcpClicSock.send(('trackLineOff').encode())
			function_stu = 0


	Btn_function_1 = tk.Button(root, width=8, text='TrackLine',fg=color_text,bg=color_btn,relief='ridge')
	Btn_function_2 = tk.Button(root, width=8, text='FindColor',fg=color_text,bg=color_btn,relief='ridge')
	Btn_function_3 = tk.Button(root, width=8, text='MotionGet',fg=color_text,bg=color_btn,relief='ridge')
	Btn_function_4 = tk.Button(root, width=8, text='Police',fg=color_text,bg=color_btn,relief='ridge')
	Btn_function_5 = tk.Button(root, width=8, text='Automatic',fg=color_text,bg=color_btn,relief='ridge')


	Btn_function_1.place(x=x,y=y)
	Btn_function_2.place(x=x+70,y=y)
	Btn_function_3.place(x=x+140,y=y)
	Btn_function_4.place(x=x+210,y=y)
	Btn_function_5.place(x=x+280,y=y)



	Btn_function_1.bind('<ButtonPress-1>', call_function_1)
	Btn_function_2.bind('<ButtonPress-1>', call_function_2)
	Btn_function_3.bind('<ButtonPress-1>', call_function_3)
	Btn_function_4.bind('<ButtonPress-1>', call_function_4)
	Btn_function_5.bind('<ButtonPress-1>', call_function_5)



def config_buttons(x,y):
	def call_SiLeft(event):
		tcpClicSock.send(('SiLeft 0').encode())

	def call_SiRight(event):
		tcpClicSock.send(('SiRight 0').encode())

	def call_SetGearMiddle(event):
		tcpClicSock.send(('PWMMS 0').encode())

	def call_PWM1MS(event):
		tcpClicSock.send(('PWMMS 1').encode())

	def call_PWM2MS(event):
		tcpClicSock.send(('PWMMS 2').encode())

	def call_PWM3MS(event):
		tcpClicSock.send(('PWMMS 3').encode())

	def call_PWM4MS(event):
		tcpClicSock.send(('PWMMS 4').encode())

	def call_MoveInit(event):
		tcpClicSock.send(('PWMINIT').encode())

	def call_PWMDefault(event):
		tcpClicSock.send(('PWMD').encode())

	Btn_SiLeft = tk.Button(root, width=16, text='<PWM0 Turn Left',fg=color_text,bg=color_btn,relief='ridge')
	Btn_SiLeft.place(x=x,y=y)
	Btn_SiLeft.bind('<ButtonPress-1>', call_SiLeft)

	Btn_SiRight = tk.Button(root, width=16, text='PWM0 Turn Right>',fg=color_text,bg=color_btn,relief='ridge')
	Btn_SiRight.place(x=x+300,y=y)
	Btn_SiRight.bind('<ButtonPress-1>', call_SiRight)

	Btn_SetGearMiddle = tk.Button(root, width=16, text='<PWM0 Middle Set>',fg=color_text,bg=color_btn,relief='ridge')
	Btn_SetGearMiddle.place(x=x+150,y=y)
	Btn_SetGearMiddle.bind('<ButtonPress-1>', call_SetGearMiddle)

	Btn_PWM1MS = tk.Button(root, width=16, text='PWM1 Middle Set',fg=color_text,bg=color_btn,relief='ridge')
	Btn_PWM1MS.place(x=x,y=y+70)
	Btn_PWM1MS.bind('<ButtonPress-1>', call_PWM1MS)

	Btn_PWM2MS = tk.Button(root, width=16, text='PWM2 Middle Set',fg=color_text,bg=color_btn,relief='ridge')
	Btn_PWM2MS.place(x=x+150,y=y+70)
	Btn_PWM2MS.bind('<ButtonPress-1>', call_PWM2MS)

	Btn_PWM3MS = tk.Button(root, width=16, text='PWM3 Middle Set',fg=color_text,bg=color_btn,relief='ridge')
	Btn_PWM3MS.place(x=x+300,y=y+70)
	Btn_PWM3MS.bind('<ButtonPress-1>', call_PWM3MS)

	Btn_PWM4MS = tk.Button(root, width=16, text='PWM4 Middle Set',fg=color_text,bg=color_btn,relief='ridge')
	Btn_PWM4MS.place(x=x,y=y+140)
	Btn_PWM4MS.bind('<ButtonPress-1>', call_PWM4MS)

	Btn_PWM5MS = tk.Button(root, width=16, text='Move to InitPos',fg=color_text,bg=color_btn,relief='ridge')
	Btn_PWM5MS.place(x=x+150,y=y+140)
	Btn_PWM5MS.bind('<ButtonPress-1>', call_MoveInit)

	Btn_PWMDefault = tk.Button(root, width=16, text='PWM Default Set',fg=color_text,bg=color_btn,relief='ridge')
	Btn_PWMDefault.place(x=x+300,y=y+140)
	Btn_PWMDefault.bind('<ButtonPress-1>', call_PWMDefault)



def loop():
	global root, var_lip1, var_lip2, var_err, var_R, var_G, var_B, var_ec#Z
	root = tk.Tk()			
	root.title('Adeept RaspTank')	  
	root.geometry('920x570')  #Z
	root.config(bg=color_bg)  

	var_lip1 = tk.StringVar()
	var_lip1.set(440)
	var_lip2 = tk.StringVar()
	var_lip2.set(380)
	var_err = tk.StringVar()
	var_err.set(20)

	var_R = tk.StringVar()
	var_R.set(80)
	var_G = tk.StringVar()
	var_G.set(80)
	var_B = tk.StringVar()
	var_B.set(80)

	var_ec = tk.StringVar() #Z
	var_ec.set(0)			#Z

	try:
		logo =tk.PhotoImage(file = 'logo.png')
		l_logo=tk.Label(root,image = logo,bg=color_bg)
		l_logo.place(x=30,y=13)
	except:
		pass

	motor_buttons(30,105)

	information_screen(330,15)

	connent_input(125,15)

	switch_button(30,195)

	servo_buttons(255,195)

	scale(30,230,203)

	function_buttons(30,360)

	scale_FL(470,0,238)

	scale_FC(470,135,238)



	config_buttons(470,360)

	root.mainloop()


if __name__ == '__main__':
	loop()
