#!/usr/bin/env python
#############################################################################################################                                                                  
# Basic example for controlling the GoPiGo using the Keyboard
# Contributed by casten on Gitub https://github.com/DexterInd/GoPiGo/pull/112
#
# This code lets you control the GoPiGo from the VNC or Pi Desktop. Also, these are non-blocking calls so it is much more easier to use too.
#
# Controls:
# 	w:	Move forward
#	a:	Turn left
#	d:	Turn right
#	s:	Move back
#	x:	Stop
#	t:	Increase speed
#	g:	Decrease speed
#	j: Move servo left
#	l: Move servo right
#	k: Move servo home
#	z: 	Exit
# http://www.dexterindustries.com/GoPiGo/                                                                
# History
# ------------------------------------------------
# Author     	Date      		Comments  
# Karan		27 June 14			Code cleanup                                                    
# Casten	31 Dec  15			Added async io, action until keyup
# Karan		04 Jan	16			Cleaned up the GUI

'''
## License
 GoPiGo for the Raspberry Pi: an open source robotics platform for the Raspberry Pi.
 Copyright (C) 2015  Dexter Industries

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/gpl-3.0.txt>.
'''     
##############################################################################################################

from gopigo import *		# Has the basic functions for controlling the GoPiGo Robot
from grove_rgb_lcd import *
import sys					# Used for closing the running program
import pygame				# Gives access to KEYUP/KEYDOWN events
import subprocess			# For IR
import grovepi				# For grovepi
import thread
import threading
import time
import random
import blescan				# For iBeacon
import bluetooth._bluetooth as bluez
import math
import grove_compass_lib	# For compass sensor
import subprocess			# For ir sensor
import json

try:
	import paho.mqtt.client as mqtt
except ImportError:
	print "mqtt error"

#Initialization for pygame
pygame.init()
setRGB(0,128,64)
screen = pygame.display.set_mode((700, 450))
pygame.display.set_caption('Remote Control Window')

# Initialization for sensor
# ultrasonic_ranger_head = 3
# ultrasonic_ranger_left = 2
# ultrasonic_ranger_right = 7
blu_led = 2
blu_led_light = False
buzzer = 3
red_led = 7
red_led_light = False
gopigo_right_led = 0
gopigo_left_led = 1
# red_led = 7
dht = 8
gas_sensor = 0
air_quality_sensor = 1
flame_sensor = 2
# light_sensor = 6
grovepi.pinMode(blu_led,"OUTPUT")
# grovepi.pinMode(gre_led,"OUTPUT")
grovepi.pinMode(red_led,"OUTPUT")
grovepi.pinMode(buzzer,"OUTPUT")
grovepi.pinMode(gas_sensor,"INPUT")
grovepi.pinMode(air_quality_sensor,"INPUT")
grovepi.pinMode(flame_sensor,"INPUT")

temperture_temp = 0
temperature_retry = 0
normal_temp = 25
normal_hum = 40

# Initialization for MQTT
routerIP = "172.17.37.185"
routerPort = "32770"
topic = "Sensor"

strPayload = {
	"frontDistance" : "0",
	"leftDistance" : "0",
	"rightDistance" : "0",
	"temperature" : "0",
	"humidity" : "0",
	"gas" : "0",
	"airQuality" : "0",
	"flame" : "0",
	"compass" : "0",
	"light" : "0",
	"lightStatus" : False,
	"fanStatus" : False
}
payload = json.loads(json.dumps(strPayload))

# Fill background
background = pygame.Surface(screen.get_size())
background = background.convert()
background.fill((250, 250, 250))

# Robot turn
nowDirection = None
divider = 4
start_Direction = 0
c = grove_compass_lib.compass()
turning = False
fanStatus = False
lightStatus = False

grovepi.digitalWrite(blu_led, 0)
grovepi.digitalWrite(red_led, 0)

# Display some text
instructions = '''
                      BASIC GOPIGO CONTROL GUI

This is a basic example for the GoPiGo Robot control 

(Be sure to put focus on thi window to control the gopigo!)

Press:
->w: Move GoPiGo Robot forward
->a: Turn GoPiGo Robot left
->d: Turn GoPiGo Robot right
->s: Move GoPiGo Robot backward
->t: Increase speed
->g: Decrease speed
->j: Move servo left
->l: Move servo right
->k: Move servo home
->z: Exit
''';

""" --- for thread start ---"""

# acoid barrier
directionDevice = ''
class Avoid_barriers(threading.Thread):
	def __init__(self, no, interval):
		threading.Thread.__init__(self)
		self.no=no
		self.interval=interval
		self.ifdo = True;
	def run(self):
		while self.ifdo == True:
			print "avoid_barrier : " + str(self.ifdo)
			avoid_barrier(directionDevice)
			time.sleep(self.interval)
		print "stop thread"
	def stop(self):
		self.ifdo=False;
		print self.ifdo

# show distance or direction by LCD
class Show_value(threading.Thread):
	def __init__(self, no, interval):
		threading.Thread.__init__(self)
		self.no=no
		self.interval=interval
		self.ifdo = True;
	def run(self):
		while self.ifdo == True:
			#print "distance" + str(self.ifdo)
			#show_ultrasonic_distance()
			#get_direction()
			get_ibeacon_distance('home')
			#get_location() # thread of get ibeacon location
			time.sleep(self.interval)
	def stop(self):
		self.ifdo=False;
		print "Show_value : " + str(self.ifdo)

# Go to the send
class Go_ibeacon(threading.Thread):
	def __init__(self, no, interval):
		threading.Thread.__init__(self)
		self.no=no
		self.interval=interval
		self.ifdo = True;
	def run(self):
		global directionDevice
		while self.ifdo == True:
			go_to_turn_on(directionDevice)
			#show_direction(get_direction())
	def stop(self):
		self.ifdo=False;
		print "distance : " + str(self.ifdo)

# Send data to Freeboard
class Freeboard(threading.Thread):
	def __init__(self, no, interval):
		threading.Thread.__init__(self)
		self.no=no
		self.interval=interval
		self.ifdo=True
	def run(self):
		while self.ifdo == True:
			send_MTQQ()
			time.sleep(self.interval)
	def stop(self):
		self.ifdo=False
		print "Freeboard : " + str(self.ifdo)

# Detect sensor data and turn
class Turn_and_turn(threading.Thread):
	def __init__(self, no , interval):
		threading.Thread.__init__(self)
		self.no=no
		self.interval=interval
		self.ifdo=True
	def run(self):
		while self.ifdo == True:
			detect_and_turn()
			# get_direction()
			time.sleep(self.interval)
	def stop(self):
		self.ifdo=False
		print "Turn_and_turn : " + str(self.ifdo)

threadone=None
threadtwo=None
threadthree=None
def threadControl(status):
#	try:
	if status == True :
		#threadone.setDaemon(True)
		#global threadone
		#threadone=Avoid_barriers(random.randint(1, 10),1)
		global threadtwo
		#threadtwo=Show_value(random.randint(10, 20),1)
		threadtwo=Freeboard(random.randint(10, 20),1)
		global threadthree
		#threadthree=Go_ibeacon(random.randint(20, 30),1)
		threadthree=Turn_and_turn(random.randint(20, 30),1)
		#threadone.start()
		threadtwo.start()
		threadthree.start()
	else:
		#threadone.stop()
		threadtwo.stop()
		threadthree.stop()
		#threadone.join()
		#threadtwo.join()
		#threadthree.join()
#	except:
#		print "threat error"

""" --- for thread end ---"""	

# Show main control panel
def main():
	size_inc=22
	index=0
	for i in instructions.split('\n'):
		font = pygame.font.Font(None, 36)
		text = font.render(i, 1, (10, 10, 10))
		background.blit(text, (10,10+size_inc*index))
		index+=1

	# Blit everything to the screen
	screen.blit(background, (0, 0))
	pygame.display.flip()

	servo_pos=90
	while True:
		event = pygame.event.wait();
		global directionDevice
		global turning
		if (event.type == pygame.KEYUP):
			stop();
			continue;
		if (event.type != pygame.KEYDOWN):
			continue;	
		char = event.unicode;
		if char=='w':
			fwd()	;# Move forward
		elif char=='a':
			left_rot();	# Turn left
		elif char=='d':
			right_rot();# Turn Right
		elif char=='s':
			bwd();# Move back
		elif char=='t':
			increase_speed();	# Increase speed
		elif char=='g':
			decrease_speed();	# Decrease speed
		elif char=='j':
			servo_pos=servo_pos+10
			servo_turn(servo_pos)
		elif char=='l':
			servo_pos=servo_pos-10
			servo_turn(servo_pos)
		elif char=='k':
			servo_pos=90
			servo_turn(servo_pos)
		elif char=='q':
			set_speed()
		elif char=='p':
			#rtn = subprocess.call(["irsend", "SEND_ONCE", "/home/pi/lircd.conf", "KEY_POWER"])
			# rtn = subprocess.call(["irsend", "SEND_ONCE", "/home/pi/lircd_fan.conf", "KEY_POWER"])
			rtn = subprocess.call(["irsend", "SEND_ONCE", "/home/pi/lircd_light.conf", "BTN_0"])
		elif char=='o':
			# rtn = subprocess.call(["irsend", "SEND_ONCE", "/home/pi/lircd.conf", "KEY_VIDEO_PREV"])
			rtn = subprocess.call(["irsend", "SEND_ONCE", "/home/pi/lircd_light.conf", "BTN_1"])
		elif char=='i':
			# rtn = subprocess.call(["irsend", "SEND_ONCE", "/home/pi/lircd.conf", "KEY_VIDEO_NEXT"])
			c.update()
			print c.headingDegrees
		elif char=='u':
			# rtn = subprocess.call(["irsend", "SEND_ONCE", "/home/pi/lircd.conf", "KEY_VOLUMEUP"])
			grovepi.digitalWrite(buzzer, 0)
		elif char=='y':
			# rtn = subprocess.call(["irsend", "SEND_ONCE", "/home/pi/lircd.conf", "KEY_VOLUMEDOWN"])
			# grovepi.digitalWrite(blu_led, 1)
			grovepi.digitalWrite(buzzer, 0)
		elif char=='m':
			#stop()
			directionDevice = 'fan'
			turning = False
			threadControl(True)
		elif char=='n':
			#stop()
			threadControl(False)
		elif char=='z':
			print "\nExiting";		# Exit
			exit()
			stop()
			sys.exit();
		elif char=='x':
			get_ibeacon_distance('fan') # To get ibeacon 

# Turn servo
def servo_turn(pos):
	if pos>180:
		pos=180
	if pos<0:
		pos=0
	servo(pos)		# This function updates the servo with the latest positon.  Move the servo.
	time.sleep(.1)			# Take a break in between operations. 

# Show ultrasonic distance
# Abandon
def show_ultrasonic_distance():
	#LCD_show("Head : " + str(grovepi.ultrasonicRead(ultrasonic_ranger_left)))
	print "head : ", str(grovepi.ultrasonicRead(ultrasonic_ranger_head)),"right : ", str(grovepi.ultrasonicRead(ultrasonic_ranger_right)),"left : ", str(grovepi.ultrasonicRead(ultrasonic_ranger_left))

# Show direction of car
def show_direction(headingDegrees):
	print "Radians : \n" + str(headingDegrees)
	LCD_show("Radians : \n" + str(headingDegrees))

# Show ibeacon distance
def show_ibeacon_distance(target, distance1, distance2):
	LCD_show(target + " : " + str(distance1) + "\nSupport : " + distance2)

# Show message on LCD
def LCD_show(text):
	try:
		setText(text)
	except:
		print "NO TEXT"

# To avoid barrier
avoid_count = 0
def avoid_barrier(avoid_barrier):
	global avoid_count
	#print "head : ", str(grovepi.ultrasonicRead(ultrasonic_ranger_head)),"right : ", str(grovepi.ultrasonicRead(ultrasonic_ranger_right)),"left : ", str(grovepi.ultrasonicRead(ultrasonic_ranger_left))
	time.sleep(.2)
	set_speed()
	if(avoid_count < 5):
		print avoid_count
		try:
			if(grovepi.ultrasonicRead(ultrasonic_ranger_head) < 40):
				bwd()
				time.sleep(.2)
				print "avoid bwd"
				avoid_count += 1
				return False
			elif(grovepi.ultrasonicRead(ultrasonic_ranger_left) < 20):
				if(avoid_barrier == 'home'):
					fwd()
				else:
					bwd()
				time.sleep(.2)
				right()
				print "avoid right"
				avoid_count += 1
				return False
			elif(grovepi.ultrasonicRead(ultrasonic_ranger_right) < 20):
				if(avoid_barrier == 'home'):
					fwd()
				else:
					bwd()
				time.sleep(.2)
				left()
				print "avoid left"
				avoid_count += 1
				time.sleep(.2)
				return False
			else:
				print "avoid pass"
				#avoid_count = 0
				return True
		except:
			print "read ultrasonic fail"
			return False
	else:
		if(avoid_barrier == 'home'):
			fwd()
		else:
			bwd()
		time.sleep(.2)
		RightOrLeft = random.randint(0,10)
		if RightOrLeft > 0 and RightOrLeft < 5:
			time.sleep(.2)
			left()
		else:
			time.sleep(.2)
			right()
		#avoid_count = 0
		return True

# Show gopigo led
def showled(direction):
	led_off(gopigo_left_led)				# Turn off the left_LED
	led_off(gopigo_right_led)				# Turn off the right_LED
	if direction == "right":
		#Blink the LED
		led_on(gopigo_right_led)			# Turn on the right_LED
		#grovepi.digitalWrite(red_led,1)	# Send HIGH to switch on LED
		time.sleep(.05)
		led_off(gopigo_right_led)			# Turn off the right_LED
		#grovepi.digitalWrite(red_led,0)	# Send HIGH to switch on LED
		time.sleep(.05)
	elif direction == "left":
		#Blink the LED
		led_on(gopigo_left_led)				# Turn on the left_LED
		#grovepi.digitalWrite(red_led,1)	# Send HIGH to switch on LED
		time.sleep(.05)
		led_off(gopigo_left_led)			# Turn off the left_LED
		#grovepi.digitalWrite(red_led,0)	# Send HIGH to switch on LED
		time.sleep(.05) 
	elif direction == "back":
		#Blink the LED
		led_on(gopigo_right_led)			# Turn on the right_LED
		led_on(gopigo_left_led)				# Turn on the left_LED
		#grovepi.digitalWrite(red_led,1)	# Send HIGH to switch on LED
		time.sleep(.05)
		led_off(gopigo_right_led)			# Turn off the right_LED
		led_off(gopigo_left_led)			# Turn off the left_LED
		#grovepi.digitalWrite(red_led,0)	# Send HIGH to switch on LED
		time.sleep(.05)
	elif direction == "forward":
		#Blink the LED
		led_on(gopigo_right_led)			# Turn on the right_LED
		led_on(gopigo_left_led)				# Turn on the left_LED
		#grovepi.digitalWrite(red_led,1)	# Send HIGH to switch on LED

#abandon
def get_location(status):
	dev_id = 0
	max_distance = 10.0
	try:
		sock = bluez.hci_open_dev(dev_id)
		#print "ble thread started"

	except:
		print "error accessing bluetooth device..."
		sys.exit(1)

        blescan.hci_le_set_scan_parameters(sock)
        blescan.hci_enable_le_scan(sock)
	#while True:
	returnedList = blescan.parse_events(sock, 10)
	
	try:
		r1 = returnedList.get('12:3b:6a:1a:5f:46')
		r2 = returnedList.get('12:3b:6a:1a:62:36')
		r3 = returnedList.get('12:3b:6a:1a:62:38')
		while(r1 == None or r2 == None or r3 == None):
			returnedList = blescan.parse_events(sock, 10)
			r1 = returnedList.get('12:3b:6a:1a:5f:46')
			r2 = returnedList.get('12:3b:6a:1a:62:36')
			r3 = returnedList.get('12:3b:6a:1a:62:38')
			time.sleep(.2)
		for a in returnedList:
			print "key : " + a + "\tvalue : " + returnedList.get(a)
		if float(r1) > max_distance:
			r1 = max_distance
		if float(r2) > max_distance:
			r2 = max_distance
		if float(r3) > max_distance:
			r3 = max_distance
		print r1, r2, r3
		
		#get the location of car.
		if(r1 != None and r2 != None and r3 != None):
			d = 10.0
			i = 10.0
			j = 10.0
			x = (math.pow(float(r1), 2.0) - math.pow(float(r2), 2.0) + math.pow(d, 2.0)) / (2.0 * d)
			y = (pow(float(r1), 2.0) - pow(float(r3) ,2.0) - pow(x, 2.0) + (pow((x - i), 2.0)) + pow(j, 2.0)) / (2.0 * j)
			print "x : " + str(x)
			print "y : " + str(y)
	except:
		print "can't get device."

#abandon
def get_ibeacon_distance(device):
	dev_id = 0
	max_distance = 30.0
	try:
		sock = bluez.hci_open_dev(dev_id)

	except:
		print "error accessing bluetooth device..."
		sys.exit(1)

        blescan.hci_le_set_scan_parameters(sock)
        blescan.hci_enable_le_scan(sock)
	#while True:
	returnedList = blescan.parse_events(sock, 10)
	if(device=='fan'):
		try:
			fan = returnedList.get('12:3b:6a:1a:5f:4f')
			while(fan == None):
				returnedList = blescan.parse_events(sock, 10)
				fan = returnedList.get('12:3b:6a:1a:5f:4f')
				time.sleep(.2)
			#for a in returnedList:
			#	print "key : " + a + "\tvalue : " + returnedList.get(a)
			print "fan Distance : ", fan
			show_ibeacon_distance(device, fan, fanSupport)
			return fan
		except:
			print "can't get device."
	elif(device=='home'):
		try:
			home = returnedList.get('12:3b:6a:1a:62:36')
			homeSupport = returnedList.get('12:3b:6a:1a:62:38')
			while(home == None or homeSupport == None):
				returnedList = blescan.parse_events(sock, 10)
				home = returnedList.get('12:3b:6a:1a:62:36')
				homeSupport = returnedList.get('12:3b:6a:1a:62:36')
				time.sleep(.2)
			print "home Distance : ", home
			print "home Support Distance : ", homeSupport
			show_ibeacon_distance(device, home, homeSupport)
			return home
		except:
			print "can't get device."

# To get the direction of car
def get_direction():
	c=grove_compass_lib.compass()
	declinationAngle = -0.6907
	PI = math.pi
	heading = c.heading + declinationAngle
	#Correct for when signs are reversed.
	if(heading < 0):
		heading += 2.0*PI
	# Check for wrap due to addition of declination.
	if(heading > 2.0*PI):
		heading -= 2.0*PI;
	headingDegrees = heading * 180.0 / math.pi
	#print ("X:",c.x,"Y:",c.y,"X:",c.z,"Heading:",heading , " Radians:", headingDegrees)
	c.update()
	#show_direction(headingDegrees)
	# print c.headingDegrees
	time.sleep(.1)
	return headingDegrees

# abandon
def go_to_turn_on(device):
	global directionDevice
	global avoid_count
	print 'directionDevice : ', directionDevice
	try:
		if(directionDevice=='fan'):
			if(avoid_barrier(directionDevice)):
				if(float(get_ibeacon_distance(directionDevice)) > .65):
					degree = get_direction()
					set_speed()
					time.sleep(.2)
					if(degree < 160 and degree > 30):
						left()
						print "left"
						time.sleep(.5)
						stop()
					elif(degree < 360 and degree > 160):
						right()
						print "right"
						time.sleep(.5)
						stop()
					else:
						print "fwd"
						global avoid_count
						avoid_count = 0
						fwd()
				else:
					time.sleep(.5)
					stop()
					rtn = subprocess.call(["irsend", "SEND_ONCE", "/home/pi/lircd_fan.conf", "KEY_POWER"])
					time.sleep(2)
					rtn = subprocess.call(["irsend", "SEND_ONCE", "/home/pi/lircd_fan.conf", "KEY_POWER"])
					time.sleep(5)
					directionDevice = 'home'
					print "fan stop"
		elif(directionDevice=='home'):
			print directionDevice
			if(avoid_barrier(directionDevice)):
				if(float(get_ibeacon_distance(directionDevice)) > .57):
					degree = get_direction()
					set_speed()
					time.sleep(.2)
					if(degree < 160 and degree > 30):
						left()
						print "left"
						time.sleep(.5)
						stop()
					elif(degree < 360 and degree > 160):
						right()
						print "right"
						time.sleep(.5)
						stop()
					else:
						print "bwd"
						avoid_count = 0
						bwd()
				else:
					time.sleep(.2)
					stop()
					threadControl(False)
					print "home stop"
	except:
		print "pass"

# Set speed for gopigo
def set_speed():
	set_right_speed(110)
	set_left_speed(110)
	#set_right_speed(193)
	#set_left_speed(200)

# read data from sensor
def read_sensor(sensor):
	try:
		# if(sensor == 'frontDistance'):
		# 	ultrasonic_ranger = grovepi.ultrasonicRead(ultrasonic_ranger_head)
		# 	if ultrasonic_ranger > 0 and ultrasonic_ranger < 1000:
		# 		return ultrasonic_ranger
		# 	else:
		# 		return -1
		# elif(sensor == 'leftDistance'):
		# 	ultrasonic_ranger = grovepi.ultrasonicRead(ultrasonic_ranger_left)
		# 	if ultrasonic_ranger > 0 and ultrasonic_ranger < 1000:
		# 		return grovepi.ultrasonicRead(ultrasonic_ranger_left)
		# 	else:
		# 		return -1
		# elif(sensor == 'rightDistance'):
		# 	ultrasonic_ranger = grovepi.ultrasonicRead(ultrasonic_ranger_right)
		# 	if ultrasonic_ranger > 0 and ultrasonic_ranger < 1000:
		# 		return ultrasonic_ranger
		# 	else:
		# 		return -1
		if(sensor == 'temperature'):
			[temp, humidity] = grovepi.dht(dht, 0)
			if temp > 0 and temp < 100:
				return temp
			else:
				return -1
		elif(sensor == 'humidity'):
			[temp, humidity] = grovepi.dht(dht, 0)
			if humidity > 0 and humidity < 100:
				return humidity
			else:
				return -1
		elif(sensor == 'gas'):
			gas = grovepi.analogRead(gas_sensor)
			if gas > 0 and gas < 1000:
				return gas
			else:
				return -1 
		elif(sensor == 'airQuality'):
			air = grovepi.analogRead(air_quality_sensor)
			if air > 0 and air < 1000:
				return air
			else:
				return -1
		elif(sensor == 'flame'):
			flame = grovepi.analogRead(flame_sensor)
			if flame > 0 and flame < 5000:
				return flame
			else:
				return -1
		elif(sensor == 'compass'):
			return get_direction()
		elif(sensor == 'light'):
			light = analogRead(1)
			if light > 0 and light < 1000: 
				return light
			else:
				return -1
		# get Gas sensor value
		#gasValue = grovepi.analogRead(gas_sensor)
		# Calculate gas density - large value means more dense gas
		#density = (float)(gasValue / 1024)
		
		# get Air quality sensor value
		#airQualityValue = grovepi.analogRead(air_quality_sensor)
		# get Flame sensor value
		#flameValue = grovepi.analogRead(flame_sensor)

		#print ("gasValue = ", gasValue, " density =", density)
		#print ("airQualityValue = ", airQualityValue)
		#print ("flameValue = ", flameValue)
		#time.sleep(.5)
	except IOError:
		print "IO read error"
		return -1
	except TypeError:
		print "TypeError"
		return -1
	except:
		print "Unexpected error:", sys.exc_info()[0]
		return -1

'''For MQII Start'''
def on_connect(mqttc, obj, flags, rc):
	print("rc: "+str(rc))

def on_message(mqttc, obj, msg):
	#print('this is payload' + str(msg.payload))
	global payload
	payload = json.loads(msg.payload)
	# print('this is payload .... \n' + str(payload))


def on_publish(mqttc, obj, mid):
	print("mid: "+str(mid))

def on_subscribe(mqttc, obj, mid, granted_qos):
	print("Subscribed: "+str(mid)+" "+str(granted_qos))

def on_log(mqttc, obj, level, string):
	print(string)

# mqtt init
mqttc = mqtt.Client()
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe
# Uncomment to enable debug messages
#mqttc.on_log = on_log
mqttc.connect(routerIP, routerPort, 60)
mqttc.loop_start()
# print('App start .... \n')
mqttc.publish("Sensor",json.dumps(payload), retain=True)
mqttc.subscribe(topic)

def send_MTQQ():
	global payload
	global fanStatus
	global lightStatus
	global temperture_temp
	global temperature_retry
	global normal_temp
	global normal_hum
	# payload['frontDistance'] = read_sensor('frontDistance')
	# payload['leftDistance'] = read_sensor('leftDistance')
	# payload['rightDistance'] = read_sensor('rightDistance')
	# if float(temperture_temp) == 0:
	# 	temperture_temp = read_sensor('temperature')
	# 	payload['temperature'] = read_sensor('temperature')
	# elif abs(float(temperture_temp) - float(read_sensor('temperature'))) < 10:
	# 	payload['temperature'] = read_sensor('temperature')
	# 	temperture_temp = payload['temperature']
	# 	temperature_retry = 0
	# elif temperature_retry >= 3:
	# 	payload['temperature'] = read_sensor('temperature')
	# 	temperture_temp = payload['temperature']
	# 	temperature_retry = 0
	# elif abs(float(temperture_temp) - float(read_sensor('temperature'))) > 10:
	# 	temperature_retry = temperature_retry + 1
	if read_sensor('temperature') == "-1":
		payload['temperature'] = normal_temp
	else:
		payload['temperature'] = read_sensor('temperature')
		normal_temp = payload['temperature']
	if read_sensor('humidity') == "-1":
		payload['humidity'] = normal_hum
	else:
		payload['humidity'] = read_sensor('humidity')
		normal_hum = payload['humidity'] 
	payload['gas'] = read_sensor('gas')
	payload['airQuality'] = read_sensor('airQuality')
	payload['flame'] = read_sensor('flame')
	payload['compass'] = read_sensor('compass')
	payload['light'] = read_sensor('light')
	payload['fanstatus'] = fanStatus
	payload['lightstatus'] = lightStatus
	effectivePayload = json.dumps(payload)
	mqttc.publish("Sensor", effectivePayload, retain=True)
'''For MQII end'''

# Detect sensor data and do turn or send IR data
def detect_and_turn():
	global turning
	global payload
	global blu_led_light
	global red_led_light
	global lightStatus
	global fanStatus
	if turning == False:
		if float(payload['airQuality']) > 100 and fanStatus == False and red_led_light == False:
		# if payload['light'] < 200 and payload['light'] != '0' and red_led_light == False and fanStatus == False:
			print "fanStatus = True"
			grovepi.digitalWrite(red_led, 1)
			robot_turn(60, 'fan')
			fanStatus = True
			time.sleep(2)
			red_led_light = True
		elif payload['airQuality'] < 100 and float(payload['airQuality']) > 0 and fanStatus == True and red_led_light == True:
		# elif payload['light'] > 700 and red_led_light == True and fanStatus == True:
			print "fanStatus = False"
			grovepi.digitalWrite(red_led, 1)
			robot_turn(60, 'fan')
			fanStatus = False
			time.sleep(2)
			red_led_light = False

		elif payload['light'] < 70 and payload['light'] != '0' and blu_led_light == False and lightStatus == False:
			print "lightStatus = True"
			grovepi.digitalWrite(blu_led, 1)
			robot_turn(290, 'light')
			time.sleep(2)
			blu_led_light = True
			
		elif payload['light'] > 400 and blu_led_light == True and lightStatus == True:
			print "lightStatus = False"
			grovepi.digitalWrite(blu_led, 1)
			robot_turn(290, 'light')
			time.sleep(2)
			blu_led_light = False
	else:
		print 'turning'

# Control the robot turn
def robot_turn(angle, machine):
	global turning
	global lightStatus
	global fanStatus
	cTempArray = []
	currentHeadingDegrees = 0
	# try:
	# start action
	turning = True
	set_speed()
	for x in range(10):
		c.update()
		time.sleep(.1)
		cTempArray.append(c.headingDegrees)
	currentHeadingDegrees = sum(cTempArray) / int(len(cTempArray))
	# print currentHeadingDegrees
	# print c.headingDegrees
	c.update()
	start = currentHeadingDegrees
	origin = currentHeadingDegrees
	target = (start+angle)%360
	right_rot()
	while True:
		current = c.headingDegrees
		print start, target, current
		if target - start > 0:
			if current > target:
				while current > target:
					left_rot()
					time.sleep(0.15)
					stop()
					time.sleep(1)
					c.update()
					current = c.headingDegrees
				stop()
				break;
		else:
			if current > target and current < start - 5:
				while current > target:
					left_rot()
					time.sleep(0.15)
					stop()
					time.sleep(1)
					c.update()
					current = c.headingDegrees
				stop()
				break;
		c.update()
	time.sleep(2)
	
	if lightStatus == False and machine == "light":
		rtn = subprocess.call(["irsend", "SEND_ONCE", "/home/pi/lircd_light.conf", "BTN_0"])
		grovepi.digitalWrite(buzzer, 1)
		time.sleep(.1)
		grovepi.digitalWrite(buzzer, 0)
		time.sleep(.5)
		lightStatus = True
		LCD_show("light on")
		grovepi.digitalWrite(blu_led, 0)
		print "light on"
	elif lightStatus == True and machine == "light":
		rtn = subprocess.call(["irsend", "SEND_ONCE", "/home/pi/lircd_light.conf", "BTN_1"])
		grovepi.digitalWrite(buzzer, 1)
		time.sleep(.1)
		grovepi.digitalWrite(buzzer, 0)
		time.sleep(.5)
		grovepi.digitalWrite(buzzer, 1)
		time.sleep(.1)
		grovepi.digitalWrite(buzzer, 0)
		time.sleep(1)
		lightStatus = False
		LCD_show("light off")
		grovepi.digitalWrite(blu_led, 0)
		print "light off"
	elif fanStatus == False and machine == "fan":
		rtn = subprocess.call(["irsend", "SEND_ONCE", "/home/pi/lircd_light.conf", "BTN_0"])
		grovepi.digitalWrite(buzzer, 1)
		time.sleep(.1)
		grovepi.digitalWrite(buzzer, 0)
		time.sleep(.5)
		fanStatus = True
		LCD_show("fan on")
		grovepi.digitalWrite(red_led, 0)
		print "fan on"
	elif fanStatus == True and machine == "fan":
		rtn = subprocess.call(["irsend", "SEND_ONCE", "/home/pi/lircd_light.conf", "BTN_1"])
		grovepi.digitalWrite(buzzer, 1)
		time.sleep(.1)
		grovepi.digitalWrite(buzzer, 0)
		time.sleep(.5)
		grovepi.digitalWrite(buzzer, 1)
		time.sleep(.1)
		grovepi.digitalWrite(buzzer, 0)
		time.sleep(1)
		fanStatus = False
		grovepi.digitalWrite(red_led, 0)
		LCD_show("fan off")
		print "fan off"
	# back to start positon
	print "back start"
	start = c.headingDegrees
	target = origin
	right_rot()
	while True:
		current = c.headingDegrees
		print start, target, current
		if target - start > 0:
			if current > target:
				while current > target:
					left_rot()
					time.sleep(0.15)
					stop()
					time.sleep(1)
					c.update()
					current = c.headingDegrees
				stop()
				break;
		else:
			if current > target and current < start - 5:
				while current > target:
					left_rot()
					time.sleep(0.15)
					stop()
					time.sleep(1)
					c.update()
					current = c.headingDegrees
				stop()
				break;
		c.update()
	time.sleep(2)
	turning = False
	# except:
	# 	print "robot error"

# Exit
def exit():
	LCD_show("")
	led_off(gopigo_left_led)				# Turn off the left_LED
	led_off(gopigo_right_led)				# Turn off the right_LED
	grovepi.digitalWrite(blu_led, 0)
	grovepi.digitalWrite(red_led, 0)

if __name__ == "__main__":
	main()
