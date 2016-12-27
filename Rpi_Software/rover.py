# Requirements
import logging
import serial
import cv2
from math import cos, sin
from sense_hat 	import SenseHat
from time import sleep, time
from picamera import PiCamera
from picamera.array import PiRGBArray

# Functions
from tools import Timer


class Rover():

	def __init__(self):
		# Process frequency
		self.t_gui = 0.0
		self.t_nav = 0.0
		self.t_con = 0.0
		self.t_vis = 0.0

		# Accelerations init
	        self.Ax = 0.0
        	self.Ay = 0.0
	        self.Ax_last = 0.0
        	self.Ay_last = 0.0
		
	        # Position init
		self.dX = 0.0
		self.dY = 0.0
		self.th = 0.0
		
		# Rotation Speed
		self.left_omega_ref = 0.0
		self.righ_omega_ref = 0.0
		self.left_omega_mes = 0.0
		self.righ_omega_mes = 0.0
		
		# IRsensor parameters
		self.left_dist = 0
		self.righ_dist = 0
		
		# For multithreading
		self.exit = False
		self.init_time = 4.95
		logging.basicConfig(level=logging.DEBUG,
                    format='[%(levelname)s] (%(threadName)-10s) %(message)s',
                    )

					
	def Guidance(self):
		start_time = time()
		period = 0.1
		logging.debug("Starting")
		Timer(self.init_time, start_time)
		
		while not self.exit:			
			start_time = time()
			
			# Calcul control reference
			self.left_omega_ref = 10.000
			self.righ_omega_ref = 10.000
			
			# Process control
			Timer(period, start_time)
			self.t_gui = time() - start_time
		
		logging.debug("Exiting")
		
		
	def Navigation(self):
		start_time = time()
		sense = SenseHat()		
		period = 0.1
		logging.debug("Starting")
		Timer(self.init_time, start_time)
		
		while not self.exit:			
			start_time = time()
			  
			# Estimatation : IMU Kalman Filter
			acceleration = sense.get_accelerometer_raw()
			self.Ax = acceleration['x']*10
			self.Ay = acceleration['y']*10

			# Observation : WHEEL ODOMETRY
			dRg = self.left_omega_mes*0.10*float(self.t_nav)
			dRd = self.righ_omega_mes*0.10*float(self.t_nav)
			dMoy = (dRg+dRg)*0.5
			dDif = (dRd-dRg)*0.5
			self.th = self.th + dDif/0.3
			self.dX = self.dX + dMoy*cos(self.th)
			self.dY = self.dY + dMoy*sin(self.th)
	
			# Process control
			Timer(period, start_time)
			self.t_nav = time() - start_time    
        	
		logging.debug("Exiting")


	def Control(self):
		start_time = time()		
		sensorsData = serial.Serial('/dev/ttyACM0', baudrate = 9600, timeout = 0.25)
		send_dummy = ''
		left_omega = ''
		righ_omega = ''
		left_dist  = ''
		righ_dist  = ''
		period = 0.1
		counter = 0     
		logging.debug("Starting")
		Timer(self.init_time, start_time)

        	while not self.exit:
			start_time = time()

			# Send datas
			sendTime = time()
			send_dummy = str(self.left_omega_ref) + ',' + str(self.righ_omega_ref) + ',' + "true" + '\n' 
			try:
				sensorsData.flush()
				sensorsData.write(unicode(send_dummy))
				send_dummy = ''
			except SerialTimeoutException:
				break
			Timer(period, sendTime)

			# Get datas
			getTime = time()
			try:
				textline = sensorsData.readline()
				dataNums = textline.split(',')
				if len(dataNums)==4:
					left_omega = float(dataNums[0])
					self.left_omega_mes = left_omega
					righ_omega = float(dataNums[1])
					self.righ_omega_mes = righ_omega
					left_dist = float(dataNums[2])
					self.left_dist = left_dist
					righ_dist = float(dataNums[3])
					self.righ_dist = righ_dist 
				left_omega = ''
				righ_omega = ''	
				left_dist  = ''
				righ_dist  = ''
			except ValueError:
				pass
			Timer(period, getTime)	
			
			# Process control
			self.t_con = time() - start_time
		
		logging.debug("Exiting")

	def Vision(self):
		start_time = time()
		cols = 320
		rows = 240
		camera = PiCamera()
		camera.resolution = (cols, rows)
		camera.framerate = 30
		rawCapture = PiRGBArray(camera, size=(cols,rows))
		period = 0.1
		logging.debug("Starting")
		Timer(self.init_time, start_time)
		
		for frame in camera.capture_continuous(rawCapture, format = "bgr", use_video_port = True):
			start_time = time()

			# Image processing			
			image = frame.array
			blur = cv2.blur(image,(5,5))
			gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

			# Break the loop
			rawCapture.truncate(0)
			if self.exit:
				camera.close()
				break			

			# Process control
			Timer(period, start_time)		
			self.t_vis = time() - start_time

		logging.debug("Exiting")
