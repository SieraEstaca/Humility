# Requirements
import logging
import serial
import cv2
from math import cos, sin, pi
from sense_hat 	import SenseHat
from time import sleep, time
from picamera import PiCamera
from picamera.array import PiRGBArray

# Functions
from tools import Timer, Filter, getDistance, Corrector, Saturation


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

		# Target point
		self.Xshift = 10
		self.Yshift = 10
		self.Wshift = 45*pi/180		

	        # Position init
		self.Xcurrent = 0.0
		self.Ycurrent = 0.0
		self.Wcurrent = 0.0

		# Localisation error
		self.distance_error = getDistance(self.Xshift-self.Xcurrent, self.Yshift-self.Ycurrent)
		self.angle_error = self.Xcurrent - self.Xshift
		
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
		
		# Init PID
		distance = Corrector(0.7, 0.0, 0.3, self.distance_error)
		angle = Corrector(0.7, 0.0, 0.3, self.angle_error)		

		# SetPoint saturation
		saturation = Saturation(15.0, 8.0)

		period = 0.1
		logging.debug("Starting")
		Timer(self.init_time, start_time)
		
		while not self.exit:			
			start_time = time()

			# Error
			self.distance_error = getDistance(self.Xshift - self.Xcurrent, self.Yshift - self.Ycurrent)
			self.angle_error = self.Wshift - self.Wcurrent			

			# PID
			dist_cmd = distance.PID(self.distance_error, self.t_gui)
			angl_cmd = angle.PID(self.angle_error, self.t_gui)

			# Commands
			self.left_omega_ref = saturation.Command(dist_cmd - angl_cmd)
			self.righ_omega_ref = saturation.Command(dist_cmd + angl_cmd)
			
			# Process control
			Timer(period, start_time)
			self.t_gui = time() - start_time
		
		logging.debug("Exiting")
		
		
	def Navigation(self):

		start_time = time()
		filter = Filter(self.Ax, self.Ay, self.t_nav)
		sense = SenseHat()		
		period = 0.1
		logging.debug("Starting")
		Timer(self.init_time, start_time)
		
		while not self.exit:			
			start_time = time()

			# Observation model : Wheel Odometry
                        dRg = self.left_omega_mes*0.10*float(self.t_nav)
                        dRd = self.righ_omega_mes*0.10*float(self.t_nav)
                        dMoy = (dRg+dRg)*0.5
                        dDif = (dRd-dRg)*0.5
                        self.Wcurrent = self.Wcurrent + dDif/0.3
                        self.Xcurrent = self.Xcurrent + dMoy*cos(self.Wcurrent)
                        self.Ycurrent = self.Ycurrent + dMoy*sin(self.Wcurrent)
			  
			### Prediction
			# 1 - Estimatation model : IMU Kalman Filter
			acceleration = sense.get_accelerometer_raw()
			self.Ax , self.Ay = filter.moving_average(acceleration['x']*10, acceleration['y']*10, self.t_nav)

			# 2 - Error Covariance

			### Correction
			# 1 - Compute Kalman gain

			# 2 - Update estimation

			# 3 - Update error covariance matric

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
#		cv2.namedWindow('Vision', cv2.WINDOW_NORMAL)
		cols = 320
		rows = 240
		camera = PiCamera()
		camera.resolution = (cols, rows)
		camera.framerate = 10
		rawCapture = PiRGBArray(camera, size=(cols,rows))
		period = 0.1
		logging.debug("Starting")
		Timer(self.init_time, start_time)
		
		for frame in camera.capture_continuous(rawCapture, format = "bgr", use_video_port = True):
			start_time = time()

			# Image processing			
			image = frame.array
#			blur = cv2.blur(image,(5,5))
#			gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
#			cv2.imshow('Vision', image)

			# Break the loop
			rawCapture.truncate(0)
			if self.exit:
				camera.close()
				break			

			# Process control
			Timer(period, start_time)		
			self.t_vis = time() - start_time 

		cv2.destroyAllWindows()
		logging.debug("Exiting")
