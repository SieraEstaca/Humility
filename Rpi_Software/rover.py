import sys
sys.path.insert(0, "/home/pi/Projects/Humility/Rpi_Software/Task/")

# Requirements
import logging
import serial
import cv2
from sys import path
from math import cos, sin, pi, fabs, atan2
from sense_hat 	import SenseHat
from time import sleep, time
from picamera import PiCamera
from picamera.array import PiRGBArray

# Functions made by ourself
from tools import Timer
from filter import Filter
from controller import Error, Reset, Corrector, Command, Derivate
from uart import Arduino

class Rover():

	def __init__(self):

		# Process frequency
		self.t_gui = 0.0
		self.t_nav = 0.0
		self.t_con = 0.0
		self.t_vis = 0.0

		# Accelerations init
	        self.Vx = 0.0
        	self.Vy = 0.0

		# Target point
		self.Xshift = 10
		self.Yshift = 10
		self.Wshift = -45*pi/180		

	        # Position init
		self.Xcurrent = 0.0
		self.Ycurrent = 0.0
		self.Wcurrent = 0.0

		# Localisation error
		# self.distance_error = Error(self.Xshift-self.Xcurrent, self.Yshift-self.Ycurrent)
		self.angle_error = atan2(self.Ycurrent-self.Yshift, self.Xcurrent-self.Xshift)
		
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
		self.GoTo = False
		self.Traj_false = False
		self.init_time = 4.95
		logging.basicConfig(level=logging.DEBUG,
                    format='[%(levelname)s] (%(threadName)-10s) %(message)s',
                    )

					
	def Guidance(self):
		start_time = time()
		
		# Init PID
		# distance = Corrector(P = 2.6, I = 0.65, D = 0.0, init_error = self.distance_error, wind_Up = True)
		average_cmd = 16.5
		angle = Corrector(P = 5.0, I = 0.0, D = 1.0, init_error = self.angle_error, wind_Up = True)

		# SetPoint saturation
		command = Command(25.0, 5.0)

		# Trajectory checker
#		trajectory = Derivate(self.distance_error)

		# Thread setting
		period = 0.1
		logging.debug("Starting")
		Timer(self.init_time, start_time)
		
		while not self.exit:			
			start_time = time()

			# GoTo waypoint up to ...
			if fabs(self.Xshift-self.Xcurrent) < 0.8 and fabs(self.Yshift-self.Ycurrent) < 0.8:
				if self.left_omega_ref < 5.0 and self.righ_omega_ref < 5.0: 
					self.GoTo = True
				self.left_omega_ref = self.left_omega_ref + self.t_gui*(0.0-self.left_omega_ref)
				self.righ_omega_ref = self.righ_omega_ref + self.t_gui*(0.0-self.righ_omega_ref)

#			self.Traj_false =  trajectory.Derivate(self.distance_error, self.t_gui)

			# Define new angle Set Point
			self.Wshift = atan2(self.Yshift-self.Ycurrent, self.Xshift-self.Xcurrent)

			# Error
			self.Wcurrent = Reset(self.Wcurrent)
			self.angle_error = self.Wshift - self.Wcurrent			

			# PID
			# dist_cmd = distance.PID(self.distance_error, self.t_gui)
			angl_cmd = angle.PID(self.angle_error, self.t_gui)

			# Commands
			if not self.GoTo:
				self.left_omega_ref = command.withSaturation(average_cmd - angl_cmd)
				self.righ_omega_ref = command.withSaturation(average_cmd + angl_cmd)
			
			# Process control
			Timer(period, start_time)
			self.t_gui = time() - start_time
		
		logging.debug("Exiting")
		
	
	def Navigation(self):
		start_time = time()

		# Init IMU
		Kalman = Filter()
		sense = SenseHat()		
		sense.set_imu_config(False, True, True)

		# Thread setting
		period = 0.1
		logging.debug("Starting")
		Timer(self.init_time, start_time)

		while not self.exit:			
			start_time = time()

			Kalman.Prediction(self.left_omega_mes, self.righ_omega_mes)
			self.Xcurrent, self.Ycurrent, self.Wcurrent, self.Vx, self.Vy = Kalman.Update()

			# Process control
			Timer(period, start_time)
			self.t_nav = time() - start_time    
        	
		logging.debug("Exiting")


	def Control(self):
		start_time = time()		

		# Init serial communication with Arduino 
		arduino = Arduino(period = 0.1)
		     
		logging.debug("Starting")
		Timer(self.init_time, start_time)

        	while not self.exit:
			start_time = time()
			
			# Bidirectionnal link with Arduino
			arduino.sendDatas(self.left_omega_ref, self.righ_omega_ref)
			self.left_omega_mes, self.righ_omega_mes, self.left_dist, self.righ_dist = arduino.getDatas()

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
