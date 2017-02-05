import numpy as np
from sense_hat  import SenseHat
from math import cos, sin, pi, pow
from controller import Reset

class Filter():

        def __init__(self):
		# Initialize SenseHat
		self.sense = SenseHat()
                self.sense.set_imu_config(False, True, True) # compass disabled
		
		# Rover constants
		self.g = 9.81 	# m/s2
		self.r = 0.045 	# Wheel Radius (m)
		self.L = 0.21 	# WheelBase (m)		
		self.Wr = 0.0	# RPM
		self.Wl = 0.0	# RPM

		# Calibration SenseHat
                self.Ax = 0.0	# m/s2
                self.Ay = 0.0	# m/s2
		T = 0.1 	# s		
		# self.Yaw = 0.0	# radians
		# self.Yaw_filt=0 # radians

		# Initialize State vector
                self.X = np.array([  	[0.0],
                     	           	[0.0]])

		# State transition matrix
		self.A = np.array([  	[1, 0], 
					[0, 0]])
		self.At = np.transpose(self.A)

		# Observation transition matrix
		self.C = np.array([  	[1, 1]])
		self.Ct = np.transpose(self.C)

		# Covariance matrix
		self.P = np.array([     [0, 0],
					[0, 0]])

		# Sensors standard deviation
		sigmaOdo = 1*(pi/180) #rad
		sigmaDerive = (0/3600)*(pi/180) #rad  # 20 deg/h
		sigmaGyro = 25*(pi/180) #rad

		# State noise matrix
                self.Q = np.array([  	[pow(sigmaOdo,2), 0],
					[0, pow(sigmaDerive,2)]])

		# Commands transition matrix
		self.B = np.array([	[(4*pi*T*self.r)/(60*self.L), -(4*pi*T*self.r)/(60*self.L)],
					[0 			    , 0]])

		# Sensors noise matrix
		self.R = np.array([	[pow(sigmaGyro,2)]])


	def Prediction(self, wR, wL):
		T = 0.1
		RPMtoRadPerSec = 2.0*pi/60.0
		
		# Inputs
		acceleration = self.sense.get_accelerometer_raw()
		Ax = acceleration['x']*self.g
		Ay = acceleration['y']*self.g
		self.Wr = wR*RPMtoRadPerSec
		self.Wl = wL*RPMtoRadPerSec

		# Commands matrix
		U = np.array([  [self.Wr],
                                [self.Wl]])		

		self.X = np.dot(self.A ,self.X) + np.dot(self.B,U)
		temp = np.dot(self.A, self.P)
		self.P = np.dot(temp, self.At) + self.Q 

	def Update(self):
		# Observation
		yaw, pitch, roll = self.sense.get_orientation_radians().values()
		Z = np.array([[Reset(-yaw)]])

		# Gain
		temp = np.dot(self.C, self.P)
		dummy = np.dot(temp, self.Ct) + self.R
		dummy = np.linalg.inv(dummy)
		temp = np.dot(self.P, self.Ct)
		K = np.dot(temp, dummy)

		# Corect covariance matrix
		self.P = self.P - np.dot(K, self.C, self.P)

		# Innovation
		S = Z - np.dot(self.C, self.X)

		# Correction
		self.X = self.X + np.dot(K,S)
		self.X[0,0] = Reset(self.X[0,0]) 
		
		return self.X[0,0], -yaw

