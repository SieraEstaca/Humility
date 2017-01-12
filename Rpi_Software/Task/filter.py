import numpy as np
from sense_hat  import SenseHat
from math import cos, sin, pi
from controller import Reset

class Filter():

        def __init__(self):
		self.sense = SenseHat()
                self.sense.set_imu_config(False, True, True)
		self.Wr = 0.0
		self.Wl = 0.0
                self.Ax = 0.0
                self.Ay = 0.0
		self.Yaw = 0.0
		self.pitch = 0.0
		self.roll = 0.0
		T = 0.1

                self.X = np.array([  	[0.0],
                     	           	[0.0],
                        		[0.0],
                                	[0.0],
                                	[0.0],
                                	[0.0]])

		# State transition matrix
		self.A = np.array([  	[1, 0, 0, T, 0, 0],
                                	[0, 1, 0, 0, T, 0],
                                	[0, 0, 1, 0, 0, T],
                                	[0, 0, 0, 0, 0, 0],
                                	[0, 0, 0, 0, 0, 0],
                                	[0, 0, 0, 0, 0, 1]])
		self.At = np.transpose(self.A)

		# Observation transition matrix
		self.C = np.array([  	[0, 0, 0, 1, 0, 1]])
		self.Ct = np.transpose(self.C)

		# Covariance matrix
		self.P = np.array([     [0, 0, 0, 0, 0, 0],
                                        [0, 0, 0, 0, 0, 0],
                                        [0, 0, 0, 0, 0, 0],
                                        [0, 0, 0, 0, 0, 0],
                                        [0, 0, 0, 0, 0, 0],
                                        [0, 0, 0, 0, 0, 0]])

		# State noise matrix
                self.Q = np.array([  	[1, 0, 0, 0, 0, 0],
                                	[0, 1, 0, 0, 0, 0],
                                	[0, 0, 1, 0, 0, 0],
                                	[0, 0, 0, 1, 0, 0],
                                	[0, 0, 0, 0, 1, 0],
                                	[0, 0, 0, 0, 0, 1]])

		# Sensors noise matrix
		self.R = np.array([	[1]])

	def Prediction(self, wR, wL):
		T = 0.1
		r = 0.045
		RPMtoRadPerSec = 2.0*pi/60.0
		L = 0.30
		
		# Inputs
		acceleration = self.sense.get_accelerometer_raw()
		Ax = acceleration['x']*10
		Ay = acceleration['y']*10
		self.Wr = wR*RPMtoRadPerSec
		self.Wl = wL*RPMtoRadPerSec
		self.Ax = self.Ax + T*(Ax - self.Ax)
		self.Ay = self.Ay + T*(Ay - self.Ay)	

		# Commands matrix
		U = np.array([  [self.Ax],
                                [self.Ay],
                                [self.Wr],
                                [self.Wl]])

		# Commands transition matrix
                B = np.array([  [T*T*0.0,0      ,0              , 0             ],
                                [0      ,T*T*0.0,0              , 0             ],
                                [0      ,0      ,r*T/L          , -r*T/L        ],
                                [0      ,0      ,r*cos(self.Yaw)*0.5 , r*cos(self.Yaw)*0.5],
                                [0      ,0      ,r*sin(self.Yaw)*0.5 , r*sin(self.Yaw)*0.5],
                                [0      ,0      ,0              , 0             ]])
		
		self.X = np.dot(self.A ,self.X) + np.dot(B,U)
		temp = np.dot(self.A, self.P)
		self.P = np.dot(temp, self.At) + self.Q

		return self.X[1,0], self.X[2,0], self.X[3,0], self.X[4,0], self.X[5,0]

	def Update(self):
		# Observation
		self.Yaw, self.pitch, self.roll = self.sense.get_orientation_radians().values()
                self.Yaw = Reset(self.Yaw)
		Z = np.array([[self.Yaw]])
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
		self.X = self.X + np.dot(K,S)

		self.Yaw = Reset(self.X[3,0]) 

		return self.X[1,0], self.X[2,0], self.X[3,0], self.X[4,0], self.X[5,0]

