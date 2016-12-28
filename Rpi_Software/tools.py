from time import time, sleep

# Synchronized all thread (or a specific task) to a precise frequency
def Timer(period, start_time):
	elapsed_time = time() - start_time
	if elapsed_time < period:
		pause = period - elapsed_time
		sleep(pause)

def getDistance(dX, dY):
	distance = dX*dX+dY*dY
	distance = pow(distance, 0.5)
	return distance	

# For main thread
class color:
	PURPLE = '\033[95m'
	CYAN = '\033[96m'
	BLUE = '\033[36m'
	BLUE = '\033[94m'
	GREEN = '\033[92m'
	YELLOW = '\033[93m'
	RED = '\033[91m'
	BOLD = '\033[1m'
	UNDERLINE = '\033[4m'
	END = '\033[0m'

# Guidance-Control corrector to master trajectory
class Corrector():
	
	def __init__(self, P, I, D, init_error, Imax = 0.1, Imin = -0.1):

		# Set all PID gains
		self.Kp = P
		self.Ki = I
		self.Kd = D

		# Set all PID terms
		self.P = 0.0
		self.I = 0.0
		self.D = 0.0

		# Additionnal parameters
		self.Imax = Imax 
		self.Imin = Imin
		self.last_error = init_error

	def PID(self, error, dt):

		# Derivate security
		if dt==0.0:
			dt = 0.1

		# Corrector terms
		self.P  = self.Kp*error
		self.I += self.Ki*error*dt
		self.D  = self.Kd*(error-self.last_error)/dt

		# Integral wind-up
		if   self.I > self.Imax:
			self.I = self.Imax
		elif self.I < self.Imax:
			self.I = self.Imin

		# Setup for next derivate	
		self.last_error = error

		# Command output
		command = self.P + self.I + self.D
		return command 

class Saturation(): 

	def __init__(self, maxSP, minSP):
		self.maxSP = maxSP 
		self.minSP = minSP

	def Command(self, SP):

		# speed command saturation
		if   SP > self.maxSP:
			SP = self.maxSP
		elif SP < self.minSP:
			SP = self.minSP

		return SP

# Navigation function to estimate a better position
class Filter():

	def __init__(self, sig1, sig2, dt):
		self.sig1_filt = 0.0
		self.sig2_filt = 0.0
		self.moving_average(sig1, sig2, dt)

	def moving_average(self, sig1, sig2, dt):
		self.sig1_filt = self.sig1_filt + dt*(sig1 - self.sig1_filt)
		self.sig2_filt = self.sig2_filt + dt*(sig2 - self.sig2_filt)
		return self.sig1_filt, self.sig2_filt
	
# Usefull to get some values to developp MATLAB model for example			
def Record():
	fichier = open('data.dat','w')
	Ax = str(self.Ax)
	Ay = str(self.Ay)
	fichier.write(Ax)
	fichier.write(',')
	fichier.write(Ay)
	fichier.write(',')
	fichier.write('\n')
	sleep(1)	
	fichier.close()
