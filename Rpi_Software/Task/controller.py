from math import pi

def Error(dX, dY):
        distance = dX*dX+dY*dY
        distance = pow(distance, 0.5)
        return distance

def Reset(angle):
	if angle < 0.0:
		angle += 2*pi
	elif angle > 2*pi:
		angle -= 2*pi
	return angle

class Corrector():

        def __init__(self, P, I, D, init_error, wind_Up = False, Imax = 0.1, Imin = -0.1):
                # Set all PID gains
                self.Kp = P
                self.Ki = I
                self.Kd = D

                # Set all PID terms
                self.P = 0.0
                self.I = 0.0
                self.D = 0.0

                # Additionnal parameters
		self.windUP = wind_Up
                self.Imax = Imax
                self.Imin = Imin
                self.last_error = init_error

        def PID(self, error, dt):
                # Derivate security
                if dt==0.0:
                        dt = 1.0

                # Corrector terms
                self.P  = self.Kp*error
                self.I += self.Ki*error*dt
                self.D  = self.Kd*(error-self.last_error)/dt

                # Integral wind-up
                if(self.windUP):
	                if   self.I > self.Imax:
				self.I = self.Imax
                	elif self.I < self.Imax:
				self.I = self.Imin

                # Setup for next derivate
                self.last_error = error

                # Command output
                command = self.P + self.I + self.D
                return command
		

class Command():

        def __init__(self, maxSP, minSP):
                self.maxSP = maxSP
                self.minSP = minSP

        def withSaturation(self, SP):
                if   SP > self.maxSP:
                        SP = self.maxSP
                elif SP < self.minSP:
                        SP = self.minSP
                return SP

