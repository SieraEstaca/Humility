import sys 
sys.path.insert(0, "/home/pi/Projects/Humility/Rpi_Software/Task/")

# Requirements
import logging 
import serial 
import cv2 
from sys import path 
from math import cos, sin, pi, fabs, atan2 
from time import sleep, time 
from picamera import PiCamera 
from picamera.array import PiRGBArray
from sense_hat import SenseHat

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

                # Define waypoints
                self.i = 0
                self.Xshift = [1.5, 1.5, 0, 0]
                self.Yshift = [0, 1.5, 0, 0]
                self.Wshift = atan2(self.Yshift[0]-0.0, self.Xshift[0]-0.0)
                
                # Position init
                self.Xcurrent = 0.0
                self.Ycurrent = 0.0
                self.Wcurrent = 0.0 # radians
		self.Wgyro = 0.0

                # Localisation error
                self.angle_error = atan2(self.Ycurrent-self.Yshift[0], self.Xcurrent-self.Xshift[0])
                
                # Rotation Speed
                self.left_omega_ref = 0.0
                self.righ_omega_ref = 0.0
                self.left_omega_mes = 0.0
                self.righ_omega_mes = 0.0
                
                # IRsensor parameters
                self.Precision = 0.15
                self.angle_precision = 5*pi/180.0
		self.obstacleDistanceStop = 150 # mm
                self.left_dist = self.obstacleDistanceStop + 100 # mm
                self.righ_dist = self.obstacleDistanceStop + 100 # mm 
		        
		# Avoidance manoeuvre
                self.angleAvoidance = 45*pi/180
                self.timingRecul = 2.0 #s
		self.timingRecover = 5 # s  35cm <=> 15RPM

		# Initialize SenseHat to save data
                self.sense = SenseHat()
                self.sense.set_imu_config(False, True, True) # compass disabled
                self.debut = time()
                
		# KALMAN Filter
		self.Kalman = Filter()
		
		# Rover Parameters
		self.R = 0.045 # m
		self.L = 0.21 #m

                # For multithreading
                self.fsm = "GoTo"
                self.sens = 'Right'
                self.exit = False
                self.obstacle = False
                self.GoTo = False
                self.Traj_false = False
                self.init_time = 4.95
                logging.basicConfig(level=logging.DEBUG,
                    format='[%(levelname)s] (%(threadName)-10s) %(message)s',
                    )

                                        
        def Guidance(self):
                start_time = time()
                
                # Init PID
                average_cmd = 15.0
                Kp = 100.0/pi
                Ki = 0.0
                Kd = 0.0*Kp/5.0
                angle = Corrector(P = Kp, I = 0.0, D = Kd, init_error = self.angle_error, wind_Up = False)

                # SetPoint saturation
                command = Command(20.0, 10.0)
                commandRecul = Command(-10.0, -20.0)

		# Switch mode 
                coeff = 2.0/3.0
                self.i = 0

		# Thread params
                period = 0.1
                counter = 0
                logging.debug("Starting")
                Timer(self.init_time, start_time)
                
                while not self.exit:                    
                        start_time = time()

                        # FINITE STATE MACHINE

                        if self.fsm == 'GoTo':
                                # Command loop to calculate new rpm setpoints
                                self.Wshift = atan2(self.Yshift[self.i]-self.Ycurrent, self.Xshift[self.i]-self.Xcurrent)
                                self.angle_error = Reset(self.Wshift - self.Wcurrent)                   
                                angl_cmd = angle.PID(self.angle_error, self.t_gui)
                                self.left_omega_ref = command.withSaturation(average_cmd - angl_cmd)
                                self.righ_omega_ref = command.withSaturation(average_cmd + angl_cmd)
                                
				# Obstacle detection
                                if self.left_dist < self.obstacleDistanceStop or self.righ_dist < self.obstacleDistanceStop:
                                        self.fsm = 'Recul'
                                        self.left_omega_ref = 0.0
                                        self.righ_omega_ref = 0.0
					left_dist_last = self.left_dist
					righ_dist_last = self.righ_dist
                                
                                # Target reached
                                if fabs(self.Xshift[self.i]-self.Xcurrent) < self.Precision and fabs(self.Yshift[self.i]-self.Ycurrent) < self.Precision:
                                        self.fsm = 'Turn'
					Wcurrent_last = self.Wcurrent
                                        self.left_omega_ref = 0.0
                                        self.righ_omega_ref = 0.0
                                        if self.i < 4:
                                                self.i = self.i + 1
                                                self.fsm = 'Turn'
                                                
                                # Last target reached
                                if self.i == len(self.Xshift):
                                        self.fsm = 'End'
                        
                        if self.fsm == 'Turn':                                  
                                # Command loop to calculate new Set Point
                                try:
                                        self.Wshift = atan2(self.Yshift[self.i]-self.Yshift[self.i-1], self.Xshift[self.i]-self.Xshift[self.i-1])
                                except ZeroDivisionError:
                                        if self.Y[shift.i] > self.Y[shift.i-1]:
                                                self.Wshift = +pi/2
                                        else:
                                                self.Wshift = -pi/2
                                self.angle_error = Reset(self.Wshift - Wcurrent_last)                   
                                if self.angle_error > 0:
                                        self.sens = 'Right'
                                        self.left_omega_ref = +average_cmd*coeff
                                        self.righ_omega_ref = -average_cmd*coeff
                                else:
                                        self.sens = 'Left'
                                        self.left_omega_ref = -average_cmd*coeff
                                        self.righ_omega_ref = +average_cmd*coeff

                                # To stop the loop
                                if fabs(Reset(self.Wshift-self.Wcurrent)) < self.angle_precision:
                                        self.fsm = 'GoTo'

			if self.fsm == 'Deviation':                           
                        	# Calculate command setpoints
                                if left_dist_last < righ_dist_last : # Left Obstacle
                                        self.Wshift = Reset(Wcurrent_last - self.angleAvoidance) 
                                        self.sens = 'Right'
                                        self.left_omega_ref = +average_cmd*coeff
                                        self.righ_omega_ref = -average_cmd*coeff
                                else : # Right Obstacle
                                        self.Wshift = Reset(Wcurrent_last + self.angleAvoidance)                  
                                        self.sens = 'Left'
                                        self.left_omega_ref = -average_cmd*coeff
                                        self.righ_omega_ref = +average_cmd*coeff

                                # To exit the loop
                                if fabs(Reset(self.Wshift-self.Wcurrent)) < self.angle_precision:
                                        self.fsm = 'Recover'
                        
                        if self.fsm == 'Recul':
                                counter = counter + 1

                                # Command new setpoints
                                self.left_omega_ref = commandRecul.withSaturation(-average_cmd)
                                self.righ_omega_ref = commandRecul.withSaturation(-average_cmd)
                                
				# Next step
                                if(counter*self.t_gui > self.timingRecul):
                                        counter = 0
                                        self.left_omega_ref = 0.0
                                        self.righ_omega_ref = 0.0
					Wcurrent_last = Reset(self.Wcurrent)
                                        self.fsm = 'Deviation'
                        
                        if self.fsm == 'Recover':
                                counter = counter + 1
                                # Command new setpoints
                                self.left_omega_ref = command.withSaturation(average_cmd)
                                self.righ_omega_ref = command.withSaturation(average_cmd)
                                
                                # If Obstacle
                                if self.left_dist < self.obstacleDistanceStop or self.righ_dist < self.obstacleDistanceStop:
                                        counter = 0
					self.fsm = 'Recul'
                                        self.left_omega_ref = 0.0
                                        self.righ_omega_ref = 0.0
					left_dist_last = self.left_dist
					righ_dist_last = self.righ_dist

                                # To exit the loop
                                if(counter*self.t_gui > self.timingRecover):
                                        counter = 0
                                        self.left_omega_ref = 0.0
                                        self.righ_omega_ref = 0.0
                                        self.obstacle = False
                                        self.fsm = 'GoTo'                       
                                
                        if self.fsm == 'End':
                                self.left_omega_ref = 0 #self.left_omega_ref + self.t_gui*(0.0-self.left_omega_ref)
                                self.righ_omega_ref = 0 #self.righ_omega_ref + self.t_gui*(0.0-self.righ_omega_ref)
                       
                        # Process control
                        Timer(period, start_time)
                        self.t_gui = time() - start_time
                
                logging.debug("Exiting")
                
        
        def Navigation(self):
                start_time = time()
		isKalmanActive = True

                # Thread setting
                period = 0.1
		convert = 2*pi/60.0
                logging.debug("Starting")
                Timer(self.init_time, start_time)

                while not self.exit:                    
                        start_time = time()

                        if not self.GoTo:

                                # TURN MODE
                                if self.fsm == 'Turn' or self.fsm == 'Deviation' :
                                        if self.sens == 'Right':
                                                omega_righ = -self.righ_omega_mes
                                                omega_left = +self.left_omega_mes
                                        else:
                                                omega_righ = +self.righ_omega_mes
                                                omega_left = -self.left_omega_mes

                                # RECUL MODE 
                                elif self.fsm == 'Recul':
                                        omega_righ = -self.righ_omega_mes
                                        omega_left = -self.left_omega_mes
                                else :
                                        omega_righ = +self.righ_omega_mes
                                        omega_left = +self.left_omega_mes

				if isKalmanActive == False :
                                        dmoy = self.R*self.t_nav*(omega_righ + omega_left)*0.5*convert
                                	temp = Reset(self.Wcurrent + 2*self.R*self.t_nav*convert*(omega_righ-omega_left)/self.L)
                               		self.Wcurrent = Reset(temp) 
                                	self.Xcurrent = self.Xcurrent + dmoy*cos(self.Wcurrent)
                                	self.Ycurrent = self.Ycurrent + dmoy*sin(self.Wcurrent)
                                if isKalmanActive == True :
					self.Kalman.Prediction(omega_righ, omega_left)
					self.Wcurrent, self.Wgyro = self.Kalman.Update()
					self.Wcurrent = Reset(self.Wcurrent)
                                        dmoy = self.R*self.t_nav*(omega_righ + omega_left)*0.5*convert
					self.Xcurrent = self.Xcurrent + dmoy*cos(self.Wcurrent)
                                	self.Ycurrent = self.Ycurrent + dmoy*sin(self.Wcurrent)

				# SAVE IN A FILE
                                yaw, pitch, roll = self.sense.get_orientation().values()
                                ax, ay, az = self.sense.get_accelerometer_raw().values()
                                fichier = open('data','a')
                                fichier.write("%.3f,%.3f,%.4f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.4f\n" % ((time()-self.debut), self.t_nav, yaw, pitch, roll, self.sense.get_temperature(), ax, ay, az, self.Xcurrent, self.Ycurrent, self.Wcurrent, omega_righ, omega_left, self.Wshift))
                                fichier.close()
        
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
#               cv2.namedWindow('Vision', cv2.WINDOW_NORMAL)
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
#                       blur = cv2.blur(image,(5,5))
#                       gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
#                       cv2.imshow('Vision', image)

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
