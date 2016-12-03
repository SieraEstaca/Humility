#!/usr/bin/python
# -*- coding: utf-8 -*-

'''
* 
* Space Rover Software Project 
*
* SiERA  : Estaca Robotics Association
* ESTACA : Transportation engineering school (France)
*
*
*
'''

import os
import logging
import serial
import cv2
from termcolor import colored, cprint
from math import cos, sin
from sense_hat import SenseHat
from threading import Thread, RLock, active_count
from time import sleep, clock, time

# Multithread tools
verrou = RLock()
exitapp = False
logging.basicConfig(level=logging.DEBUG,
                    format='[%(levelname)s] (%(threadName)-10s) %(message)s',
                    )




# Main class for rover guidance-navigation-control and vision
class Software():

	# Constructor
	def __init__(self):
        
		# Rotation Speed
		self.left_omega_ref = 0.0
		self.righ_omega_ref = 0.0
	        self.left_omega_mes = 0.0
		self.righ_omega_mes = 0.0		

		self.erreur = 0.0
        	self.th = 0.0
        
                # Accelerations init
                self.Ax = 0.0
                self.Ay = 0.0
                self.Ax_last = 0.0
                self.Ay_last = 0.0

                # Position init
                self.dX = 0.0
                self.dY = 0.0

		# Process frequency
                self.dT_navigation = 0.0
		self.dT_guidance = 0.0
	


        # Serial link between Raspberry Pi and Arduino Mega
        def Guidance(self):

                # Init thread
                logging.debug("Starting")
		period = 0.1
                sensorsData = serial.Serial('/dev/ttyACM0',9600)
                sleep(4)

                # Get arduino measurement
                while not exitapp:
			t = clock()

			# --------------- Initialize 
			send_dummy = ''
			left_omega = ''
			righ_omega = ''

			# --------------- Send datas
			start_time = time()
			self.left_omega_ref = 14.000
			self.righ_omega_ref = 12.000
			send_dummy = str(self.left_omega_ref) + ',' + str(self.righ_omega_ref) + '\n' 
			sensorsData.write(send_dummy)
			send_dummy = ''

                        # --------------- Waiting to get datas
			elapsed_time = time() - start_time
			if elapsed_time < period:
				pause = period - elapsed_time
				sleep(pause)

			# --------------- Get datas
			start_time = time()
                        try:
				textline = sensorsData.readline()
				dataNums = textline.split(',')
				if len(dataNums)==2:
					left_omega = float(dataNums[0])
					self.left_omega_mes = left_omega
					righ_omega = float(dataNums[1])
					self.righ_omega_mes = righ_omega
				left_omega = ''
				righ_omega = ''	
                        except ValueError:
				pass

			# --------------- Check frequency
			elapsed_time = time() - start_time
			if elapsed_time < period:
				pause = period - elapsed_time
				sleep(pause)
			
			self.dT_guidance = clock() - t

                # Thread stop
                logging.debug("Exiting")
 
 
 
        # Position estimation
        def Navigation(self):

                # Thread init
                logging.debug("Starting")
                sense = SenseHat()		
		sleep(1)

                # All mathematicals calculation 
                while not exitapp:			
                        t = clock()
			  
			# Acceleration
                        acceleration = sense.get_accelerometer_raw()
                        self.Ax = acceleration['x']*10
                        self.Ay = acceleration['y']*10

			# Odometry
                        dRg = self.left_omega_mes*0.10*float(self.dT_navigation)
			dRd = self.righ_omega_mes*0.10*float(self.dT_navigation)
                        dMoy = (dRg+dRg)*0.5
			dDif = (dRd-dRg)*0.5
                        self.th = self.th + dDif/0.3
                        self.dX = self.dX + dMoy*cos(self.th)
                        self.dY = self.dY + dMoy*sin(self.th)

			# Sampling Rate
                        self.dT_navigation = clock() - t

                # Thread stop
                logging.debug("Exiting")
                


        # Record all useful datas		
        def Record(self):

                # Thread init
                logging.debug("Starting")
                fichier = open('data.dat','w')
                sleep(5)

                # Record start
                while not exitapp:
                	Ax = str(self.Ax)
                        Ay = str(self.Ay)
                        fichier.write(Ax)
                        fichier.write(',')
                        fichier.write(Ay)
                        fichier.write(',')
                        fichier.write('\n')
                        sleep(1)
                        
                fichier.close()
                
                # Thread stop
                logging.debug("Exiting")



        # Print value in shell
        def Print(self):
                      
                # Thread init
                logging.debug("Starting")
                sleep(5)

                # Start print
                while not exitapp:
			os.system('cls' if os.name == 'nt' else 'clear')	
			print colored('NAVIGATION','blue')
			print "%-10r %-10s" %("t:", self.dT_navigation)
			print "%-10r %-10s %-10r %-10s" %("x:", self.dX, "y:", self.dY)
			print " "
                        print colored('CONTROL','red')
			print "%-10r %-10s" %("t:", self.dT_guidance)
			print "%-10r %-10s %-10r %-10s" %("left_ref:", self.left_omega_ref, "right_ref:", self.righ_omega_ref)
			print "%-10r %-10s %-10r %-10s" %("left_mes:", self.left_omega_mes, "right_mes:", self.righ_omega_mes)
			print " "
                        sleep(1)

		# Thread stop
                logging.debug("Exiting")




# MAIN PROGRAM 

try:

        # Define all objects
        Software = Software()
          
        # Create all threads
        Connect    = Thread(name = "Thread_1", target = Software.Guidance   )
        Navigation = Thread(name = "Thread_2", target = Software.Navigation )
#	Record     = Thread(name = "Thread_3", target = Software.Record     )
        Print      = Thread(name = "Thread_4", target = Software.Print      )

        # Daemonize thread
        Connect.daemon    = True
        Navigation.daemon = True
#	Record.daemon     = True
        Print.daemon      = True

        # Launch thread
        Connect.start()
        Navigation.start()
        #Record.start()
 	Print.start()

	# Main thread
	sleep(1)
	print colored('Starting in five seconds ...','red')	
        while active_count() > 0:
                        sleep(0.1)

except KeyboardInterrupt:
          exitapp = True
          raise



