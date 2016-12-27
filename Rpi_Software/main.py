#!/usr/bin/python
# -*- coding: utf-8 -*-

# Requirements
from threading import Thread, active_count
from termcolor import colored
from time import sleep, time
from os import system, name

# Functions
from rover import Rover 
from tools import Timer

try:
	# Initialize
	Rover = Rover()
          
	# Create all threads
	Guidance = Thread(name = "GUIDANCE", target = Rover.Guidance)
	Navigation = Thread(name = "NAVIGATION", target = Rover.Navigation)
	Control = Thread(name = "CONTROL", target = Rover.Control)
	Vision = Thread(name = "VISION", target = Rover.Vision)
	
	# Daemonize thread
	Guidance.daemon	= True
    	Navigation.daemon = True
	Control.daemon = True
	Vision.daemon = True

	# Launch thread
	Guidance.start()
    	Navigation.start()
    	Control.start()
	Vision.start()
	
	# Print data thread
	sleep(5)	
	while active_count() > 0:
		start_time = time()
		system('cls' if name == 'nt' else 'clear')
		print colored('GUIDANCE', 'green')
		print "%-20r %-5s" %("time process", Rover.t_gui)
		print "%-20r %-20s %-20r %-20s" %("shift_X", Rover.dX, "shift_Y", Rover.dY)
		print "%-20r %-20s %-20r %-20s" %("left_obstacle", Rover.left_dist, "righ_obstacle", Rover.righ_dist) 
		print " "
		print colored('NAVIGATION', 'blue')
		print "%-20r %-5s" %("time process", Rover.t_nav)
		print "%-20r %-20s %-20r %-20s" %("current_X", Rover.dX, "current_Y", Rover.dY)
		print "%-20r %-20s %-20r %-20s" %("left_obstacle", Rover.left_dist, "righ_obstacle", Rover.righ_dist) 
		print " "
		print colored('VISION', 'magenta')
		print "%-20r %-5s" %("time process", Rover.t_vis)
		print " "
		print colored('CONTROL', 'red')
		print "%-20r %-5s" %("time process", Rover.t_con)
		print "%-20r %-20s %-20r %-20s" %("left_speed_ref", Rover.left_omega_ref, "right_ref", Rover.righ_omega_ref)
		print "%-20r %-20s %-20r %-20s" %("left_speed_mes", Rover.left_omega_mes, "right_mes", Rover.righ_omega_mes)
		print " "
		Timer(1.0, start_time)

except KeyboardInterrupt:
	Rover.exit = True
	raise
