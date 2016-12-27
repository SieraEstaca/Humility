#!/usr/bin/python
# -*- coding: utf-8 -*-

# Requirements
import logging
import psutil
from threading import Thread, active_count
from termcolor import colored
from time import sleep, time
from os import system, name

# Functions
from rover import Rover 
from tools import Timer, color

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
	logging.debug("Starting")
	sleep(5)	
	while active_count() > 0:
		start_time = time()
		system('cls' if name == 'nt' else 'clear')
		print color.BOLD + '-------------------- LUNAR ROVER SOFTWARE ' + color.END
		print "%-20r %-10s" %("CPU (%)", psutil.cpu_percent(interval = None, percpu = True))
 		print " "
		print " "
		print color.BOLD + color.GREEN + 'GUIDANCE' + color.END 
		print "%-20r %-10s" %("time process", round(Rover.t_gui,3))
		print "%-20r %-10s %-20r %-10s" %("Distance error", round(Rover.distance_error,2), "Yaw error", round(Rover.angle_error,2))
		print "%-20r %-10s %-20r %-10s" %("left_obstacle", Rover.left_dist, "righ_obstacle", Rover.righ_dist) 
		print " "
		print color.BOLD + color.BLUE + 'NAVIGATION' + color.END
		print "%-20r %-10s" %("time process", round(Rover.t_nav,3))
		print "%-20r %-10s %-20r %-10s" %("current_X", round(Rover.Xcurrent,3), "current_Y", round(Rover.Ycurrent,3))
		print "%-20r %-10s %-20r %-10s" %("Ax", round(Rover.Ax,3), "Ay", round(Rover.Ay,3))
		print " " 
		print color.BOLD + color.PURPLE + 'CONTROL' + color.END
		print "%-20r %-10s" %("time process", round(Rover.t_con,3))
		print "%-20r %-10s %-20r %-10s" %("left_speed_ref", Rover.left_omega_ref, "right_ref", Rover.righ_omega_ref)
		print "%-20r %-10s %-20r %-10s" %("left_speed_mes", Rover.left_omega_mes, "right_mes", Rover.righ_omega_mes)
		print " "
		print " "
		print color.BOLD + color.RED + 'VISION' + color.END
		print "%-20r %-10s" %("time process", round(Rover.t_vis, 3))
		print " "
		print " "
		print "SiERA Rover, Team Humility"
		Timer(1.0, start_time)

except KeyboardInterrupt:
	Rover.exit = True
	logging.debug("Exiting")
	raise
