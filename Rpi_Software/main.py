#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
sys.path.insert(0, "/home/pi/Projects/Humility/Rpi_Software/Task/")

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
	Traj = True
	to_angle = 180/3.14
	grid = '--------------------'	
	while active_count() > 0:
		start_time = time()
		Traj != Rover.Traj_false
#		system('cls' if name == 'nt' else 'clear')
		print color.BOLD + grid + ' MARS ROVER SOFTWARE ' + grid + grid + grid + color.END
		print "%-20r %-10s" %("CPU (%)", psutil.cpu_percent(interval = None, percpu = True))
		print "%-20r %-10s %-20r %-10s" %("Target Reached", Rover.GoTo, "Trajectory", Traj)
 		print color.BOLD + grid + grid + grid + grid + grid + color.END
		print " "
		print color.BOLD + color.GREEN + 'GUIDANCE' + color.END 
		print "%-20r %-10s" %("time process", round(Rover.t_gui,3))
		print "%-20r %-10s %-20r %-10s" %("Distance error", round(Rover.Xcurrent,2), "Yaw error", round(Rover.angle_error*to_angle,2))
		print "%-20r %-10s %-20r %-10s" %("left_obstacle", Rover.left_dist, "righ_obstacle", Rover.righ_dist) 
		print " "
		print color.BOLD + color.BLUE + 'NAVIGATION' + color.END
		print "%-20r %-10s" %("time process", round(Rover.t_nav,3))
		print "%-20r %-10s %-20r %-10s %-20r %-10s" %("Xshift", Rover.Xshift, "Yshift", Rover.Yshift, "Heading shift", round(Rover.Wshift*to_angle,3))
		print "%-20r %-10s %-20r %-10s %-20r %-10s" %("Xcurrent", round(Rover.Xcurrent,3), "Ycurrent", round(Rover.Ycurrent,3), "Heading current", round(Rover.Wcurrent*to_angle,3))
#		print "%-20r %-10s %-20r %-10s" %("Ax", round(Rover.Ax,3), "Ay", round(Rover.Ay,3))
		print " " 
		print color.BOLD + color.PURPLE + 'CONTROL' + color.END
		print "%-20r %-10s" %("time process", round(Rover.t_con,3))
		print "%-20r %-10s %-20r %-10s" %("left_speed_ref", round(Rover.left_omega_ref,3), "right_ref", round(Rover.righ_omega_ref,3))
		print "%-20r %-10s %-20r %-10s" %("left_speed_mes", round(Rover.left_omega_mes,3), "right_mes", round(Rover.righ_omega_mes,3))
		print " "
		print color.BOLD + grid + grid + grid + grid + grid + color.END
		print " "
		print color.BOLD + color.RED + 'VISION' + color.END
		print "%-20r %-10s" %("time process", round(Rover.t_vis, 3))
		print " "
		print " "
#		print color.BOLD + "SiERA Rover, Team Humility" + color.END
		Timer(1.0, start_time)

except KeyboardInterrupt:
	Rover.exit = True
	logging.debug("Exiting")
	raise
