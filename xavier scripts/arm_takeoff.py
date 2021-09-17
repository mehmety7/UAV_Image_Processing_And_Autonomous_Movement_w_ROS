#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
print "Start simulator (SITL)"
import dronekit_sitl
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()
"""
# Import DroneKit-Python
from dronekit import connect, VehicleMode
import time



#1. yöntem udp
iha = connect('udp:127.0.0.1:14551')

#2.yöntem tcp
#iha = connect('tcp:127.0.0.1:5762', wait_ready=True)


def arm_and_takeoff(alt): #altitude
	while iha.is_armable==False:
		print("Not ready to arm!")
		time.sleep(1)
	print("Ready to arm!")
	
	iha.mode=VehicleMode("GUIDED")
	while iha.mode=="GUIDED":
		print("Mode switching to mode GUIDED!")
		time.sleep(1.5)

	print("Drone switched to mode GUIDED!")
	iha.armed=True
	while not iha.armed:
		print("Waiting for arming!")
		time.sleep(1)
	
	print("Drone armed")

	iha.simple_takeoff(alt)
	while iha.location.global_relative_frame.alt <= alt*0.95:
		print("Current height: {}".format(iha.location.global_relative_frame.alt))
		time.sleep(0.5)

	print("Reached to target altitude!")
	time.sleep(1)

	print("Drone is landing!")
	iha.mode=VehicleMode("LAND")

arm_and_takeoff(10)
