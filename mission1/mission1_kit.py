#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import DroneKit-Python
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
import time
import rospy
from pymavlink import mavutil # Needed for command message definitions
import math

def save_mission(aFileName):
    """
    Save a mission in the Waypoint file format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    missionlist = download_mission()
    output='QGC WPL 110\n'
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        file_.write(output)


def download_mission():
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist
    



def upload_mission(aFileName):
        """
        Upload a mission from a file.
        """
        #Read mission from file
        missionlist = readmission(aFileName)

        print "\nUpload mission from a file: %s" % aFileName
        #Clear existing mission from vehicle
        print ' Clear mission'
        cmds = vehicle.commands
        cmds.clear()
        #Add new mission to vehicle
        for command in missionlist:
            cmds.add(command)
        print ' Upload mission'
        vehicle.commands.upload()

def readmission(aFileName):
    """
    Load a mission from a file into a list.
    This function is used by upload_mission().
    """
    print "Reading mission from file: %s\n" % aFileName
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def changeMode(vehicle, mode):
    while vehicle.mode != VehicleMode(mode):
            vehicle.mode = VehicleMode(mode)
            time.sleep(0.5)
    return True

def add_last_waypoint_to_mission(                                       #--- Adds a last waypoint on the current mission file
        vehicle,            #--- vehicle object
        wp_Last_Latitude,   #--- [deg]  Target Latitude
        wp_Last_Longitude,  #--- [deg]  Target Longitude
        wp_Last_Altitude):  #--- [m]    Target Altitude
    """
    Upload the mission with the last WP as given and outputs the ID to be set
    """
    # Get the set of commands from the vehicle
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    # Save the vehicle commands to a list
    missionlist=[]

    #wpFirstObject = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
    #                       wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
    #
    #missionlist.append(wpFirstObject)

    for cmd in cmds:
        missionlist.append(cmd)

    # Modify the mission as needed. For example, here we change the
    wpLastObject = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                           wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
    missionlist.append(wpLastObject)

    # Clear the current mission (command is sent when we call upload())
    cmds.clear()

    #Write the modified mission and flush to the vehicle
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()
    
    return (cmds.count)


def clear_mission(vehicle):
    """
    Clear the current mission.
    """
    cmds = vehicle.commands
    vehicle.commands.clear()
    vehicle.flush()

    # After clearing the mission you MUST re-download the mission from the vehicle
    # before vehicle.commands can be used again
    # (see https://github.com/dronekit/dronekit-python/issues/230)
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()



if __name__ == '__main__':
    
    rospy.init_node('mission1_node', anonymous=True)

    #-- Connect to the vehicle
    print('Connecting...')
    vehicle = connect('udp:127.0.0.1:14551')
    #vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)



    #save_mission("waypoints1.txt")


    upload_mission("/home/mehmety7/catkin_ws/src/drone/scripts/waypoints1.txt")


    add_last_waypoint_to_mission(vehicle, vehicle.location.global_relative_frame.lat, 
                                           vehicle.location.global_relative_frame.lon, 
                                           vehicle.location.global_relative_frame.alt)
    print("Home waypoint added to the mission")
    time.sleep(1)

    arm_and_takeoff(10)
    changeMode(vehicle, "AUTO")
    time.sleep(2)


    while True:

        print ("Current WP: %d of %d "%(vehicle.commands.next, vehicle.commands.count))

        if vehicle.commands.next == 4:
            print("Su alma alanı")
        elif vehicle.commands.next == 5:
            print("Su bırakma bölgesi")        

        if vehicle.commands.next == vehicle.commands.count:
            print ("Final waypoint reached: go back home")
            time.sleep(10)
            #-- First we clear the flight mission
            clear_mission(vehicle)
            print ("Mission deleted")
            
            #-- We go back home
            #changeMode(vehicle,"RTL")
            #mode = "BACK"
            time.sleep(0.5)
            print("Mission Complated - Vehicle is Landing!!")
            changeMode(vehicle, "LAND")#yere çakılma sebebi

            break

        time.sleep(0.5)
