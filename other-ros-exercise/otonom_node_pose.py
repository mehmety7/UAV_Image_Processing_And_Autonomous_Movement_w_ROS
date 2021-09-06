#!/usr/bin/env python


import rospy
from std_msgs.msg import String, Float64
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import *
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import Odometry

from math import cos, sin, asin, sqrt, atan2
import math

# ################################################################################################################################################
# -------------------------------------------------------------- GLOBAL DEFINITIONS ---------------------------------------------------
# ################################################################################################################################################

pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

# define msgs
correction_vector_g = Pose() 
local_offset_pose_g = Point() # float x,y,z
current_pose_g = Odometry()
waypoint_g = PoseStamped()

# States info
okConnected = False
okArmed = False
okGuided = False

# initialize heading of drone
current_heading_g = 0.0
local_offset_g = 0.0
correction_heading_g = 0.0
local_desired_heading_g = 0.0

# define waypoint structure
class Waypoint():
	def __init__(self, x, y, z, psi):
		self.x = x
		self.y = y
		self.z = z
		self.psi = psi

# position of pools
#red(-30.4,-11.5) blue(-30.9,16.7)	
redx = 0.0
redy = 0.0
bluex = 0.0
bluey = 0.0

# ################################################################################################################################################
# -------------------------------------------------------------- STATES INSTRUCTIONS ---------------------------------------------------
# ################################################################################################################################################

def setArm():
	global waypoint_g

	set_destination(0,0,0,0)
	for i in range(100):
		pose_pub.publish(waypoint_g)
		rospy.sleep(0.01)

	rospy.wait_for_service('/mavros/cmd/arming')
	try:
		armService = rospy.ServiceProxy(
			'/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
		armService(True)
	except rospy.ServiceException as e:
		print ("Service arm call failed: %s" % e)



def setTakeoffMode(alt):
	global waypoint_g

	set_destination(0,0,int(alt),0)
	for i in range(100):
		pose_pub.publish(waypoint_g)
		rospy.sleep(0.01)

	rospy.wait_for_service('/mavros/cmd/takeoff')
	try:
		takeoffService = rospy.ServiceProxy(
			'/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
		takeoffService(altitude=int(alt), latitude=0,
		           longitude=0, min_pitch=0, yaw=0)
	except rospy.ServiceException as e:
		print ("Service takeoff call failed: %s" % e)



def setLandMode():
	rospy.wait_for_service('/mavros/cmd/land')
	try:
		takeoffService = rospy.ServiceProxy(
			'/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
		takeoffService(altitude=0, latitude=0,
		           longitude=0, min_pitch=0, yaw=0)
	except rospy.ServiceException as e:
		print ("Service takeoff call failed: %s" % e)



def setGuidedMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        #http://wiki.ros.org/mavros/CustomModes for custom modes
        isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
    except rospy.ServiceException as e:
        print ("service set_mode GUIDED call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled"%e)


# ################################################################################################################################################
# -------------------------------------------------------------- CALLBACK FUNCTIONS ----------------------------------------------------------
# ################################################################################################################################################

def globalAltitudeCallback(globalAltitudeCB):
	global alt
	alt = globalAltitudeCB.data

def globalStateCallback(globalStateCB):
	global okConnected
	global okArmed
	global okGuided
	okConnected = globalStateCB.connected
	okArmed = globalStateCB.armed
	okGuided = globalStateCB.guided

def colorPositionCallback(colorPositionCallback):
	global current_pose_g
	global redx
	global redy
	global bluex
	global bluey

	okColor = colorPositionCallback.data
	pos = getCurrentLocation()

	if okColor == "Red":
		redx = pos.x
		redy = pos.y

	if okColor == "Blue":
		bluex = pos.x
		bluey = pos.y

def enu2local(data):
	global local_offset_g

	point = Point()
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	z = data.pose.pose.position.z
	deg2rad = (math.pi/180)
	point.x = x*math.cos((local_offset_g - 90)*deg2rad) - y*math.sin((local_offset_g - 90)*deg2rad)
	point.y = x*math.sin((local_offset_g - 90)*deg2rad) + y*math.cos((local_offset_g - 90)*deg2rad)
	point.z = z

	return point


def poseCallback(data):
	global current_heading_g
	global current_pose_g
	global local_offset_g

	current_pose_g = data
	enu2local(current_pose_g)
	q0 = current_pose_g.pose.pose.orientation.w
	q1 = current_pose_g.pose.pose.orientation.x
	q2 = current_pose_g.pose.pose.orientation.y
	q3 = current_pose_g.pose.pose.orientation.z
	psi = math.atan2((2*(q0*q3 + q1*q2)), (1 - 2*(math.pow(q2,2) + math.pow(q3,2))))

	current_heading_g = psi*(180/math.pi) - local_offset_g


def getCurrentLocation():
	global current_pose_g

	point = Point()
	point = enu2local(current_pose_g)
	return point

def getCurrentHeading():
	global current_heading_g

	return current_heading_g


# ################################################################################################################################################
# -------------------------------------------------------------- MOVEMENENT ---------------------------------------------------
# ################################################################################################################################################



def initialize_local_frame():

	global local_offset_g
	global local_offset_pose_g
	global current_pose_g


	local_offset_g = 0

	for i in range(30):
		
		rospy.sleep(0.1)


		q0 = current_pose_g.pose.pose.orientation.w
		q1 = current_pose_g.pose.pose.orientation.x
		q2 = current_pose_g.pose.pose.orientation.y
		q3 = current_pose_g.pose.pose.orientation.z
		psi = math.atan2((2*(q0*q3 + q1*q2)), (1 - 2*(math.pow(q2,2) + math.pow(q3,2))) )

		local_offset_g = local_offset_g + psi*(180/math.pi)

		local_offset_pose_g.x = local_offset_pose_g.x + current_pose_g.pose.pose.position.x
		local_offset_pose_g.y = local_offset_pose_g.y + current_pose_g.pose.pose.position.y
		local_offset_pose_g.z = local_offset_pose_g.z + current_pose_g.pose.pose.position.z

	local_offset_pose_g.x = local_offset_pose_g.x/30
	local_offset_pose_g.y = local_offset_pose_g.y/30
	local_offset_pose_g.z = local_offset_pose_g.z/30
	local_offset_g = local_offset_g / 30



def set_heading(heading):
	global local_desired_heading_g
	global correction_heading_g
	global local_offset_g
	global waypoint_g

	local_desired_heading_g = heading
	heading = heading + correction_heading_g + local_offset_g

	yaw = heading*(math.pi/180)
	pitch = 0
	roll = 0

	cy = math.cos(yaw * 0.5)
	sy = math.sin(yaw * 0.5)
	cr = math.cos(roll * 0.5)
	sr = math.sin(roll * 0.5)
	cp = math.cos(pitch * 0.5)
	sp = math.sin(pitch * 0.5)

	qw = cy * cr * cp + sy * sr * sp
	qx = cy * sr * cp - sy * cr * sp
	qy = cy * cr * sp + sy * sr * cp
	qz = sy * cr * cp - cy * sr * sp

	waypoint_g.pose.orientation.w = qw
	waypoint_g.pose.orientation.x = qx
	waypoint_g.pose.orientation.y = qy
	waypoint_g.pose.orientation.z = qz


def set_destination(x, y, z, psi):
	global correction_heading_g
	global local_offset_g
	global correction_vector_g
	global local_offset_pose_g
	global waypoint_g

	set_heading(psi)
	deg2rad = (math.pi/180)
	Xlocal = x*math.cos((correction_heading_g + local_offset_g - 90)*deg2rad) - y*math.sin((correction_heading_g + local_offset_g - 90)*deg2rad)
	Ylocal = x*math.sin((correction_heading_g + local_offset_g - 90)*deg2rad) + y*math.cos((correction_heading_g + local_offset_g - 90)*deg2rad)
	Zlocal = z

	x = Xlocal + correction_vector_g.position.x + local_offset_pose_g.x
	y = Ylocal + correction_vector_g.position.y + local_offset_pose_g.y
	z = Zlocal + correction_vector_g.position.z + local_offset_pose_g.z

	waypoint_g.pose.position.x = x
	waypoint_g.pose.position.y = y
	waypoint_g.pose.position.z = z

	pose_pub.publish(waypoint_g)


def checkWaypointReached(posTolerance = 0.3, headingTolerance = 0.01):

	global local_desired_heading_g
	global waypoint_g
	global current_pose_g
	global current_heading_g

	pose_pub.publish(waypoint_g)
	
	deltaX = float(abs(waypoint_g.pose.position.x - current_pose_g.pose.pose.position.x))
	deltaY = float(abs(waypoint_g.pose.position.y - current_pose_g.pose.pose.position.y))
	deltaZ = float(abs(waypoint_g.pose.position.z - current_pose_g.pose.pose.position.z))
	dMag = math.sqrt( math.pow(deltaX, 2) + math.pow(deltaY, 2) + math.pow(deltaZ, 2) )

	cosErr = float(math.cos(current_heading_g*(math.pi/180)) - math.cos(local_desired_heading_g*(math.pi/180)))
	sinErr = float(math.sin(current_heading_g*(math.pi/180)) - math.sin(local_desired_heading_g*(math.pi/180)))

	headingErr = float(math.sqrt( math.pow(cosErr, 2) + math.pow(sinErr, 2) ))

	if dMag < posTolerance and headingErr < headingTolerance:
		return 1
	else:
		return 0

def setSpeed(speed): # speed -- mps
	
	rospy.wait_for_service('/mavros/cmd/command')
	try:
		clientService = rospy.ServiceProxy(
			'/mavros/cmd/command', mavros_msgs.srv.CommandLong)
		clientService(command=178, param1=1, param2=speed, param3=-1, param4=0)
	except rospy.ServiceException as e:
		print ("Service speed setting failed: %s" % e)



# ################################################################################################################################################
# -------------------------------------------------------------- TESTS ---------------------------------------------------
# ################################################################################################################################################

'''
# ################################################################################################################################################
-------------------------------------------------------------------- MAP ------------------------------------------------------------------------

					(-30,-30)					red(-30.4,-11.5)  (-30.0)		blue(-30.9,16.7)		   (-30,30)

					(0,-30)										   (0,0)									(0,30)

					(30, -30)									   (30,0)               		            (30,30)

 ################################################################################################################################################
'''

	# right - forward - up  -------------  positive distance
	# 0(default) - 90(left) - 180(behind) - -90(right)  --------------- angular system

def test(list):
	list.append(Waypoint(0,0,3,0))
	list.append(Waypoint(0,0,10,0))


def mission2_1(listt):

	listt.append(Waypoint(0,30,6,0))
	listt.append(Waypoint(-30,30,6,90))
	listt.append(Waypoint(-30,-30,6,180))
	listt.append(Waypoint(0,-30,6,-90))
	
	listt.append(Waypoint(0,30,6,0))
	listt.append(Waypoint(-30,30,6,90))
	listt.append(Waypoint(-30,30,6,180))



def mission2_2(listt, redx, redy, bluex, bluey):

	listt.append(Waypoint(bluex,bluey,6,180))
	listt.append(Waypoint(bluex,bluey,0.8,180))
	listt.append(Waypoint(bluex,bluey,6,180))
	listt.append(Waypoint(redx,redy,6,180))
	listt.append(Waypoint(redx,redy,0.8,180))
	listt.append(Waypoint(redx,redy,6,180))
	listt.append(Waypoint(-30,-30,6,180))
	listt.append(Waypoint(0,-30,6,-90))
	listt.append(Waypoint(0,0,6,0))



if __name__ == '__main__':

	rospy.init_node('otonom_pose_node', anonymous=True)
	rate = rospy.Rate(2.0)
	
	rospy.Subscriber("/mavros/global_position/rel_alt", Float64, globalAltitudeCallback)
	rospy.Subscriber("/mavros/state", State, globalStateCallback)
	rospy.Subscriber("/mavros/global_position/local", Odometry, poseCallback)
	rospy.Subscriber("/is_find_color", String, colorPositionCallback)

	# Conecting check
	while not okConnected:
		rospy.sleep(0.01)

	# Guided check
	setGuidedMode()
	while not okGuided:
		rospy.sleep(0.01)
		setGuidedMode()

	print("Is Drone Guided:{}".format(okGuided))

	# Create local reference frame 
	initialize_local_frame()

	# Armed check
	setArm()
	while not okArmed:
		pose_pub.publish(waypoint_g)
		rospy.sleep(0.1)
		setArm()

	print("Drone Arm Status:{}".format(okArmed))

	# Altitude check
	setTakeoffMode(6)
	while alt < 5.8:
		rospy.sleep(0.1)

	setSpeed(10.0)

	list1 = []
	mission2_1(list1)

	rx, ry, bx, by = 0.0 , 0.0, 0.0, 0.0
	counter = 0
	while not rospy.is_shutdown():
		rate.sleep()
		if checkWaypointReached(.3) == 1:
			if counter < len(list1):
				
				set_destination(list1[counter].x, list1[counter].y, list1[counter].z, list1[counter].psi)
				
				if counter == 3:
					print("Red X: {:.7f} , RedY: {:.7f}".format(redx, redy))
					print("Blue X: {:.7f} , BlueY: {:.7f}".format(bluex, bluey))
					bx, by = bluex, bluey
					rx, ry = redx, redy
				
				counter = counter + 1
			else:
				break

	

	list2 = []
	mission2_2(list2, rx, ry, bx, by)

	counter = 0
	while not rospy.is_shutdown():
		rate.sleep()
		if checkWaypointReached(.3) == 1:
			if counter < len(list2):
				set_destination(list2[counter].x, list2[counter].y, list2[counter].z, list2[counter].psi)

				if counter == 1 or counter == 4:
					print("Inis yapiliyor : X: {:.7f} , Y: {:.7f}".format(getCurrentLocation().x, getCurrentLocation().y))

				counter = counter + 1
			else:
				break

	setLandMode()

	del list2[:]
	del list1[:]


	# MAVI - SU ALMA 
	# KIRMIZI - SU BIRAKMA