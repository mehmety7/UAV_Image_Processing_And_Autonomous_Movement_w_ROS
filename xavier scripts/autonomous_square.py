#!/usr/bin/env python


import rospy
from std_msgs.msg import String, Float64
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import *
from geometry_msgs.msg import Twist, Pose, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from math import cos, sin, asin, sqrt, atan2
import math
from datetime import datetime

# ################################################################################################################################################
# -------------------------------------------------------------- GLOBAL DEFINITIONS ---------------------------------------------------
# ################################################################################################################################################

pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

# define msgs
correction_vector_g = Pose()
local_offset_pose_g = Point()  # float x,y,z
current_pose_g = Odometry()
waypoint_g = PoseStamped()

# States info
okConnected = False
okArmed = False
okGuided = False

# altitude
alt = 0.0

# navsatfix altitude
isFirstAlt = True
first_alt_navsatfix = 0.0
alt_navsatfix = 0.0

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


# ################################################################################################################################################
# -------------------------------------------------------------- STATES INSTRUCTIONS ---------------------------------------------------
# ################################################################################################################################################

def setArm():
    global waypoint_g

    set_destination(0, 0, 0, 0)
    for i in range(30):
        pose_pub.publish(waypoint_g)
        rospy.sleep(0.01)

    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException as e:
        print("Service arm call failed: %s" % e)


def setTakeoffMode(altitude_):
    global waypoint_g

    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
        takeoffService(altitude=int(altitude_), latitude=0,longitude=0, min_pitch=0, yaw=0)
    except rospy.ServiceException as e:
        print("Service takeoff call failed: %s" % e)


def setLandMode():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        takeoffService(altitude=0, latitude=0,longitude=0, min_pitch=0, yaw=0)
    except rospy.ServiceException as e:
        print("Service takeoff call failed: %s" % e)


def setGuidedMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='GUIDED')                                                               
        # return true or false
    except rospy.ServiceException as e:
        print("service set_mode GUIDED call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" % e)

def setStreamRate():
    rospy.wait_for_service('/mavros/set_stream_rate')
    try:
        streamService = rospy.ServiceProxy('/mavros/set_stream_rate', mavros_msgs.srv.StreamRate)
        streamService(stream_id=0, message_rate=10,on_off=1)
    except rospy.ServiceException as e:
        print("Service stream call failed: %s" % e)


# ################################################################################################################################################
# -------------------------------------------------------------- CALLBACK FUNCTIONS ----------------------------------------------------------
# ################################################################################################################################################

def globalPositionCallback(globalPositionCallback):
    global first_alt_navsatfix
    global alt_navsatfix
    global isFirstAlt
    global alt
    if isFirstAlt:
        first_alt_navsatfix = globalPositionCallback.altitude
        isFirstAlt = False
    else:
        alt_navsatfix = globalPositionCallback.altitude - first_alt_navsatfix
    alt = alt_navsatfix


def globalAltitudeCallback(globalAltitudeCB):
    global alt

def globalStateCallback(globalStateCB):
    global okConnected
    global okArmed
    global okGuided
    okConnected = globalStateCB.connected
    okArmed = globalStateCB.armed
    okGuided = globalStateCB.guided

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
    psi = math.atan2((2*(q0*q3 + q1*q2)),(1 - 2*(math.pow(q2, 2) + math.pow(q3, 2))))

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
# -------------------------------------------------------------- MOVEMENT ---------------------------------------------------
# ################################################################################################################################################


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
    Xlocal = x*math.cos((correction_heading_g + local_offset_g - 90)*deg2rad) - \
        y*math.sin((correction_heading_g + local_offset_g - 90)*deg2rad)
    Ylocal = x*math.sin((correction_heading_g + local_offset_g - 90)*deg2rad) + \
        y*math.cos((correction_heading_g + local_offset_g - 90)*deg2rad)
    Zlocal = z

    x = Xlocal + correction_vector_g.position.x + local_offset_pose_g.x
    y = Ylocal + correction_vector_g.position.y + local_offset_pose_g.y
    z = Zlocal + correction_vector_g.position.z + local_offset_pose_g.z

    waypoint_g.pose.position.x = x
    waypoint_g.pose.position.y = y
    waypoint_g.pose.position.z = z

    print("Set destination working..")
    pose_pub.publish(waypoint_g)

def initialize_local_frame():

    global local_offset_g
    global local_offset_pose_g
    global current_pose_g

    local_offset_g = 0

    for i in range(50):

        rospy.sleep(0.1)

        q0 = current_pose_g.pose.pose.orientation.w
        q1 = current_pose_g.pose.pose.orientation.x
        q2 = current_pose_g.pose.pose.orientation.y
        q3 = current_pose_g.pose.pose.orientation.z
        psi = math.atan2((2*(q0*q3 + q1*q2)),
                         (1 - 2*(math.pow(q2, 2) + math.pow(q3, 2))))

        local_offset_g = local_offset_g + psi*(180/math.pi)

        local_offset_pose_g.x = local_offset_pose_g.x + \
            current_pose_g.pose.pose.position.x
        local_offset_pose_g.y = local_offset_pose_g.y + \
            current_pose_g.pose.pose.position.y
        local_offset_pose_g.z = local_offset_pose_g.z + \
            current_pose_g.pose.pose.position.z

    local_offset_pose_g.x = local_offset_pose_g.x/30
    local_offset_pose_g.y = local_offset_pose_g.y/30
    local_offset_pose_g.z = local_offset_pose_g.z/30
    local_offset_g = local_offset_g / 30

def checkWaypointReached(posTolerance=0.8, headingTolerance=0.01):

    global local_desired_heading_g
    global waypoint_g
    global current_pose_g
    global current_heading_g

    pose_pub.publish(waypoint_g)

    deltaX = abs(waypoint_g.pose.position.x - current_pose_g.pose.pose.position.x)
    deltaY = abs(waypoint_g.pose.position.y - current_pose_g.pose.pose.position.y)
    deltaZ = abs(waypoint_g.pose.position.z - current_pose_g.pose.pose.position.z)
    
    dMag = math.sqrt(math.pow(deltaX, 2) + math.pow(deltaY, 2) + math.pow(deltaZ, 2))

    cosErr = math.cos(current_heading_g*(math.pi/180)) - math.cos(local_desired_heading_g*(math.pi/180))
    sinErr = math.sin(current_heading_g*(math.pi/180)) -math.sin(local_desired_heading_g*(math.pi/180))

    headingErr = math.sqrt(math.pow(cosErr, 2) + math.pow(sinErr, 2))

    if dMag < posTolerance and headingErr < headingTolerance:
        print("checkWayPoint : YESSS..")
        return 1
    else:
        print("checkWayPoint : NOO..")
        return 0


def setSpeed(speed):  # speed -- mps

    rospy.wait_for_service('/mavros/cmd/command')
    try:
        clientService = rospy.ServiceProxy(
            '/mavros/cmd/command', mavros_msgs.srv.CommandLong)
        clientService(command=178, param1=1, param2=speed, param3=-1, param4=0)
    except rospy.ServiceException as e:
        print("Service speed setting failed: %s" % e)

# ################################################################################################################################################
# -------------------------------------------------------------- TESTS ---------------------------------------------------
# ################################################################################################################################################
'''
# ################################################################################################################################################
-------------------------------------------------------------------- MAP ------------------------------------------------------------------------
		(-30,-30)				                      (-30.0)                             (-30,30)
		(0,-30)										   (0,0)				               (0,30)
		(30, -30)									   (30,0)               	           (30,30)
 ################################################################################################################################################
'''

# right - forward - up  -------------  positive sides
# 0(default) - 90(left) - 180(behind) - -90(right)  --------------- angular system

def square_test(h):

    listt = []
    listt.append(Waypoint(0, 0, h, 0))
    listt.append(Waypoint(0, 5, h, 0))
    listt.append(Waypoint(-5, 5, h, 90))
    listt.append(Waypoint(-5, -5, h, 180))
    listt.append(Waypoint(0, -5, h, -90))
    listt.append(Waypoint(0, 0, h, 0))

    counter = 1
    

    isCheck = True
    start_time = datetime.now().second

    while not rospy.is_shutdown():
        print("Loop inside..")
        set_destination(listt[0].x, listt[0].y, listt[0].z, listt[0].psi)
        current_time = datetime.now().second
        if abs(current_time - start_time) > 15 and isCheck:
            print("Chechwaypoint is not working..")
            print("Landing")
            setLandMode()
            break
        rospy.sleep(.1)
        if checkWaypointReached(0.8) == 1:
            isCheck = False
            if counter < len(listt):
                set_destination(listt[counter].x, listt[counter].y, listt[counter].z, listt[counter].psi)
                print("Set new destination: ", listt[counter].x, listt[counter].y, listt[counter].z, listt[counter].psi)
                counter = counter + 1
                rospy.sleep(.1)
                print("Road number: ", counter)
            else:
                break

if __name__ == '__main__':

    rospy.init_node('otonom_kit', anonymous=True)

    rate = rospy.Rate(3)

    setStreamRate()

    rospy.Subscriber("/mavros/global_position/rel_alt",Float64, globalAltitudeCallback)
    rospy.Subscriber("/mavros/state", State, globalStateCallback)
    rospy.Subscriber("/mavros/global_position/local", Odometry, poseCallback)
    rospy.Subscriber("/chatter", Float64, distanceCallback)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, globalPositionCallback)

    # Conecting check
    while not okConnected:
        rospy.sleep(0.01)

    setStreamRate()

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
        rospy.sleep(0.1)
        setArm()

    print("Drone Arm Status:{}".format(okArmed))

    h = 10.0
    setTakeoffMode(h)
    rospy.sleep(3.0)
    # Altitude check
    temp = 0
    while alt < h:
        rospy.sleep(0.1)
        temp = temp + 1
        if temp == 300:
            print("Drone Not Enough Altitude:{}".format(alt))
            print("Landing ...")
            setLandMode()
            exit()
    
    print("Drone Final Altitude:{}".format(alt))
    rate.sleep()
    square_test(h)
    rospy.sleep(1.0)

    #setSpeed(10.0)
    setLandMode()
