#!/usr/bin/env python


import rospy
from std_msgs.msg import String, Float64
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import *
from geometry_msgs.msg import Twist, Pose, PoseStamped, Point
from nav_msgs.msg import Odometry

from math import cos, sin, asin, sqrt, atan2
import math

# ################################################################################################################################################
# -------------------------------------------------------------- GLOBAL DEFINITIONS ---------------------------------------------------
# ################################################################################################################################################

pose_pub = rospy.Publisher(
    '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
vel_pub = rospy.Publisher(
    '/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
img_pub = rospy.Publisher("/img_node_state", String, queue_size=1)

# define msgs
correction_vector_g = Pose()
local_offset_pose_g = Point()  # float x,y,z
current_pose_g = Odometry()
waypoint_g = PoseStamped()

velocity_msg = Twist()

# States info
okConnected = False
okArmed = False
okGuided = False

# altitude
alt = 0.0

# initialize heading of drone
current_heading_g = 0.0
local_offset_g = 0.0
correction_heading_g = 0.0
local_desired_heading_g = 0.0

# basic movement
align_x = ""
align_y = ""

# position of pools
# red(-30.4,-11.5) blue(-30.9,16.7)
okColor = "None"
redx = 0.0
redy = 0.0
bluex = 0.0
bluey = 0.0

# to close img node
msg_for_img = "0"

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
    for i in range(100):
        pose_pub.publish(waypoint_g)
        rospy.sleep(0.01)

    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy(
            '/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException as e:
        print("Service arm call failed: %s" % e)


def setTakeoffMode(alt):
    global waypoint_g

    set_destination(0, 0, int(alt), 0)
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
        print("Service takeoff call failed: %s" % e)


def setLandMode():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        takeoffService = rospy.ServiceProxy(
            '/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        takeoffService(altitude=0, latitude=0,
                       longitude=0, min_pitch=0, yaw=0)
    except rospy.ServiceException as e:
        print("Service takeoff call failed: %s" % e)


def setGuidedMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy(
            '/mavros/set_mode', mavros_msgs.srv.SetMode)
        # http://wiki.ros.org/mavros/CustomModes for custom modes
        isModeChanged = flightModeService(
            custom_mode='GUIDED')  # return true or false
    except rospy.ServiceException as e:
        print("service set_mode GUIDED call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" % e)


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
    global okColor

    okColor = colorPositionCallback.data
    pos = getCurrentLocation()

    if okColor == "Red":
        redx = pos.x
        redy = pos.y

    if okColor == "Blue":
        bluex = pos.x
        bluey = pos.y


def alignCallback(alignCallback):
    global align_x
    global align_y

    align = alignCallback.data
    if len(align) == 2:
        align_x = align[0]
        align_y = align[1]


def enu2local(data):
    global local_offset_g

    point = Point()
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    deg2rad = (math.pi/180)
    point.x = x*math.cos((local_offset_g - 90)*deg2rad) - \
        y*math.sin((local_offset_g - 90)*deg2rad)
    point.y = x*math.sin((local_offset_g - 90)*deg2rad) + \
        y*math.cos((local_offset_g - 90)*deg2rad)
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
    psi = math.atan2((2*(q0*q3 + q1*q2)),
                     (1 - 2*(math.pow(q2, 2) + math.pow(q3, 2))))

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

    pose_pub.publish(waypoint_g)


def checkWaypointReached(posTolerance=0.3, headingTolerance=0.01):

    global local_desired_heading_g
    global waypoint_g
    global current_pose_g
    global current_heading_g

    pose_pub.publish(waypoint_g)

    deltaX = float(abs(waypoint_g.pose.position.x -
                   current_pose_g.pose.pose.position.x))
    deltaY = float(abs(waypoint_g.pose.position.y -
                   current_pose_g.pose.pose.position.y))
    deltaZ = float(abs(waypoint_g.pose.position.z -
                   current_pose_g.pose.pose.position.z))
    dMag = math.sqrt(math.pow(deltaX, 2) +
                     math.pow(deltaY, 2) + math.pow(deltaZ, 2))

    cosErr = float(math.cos(current_heading_g*(math.pi/180)) -
                   math.cos(local_desired_heading_g*(math.pi/180)))
    sinErr = float(math.sin(current_heading_g*(math.pi/180)) -
                   math.sin(local_desired_heading_g*(math.pi/180)))

    headingErr = float(math.sqrt(math.pow(cosErr, 2) + math.pow(sinErr, 2)))

    if dMag < posTolerance and headingErr < headingTolerance:
        return 1
    else:
        return 0


def setSpeed(speed):  # speed -- mps

    rospy.wait_for_service('/mavros/cmd/command')
    try:
        clientService = rospy.ServiceProxy(
            '/mavros/cmd/command', mavros_msgs.srv.CommandLong)
        clientService(command=178, param1=1, param2=speed, param3=-1, param4=0)
    except rospy.ServiceException as e:
        print("Service speed setting failed: %s" % e)


def moveX(speed, time):

    velocity_msg.linear.x = speed

    vel_pub.publish(velocity_msg)

    rospy.sleep(time)

    stop()


def moveY(speed, time):

    velocity_msg.linear.y = speed

    vel_pub.publish(velocity_msg)

    rospy.sleep(time)

    stop()


def moveZ(speed, time):

    velocity_msg.linear.z = speed

    vel_pub.publish(velocity_msg)

    rospy.sleep(time)

    stop()


def move_down():
    
    global alt

    velocity_msg.linear.z = -0.5

    while True:

        vel_pub.publish(velocity_msg)

        if alt <= 1.5:                          # hc-sr mesaji burdan dinlenecek
            break

        rate.sleep()

    stop()
    rospy.sleep(2.0)


def move_up(h):
    global alt

    velocity_msg.linear.z = 0.5
    #velocity_msg.linear.y = -0.1               # kalkista care olmazsa sil
    while True:

        vel_pub.publish(velocity_msg)

        if alt >= h:
            break

        rate.sleep()

    stop()


def stop():

    velocity_msg.linear.x = 0
    velocity_msg.linear.y = 0
    velocity_msg.linear.z = 0

    vel_pub.publish(velocity_msg)
    rospy.sleep(1.0)


def alignment_red():
    global align_x
    global align_y

    while True:
        if align_x == "" and align_y == "":
            return False
        if align_x == "0" and align_y == "0":
            print("Hizalama tamam!")
            stop()
            return True
        if align_y == "0":
            if align_x == "-":
                print("Hedef +X yonunde!")
                moveX(-0.4, .8)
            else:
                print("Hedef -X yonunde!")
                moveX(0.4, .8)
        else:
            if align_y == "-":
                print("Hedef +Y yonunde!")
                moveY(-0.4, .8)
            else:
                print("Hedef -Y yonunde!")
                moveY(0.4, .8)
                
                
def alignment_blue():
    global align_x
    global align_y

    while True:
        if align_x == "" and align_y == "":
            return False
        if align_x == "0" and align_y == "0":
            print("Hizalama tamam!")
            rospy.sleep(.1)
            stop()
            return True
        if align_y == "0":
            if align_x == "-":
                print("Hedef +X yonunde!")
                moveY(0.4, 1.0)
            else:
                print("Hedef -X yonunde!")
                moveY(-0.4, 1.0)
        else:
            if align_y == "-":
                print("Hedef +Y yonunde!")
                moveX(-0.4, 1.0)
            else:
                print("Hedef -Y yonunde!")
                moveX(0.4, 1.0)


# ################################################################################################################################################
# -------------------------------------------------------------- TESTS ---------------------------------------------------
# ################################################################################################################################################
'''
# ################################################################################################################################################
-------------------------------------------------------------------- MAP ------------------------------------------------------------------------
		(-30,-30)					red(-30.4,-11.5)                   (-30.0)		blue(-30.9,16.7)  (-30,30)
		(0,-30)										   (0,0)				  (0,30)
		(30, -30)									   (30,0)               	          (30,30)
 ################################################################################################################################################
'''

# right - forward - up  -------------  positive sides
# 0(default) - 90(left) - 180(behind) - -90(right)  --------------- angular system

def alignment_test(h):
    global okColor
    global msg_for_img

    list1 = []
    list1.append(Waypoint(0, 0, h, 90))
    list1.append(Waypoint(-30, 0, h, 90))
    counter = 0
    alg_red = False
    alg_blue = False
    while not rospy.is_shutdown():
        img_pub.publish(msg_for_img)
        rospy.sleep(.2)
        '''
        if okColor == "Red" and not alg_red:
            rospy.sleep(0.7)
            stop()
            if alignment_red():
                msg_for_img = "2"
                print("Kirmizya inis basliyor ..!")
                move_down()
                print("Kirmizidan kalkis basliyor ..!")
                move_up(h)
                set_destination(
                    list1[counter-1].x, list1[counter-1].y, list1[counter-1].z, list1[counter-1].psi)
                alg_red = True
        '''        
        if okColor == "Blue" and not alg_blue:
            rospy.sleep(0.5)
            stop()
            if alignment_blue():
                msg_for_img = "1"
                print("Maviye inis basliyor ..!")
                move_down()
                print("Maviden kalkis basliyor ..!")
                move_up(h)
                set_destination(
                    list1[counter-1].x, list1[counter-1].y, list1[counter-1].z, list1[counter-1].psi)
                alg_blue = True
                
        if checkWaypointReached(.3) == 1:
            if counter < len(list1):

                set_destination(
                    list1[counter].x, list1[counter].y, list1[counter].z, list1[counter].psi)

                counter = counter + 1
            else:
                break



def square_test(h):

    listt = []
    listt.append(Waypoint(0, 5, h, 0))
    listt.append(Waypoint(-5, 5, h, 90))
    listt.append(Waypoint(-5, -5, h, 180))
    listt.append(Waypoint(0, -5, h, -90))
    listt.append(Waypoint(0, 0, h, 0))

    counter = 0
    while not rospy.is_shutdown():

        img_pub.publish(msg_for_img)
        rospy.sleep(.1)     
        if checkWaypointReached(.3) == 1:
            if counter < len(listt):
                print(listt[counter].x, listt[counter].y, listt[counter].z, listt[counter].psi)
                set_destination(
                    listt[counter].x, listt[counter].y, listt[counter].z, listt[counter].psi)

                counter = counter + 1
                print(counter)
            else:
                break

def mission2(h):

    global okColor
    global msg_for_img
    global redx
    global redy

    listt = []
    listt.append(Waypoint(0, 30, h, 0))
    listt.append(Waypoint(-30, 30, h, 90))
    listt.append(Waypoint(-30, -30, h, 180))
    listt.append(Waypoint(0, -30, h, -90))

    listt.append(Waypoint(0, 30, h, 0))
    listt.append(Waypoint(-30, 30, h, 90))
    listt.append(Waypoint(-30, -30, h, 180))
    listt.append(Waypoint(0, -30, h, -90))
    listt.append(Waypoint(0, 0, h, 0))

    counter = 0
    alg_red = False
    alg_blue = False
    target_save = False
    bluesX = []
    bluesY = []
    redsX = []
    redsY = []
    while not rospy.is_shutdown():

        img_pub.publish(msg_for_img)
        rospy.sleep(.1)
        
        if counter == 2 or counter == 4:
            msg_for_img = "0"
            
        if counter == 3:
            msg_for_img = "1"
            
        if counter == 2 and okColor == "Blue":
            bluesX.append(bluex)
            bluesY.append(bluey)
            
        if counter == 3 and okColor == "Red":
            redsX.append(redx)
            redsY.append(redy)
            
        if not target_save and counter == 4:
            Xred = redsX[int(len(redsX)/2)]
            Yred = redsY[int(len(redsY)/2)]
            Xblue = bluesX[int(len(bluesX)/2)]
            Yblue = bluesY[int(len(bluesY)/2)]
            print("Red X: {:.7f} , Red Y: {:.7f}".format(Xred, Yred))
            print("Blue X: {:.7f} , Blue Y: {:.7f}".format(Xblue, Yblue))
            target_save = True
        
        if counter == 6 and okColor == "Blue" and not alg_blue:
            rospy.sleep(0.4)
            stop()
            if alignment_blue():
                if alignment_blue():
                    print("Maviye inis basliyor ..!")
                    move_down()
                    print("Maviden kalkis basliyor ..!")
                    move_up(h)
                    set_destination(Xred, Yred, h, 180)
                    alg_blue = True
                    msg_for_img = "1"
        if (counter == 7 or counter == 6) and okColor == "Red" and not alg_red and alg_blue:
            rospy.sleep(0.4)
            stop()
            if alignment_red():
                if alignment_red():
                    print("Kirmizya inis basliyor ..!")
                    move_down()
                    print("Kirmizidan kalkis basliyor ..!")
                    move_up(h)
                    counter = 7
                    set_destination(
                        listt[counter-1].x, listt[counter-1].y, listt[counter-1].z, listt[counter-1].psi)
                    print(listt[counter-1].x, listt[counter-1].y, listt[counter-1].z, listt[counter-1].psi)
                    alg_red = True
                    msg_for_img = "2"

        if checkWaypointReached(.3) == 1:
            if counter < len(listt):
                print(listt[counter].x, listt[counter].y, listt[counter].z, listt[counter].psi)
                set_destination(
                    listt[counter].x, listt[counter].y, listt[counter].z, listt[counter].psi)

                counter = counter + 1
                print(counter)
            else:
                break


if __name__ == '__main__':

    rospy.init_node('otonom_kit', anonymous=True)

    rate = rospy.Rate(2.0)

    rospy.Subscriber("/mavros/global_position/rel_alt",
                     Float64, globalAltitudeCallback)
    rospy.Subscriber("/mavros/state", State, globalStateCallback)
    rospy.Subscriber("/mavros/global_position/local", Odometry, poseCallback)
    rospy.Subscriber("/is_find_color", String, colorPositionCallback)
    rospy.Subscriber("/is_align", String, alignCallback)

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
    h = 10.0
    setTakeoffMode(h)
    while alt < h - 0.2:
        rospy.sleep(0.01)
        
    setSpeed(10.0)

    #alignment_test(h)
    #square_test(h)
    mission2(h)
    setLandMode()
    
    msg_for_img = "3"
    img_pub.publish(msg_for_img)

    # MAVI - SU ALMA
    # KIRMIZI - SU BIRAKMA
