#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *
from geometry_msgs.msg import TwistStamped
from control import *
   
class Movement():

    def __init__(self):
        rospy.init_node('move_exercise', anonymous=True)
        
        self.publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.globalPositionCallback)

        self.stamped = TwistStamped()
        self.r = rospy.Rate(10.0)
        
        self.latitude = 0.0
        self.longitude = 0.0


        
        while not rospy.is_shutdown():
            setGuidedMode()
            self.publisher.publish(self.stamped)
            setArm()
            setTakeoffMode()
            

            self.moveSquare()
            self.rate.sleep()
            
            #self.moveCircle()
            
            #self.moveSpiral()
            
            #rospy.sleep(5)

            #self.stop()
            #rospy.sleep(5)

            #rospy.sleep(5)

            setLandMode()
            setDisarm()

        #move_exercise()

    def globalPositionCallback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        #print ("longitude: %.7f" %longitude)
        #print ("latitude: %.7f" %latitude)

    def moveSquare(self):
        #square_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        #square = TwistStamped()

        #user_input = raw_input("Enter the square side: "); # take from user
        side_length = 10
        flag_x = 1
        flag_y = 1
        rospy.loginfo('Moving...')

        for x in range(1,6):
            if 4%x == 0:
                self.stamped.twist.linear.y = side_length
                flag_x= -1
            else:
                self.stamped.twist.linear.x = side_length
                flag_y= -1

            self.stamped.twist.linear.x=0;
            self.stamped.twist.linear.y=0;
            self.publisher.publish(self.stamped);
            rospy.sleep(2);

            if flag_x == -1 and flag_y == -1:
                side_length *= -1
                flag_x = 1
                flag_y = 1

    def moveCircle(self):

        #circle_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        #circle = TwistStamped()


        self.stamped.twist.linear.x=5
        self.stamped.twist.linear.z=5
        self.publisher.publish(self.stamped)
        rospy.loginfo('Moving...')



        
    def moveSpiral(self):
        self.constant_speed = 2
        self.rk = 1.0
        self.rk_step = 0.1
        self.duration = 3.0 #10 seconds
        self.rate = 3.0
        self.number_of_iteration = self.duration * self.rate
        rospy.loginfo('Moving...')
        
        i=0

        while(i<self.number_of_iteration):

            #print t1-t0
            i=i+1
            self.rk=self.rk+self.rk_step
            self.stamped.twist.linear.x =0
            self.stamped.twist.linear.y =self.rk
            self.stamped.twist.linear.z =0
            self.stamped.twist.angular.x = 0
            self.stamped.twist.angular.y = 0
            self.stamped.twist.angular.z =self.constant_speed
            

            #publish the velocity
            self.publisher.publish(self.stamped)
            self.r.sleep()

    def stop(self): 

        #self.publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        #stop = TwistStamped()
     
        self.stamped.twist.linear.x=0.0
        self.stamped.twist.linear.y=0.0
        self.stamped.twist.angular.z=0.0 
        self.publisher.publish(self.stamped)
        rospy.loginfo('Stopping...')

'''    

    def menu(self):
        print "Press"
        print "1: to move on square"
        print "2: to move on circle"
        print "3: to move on spiral"
        print "4: to stop"
        
    def move_exercise(self):
        x='1'
        while ((not rospy.is_shutdown()) and (x in ['1','2','3','4'])):
            self.menu()
            x = raw_input("Enter your input: ")
            if (x=='1'):
                setGuidedMode()
                setArm()
                setTakeoffMode()
                moveSquare()
                setLandMode()
            elif(x=='2'):
                setGuidedMode()
                setArm()
                setTakeoffMode()
                moveCircle()
            elif(x=='3'):
                setGuidedMode()
                setArm()
                setTakeoffMode()
                moveSpiral()
            elif(x=='4'):
                stop()
                setLandMode()
            else: 
                print "Exit"

'''

if __name__ == "__main__":
    Movement()