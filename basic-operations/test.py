#!/usr/bin/env python

# OPENCV
import cv2


# ROS LIBRARIES
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

import sys

# Bridge for OpenCV and ROS Images
image_bridge = CvBridge()
global_image = None



lower_blue = (70, 50, 50)
upper_blue = (130, 255, 255)


#Change these fov values with Raspberry Pi Cam Module values
horizontal_fov = 118.2 * math.pi/180
vertical_fov = 69.5 * math.pi/180
horizontal_resolution = 1280
vertical_resolution = 720


def image_callback(ros_image):
    global image_bridge
    try:
        cv_image = bridge.imsgs_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    global_image = cv_image




if __name__ == '__main__':
    rospy.init_node("image_node", anonymous = True)
    rospy.Subscriber("/gscam/gscam/image_raw", Image, image_callback)

    # Publish geometric info about where object is (x,y coordiantes to operate drone)
    image_pub = rospy.Publisher('image_location', String, queue_size = 10)

    rate = rospy.Rate(20)

    # TODO:: Implement OPENCV detection here
    # publish image_pub

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        frame = global_image

        if frame is None:
            continue

    width = horizontal_resolution
        height = vertical_resolution
        dim = (width, height)
        frame = cv2.resize(frame, dim)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (11, 11), 0)
        rows = gray.shape[0]

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            #if(radius > 10):
                #cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                #cv2.circle(mask, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.drawContours(frame,c,-1, (0, 255, 255), 2)
            cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
            cv2.putText(frame, "Radius: " + str(radius), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 1)
            cv2.putText(frame, "cX:     " + str(frame.shape[1]/2 - cX), (50,80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 1)
            cv2.putText(frame, "cY:     " + str(cY - frame.shape[0]/2), (50,110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 1)
            
            cv2.putText(frame, "Message X:     {:.3f}".format((cX-horizontal_resolution/2)*horizontal_fov/horizontal_resolution), (50,140), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 1)
            cv2.putText(frame, "Message Y:     {:.3f}".format((cY-vertical_resolution/2)*vertical_fov/vertical_resolution) , (50,170), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 1)
            image_pub.publish(str((cX-horizontal_resolution/2)*horizontal_fov/horizontal_resolution) + "," + str((cY-vertical_resolution/2)*vertical_fov/vertical_resolution)
                + "," + str(radius))
        rate.sleep()

    cv2.imshow("org frame", frame)
    #cv2.imshow("mask", mask)
    if cv2.waitKey(1) == ord('q'):
        break

        