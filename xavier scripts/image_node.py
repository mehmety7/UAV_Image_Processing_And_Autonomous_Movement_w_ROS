#!/usr/bin/env python

import sys
import rospy

import numpy as np
import cv2
import math

from std_msgs.msg import String



def gstreamer_pipeline(
	capture_width=1280,
	capture_height=720,
	display_width=640,
	display_height=360,
	framerate=60,
	flip_method=0,
):
	return (
		"nvarguscamerasrc ! "
		"video/x-raw(memory:NVMM), "
		"width=(int)%d, height=(int)%d, "
		"format=(string)NV12, framerate=(fraction)%d/1 ! "
		"nvvidconv flip-method=%d ! "
		"video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
		"videoconvert ! "
		"video/x-raw, format=(string)BGR ! appsink"
		% (
			capture_width,
			capture_height,
			framerate,
			flip_method,
			display_width,
			display_height,
		)
	)


# msg to define which state for img processing
state = "1"


def state_callback(msg):
	global state

	state = msg.data


if __name__ == '__main__':

	rospy.init_node("image_kit", anonymous=True)
	rospy.Subscriber("/img_node_state", String, state_callback)

	detect_pub = rospy.Publisher("/is_find_color", String, queue_size=1)
	align_pub = rospy.Publisher("/is_align", String, queue_size=1)

	rate = rospy.Rate(2.0)

	color = "None"
	align = ""

	for i in range(0, 5):
		camera = cv2.VideoCapture(i, cv2.CAP_V4L2)
		if camera.isOpened():
			break
		if i == 5:
			print("Cam not found..")
			exit()
	width = 640
  	height = 480
	camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
  	camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
	camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))

  	

	if camera.isOpened():

		cv2.namedWindow("Gokmen UAV Cam", cv2.WINDOW_AUTOSIZE)

		while not rospy.is_shutdown():

			# print(int(camera.get(cv2.CAP_PROP_FRAME_WIDTH)))
			# print(int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT)))

			ret, imageFrame = camera.read()

			color = "None"
			bigColor = "None"
			align = ""

			horizontal_fov = 62.2 * math.pi/180
			vertical_fov = 48.8 * math.pi/180
			horizontal_resolution = 840
			vertical_resolution = 680

			width = horizontal_resolution
			height = vertical_resolution
			dim = (width, height)
			imageFrame = cv2.resize(imageFrame, dim)

			if state == "0":  #blue
				
				blue_lower = np.array([70, 90, 50], np.uint8) 
				blue_upper = np.array([130, 255, 255], np.uint8)

				hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

		
				blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

				kernal_blue = np.ones((5, 5), "uint8")

				blue_mask = cv2.dilate(blue_mask, kernal_blue)

				contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

				if len(contours) > 0:
					c = max(contours, key=cv2.contourArea)

					bigColor = "Blue"
		
					x, y, w, h = cv2.boundingRect(c)
					cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
		
					centerHavuzX = int((x+x+w)/2) # yatayda
					centerHavuzY = int((y+y+h)/2) # dikeyde
					
					area = cv2.contourArea(c)
					
					if(area >= 10000):
	   
						threshDikey = int((height//2)*.2)
						threshYatay = int((width//2)*.2)
		
						color = bigColor
					
						if (((centerHavuzY <= (height//2 + threshDikey)) and (centerHavuzY >= (height//2 - threshDikey))) and 
																((centerHavuzX <= (width//2 + threshYatay)) and (centerHavuzX >= (width//2 - threshYatay)))):
							align = "00"
						elif ((centerHavuzY <= (height//2 + threshDikey)) and (centerHavuzY >= (height//2 - threshDikey))):
							if (centerHavuzX > (width//2 + threshYatay)):
								align = "-0"
							elif (centerHavuzX < (width//2 - threshYatay)):
								   align = "+0"
						elif ((centerHavuzX <= (width//2 + threshYatay)) and (centerHavuzX >= (width//2 - threshYatay))):
							if (centerHavuzY > (height//2 + threshDikey)):
								align = "0+"
							elif (centerHavuzY < (height//2 - threshDikey)):
								align = "0-"
						else:
							if (centerHavuzX > (width//2 + threshYatay)):
								align = align + "-"
							if (centerHavuzX < (width//2 - threshYatay)):
								align = align + "+"
							if (centerHavuzY > (height//2 + threshDikey)):
								align = align + "+"
							if (centerHavuzY < (height//2 - threshDikey)):
								align = align + "-"
						
						print(color)
						print(area)
						print(align)

				cv2.imshow("Gokmen UAV Cam", imageFrame)
				keyCode = cv2.waitKey(30) & 0xFF
				# Stop the program on the ESC key
				if keyCode == 27:
					break
					
				detect_pub.publish(color)
				align_pub.publish(align)

			elif state == "1":  #red

				lower1 = np.array([0,100,20])
				upper1 = np.array([10,255,255])

				lower2 = np.array([160,100,20])
				upper2 = np.array([179,255,255])
				
	   			hsv = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
		  
		  
		  		lower_mask = cv2.inRange(hsv, lower1, upper1)
				upper_mask = cv2.inRange(hsv, lower2, upper2)

				red_mask = lower_mask + upper_mask

				kernal_red = np.ones((5, 5), "uint8")

				red_mask = cv2.dilate(red_mask, kernal_red)

				contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

				if len(contours) > 0:
					c = max(contours, key=cv2.contourArea)

					bigColor = "Red"
		
					x, y, w, h = cv2.boundingRect(c)
					cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
		
					centerHavuzX = int((x+x+w)/2) # yatayda
					centerHavuzY = int((y+y+h)/2) # dikeyde
					
					area = cv2.contourArea(c)
					
					if(area >= 200):
	   
						threshDikey = int((height//2)*.2)
						threshYatay = int((width//2)*.2)
		
						color = bigColor
					
						if (((centerHavuzY <= (height//2 + threshDikey)) and (centerHavuzY >= (height//2 - threshDikey))) and 
																((centerHavuzX <= (width//2 + threshYatay)) and (centerHavuzX >= (width//2 - threshYatay)))):
							align = "00"
						elif ((centerHavuzY <= (height//2 + threshDikey)) and (centerHavuzY >= (height//2 - threshDikey))):
							if (centerHavuzX > (width//2 + threshYatay)):
								align = "-0"
							elif (centerHavuzX < (width//2 - threshYatay)):
								   align = "+0"
						elif ((centerHavuzX <= (width//2 + threshYatay)) and (centerHavuzX >= (width//2 - threshYatay))):
							if (centerHavuzY > (height//2 + threshDikey)):
								align = "0+"
							elif (centerHavuzY < (height//2 - threshDikey)):
								align = "0-"
						else:
							if (centerHavuzX > (width//2 + threshYatay)):
								align = align + "-"
							if (centerHavuzX < (width//2 - threshYatay)):
								align = align + "+"
							if (centerHavuzY > (height//2 + threshDikey)):
								align = align + "+"
							if (centerHavuzY < (height//2 - threshDikey)):
								align = align + "-"
						
						print(color)
						print(area)
						print(align)

				cv2.imshow("Gokmen UAV Cam", imageFrame)
				keyCode = cv2.waitKey(30) & 0xFF
				# Stop the program on the ESC key
				if keyCode == 27:
					break

				detect_pub.publish(color)
				align_pub.publish(align)

			elif state == "2":
				continue

			elif state == "3":
				break
			
			else:
				print("Check the state value")
		
  	else:
		print("Unable camera")
