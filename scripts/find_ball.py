#!/usr/bin/env python

import sys
import os
import subprocess
import rospy
import cv2
import numpy as np
import pickle
import time

from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import CameraInfo
from turtlebot3_soccer.msg import ObjDetection

from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry

detectionParamFile = "ball_param.p"

i1 = None
i2 = None
minH = None
minS = None
minV = None
maxH = None
maxS = None
maxV = None
fSize = None
raio = None

robotToInertialMat = None
robotPosition = None

debug = False

def nothing(x):
	pass

# ball detection algorithm
def detectBall(frame):
	global minH, maxH

	# init return variables
	ballPosX = 0
	ballPosY = 0
	ballExist = False
	
	minH  = 15
	maxH = 23
	
	# up and low color boundaries
	colorLower = (minH, minS, minV)
	colorUpper = (maxH, maxS, maxV)

	# is possible to define image size direct in usb_cam package
	#frameResized = cv2.resize(frame, None, fx=0.4, fy=0.4, interpolation=cv2.INTER_CUBIC)
	
	# processing the image
	# apply blur, converto to HSV color space, create mask based on color, erode and dilate
	blurred = cv2.GaussianBlur(frame, (fSize, fSize), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, colorLower, colorUpper)
	mask = cv2.erode(mask, None, iterations=i1)
	mask = cv2.dilate(mask, None, iterations=i2)
	
	# Detect contours
	(_, cnts, _) = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	
	# find the largest contour in the mask, then use it
	# to compute the minimum enclosing circle and centroid
	if len(cnts) > 0:
		
		# get the contour with the larger area
		c = max(cnts, key=cv2.contourArea)
		
		# find a circle of the minimum area enclousing some points
		((x,y), radius) = cv2.minEnclosingCircle(c)
		
		
		# if the circle has a certain size, it is the ball
		if radius > 5:
			
			# Draw ball contour (debug mode)
			if debug:
				# find the moments to define the center of the circle
				(startX, endX) = (int(x - 20), int(x + 20))
				(startY, endY) = (int(y - 20), int(y + 20))
				cv2.line(frame, (startX, int(y)), (endX, int(y)), (0, 0, 0), 3)
				cv2.line(frame, (int(x), startY), (int(x), endY), (0, 0, 0), 3)

				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
				cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
			
				#print(int(x),int(y))
				height, width, channels = frame.shape
			
				centerX = x/width
				centerY = y/height
	
				#centerCameraCord = np.float32((centerX, centerY, 1)).reshape(3,1)
			
				#resTrans = np.matmul(inverseHomographyMatrix, centerCameraCord)
			
				#centerPlaneCord = np.float32((resTrans[0]/resTrans[2], resTrans[1]/resTrans[2]))
		
				#print("X: " + str(centerPlaneCord[0][0]) + " Y: " + str(centerPlaneCord[1][0]))
			
				#goal = np.array((centerPlaneCord[1][0], centerPlaneCord[0][0],1)).reshape(3,1)
	
				#goalInertialFrame = np.matmul(robotToInertialMat, goal)
			
				#print(goalInertialFrame[0][0], goalInertialFrame[1][0])
			
			# set return variables if there is a ball on screen
			ballPosX = int(x)
			ballPosY = int(y)
			ballExist = True
			
	# Display image and draw control lines (debug mode)
	if debug:
		lVertLine = int(frame.shape[1]*0.2)
		rVertLine = int(frame.shape[1]*0.8)
		horLine = int(frame.shape[0]*0.7)

		cv2.line(frame, (lVertLine,0), (lVertLine, frame.shape[0]), (0,0,0), 2)
		cv2.line(frame, (rVertLine,0), (rVertLine, frame.shape[0]), (0,0,0), 2)
		cv2.line(frame, (0, horLine), (frame.shape[1], horLine), (0,0,0), 2)
	
		cv2.imshow('original', frame)
		cv2.imshow('mask', mask)
		cv2.waitKey(1)
	
	return ballPosX, ballPosY, ballExist

# Callback called which time image is received
def image_callback(data):
	
	# convert receveid image to RGB numpy
	image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
	h, w = image.shape[:2]
	
	# display original image
	#cv2.imshow('frame', image)
	
	# run ball detection
	ballPosX, ballPosY, ballExist = detectBall(image)
	
	ball_detection = ObjDetection()
	ball_detection.objExist = ballExist
	ball_detection.objPosX = ballPosX
	ball_detection.objPosY = ballPosY
	
	pub_ball_detection.publish(ball_detection)

def odometryHandler(data):
	global robotToInertialMat, robotPosition

	robotPosX = data.pose.pose.position.x*1000
	robotPosY = data.pose.pose.position.y*1000
	robotPosition = (robotPosX, robotPosY)

	robotOriX = data.pose.pose.orientation.x
	robotOriY = data.pose.pose.orientation.y
	robotOriZ = data.pose.pose.orientation.z
	robotOriW = data.pose.pose.orientation.w
	
	quaternion = (robotOriX, robotOriY, robotOriZ, robotOriW)
	euler = euler_from_quaternion([robotOriX, robotOriY, robotOriZ, robotOriW])
	
	yaw = euler[2]
	
	cosTheta = np.cos(-yaw)
	sinTheta =np.sin(-yaw)
	
	# Rotation matrix of robot frame to inertial frame
	rotationMatrix = np.array(	((cosTheta,	 -sinTheta,	0),
													(sinTheta, 	cosTheta, 	0),
													(0,				0,					1)))
													
	translationMatrix = np.array(	((1,	0,		robotPosX),
														(0,	1,		robotPosY),
														(0,	0,		1)))	
	
	
	robotToInertialMat = np.matmul(rotationMatrix, translationMatrix)
	
	#print(robotPosX, robotPosY)
	
if __name__ == '__main__':
	print("Starting find ball")

	global homographyMatrix, inverseHomographyMatrix

	# Get parameter file
	detectionParamFilePath = "/home/pi/catkin_ws/src/turtlebot3_soccer/src/" + detectionParamFile
	homographyMatrixPath = "/home/pi/catkin_ws/src/turtlebot3_soccer/src/ball_homography_tf_matrix.npy"
	
	# if detection param file doesnt exist open the configure detection script
	if os.path.isfile(detectionParamFilePath) == False:
		print("Ball detection parameters file not found!")
		print("Generate file with configure_detection script")
		exit(0)
		
	if os.path.isfile(homographyMatrixPath) == False:
		print("Homography matrix file not found!")
		print("Use calculate_homography script")
		exit(0)
		
	# load and set filter parameters of the file 
	filterParameters = pickle.load(open(detectionParamFilePath, "rb"))
		
	i1 = filterParameters.get('Erode') 
	i2 = filterParameters.get('Dilate') 
	minH = filterParameters.get('minH')
	minS = filterParameters.get('minS') 
	minV = filterParameters.get('minV')
	maxH = filterParameters.get('maxH')
	maxS = filterParameters.get('maxS') 
	maxV = filterParameters.get('maxV') 
	fSize = (filterParameters.get('fSize') *2) + 1
	raio = filterParameters.get('radius')
	
	# load homography matrix and calculate the inverse matrix
	homographyMatrix = np.load(homographyMatrixPath)
	inverseHomographyMatrix = (np.linalg.inv(homographyMatrix))

	#initialize the node
	rospy.init_node('find_ball', anonymous=True)
	
	# Subscribe to usb_cam and receive the image
	img_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, image_callback)
	bridge = CvBridge()
	
	#sub_odom = rospy.Subscriber("/odom", Odometry, odometryHandler)
		
	# Publish (x,y) coordinates over the ball_position topic
	pub_ball_detection = rospy.Publisher('/ball_detection', ObjDetection, queue_size=5)
	
	# not return until the node has been shutdown
	rospy.spin()
		
