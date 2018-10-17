#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import time

from geometry_msgs.msg import Twist
from turtlebot3_soccer.msg import ObjDetection
from sensor_msgs.msg import CameraInfo 
from sensor_msgs.msg import CompressedImage

# camera dimensions
camHeight = 0
camWidth = 0

def drive_callback(data):
	
	# create ball boundaries to control turtlebot
	lVertLine = int(camWidth*0.35)
	rVertLine = int(camWidth*0.65)
	horLine = int(camHeight*0.7)
	
	# if there is a ball in the screen
	if data.objExist:
	
		# based on the ball quadrant, execute an action
		# quadrant left-up or left-down (rotate left)
		if data.objPosX <= lVertLine:
			vel_msg.linear.x = 0
			vel_msg.angular.z = 0.5
			print("rotate left")
			
		# quadrant right-up or right-down (rotate right)
		elif data.objPosX >= rVertLine:
			vel_msg.linear.x = 0
			vel_msg.angular.z = -0.5
			print("rotate right") 	
		
		# quadrant center-up (drive foward - fast)
		elif data.objPosX > lVertLine and data.objPosX < rVertLine and data.objPosY <= horLine:
			vel_msg.linear.x = 0.1
			vel_msg.angular.z = 0
			print("drive forward - fast")
			
		else:
			vel_msg.linear.x = 0.5
			vel_msg.angular.z = 0
			print("dribla")
		
		
	# if there is no ball in the screen, go search
	else:
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		print("search ball")
	
	# write velocity to the motors
	pub_vel.publish(vel_msg)
	
# get camera information 
def cam_info_callback(data):
	global camHeight, camWidth
	camHeight = data.height
	camWidth = data.width
	
if __name__ == '__main__':
 	print("Starting decision making")
 	
	global pub_vel, vel_msg
	
	# initialize linear and angular velocity
	vel_msg = Twist()
	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0
 	
 	# Initialize the node
 	rospy.init_node('decision_making', anonymous=True)
 
 	# Subscribe to find_ball topics
 	sub_ball_detection = rospy.Subscriber("/ball_detection", ObjDetection, drive_callback)
 	
 	# Subscribe to usb_cam
 	sub_cam_info = rospy.Subscriber("/usb_cam/camera_info", CameraInfo, cam_info_callback)
 	
 	# Publish to cmd_vel topic to drive turtlebot
 	pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
 	
 	
 	
 	rospy.spin()
