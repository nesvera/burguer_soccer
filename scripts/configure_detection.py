import cv2
import numpy as np
import math
from collections import deque
import imutils
import os
import signal
import pickle
import sys
import subprocess

filterParameters = None
parameterFilePath = None
v4l2ucpProc = None

positionBuffer = deque(maxlen=10)

def nothing(x):
	pass

def mouse_handler(event, x, y, flags, param):
	print("Mouse position: (" + str(x) + " , " + str(y) +")")

def main():
	# open v4l2 to configure the camera
	#os.system("v4l2ucp &")
	v4l2ucpProc = subprocess.Popen(['v4l2ucp'])

	loadFile = os.getcwd() + "/" + parameterFilePath

	# Create the tracking window
	cv2.namedWindow('Tracker')
	cv2.createTrackbar('minH','Tracker', 0,255, nothing)
	cv2.createTrackbar('minS','Tracker', 0,255, nothing)
	cv2.createTrackbar('minV','Tracker', 0,255, nothing)
	cv2.createTrackbar('maxH','Tracker', 255,255, nothing)
	cv2.createTrackbar('maxS','Tracker', 255,255, nothing)
	cv2.createTrackbar('maxV','Tracker', 255,255, nothing)
	cv2.createTrackbar('Erode' ,'Tracker',  0, 10, nothing)
	cv2.createTrackbar('Dilate' ,'Tracker',  0, 10, nothing)
	cv2.createTrackbar('filterSize', 'Tracker', 0, 100, nothing)
	cv2.createTrackbar('radius', 'Tracker', 0, 100, nothing)

	# if file already exist, load the values
	if os.path.isfile(loadFile):
		filterParameters = pickle.load(open(loadFile, "rb"))

		cv2.setTrackbarPos('minH','Tracker', filterParameters.get('minH') )
		cv2.setTrackbarPos('minS','Tracker', filterParameters.get('minS') )
		cv2.setTrackbarPos('minV','Tracker', filterParameters.get('minV') )
		cv2.setTrackbarPos('maxH','Tracker', filterParameters.get('maxH') )
		cv2.setTrackbarPos('maxS','Tracker', filterParameters.get('maxS') )
		cv2.setTrackbarPos('maxV','Tracker', filterParameters.get('maxV') )
		cv2.setTrackbarPos('Erode' ,'Tracker', filterParameters.get('Erode') )
		cv2.setTrackbarPos('Dilate' ,'Tracker', filterParameters.get('Dilate') )
		cv2.setTrackbarPos('filterSize', 'Tracker', filterParameters.get('fSize') )
		cv2.setTrackbarPos('radius', 'Tracker', filterParameters.get('radius'))

	# Read from webcam
	camera = cv2.VideoCapture(0)

	# Loop through each webcam frame
	while(True):


		if camera.isOpened():
			ret, frame = camera.read()

			# Get values of the GUI
			i1 = cv2.getTrackbarPos('Erode', 'Tracker')
			i2 = cv2.getTrackbarPos('Dilate', 'Tracker')
			minH = cv2.getTrackbarPos('minH', 'Tracker')
			minS = cv2.getTrackbarPos('minS', 'Tracker')
			minV = cv2.getTrackbarPos('minV', 'Tracker')
			maxH = cv2.getTrackbarPos('maxH', 'Tracker')
			maxS = cv2.getTrackbarPos('maxS', 'Tracker')
			maxV = cv2.getTrackbarPos('maxV', 'Tracker')
			fSize = (cv2.getTrackbarPos('filterSize', 'Tracker')*2) + 1
			raio = cv2.getTrackbarPos('radius', 'Tracker')

			# up and low color boundaries
			colorLower = (minH,minS,minV)
			colorUpper = (maxH,maxS,maxV)

			#frame_r = cv2.resize(frame, None, fx=0.4, fy=0.4, interpolation=cv2.INTER_CUBIC)

			# converto to HSV color space, create mask based on the color, erode and dilate 
			blurred = cv2.GaussianBlur(frame, (fSize, fSize), 0)
			hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
			mask = cv2.inRange(hsv, colorLower, colorUpper)
			mask = cv2.erode(mask, None, iterations=i1)
			mask = cv2.dilate(mask, None, iterations=i2)

			# Detect contours
			(_,cnts, _) = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			if len(cnts) > 0:

				# get the contour with the larger
				c = max(cnts, key=cv2.contourArea)

				# find a circle of the minimum area enclosing some points
				((x, y), radius) = cv2.minEnclosingCircle(c)

				# just draw if there is a certain size
				if radius > raio:

					# find the moments to define de center of the circle
					M = cv2.moments(c)
					center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
					(cX, cY) = center
					(startX, endX) = (int(cX - 20), int(cX + 20))
					(startY, endY) = (int(cY - 20), int(cY + 20))
					cv2.line(frame, (startX, cY), (endX, cY), (0, 0, 0), 3)
					cv2.line(frame, (cX, startY), (cX, endY), (0, 0, 0), 3)

					# draw the circle and centroid on the frame,
					# then update the list of tracked points
					cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
					cv2.circle(frame, center, 5, (0, 0, 255), -1)

					#print(center)

			lVertLine = int(frame.shape[1]*0.2)
			rVertLine = int(frame.shape[1]*0.8)
			horLine = int(frame.shape[0]*0.8)

			cv2.line(frame, (lVertLine,0), (lVertLine, frame.shape[0]), (0,0,0), 5)
			cv2.line(frame, (rVertLine,0), (rVertLine, frame.shape[0]), (0,0,0), 5)
			cv2.line(frame, (0, horLine), (frame.shape[1], horLine), (0,0,0), 5)

			# Display the results
			cv2.imshow('Original',frame)
			#cv2.imshow('blurred', blurred)
			cv2.imshow('mask',mask)

			cv2.setMouseCallback('Original', mouse_handler)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	camera.release()
	cv2.destroyAllWindows()

def exit_handler(signal, frame):

	# Get values of the GUI
	i1 = cv2.getTrackbarPos('Erode', 'Tracker')
	i2 = cv2.getTrackbarPos('Dilate', 'Tracker')
	minH = cv2.getTrackbarPos('minH', 'Tracker')
	minS = cv2.getTrackbarPos('minS', 'Tracker')
	minV = cv2.getTrackbarPos('minV', 'Tracker')
	maxH = cv2.getTrackbarPos('maxH', 'Tracker')
	maxS = cv2.getTrackbarPos('maxS', 'Tracker')
	maxV = cv2.getTrackbarPos('maxV', 'Tracker')
	fSize = (cv2.getTrackbarPos('filterSize', 'Tracker'))
	raio = cv2.getTrackbarPos('radius', 'Tracker')

	filterParameters = {	"Erode": i1,
							"Dilate": i2,
							"minH": minH,
							"minS": minS,
							"minV": minV,
							"maxH": maxH,
							"maxS": maxS,
							"maxV": maxV,
							"fSize": fSize,
							"radius": raio}

	filePath = "./" + parameterFilePath
	pickle.dump(filterParameters, open(filePath, "wb"))

	exit(0)

if __name__ == "__main__":

	if len(sys.argv) != 2:
		print("Usage: " + sys.argv[0] +  " <parameter file path>")
		exit(0)
	
	# get file from parameter
	parameterFilePath = sys.argv[1]	
	
	print("Press CTRL+C to close the script ...")
	print("The filter values will be saved.")

	# set function to save when exit de script
	signal.signal(signal.SIGINT, exit_handler)

	main()


