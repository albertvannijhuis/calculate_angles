# Detecting angle between two green post-its with using a given camera  
#
# File:		calculate_angle.py
# Author:	Albert van Nijhuis										
# Date:	    11-03-2021											

### Imports 
import numpy as np
import cv2
import imutils
import math
import time
import argparse 
import sys
import requests
from picamera.array import PiRGBArray
from picamera import PiCamera

### Class for calculating the angle between two green objects with the webcam 
class CalculateAngle:
	## Initializing functon
	def __init__(self, parWidth, parLowerColor, parUpperColor, parMinContourArea):
		# Initialize variables
		self.varWidth = parWidth
		self.varLowerColor = parLowerColor
		self.varUpperColor = parUpperColor
		self.varMinContourArea = parMinContourArea
		self.varAngleData = []
		self.varIndex = 0
		
		# allow the camera or video file to warm up
		time.sleep(2.0)
	
	## Function that preprocesses a frame
	def PreProcessFrame(self, parFrame):
		# Blur the frame to reduce noise for detecting edged
		tempBlurred = cv2.GaussianBlur(parFrame, (11, 11), 0)
		
		# Saturate the blurred frame so red, green and blue stand out more
		tempProcessedFrame = cv2.cvtColor(tempBlurred, cv2.COLOR_BGR2HSV)
		
		# Lastly, return the preprocesssed frame
		return tempProcessedFrame
	
	## Fucntion that returns a mask that only detects objects in a ginven color-range
	def CreateMask(self, parProcessedFrame):
		# First, we need to set the color range
		tempMask = cv2.inRange(parProcessedFrame, self.varLowerColor, self.varUpperColor)
		
		# Eroding the mask to further reduce noise 
		tempMask = cv2.erode(tempMask, None, iterations = 2)
		
		# Dialate the green objects
		tempMask = cv2.dilate(tempMask, None, iterations = 2)
	
		# Return the mask
		return tempMask
	
	## Function that calculates and returns the centerpoints 
	def CalculateCenterPoints(self, parMask):
		# Get contour values 
		tempContours, tempHierachie = cv2.findContours(parMask.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
	
		# Get center points of contours and draw it to the image. We only process contours that have a area
		# bigger then the given area in pixels
		tempCentres = []
		for i in range(len(tempContours)):
			if cv2.contourArea(tempContours[i]) < self.varMinContourArea:
				continue

			# Ger the importens points of the contours (middle, edges) with the moments function
			tempMoments = cv2.moments(tempContours[i])
			tempCentres.append((int(tempMoments['m10'] / tempMoments['m00']), 
								int(tempMoments['m01'] / tempMoments['m00'])))
		
		# Check if there are 2 centerpoints, if so return it, else return 0
		if len(tempCentres) == 2:
			return tempCentres
		else:
			return 0
	
	## Funciton that calculates and returns the angel in degrees
	def CalculateAngle(self, parCentres):
		# Get the max x, min x, max y and min y values for calculating the angle
		tempMax_x = max(parCentres[0][0], parCentres[1][0])
		tempMin_x = min(parCentres[0][0], parCentres[1][0])
		tempMax_y = max(parCentres[0][1], parCentres[1][1])
		tempMin_y = min(parCentres[0][1], parCentres[1][1])
		
		# Calculate the angle with the tan function and convert it to degrees 
		tempTan = math.atan2(tempMax_y - tempMin_y, tempMax_x - tempMin_x)
		tempDegree = math.degrees(tempTan)
		
		# If the left centerpoint is higher then the right centerpoint
		# multiply the value *-1, so the values won't be the same
		if parCentres[0][0] > parCentres[1][0]:
			tempDegree = tempDegree * -1
		
		# Return the degrees 
		return tempDegree

	## Function that removes all the outliers in an given numpy array. 
	## The m parameter determines how big or small a outlier is 
	def RemoveOutliers(self, parData, parM):
		return parData[abs(parData - np.mean(parData)) < parM * np.std(parData)] 
	
	## Function for posting json data to the nodered server
	def PostJsonData(self, parUrl, parPayload):
		try:
			r = requests.post(parUrl, json = parPayload)
		except requests.exceptions.RequestException as e:  # This is the correct syntax
			raise SystemExit(e)
	
	
### Main function	
if __name__ == "__main__":
	# Initialize webcam and set the fps 
	varCap = PiCamera()
	varCap.resolution = (640, 480)
	varCap.framerate = 32
	varRawCap = PiRGBArray(varCap, size=(640, 480))

	# Create object of class and parsing in width, color boundaries and the minim	
	CalcAngle = CalculateAngle(parWidth = 600, 
							   parUpperColor = (64, 255, 255), 
							   parLowerColor = (29, 86, 6), 
							   parMinContourArea = 200)
	
	# Initialize variables
	varAngleData = []
	varIndex = 0
	
	# StarCalcAngleting loop	
	for tempFrame in varCap.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		# Preprocess the frame and create a mask of it
		tempProcessedFrame = CalcAngle.PreProcessFrame(tempFrame)
		
		# Create a mask
		tempMask = CalcAngle.CreateMask(tempProcessedFrame)
		
		# Get centerpoints
		tempCentres = CalcAngle.CalculateCenterPoints(tempMask)
		
		# Continue if the function returned a array with 2 centerpoints
		if tempCentres != 0:
			# Get the degree
			tempDegree = CalcAngle.CalculateAngle(tempCentres)
			
			# For visualisation, draw the centerpoints, a line between them and put the angel on the screen 
			cv2.circle(tempFrame, tempCentres[-1], 3, (0, 0, 0), -1)
			cv2.line(tempFrame, tempCentres[0], tempCentres[1], (255,255,255), 3)
			cv2.putText(tempFrame,
						str(round(tempDegree, 2)),
						(50, 50),
						cv2.FONT_HERSHEY_SIMPLEX,
						1,
						(255,255,255),
						2,
						cv2.LINE_AA)
		
			# Put the data in a JSON file. Every 10th iteration the average of the data will be calculated
			# and the outliers will be taken away. After that the average will be put into a JSON file
			varIndex = varIndex + 1
			
			if varIndex < 10:
				varAngleData.append(tempDegree)
			else:
				# Reset index
				varIndex = 0
				
				# Convert array to numpy array so calculations can be done easier
				tempNpArray = np.asarray(varAngleData)
				
				# Remove outliers from numpy array
				tempNpArray = CalcAngle.RemoveOutliers(tempNpArray, 3)
				
				# Get average from array
				tempAvg = np.mean(tempNpArray)
				
				# Create payload
				tempPayload = {'angle' : int(tempAvg)}
				
				# Post the data
				CalcAngle.PostJsonData("http://192.168.137.3:1880", tempPayload)
				
		# output the image
		cv2.imshow("Calculated angle of two green objects", tempFrame)
			
		# Get pressed key
		key = cv2.waitKey(1) & 0xFF
	
		# if the 'q' key is pressed, stop the loop
		if key == ord("q"):
			break
