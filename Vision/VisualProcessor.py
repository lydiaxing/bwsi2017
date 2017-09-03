import rospy
import numpy as np
import cv2

from line_follower.msg import ErrorMsg

class VisualProcessor:
    def __init__(self):
	self.HSV_RANGES = [(19, 156, 85),(92, 255, 255)] #orange[(10, 90, 140),(22, 245, 255)] 

    def computeError(self, image):
	cv2.rectangle(image, (0, 0), (1280, 450), (0, 0, 0), -1)
	cv2.rectangle(image, (0, 500), (1280, 720), (0, 0, 0), -1)
	contours, hsv, thresh = self.process(image)

	if len(contours) > 0:
		largest_contour = self.sortContours(contours)[len(contours)-1]
		self.drive = True
	else:
		self.drive = False
		largest_contour = None
		
	#make the contours image into rectangles, (x,y) is the top left corner, w is width, h is height
	x, y, w, h = cv2.boundingRect(largest_contour)
	final_image = cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
	#horiz_center = x + (w / 2)
	horiz_center = x + (w / 2)
	vert_center = y + (h/2)

	debugImage = image.copy()

	errorMsg = ErrorMsg()
        errorMsg.targetCenter = horiz_center
	errorMsg.targetHeight = vert_center 


        return errorMsg, debugImage, self.drive

    def getContours(self, image):
	new_image, contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	return contours, new_image

    def sortContours(self, contours):
	sort = sorted(contours, key=cv2.contourArea)
	return sort

    def process(self, cv_image):
	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
	thresh = cv2.inRange(hsv, self.HSV_RANGES[0], self.HSV_RANGES[1])
	contours, contour_labeled_image = self.getContours(thresh)
	return contours, hsv, thresh
