#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from line_follower.msg import ErrorMsg
from std_msgs.msg import Bool

from LineFollower.Vision.VisualProcessor import VisualProcessor

class VisionNode:

    def __init__(self):
	self.drive = False
        # Initialize the visual processor
        self.visualProcessor = VisualProcessor()

        # Get some constants
        cameraTopic = rospy.get_param('/line_follower/camera_topic')
        errorTopic = rospy.get_param('/line_follower/error_topic')
        visualDebugTopic = rospy.get_param('/line_follower/visual_debug_topic')

        # Create a bridge to convert ros image
        # messages to opencv images
        self.bridge = CvBridge()
	
	self.drivePub = rospy.Publisher(
		'/line_follower/driveBool',
		Bool,
		queue_size = 1)

        # Initialize a topic to publish
        # the error message for the controller
        self.errorPub = rospy.Publisher(
                errorTopic,
                ErrorMsg,
                queue_size = 1)

        # Subscribe to the camera
        self.imageSub = rospy.Subscriber(
                cameraTopic,
                Image, 
                self.cameraCallback, 
                queue_size=1)

        # Publish a debug image
        self.debugImagePub = rospy.Publisher(
                visualDebugTopic,
                Image, 
                queue_size=1)

    def cameraCallback(self, msg):
        # Convert the message to an opencv image
        image = self.bridge.imgmsg_to_cv2(msg)

        # Get an error message
        errorMsg, debugImage, self.drive = self.visualProcessor.computeError(image)

        # Publish the error message
        self.errorPub.publish(errorMsg)

        # Publish the debug image
        debugImageMsg = self.bridge.cv2_to_imgmsg(debugImage,"bgr8")
        self.debugImagePub.publish(debugImageMsg)

	self.drivePub.publish(self.drive)

if __name__ == "__main__":
    rospy.init_node('vision_node')
    visualNode = VisionNode()
    rospy.spin()
