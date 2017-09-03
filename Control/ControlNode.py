#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Header
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

from line_follower.msg import ErrorMsg

from LineFollower.Control.Controller import Controller

class ControlNode:

    def __init__(self):
	self.drive = False
        # Initialize the controller
        self.controller = Controller()

        # Get some parameters
        self.velocity  = 2
        errorTopic = rospy.get_param('/line_follower/error_topic')
        navigationTopic = rospy.get_param('/line_follower/navigation_topic')

        # Subscribe to the goal point topic
        self.errorSub = rospy.Subscriber(
                errorTopic,
                ErrorMsg, 
                self.errorCallback, 
                queue_size=1)
	
	self.driveSub = rospy.Subscriber(
		'/line_follower/driveBool',
		Bool,
		self.driveCallback,
		queue_size = 1)

        # Publish to the command mux
        self.commandPub = rospy.Publisher(
                #navigationTopic,
		"/line_follow",
                AckermannDriveStamped,
                queue_size = 1)

    def errorCallback(self, errorMsg):
        # Use the controller to get the steering angle
        steeringAngle = -self.controller.getSteeringAngle(errorMsg)
	self.velocity = abs(steeringAngle)*-4.41176 + 4
        # Output the velocity and the steering angle
        self.publishCommand(self.velocity, steeringAngle)

    def driveCallback(self, driveBool):
	self.drive = driveBool.data

    def publishCommand(self, velocity, angle):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "line_follower"

        drivingCommand = AckermannDriveStamped()

	if self.drive:
	        drivingCommand.drive.speed = velocity
	else:
		drivingCommand.drive.speed = 0

        drivingCommand.drive.steering_angle = angle

        self.commandPub.publish(drivingCommand)
        
if __name__=="__main__":
    rospy.init_node("ControlNode") 
    lineFollower= ControlNode()
    rospy.spin()
