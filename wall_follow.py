#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
import numpy as np

#follows the right wall
class WallFollowNode():
    def __init__(self):
        self.driving = AckermannDriveStamped()
        self.driving.header.stamp = rospy.Time.now()

        self.driving.drive.speed = 1.5
    	self.dist_setpt = .6
        self.right_wall = 1  #+1 for right, -1 for left
        #self.bang_pub = rospy.Publisher("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
	self.bang_pub = rospy.Publisher("/wall_follow", AckermannDriveStamped, queue_size=1)
        self.scan = rospy.Subscriber("/scan", LaserScan, self.callback)
	self.err = 0
	self.queLength = 30
	self.que = [0]*self.queLength
	self.kd = 0.2
	self.dval = 0

    def callback(self, msg):
        self.driving.drive.steering_angle = 0

        #right wall follower

        if self.right_wall==1:
            my_ranges = msg.ranges[140:220]
        else:
            my_ranges = msg.ranges[860:940]

	error = self.dist_setpt - min(my_ranges)
    

	
        err = self.que[-1] - self.que[0]
        self.dval = self.kd*err
        self.que.append(error)
        self.que.pop(0)
	'''self.que.append(self.derr)
	derr = (self.que[-1] - self.que[0])
	#self.kd *= derr
	self.dval = self.kd*derr
	self.que.pop(0)'''

        kp = 0.2
        control_u = self.right_wall*error*kp+self.dval
        print (self.dval)
	if control_u > 0.34:
		control_u = 0.34
	elif control_u < -0.34:
		control_u = -0.34

        self.driving.drive.steering_angle=control_u     
        self.bang_pub.publish(self.driving)

if __name__ == '__main__':
	rospy.init_node("WallFollowNode")
	node = WallFollowNode()
	rospy.spin()
