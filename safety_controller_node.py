#!/usr/bin/python

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import numpy as np
import math

class SafetyControllerNode:
	def __init__(self):
		#threshold distance where the robot is still seeing itself (meters)
		self.thresholdRobotSelfSeeing = 0.5
		#number of laser scan points to average together
		self.k = 20

		rospy.Subscriber("/scan", LaserScan, self.laser_callback)
		rospy.Subscriber("/ackermann_cmd_mux/output", AckermannDriveStamped,self.ackermann_cmd_input_callback)
		self.cmd_pub = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size=10)
		self.odom_cb = rospy.Subscriber("/odom", Odometry,self.odom_callback, queue_size=10)
		self.SAFE = True
		self.SIDE_DISTANCE = 0.27
		self.FRONT_DISTANCE = 0
		self.currentSpeed = 0
		self.currentAngleOffset = 0
		self.turnOffset = 0
		self.currentOdom = Odometry()
		self.minRange = 1000
		self.currentStep = 0
		self.carLength = 0.32
		self.carWidth = 0.28
		self.laserOffset = 0.05
		self.currentTurnAngle = 0
		self.margin = 0.15
		self.SAFETY_MODE = 0 # 0 for normal operation, 1 for ar-tag conga line


	def odom_callback(self,msg):
		self.currentOdom = msg

	def getStep(self,msg):
    		for x in range(80 ,1000):
			if self.minRange > msg.ranges[x] :
				self.minRange = msg.ranges[x]

		for x in range(80,1000):
			if self.minRange == msg.ranges[x]:
				self.step = x
				break
		return self.step

	def curveController(self,msg):
		self.h = self.carLength / math.tan(self.currentTurnAngle)
		self.turnRadius = self.carLength / math.sin(self.currentTurnAngle)
		self.count = 0
		for x in range(180,900):
			self.currentRange = msg.ranges[x]
			self.currentAngle = (x - 180) / 4
			self.currentAngle = math.radians(self.currentAngle)
			self.posX = self.currentRange * math.cos(self.currentAngle)
			self.posY = self.currentRange * math.sin(self.currentAngle) + self.laserOffset
			self.circle = (self.posX - self.h)**2 + (self.posY + self.carLength)**2
			self.outerMargin = (self.turnRadius + self.margin)**2
			self.innerMargin = (self.turnRadius - self.margin)**2
			self.rangeToObject = math.sqrt(self.posX**2 + self.posY**2)
			if self.circle > self.innerMargin and self.circle < self.outerMargin and self.rangeToObject < self.FRONT_DISTANCE:
				self.count += 1
		if self.count > 2:
			self.safe = False
		else :
			self.safe = True
		return self.safe

	def straightController(self,msg):
		self.currentRange = 0
		self.currentAngle = 0
		self.posX = 0
		self.posY = 0
		self.counter = 0
		self.turnOffset = int(math.degrees(self.currentAngleOffset) * 4)
		self.minRange = 1000
		for x in range(180 - self.turnOffset,900 - self.turnOffset) :
			self.currentRange = msg.ranges[x]
			self.currentAngle = (x - 180 - self.turnOffset) / 4
			self.currentAngle = math.radians(self.currentAngle)
			self.posX = self.currentRange * math.cos(self.currentAngle)
			self.posY = self.currentRange * math.sin(self.currentAngle)
                        if self.posX < self.SIDE_DISTANCE and self.posX > -self.SIDE_DISTANCE and self.posY < self.FRONT_DISTANCE:
                                self.counter = self.counter + 1
		if self.counter > 2  :
			self.safe = False
		else :
			self.safe = True
		return self.safe
                      
		
	
	def laser_callback(self, msg):
		self.currentStep = self.getStep(msg)
		if self.currentTurnAngle != 0:
			self.curveSafe = self.curveController(msg)
		else :
			self.curveSafe = False
		self.straightSafe = self.straightController(msg)
		if self.straightSafe or self.curveSafe :
			self.SAFE = True
		else :
			self.SAFE = False
		print self.SAFE

		
	def calculateSafeDistance(self,msg):
		self.currentSpeed = msg.drive.speed
		self.currentTurnAngle = msg.drive.steering_angle
		if self.SAFE:	
			self.FRONT_DISTANCE = 0.223607 * (1.794144**self.currentOdom.twist.twist.linear.x) + 0.1
		elif self.currentOdom.twist.twist.linear.x < 0.01:
			self.FRONT_DISTANCE = 0.223607 * (1.794144**self.currentOdom.twist.twist.linear.x) + 0.1
		if self.currentOdom.twist.twist.linear.x < 0 :
			self.FRONT_DISTANCE = .5

	def calculateAvoidance(self,back_up_msg):
                back_up_msg.drive.speed = 0

		if msg.drive.speed != 0 and self.currentOdom.twist.twist.linear.x < 0.01 :
			back_up_msg.drive.speed = -0.5
			if self.currentStep > 540 :
				back_up_msg.drive.steering_angle = 3
			else  :
				back_up_msg.drive.steering_angle = -3
			
		if self.currentOdom.twist.twist.linear.x > 0:
			if self.currentStep > 540 :
				back_up_msg.drive.steering_angle = -3
			else  :
				back_up_msg.drive.steering_angle = 3
		return back_up_msg

	def ackermann_cmd_input_callback(self, msg):
		self.calculateSafeDistance(msg)

		if self.SAFE :
			self.cmd_pub.publish(msg)
		else:
			back_up_msg = AckermannDriveStamped()
			if self.SAFETY_MODE == 0:
				back_up_msg = self.calculateAvoidance(back_up_msg)
			else:
				back_up_msg.drive.speed = 0
			back_up_msg.header.stamp = rospy.Time.now()
			self.cmd_pub.publish(back_up_msg)

if __name__ == '__main__':
	rospy.init_node("SafetyControllerNode")
	node = SafetyControllerNode()
	rospy.spin()
