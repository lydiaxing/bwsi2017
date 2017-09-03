#!/usr/bin/python

import rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from ackermann_msgs.msg import AckermannDriveStamped
import tf

class ARMonitorNode:
	def __init__(self):
		rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback)
		self.ar_pub = rospy.Publisher("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)
		self.KSPEED = 0.7
		self.CAR_LENGTH = .425 # in meters
		self.kp = 0.3
		self.speedKp = 1
		self.distReference = 0.7

	def ar_callback(self, msg):
		cmd = AckermannDriveStamped()
		cmd.header.stamp = rospy.Time.now()
		
		
		for marker in msg.markers:
			dist = marker.pose.pose.position.z
                        self.error = self.distReference - dist
                        self.KSPEED = -self.error * self.speedKp + 0.2
                        if self.KSPEED > 0.7 :
                                self.KSPEED = 0.7
			offset  = marker.pose.pose.position.x    
                	offset = offset * self.kp        
			cmd.drive.speed = self.KSPEED
			cmd.drive.steering_angle = -offset 
			
			#goalx = offset
			#goaly = marker.pose.pose.position.y
			#quaternion = marker.pose.pose.orientation
			'''quaternion = (marker.pose.pose.orientation.x,marker.pose.pose.orientation.y,marker.pose.pose.orientation.z,marker.pose.pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quaternion)'''
			#-self.getPurePursuitAngle(goalx, goaly)
			#offset(tested), euler(untested)
			
               	
		self.ar_pub.publish(cmd)
		
                        
	def getPurePursuitAngle(self, goalx, goaly):
		norm = np.linalg.norm((goalx, goaly))
		sina = goalx / norm
		k = 2*sina / norm
		angle = np.arctan(k*self.CAR_LENGTH)
		return angle

if __name__ == '__main__':
	rospy.init_node("ARMonitorNode")
	node = ARMonitorNode()
	rospy.spin()
