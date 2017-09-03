#!/usr/bin/python

import rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from ackermann_msgs.msg import AckermannDriveStamped
import tf
from std_msgs.msg import Float32

class ARStateChanger:
	def __init__(self):
		rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback)
		self.pub = rospy.Publisher("/state", Float32, queue_size=1)
		
		self.current_ID = 16

		self.START_ID = 16
		self.RW_ID = 17
		self.HAIR_ID = 18
		self.LINE_ID = 1 #TODO: BACK TO 20!!!
		self.UNDER_ID = 21
		self.BOA_ID = 22
		self.WALL_ID = 23

		self.LOCAL = 1
		self.WALL = 2
		self.LINE = 3

		self.over_pub = rospy.Publisher("/overpass", Float32, queue_size=1) 
		
		self.OVER_ID = 19

		self.NORM = 1
		self.SHORT = 2
		print "subsribed"

	def ar_callback(self, msg):
                #print "callback"
		#cmd = AckermannDriveStamped()
		#cmd.header.stamp = rospy.Time.now()
		
		for marker in msg.markers:
			if marker.id >= 1 and marker.id <= 23: #TODO: BACK TO 16 from 1!!!!!
				if marker.id != self.current_ID and marker.pose.pose.position.z < 1.5: #and marker.pose.pose.position.z < 1#TODO: MAKE SURE AR DIST IS RIGHT
					self.current_ID = marker.id
                                        print(self.current_ID)
		
		if(self.current_ID == self.START_ID):
			self.pub.publish(self.LOCAL)
			self.over_pub.publish(self.NORM)
		elif(self.current_ID == self.RW_ID):
			self.pub.publish(self.LOCAL)
			self.over_pub.publish(self.NORM)
		elif(self.current_ID == self.HAIR_ID):
			self.pub.publish(self.LOCAL)
			self.over_pub.publish(self.NORM)
		elif(self.current_ID == self.OVER_ID):
			self.pub.publish(self.LOCAL)
			self.over_pub.publish(self.SHORT)
		elif(self.current_ID == self.LINE_ID):
			self.pub.publish(self.LINE)
			self.over_pub.publish(self.NORM)
		elif(self.current_ID == self.UNDER_ID):
			self.pub.publish(self.LOCAL)
			self.over_pub.publish(self.NORM)
		elif(self.current_ID == self.BOA_ID):
			self.pub.publish(self.LOCAL)
			self.over_pub.publish(self.NORM)
		elif(self.current_ID == self.WALL_ID):
			self.pub.publish(self.WALL)
			self.over_pub.publish(self.NORM)
		else:
                        self.pub.publish(self.current_ID)

if __name__ == '__main__':
	rospy.init_node("ARStateChanger")
	node = ARStateChanger()
	rospy.spin()
