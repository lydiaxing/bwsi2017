#!/usr/bin/python

import rospy
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped

class StateMachineNode():
    def __init__(self):
	self.LOCAL = 1
	self.WALL = 2
	self.LINE = 3

	self.NORM = 1
        self.SHORT = 2
		
	self.localmsg = AckermannDriveStamped()	
	self.wallmsg = AckermannDriveStamped()
	self.linemsg = AckermannDriveStamped()
	self.hairmsg = AckermannDriveStamped()

	self.state = rospy.Subscriber("/state", Float32, self.state) 
	self.localSub = rospy.Subscriber("/local_potential", AckermannDriveStamped, self.local) 
	self.wallSub = rospy.Subscriber("/wall_follow", AckermannDriveStamped, self.wall) 
	self.lineSub = rospy.Subscriber("/line_follow", AckermannDriveStamped, self.line) 

	self.pub = rospy.Publisher("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)

	self.overmsg = AckermannDriveStamped()
	self.normmsg = AckermannDriveStamped()
	self.overSub = rospy.Subscriber("/oversaf", AckermannDriveStamped, self.over) 
	self.normSub = rospy.Subscriber("/normsaf", AckermannDriveStamped, self.norm)
	self.stateSafSub = rospy.Subscriber("/overpass", Float32, self.safState)
	self.saf_pub = rospy.Publisher("/ackermann_cmd_mux/input/default", AckermannDriveStamped, queue_size=1)

    def state(self, msg):
	if(msg.data == self.LOCAL):
		self.pub.publish(self.localmsg)
	elif(msg.data == self.WALL):
		self.pub.publish(self.wallmsg)
	elif(msg.data == self.LINE):
		self.pub.publish(self.linemsg)

    def safState(self, msg):
        if(msg.data == self.NORM):
                self.saf_pub.publish(self.normmsg)
        elif(msg.data == self.SHORT):
                self.saf_pub.publish(self.overmsg)

    def local(self, msg):
	self.localmsg = msg

    def wall(self, msg):
	self.wallmsg = msg

    def line(self, msg):
	self.linemsg = msg
	
    def over(self, msg):
	self.overmsg = msg

    def norm(self, msg):
	self.normmsg = msg
        
if __name__ == '__main__':
	rospy.init_node("StateMachineNode")
	node = StateMachineNode()
	rospy.spin()
