#!/usr/bin/python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from ackermann_msgs.msg import AckermannDriveStamped
from rospy.numpy_msg import numpy_msg

class LocalPotentialField:
    def __init__(self):
        rospy.Subscriber("/scan", numpy_msg(LaserScan), self.laser_scan)
	self.cmd_pub = rospy.Publisher("/local_potential", AckermannDriveStamped, queue_size=1) 

        #self.cmd_pub = rospy.Publisher("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
	# rospy.Subscriber("/ackermann_cmd_mux/output", AckermannDriveStamped, self.output_scan)

	#self.cmd_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
	#rospy.Subscriber("/vesc/high_level/ackermann_cmd_mux/output", AckermannDriveStamped, self.output_scan)

	#rospy.Subscriber("/ackermann_cmd_mux/output", AckermannDriveStamped, self.output_scan)

        self.k_front = 0.1 #right
        self.k_side = 0.1 #right, wall straight is 0.1
        self.kback = 25.0
        self.backDist = 1000000
        self.kpSpeed = 0.000004
	self.kdAngle =  -0.001
        self.kpAngle = 0.00001
	self.dval = 0
	self.wantAngle = 0
	self.angleList = [0]*10
	self.max =0
	self.step =0

    def laser_scan(self, msg):
        forceX = 0
        forceY = 0

        x = np.arange(280, 800)
        currentAngle = (x - 180) / 4.0
        currentAngle = np.radians(currentAngle)
        compX = msg.ranges[x] * np.cos(currentAngle)
        compY = msg.ranges[x] * -np.sin(currentAngle)

        forceX = np.sum(np.divide(15, np.square(compX) * np.sign(compX) * np.square(msg.ranges[x]) + 0.001 * np.sign(compX)))
        forceY = np.sum(np.divide(20, np.square(compY) * np.sign(compY) * np.square(msg.ranges[x]) + 0.001 * np.sign(compY)))
                
        #forceX += 4 / ((((compX )**2 * np.sign(compX)) * msg.ranges[x]**2) + 0.01 * np.sign(compX))
        #forceY += 2 / ((((compY )**2 * np.sign(compY)) * msg.ranges[x]**2) + 0.01 * np.sign(compY))

                
        forceY = forceY + self.backDist
	temp = forceX * self.kpAngle + self.dval
    
	driveMsg = AckermannDriveStamped()
	if temp > 0.34:
		driveMsg.drive.steering_angle = 0.34
	elif temp < -0.34:
		driveMsg.drive.steering_angle = -0.34
	else:
		driveMsg.drive.steering_angle = temp
	
	self.speed = forceY * self.kpSpeed
        #3 on straightaways
	print self.dval
     
	self.speed = -4.411716 * abs(temp) + 4
	if (self.speed < 2):
            self.speed = 2
        if (self.speed > 4):
            self.speed = 4
        driveMsg.drive.speed = self.speed #(self.kpSpeed * np.divide(totalForceX, totalForceX) * math.sqrt(totalForceX**2 + totalForceY**2))*2
        
        self.cmd_pub.publish(driveMsg)

    def output_scan(self, msg):
	error = self.angleList[-1] - self.angleList[0]
	self.dval = self.kdAngle*error
	self.angleList.append(msg.drive.steering_angle)
	self.angleList.pop(0)

if __name__ == "__main__":
    rospy.init_node("local_potential_controller")
    node = LocalPotentialField()
rospy.spin()
