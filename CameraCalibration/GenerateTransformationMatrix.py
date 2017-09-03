#!/usr/bin/python

import rospy
from geometry_msgs.msg import Point

import numpy as np

from LineFollower.CameraCalibration.Constants import *

class GenerateTransformationMatrix:
    
    def __init__(self):
        # Get the coordinates to be clicked
        self.corners = rospy.get_param('/line_follower/camera_calibration_corners').items()

        self.numPoints = len(self.corners)

        # The clicked pixels
        self.pixelCoordinates = []

        cameraTopic = rospy.get_param('/line_follower/camera_topic')
        clickTopic = rospy.get_param('/line_follower/click_topic')

        # Subscribe to clicks
        self.clickSub = rospy.Subscriber(
                clickTopic,
                Point, 
                self.pointClicked, 
                queue_size=1)

        out = "open rqt_image_view and subscribe to " + cameraTopic + "\n"
        out += "Use the check box to select " + clickTopic + " to publish mouse clicks.\n"
        out += "Place a piece of paper with the long side 63cm from the back wheel of the car."

        rospy.loginfo(out)

        rospy.loginfo("Click on the " + str(self.corners[0][0]) + " corner.")

        # Wait for all points to be clicked
        self.allPointsClicked = False
        rate = rospy.Rate(1)
        while (not rospy.is_shutdown()) and (not self.allPointsClicked):
            rate.sleep()

        if self.allPointsClicked:
            self.generateTransformation()

    def pointClicked(self, msg):
        if len(self.pixelCoordinates) >= self.numPoints:
            return

        # Add the pixel value
        x = msg.x
        y = msg.y
        point = np.array([x, y, 1.])
        self.pixelCoordinates.append(point)

        if len(self.pixelCoordinates) < self.numPoints:
            rospy.loginfo("Click on the " + self.corners[len(self.pixelCoordinates)][0] + " corner.")
        else:
            self.allPointsClicked = True
        
    @staticmethod
    def approximateProjective(sources, destinations):
        """
        Given source points [X0, X1, ...]
        and destination points [Y0, Y1, ...]
        We want to find projective transformation matrix H

        H X ~ Y for all X, Y

        [X0 Y0 1  0  0  0  0  0  -X0' 0  0  0 ] [H0] = [0 ]
        [0  0  0  X0 Y0 1  0  0  -Y0' 0  0  0 ] [H1]   [0 ]
        [0  0  0  0  0  0  X0 Y0 -1   0  0  0 ] [H1]   [-1]
                .                               [H2]    .
                .                               [H3]    .
                .                               [H4]    .
                                                [H5]
                                                [H6]
                                                [H7]
                                                [W0]
                                                [W1]
                                                [W2]
                                                [W3]
        """

        numPoints = len(sources)
        X = np.zeros((3*numPoints, 12))
        Y = np.zeros(3*numPoints)

        for i in xrange(numPoints):
            # Populate X
            X[3*i, 0] = sources[i][0]
            X[3*i, 1] = sources[i][1]
            X[3*i, 2] = 1.
            X[3*i + 1, 3] = sources[i][0]
            X[3*i + 1, 4] = sources[i][1]
            X[3*i + 1, 5] = 1.
            X[3*i + 2, 6] = sources[i][0]
            X[3*i + 2, 7] = sources[i][1]

            X[3*i + 0, 8+i] = -destinations[i][0]
            X[3*i + 1, 8+i] = -destinations[i][1]
            X[3*i + 2, 8+i] = -1.

            # Populate Y
            Y[3*i + 2] = -1.

        H = np.dot(np.linalg.pinv(X), Y)

        w = [H[8], H[9], H[10], H[11]]

        H = np.array(
                [[H[0], H[1], H[2]],
                 [H[3], H[4], H[5]],
                 [H[6], H[7], 1.  ]])

        return H

    def generateTransformation(self):
        rospy.loginfo("Generating transformation matrix...")

        # Set the sources
        worldCoordinates = [0] * self.numPoints
        for i, (name, corner) in enumerate(self.corners):
            worldCoordinates[i] = np.array(corner)

        transformationMatrix = GenerateTransformationMatrix.approximateProjective(
                self.pixelCoordinates,
                worldCoordinates)

        # Write to file
        np.savetxt(TRANSFORMATION_MATRIX_FILE, transformationMatrix)
        np.savetxt(TRANSFORMATION_MATRIX_INV_FILE, np.linalg.inv(transformationMatrix))
        rospy.loginfo("Matrix written to " + TRANSFORMATION_MATRIX_FILE)

if __name__ == "__main__":
    rospy.init_node('generate_transformation_matrix')
    g = GenerateTransformationMatrix()
