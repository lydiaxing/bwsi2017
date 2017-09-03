import rospy
import math

from LineFollower.Vision.CoordinateTransformations import CoordinateTransformations

class Controller:

    def __init__(self):
	self.HORIZ_FOV = 110 # degrees
	self.HORIZ_PX = 672
	self.length_of_car = .425
    
    def getSteeringAngle(self, errorMessage):
	angle_px = errorMessage.targetCenter - (self.HORIZ_PX / 2)
	angle_degrees = (angle_px / self.HORIZ_PX)*self.HORIZ_FOV
	angle_radians = math.radians(angle_degrees)
		
	if(errorMessage.targetCenter is not None):
		steeringAngle = .55*angle_radians
	else:
		steeringAngle = 0

	if steeringAngle > 0.34:
		steeringAngle = 0.34
	elif steeringAngle < -0.34:
		steeringAngle = -0.34
	else:
		steeringAngle = steeringAngle
	
        return steeringAngle

    def getPurePursuitSteeringAngle(self, errorMessage):

	pixels = (errorMessage.targetCenter, errorMessage.targetHeight)

	world_x, world_y = CoordinateTransformations.pixelsToWorld(pixels)
	world_coord_arr = np.asArray([world_x, world_y])

	look_ahead_distance = np.linalg.norm(world_coord_arr)
	sin_axle_angle = (world_x / look_ahead_distance)
	curvature = (2 * sin_axle_angle) / look_ahead_distance

	if(errorMessage.targetCenter is not None):
		purePursuitSteeringAngle = numpy.arctan(curvature * length_of_car)
	else:
		purePursuitSteeringAngle = 0

	if purePursuitSteeringAngle > 0.34:
		purePursuitSteeringAngle = 0.34
	elif purePursuitSteeringAngle < -0.34:
		purePursuitSteeringAngle = -0.34
	else:
		purePursuitSteeringAngle = purePursuitSteeringAngle
	
	return purePursuitSteeringAngle

