import numpy as np

from LineFollower.CameraCalibration.Constants import *

transformationMatrix = np.loadtxt(TRANSFORMATION_MATRIX_FILE)
transformationMatrixInv = np.loadtxt(TRANSFORMATION_MATRIX_INV_FILE)

'''
To access these functions in another file add an import statement:
from LineFollower.Control.CoordinateTransformations import CoordinateTransformations
Then you can call these functions:
world = CoordinateTransformations.pixelsToWorld(pixels)
'''

# EVERYTHING I RETURN IS Y,X BECAUSE ROW, COLUMN

class CoordinateTransformations:

    @staticmethod
    def pixelsToWorld(pixels):
        '''
        TODO
        Compute the world coordinates given the pixel coordinates
        and a transformation matrix. You have access to the
        transformationMatrix variable above.
        '''
	y,x = pixels

	coordMatrix = np.array([x,y,1])

	transform_arr = np.dot(transformationMatrix, coordMatrix)
	transform_list = np.toList(transform_arr)

	w = transform_list[2]
	
	y_world = transform_list[1]/w
	x_world = transform_list[0]/w	
	
	world_coord = (y_world, x_world)

        return world_coord
       
    @staticmethod
    def worldToPixels(world):
        '''
        TODO
        Compute the pixel coordinates given the world coordinates
        and an inverse transformation matrix. You have access to the
        transformationMatrixInv variable above.
        '''

	world_y, world_x = world
	world_arr = np.asArray([world_x, world_y, 1])

	scaled_pixel_arr = np.dot(transformationMatrixInv, world_arr)
	scaled_pixel_list = np.toList(scaled_pixel_arr)

	w_inverse = scaled_pixel_list[2]

	pixel_x = scaled_pixel_list[0]/w_inverse
	pixel_y = scaled_pixel_list[1]/w_inverse

	pixelCoord = (pixel_y, pixel_x)

        return pixelCoord

