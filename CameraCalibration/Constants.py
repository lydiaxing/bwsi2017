import os
VISION_PATH = os.path.split(__file__)[0]
MATRICES_PATH = os.path.join(VISION_PATH, "Matrices")

TRANSFORMATION_MATRIX_FILE = os.path.join(MATRICES_PATH, "TransformationMatrix.txt")
TRANSFORMATION_MATRIX_INV_FILE = os.path.join(MATRICES_PATH, "TransformationMatrixInv.txt")
