"""
This class implements a automated curve/edge detection script.
"""
import cv2
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn import datasets, linear_model
from sklearn.linear_model import Ridge
import pickle
import sys

class 

def detectBlob(color, left_image, right_image):
		
	# Set up the detector with default parameters.
	detector = cv2.SimpleBlobDetector()

	# Detect blobs.
	keypoints = detector.detect(im)
 

def fitPlane (left_image, right_image):
	pass


def __initDetector ():
	# Setup SimpleBlobDetector parameters.
	params = cv2.SimpleBlobDetector_Params()
	 
	# Change thresholds
	params.minThreshold = 10;
	params.maxThreshold = 200;
	 
	# Filter by Area.
	params.filterByArea = True
	params.minArea = 1500
	 
	# Filter by Circularity
	params.filterByCircularity = True
	params.minCircularity = 0.1
	 
	# Filter by Convexity
	params.filterByConvexity = True
	params.minConvexity = 0.87
	 
	# Filter by Inertia
	params.filterByInertia = True
	params.minInertiaRatio = 0.01
	 
	# Create a detector with the parameters
	ver = (cv2.__version__).split('.')
	if int(ver[0]) < 3 :
	    detector = cv2.SimpleBlobDetector(params)
	else : 
	    detector = cv2.SimpleBlobDetector_create(params)

def get_disparity(imL, imR):
	import numpy as np
	import cv2
	from matplotlib import pyplot as plt

	# imgL = cv2.imread('Yeuna9x.png',0)
	# imgR = cv2.imread('SuXT483.png',0)

	stereo = cv2.StereoBM(1, 16, 15)
	disparity = stereo.compute(imgL, imgR)

	plt.imshow(disparity,'gray')
	plt.show()
	return disparity

if __name__ = "__main__":


