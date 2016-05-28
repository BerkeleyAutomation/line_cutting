"""
This class implements a manual edge annotation
script.
"""

import cv2
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn import datasets, linear_model
from sklearn.linear_model import Ridge
import pickle
import sys


#left state variables

class LeftRightState:
	def __init__(self):
		self.lstart = False
		self.left_pixels = []
		self.rstart = False
		self.right_pixels = []

	def fitLine(self, steps=50):
		data_matrix = np.zeros((len(self.left_pixels),2))
		for i in range(0,len(self.left_pixels)):
			data_matrix[i,0] = self.left_pixels[i][0]
			data_matrix[i,1] = self.left_pixels[i][1]

		# Create linear regression object
		regrL = linear_model.LinearRegression()

		# Train the model using the data
		regrL.fit(data_matrix[:,[0]], data_matrix[:,[1]])

		pointsL = np.linspace(np.min(data_matrix[:,0]), np.max(data_matrix[:,0]), steps)

		predictedL = []
		for i in pointsL:
			predictedL.append((i,np.squeeze(regrL.predict([i]))[0]))

		data_matrix = np.zeros((len(self.right_pixels),2))
		for i in range(0,len(self.right_pixels)):
			data_matrix[i,0] = self.right_pixels[i][0]
			data_matrix[i,1] = self.right_pixels[i][1]

		# Create linear regression object
		regrR = linear_model.LinearRegression()

		# Train the model using the data
		regrR.fit(data_matrix[:,[0]], data_matrix[:,[1]])

		pointsR = np.linspace(np.min(data_matrix[:,0]), np.max(data_matrix[:,0]), steps)

		predictedR = []
		for i in pointsR:
			predictedR.append((i,np.squeeze(regrR.predict([i]))[0]))

		return (predictedL, predictedR)

def onclickLeft(event, lrstate):
	if not lrstate.lstart:
		lrstate.lstart = True
		print "Started Left", event.xdata, event.ydata
	else:
		lrstate.lstart = False
		print "Ended Left", event.xdata, event.ydata

def onmoveLeft(event, lrstate):
	if lrstate.lstart:
		lrstate.left_pixels.append((event.xdata, event.ydata))
		print "Added Left", event.xdata, event.ydata

def onclickRight(event, lrstate):
	if not lrstate.rstart:
		lrstate.rstart = True
		print "Started Right", event.xdata, event.ydata
	else:
		lrstate.rstart = False
		print "Ended Right", event.xdata, event.ydata

def onmoveRight(event, lrstate):
	if lrstate.rstart:
		lrstate.right_pixels.append((event.xdata, event.ydata))
		print "Added Right", event.xdata, event.ydata

if __name__ == "__main__":

	print "Starting Edge Detection"
	if len(sys.argv) != 3:
		print "Usage: GuidedEdgeDetector.py left.jpg right.jpg" 
		exit()

	image_file_left = sys.argv[1]
	image_file_right = sys.argv[2]

	#left circle
	img1 = cv2.imread('left1.jpg',0)

	fig = plt.figure()
	plt.imshow(img1,cmap = 'gray')
	
	state = LeftRightState()

	cid = fig.canvas.mpl_connect('button_press_event', lambda x: onclickLeft(x,state))
	cid2 = fig.canvas.mpl_connect('motion_notify_event', lambda x: onmoveLeft(x,state))

	plt.show()


	#right circle
	img1 = cv2.imread('right1.jpg',0)

	fig = plt.figure()
	plt.imshow(img1,cmap = 'gray')

	cid = fig.canvas.mpl_connect('button_press_event', lambda x: onclickRight(x,state))
	cid2 = fig.canvas.mpl_connect('motion_notify_event', lambda x: onmoveRight(x,state))

	plt.show()

	print state.fitLine(50)



	"""
	#hard-coded image workspace right
	workspacer = workspace_mask([175, 1400, 350,  700],plot=True)

	imgER = workspacer(segment_edge(img1, confidence=200, gsigma=10,plot=True))

	maskedR = np.multiply(img1, imgER).astype('uint8')


	img2 = cv2.imread('left1.jpg',0)

	workspacel = workspace_mask([175, 1400, 350,  700],plot=True)

	imgEL = workspacel(segment_edge(img2, confidence=200, gsigma=10,plot=True))

	maskedL = np.multiply(img2, imgER).astype('uint8')



	#outs = getStereoPairPointCloud(maskedL, maskedR)

	#print get_line(outs)
	"""

	

