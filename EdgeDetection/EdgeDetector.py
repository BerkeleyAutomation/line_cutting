"""
This class implements some basic utilities for edge detection 
from endoscope images using opencv
"""

import cv2
import numpy as np
from matplotlib import pyplot as plt
from sklearn import datasets, linear_model

"""
This function calculates a cleaned up laplacian of the
image. 

--Returns a mask over the full image, usually catches
the edges of the workspace

confidence - threshold on the deriv to detect an edge
mkernel - median filtering kernel
lkernel - laplace deriv aperture
gkernel/gsigma - noise cleanup
"""
def segment_edge(img, 
					  confidence=240, 
					  mkernel=31,
					  lkernel=31, 
					  gkernel=31, 
					  gsigma=10,
					  plot=False):

	#first equalize the image
	equ = cv2.medianBlur(img,mkernel)

	#then take a derivative filter
	laplacian = cv2.Laplacian(equ,cv2.CV_8U,ksize = lkernel)

	#invert
	ilaplacian = 255 - laplacian
	ilaplacian = cv2.GaussianBlur(ilaplacian, (gkernel,gkernel), gsigma)

	if plot:
		plt.figure()
		plt.subplot(121),plt.imshow(img,cmap = 'gray')
		plt.title('Original Image'), plt.xticks([]), plt.yticks([])
		plt.subplot(122),plt.imshow(255*(ilaplacian > confidence),cmap = 'gray')
		plt.title('Edge Image'),
		plt.xticks([]), plt.yticks([])
		plt.show()

	return 255*(ilaplacian > confidence)

"""
Bounding Box mask

Only keeps everything within a bounding box 
(standard range semantics on inclusion/exclusion)

args are normal, y is x/ x is y in code
"""
def bounding_box(img, xmin, xmax, ymin, ymax, plot=False):
	size = np.shape(img)
	new_image = np.copy(img)
	for x in range(0, size[0]):
		for y in range(0, size[1]):
			if y < xmin or y >= xmax or x < ymin or x >= ymax:
				new_image[x,y] = 0

	if plot:
		plt.figure()
		plt.imshow(new_image,cmap = 'gray')
		plt.title('Masked workspace')
		plt.xticks([])
		plt.yticks([])
		plt.show()

	return new_image


"""
This is a lambda that you can use 
to set the workspace calls bounding_box
[xmin, xmax, ymin, ymax]
"""
def workspace_mask(workspace, plot=False):
	return lambda img: bounding_box(img, 
									workspace[0], 
									workspace[1], 
									workspace[2], 
									workspace[3],
									plot)


"""
Given a point x, y this gets the secant line (as an sklearn model)

args are normal, y is x/ x is y in code
"""
def get_secant_line(img, xs, ys, n=100, plot=False):
	points = []
	for x in range( max(xs-n,0), xs+n):
		for y in range( max(ys-n,0), ys+n):

			try:
				if img[y,x] > 0:
					points.append((y,x))
			except IndexError:
				pass

	if len(points) == 0:
		raise ValueError("There are no detected edges in the neighborhood")
	
	#stupid data type conversion could/should optimize
	data_matrix = np.zeros((len(points),2))
	for i,v in enumerate(points):
		data_matrix[i,0] = v[0] 
		data_matrix[i,1] = v[1] 

	# Create linear regression object
	regr = linear_model.LinearRegression()

	# Train the model using the data
	regr.fit(data_matrix[:,[0]], data_matrix[:,[1]])

	#if plotting is on
	if plot:
		plt_img = np.copy(img)
		for y in range(0, np.shape(img)[0]):
			x = int(round(np.squeeze(regr.predict([y]))))
			
			try:
				plt_img[max(y,0),max(x,0)] = 122
			except IndexError:
				pass

		plt.figure()
		plt.imshow(plt_img,cmap = 'gray')
		plt.title('Detected Line')
		plt.xticks([])
		plt.yticks([])
		plt.show()

	return regr

if __name__ == "__main__":

	#right circle
	img = cv2.imread('right_circle2.jpg',0)

	#hard-coded image workspace right
	workspacer = workspace_mask([450, 1340, 350, 1000], plot=True)

	get_secant_line(workspacer(segment_edge(img, plot=True)), 991, 394, plot=True)


	#left circle
	img = cv2.imread('left_circle2.jpg',0)

	workspacel = workspace_mask([450, 1340, 350, 1000], plot=True)

	get_secant_line(workspacel(segment_edge(img, plot=True)), 1280, 633, plot=True)

	

	#right line
	img = cv2.imread('right_line3.jpg',0)

	workspacel = workspace_mask([100, 1400, 300, 700], plot=True)

	get_secant_line(workspacel(segment_edge(img, plot=True)), 710, 496, n=200, plot=True)


	#left line
	img = cv2.imread('left_line3.jpg',0)

	workspacel = workspace_mask([100, 1400, 300, 700], plot=True)

	get_secant_line(workspacel(segment_edge(img, plot=True)), 454, 470, n=200, plot=True)


