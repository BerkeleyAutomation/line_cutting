"""
This class implements some basic utilities for edge detection 
from endoscope images using opencv
"""

import cv2
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn import datasets, linear_model
from sklearn.linear_model import Ridge
import pickle

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

	return (ilaplacian > confidence)

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
def get_secant_line2D(img, xs, ys, n=100, plot=False, flag=True):
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
	if flag == True:
		regr.fit(data_matrix[:,[1]], data_matrix[:,[0]])
	else:
		regr.fit(data_matrix[:,[0]], data_matrix[:,[1]])

	#if plotting is on
	if plot:
		plt_img = np.copy(img)
		for x in range(0, np.shape(img)[1]):
			y = int(round(np.squeeze(regr.predict([x]))))
			
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

"""

"""
def get_line(outs, k=2, step=50):
	model = Ridge(alpha=10.0, fit_intercept=True, normalize=False, copy_X=True, max_iter=None, tol=0.001)
	model.fit(outs[:,[1]], outs[:,0])
	ymax = np.max(outs[:,1])
	ymin = np.min(outs[:,1])
	points = np.linspace(ymin,ymax,step)

	output = np.zeros((step, 3))
	for i,v in enumerate(points):
		output[i,:] = np.array([np.squeeze(model.predict([v])),v, outs[0,2]])

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	
	ax.scatter(outs[:,0], outs[:,1], outs[:,2], c='b', marker='x')

	ax.scatter(output[:,0], output[:,1], output[:,2], c='r')
	plt.show()

	return output

"""
Given masked images aligns two point clouds
"""
def getStereoPairPointCloud(imgL, imgR, calibration_data='../calibration_data/camera_left2'):
	stereo = cv2.StereoBM(cv2.STEREO_BM_BASIC_PRESET,ndisparities=16)
	disparity = stereo.compute(imgL, imgR)
	
	cF = open(calibration_data, 'rb')
	dL = pickle.load(cF)

	fx = dL['K'][0]
	fy = dL['K'][4]
	cx = dL['K'][2]
	cy = dL['K'][5]
	Tx = dL['P'][3]
	Ty = dL['P'][7]
	T = -Tx/fx

	Q = np.zeros((4,4))
	Q[0,0] = 1
	Q[0,3] = -cx
	Q[1,3] = -cy
	Q[1,1] = 1
	Q[2,3] = np.sqrt(fx**2 + fy**2)
	#Q[3,3] = 1
	Q[3,2] = -1/T

	dispindices = np.where(disparity > 0)
	N = len(dispindices[0])
	sparseline = np.zeros((N,3))

	for i in range(0,N):
		sparseline[i,:] = np.array((dispindices[0][i], dispindices[1][i], disparity[dispindices[0][i], dispindices[1][i]]))

	sparseline[:,2] = np.mean(sparseline[:,2])

	print sparseline[:,2]
	print np.mean(sparseline[:,2])

	plt.figure()
	plt.imshow(disparity,cmap = 'gray')
	plt.show()

	out = cv2.perspectiveTransform(sparseline[None, :, :], Q)
	return -np.squeeze(out)

if __name__ == "__main__":


	#right circle
	img1 = cv2.imread('right1.jpg',0)

	fig = plt.figure()
	plt.imshow(img1,cmap = 'gray')
	
	def onclick(event):
		print('button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
          (event.button, event.x, event.y, event.xdata, event.ydata))

	cid = fig.canvas.mpl_connect('button_press_event', onclick)

	plt.show()



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

	

