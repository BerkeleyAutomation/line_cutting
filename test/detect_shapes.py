# import the necessary packages
# from pyimagesearch.shapedetector import ShapeDetector
import argparse
import imutils
import cv2
import numpy as np

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
	help="path to the input image")
args = vars(ap.parse_args())

# load the image and 
image = cv2.imread(args["image"])

# resize it to a smaller factor so that
# the shapes can be approximated better
resized = imutils.resize(image, width=300)
ratio = image.shape[0] / float(resized.shape[0])

# convert the resized image to grayscale, blur it slightly,
# and threshold it
gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

# find contours in the thresholded image 
cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
cnts = cnts[0] if imutils.is_cv2() else cnts[1]


# initialize the shape detector
# sd = ShapeDetector()

# loop over the contours
for c in cnts:
	# compute the center of the contour, then detect the name of the
	# shape using only the contour
	M = cv2.moments(c)
	cX = int((M["m10"] / M["m00"]) * ratio) if M["m00"] != 0 else 0
	cY = int((M["m01"] / M["m00"]) * ratio) if M["m00"] != 0 else 0
	# shape = sd.detect(c)

	# #find mean color
	# mask = np.zeros(gray.shape,np.uint8)
	# mean_val = cv2.mean(gray,mask = mask)

	# multiply the contour (x, y)-coordinates by the resize ratio,
	# then draw the contours and the name of the shape on the image
	# c = c.astype("float")
	# c *= ratio
	# c = c.astype("int")
	# cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
	# cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
	# 	0.5, (255, 255, 255), 2)

	# # show the output image
	# cv2.imshow("Image", image)
	# cv2.waitKey(0)
	print (cX, cY)
	