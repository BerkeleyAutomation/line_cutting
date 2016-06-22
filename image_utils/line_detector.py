# import the necessary packages
# from pyimagesearch.shapedetector import ShapeDetector
import argparse
import imutils
import cv2
import numpy as np
import matplotlib.pyplot as plt
import IPython
import pickle
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
import sys
import time
import image_geometry
import scipy.interpolate
from sklearn.neighbors import KNeighborsClassifier
import scipy.ndimage

### TO DO: right image/left image preprocessing to increase robustness

def line_detector_drawn(image, show_plots = False):
    # resize it to a smaller factor so that
    # the shapes can be approximated better
    resized = imutils.resize(image, width=int(np.ceil(image.shape[1]/2)))
    ratio = image.shape[0] / float(resized.shape[0])

    # convert the resized image to grayscale, blur it slightly,
    # and threshold it
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (45, 45), 0)

    # thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, 19, 2)
    kernel1 = np.ones((7, 7), np.uint8)
    thresh = 255 - thresh

    thresh = cv2.dilate(thresh, kernel1, iterations=1)
    kernel2 = np.ones((7, 7), np.uint8)
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel2)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel2)
    thresh = closing
    # if show_plots:
    #     cv2.imshow("Thresh", thresh)
    #     cv2.waitKey(0)

    return remove_blobs(image, resized, thresh, ratio, show_plots), ratio



def line_detector_painted(image, show_plots = False):
    # resize it to a smaller factor so that
    # the shapes can be approximated better
    resized = imutils.resize(image, width=int(np.ceil(image.shape[1]/2)))
    ratio = image.shape[0] / float(resized.shape[0])

    # convert the resized image to grayscale, blur it slightly,
    # and threshold it
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (35, 35), 0)
    # thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, 15, 2)

    kernel = np.ones((5, 5), np.uint8)
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
    thresh = 255 - closing

    if show_plots:
        cv2.imshow("Thresh", thresh)
        cv2.waitKey(0)
    return remove_blobs(image, resized, thresh, ratio, show_plots), ratio

def remove_blobs(full_image, resized_image, gray, ratio, show_plots=False):
    if show_plots:
        cv2.imshow("Thresh", gray)
        cv2.waitKey(0)

    cnts = cv2.findContours(gray.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]

    # loop over the contours

    hsv = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)
    for c in cnts:
        mask = np.zeros(gray.shape,np.uint8)
        cv2.drawContours(mask,[c],0,255,-1)
        mean_val = np.array(cv2.mean(hsv,mask = mask))
        if np.max(mean_val) < 100:
           continue
        else:
            cv2.drawContours(gray, [c], -1, (0, 0, 0), -1)
    if show_plots:
        cv2.imshow("a", gray)
        cv2.waitKey(0)
    return gray

def detect_relative_position(cur_position, next_position, image, ratio, rect_width=400, rect_height=50, show_plots=False):
    """
    Takes in current and next robot position in pixel space and a full image and returns whether or not the robot is to the right or left.
    """
    cur_position, next_position = np.array(cur_position), np.array(next_position)
    delta = next_position - cur_position
    slope = np.array((delta[0], delta[1]))
    if np.linalg.norm(slope) != 0:
        slope =  slope / np.linalg.norm(slope)
    perp_slope = np.array((-delta[1], delta[0]))
    if np.linalg.norm(perp_slope) != 0:
        perp_slope = perp_slope / np.linalg.norm(perp_slope)
    pts = []
    # points defining the corners of a rectangle
    pts.append(next_position + perp_slope * rect_width / 2)
    pts.append(pts[-1] + slope * rect_height/2)
    pts.append(pts[-1] - slope * rect_height)
    pts.pop(-3)
    pts.append(next_position - perp_slope * rect_width / 2)
    pts.append(pts[-1] + slope * rect_height/2)
    pts.append(pts[-1] - slope * rect_height)
    pts.pop(-3)
    pts = np.array([pt.tolist() for pt in pts]) / ratio

    xmax = int(np.ceil(np.max(pts[:,0].tolist() + [image.shape[0]])))
    xmin = int(np.floor(np.min(pts[:,0].tolist() + [0])))

    ymax = int(np.ceil(np.max(pts[:,1].tolist() + [image.shape[1]])))
    ymin = int(np.floor(np.min(pts[:,1].tolist() + [0])))



    xstart = -xmin
    xend = xstart + image.shape[0]

    ystart = -ymin
    yend = ystart + image.shape[1]

    newshape = (xmax - xmin, ymax - ymin)

    new_image = np.zeros(newshape)
    new_image[xstart:xend,ystart:yend] = image
    new_image = new_image.astype(np.uint8)

    newpts = []
    for i in range(pts.shape[0]):
        pt = np.floor(pts[i,:] + np.array((xstart, ystart))).astype(int)
        newpts.append(pt.tolist())
    newpts[2], newpts[3] = newpts[3], newpts[2]

    cv2.drawContours(new_image, [np.array(newpts)], -1, (0,255,0), 3)


    mask = np.zeros(np.array(new_image.shape)[:2],np.uint8)
    cv2.drawContours(mask,[np.array(newpts)],0,255,-1)

    new_image *= mask
    if show_plots:
        plt.imshow(image)
        plt.show()
        plt.imshow(new_image, cmap='Greys_r')
        plt.show()
        plt.imshow(mask, cmap='Greys_r')
        plt.show()
        plt.imshow(new_image[xstart:xend,ystart:yend], cmap='Greys_r')
        plt.show()


    cnts = cv2.findContours(new_image[xstart:xend,ystart:yend], cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]

    dist = float('inf')
    best_center = None
    for c in cnts:
        M = cv2.moments(c)
        cX = int((M["m10"] / M["m00"]) * ratio) if M["m00"] != 0 else 0
        cY = int((M["m01"] / M["m00"]) * ratio) if M["m00"] != 0 else 0
        center = np.array((cX, cY))
        if np.linalg.norm(center - next_position) < dist:
            dist = np.linalg.norm(center - next_position)
            best_center = center
            
    center_of_mass = best_center
    if center_of_mass == None:
        return -1
    line_vector = center_of_mass - np.array(cur_position)
    planned_vector = np.array(next_position) - np.array(cur_position)

    cpdt = np.cross(line_vector, planned_vector)
    if cpdt < 0:
        return 1 # line is to the right
    else:
        return 0 # line is to the left



if __name__ == "__main__":

    SHOW_PLOTS = True

    left_image = cv2.imread("left12.jpg")

    cur_position = (1000, 300)
    next_position = (990, 290)

    left_gray, ratio = line_detector_drawn(left_image, SHOW_PLOTS)
    rel = detect_relative_position(cur_position, next_position, left_gray, ratio, show_plots=True)
    print rel
