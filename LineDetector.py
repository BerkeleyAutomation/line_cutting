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

"""
This class detects a black line in a given image
"""
class LineDetector(object):

    def __init__(self, 
                 image, 
                 show_plots = False,
                 painted=False):

        self.image = image
        self.show_plots = show_plots
        self.info = {}
        self.transform = self.load_transform_matrix()

        if painted:
            self.__linedetector_fn = self.line_detector_painted
        else:
            self.__linedetector_fn = self.line_detector_drawn

        rospy.Subscriber("/endoscope/left/image_rect_color", Image,
                         self.left_image_callback, queue_size=1)
        rospy.Subscriber("/endoscope/right/image_rect_color", Image,
                         self.right_image_callback, queue_size=1)
        # info subscribers
        rospy.Subscriber("/endoscope/left/camera_info",
                         CameraInfo, self.left_info_callback)
        rospy.Subscriber("/endoscope/right/camera_info",
                         CameraInfo, self.right_info_callback)


    def left_info_callback(self, msg):
        if self.info['l']:
            return
        self.info['l'] = msg

    def right_info_callback(self, msg):
        if self.info['r']:
            return
        self.info['r'] = msg

    def right_image_callback(self, msg):
        if rospy.is_shutdown():
            return
        self.right_timestamp = msg.header.stamp
        self.right_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

    def left_image_callback(self, msg):
        if rospy.is_shutdown():
            return
        self.left_timestamp = msg.header.stamp
        self.left_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        self.image = self.left_image

### TO DO: right image/left image preprocessing to increase robustness

    def line_detector_drawn(self, image, show_plots = False):
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
        # if self.show_plots:
        #     cv2.imshow("Thresh", thresh)
        #     cv2.waitKey(0)

        return self.remove_blobs(image, resized, thresh, ratio, show_plots), ratio




    def line_detector_painted(self, image, show_plots = False):
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

        if self.show_plots:
            cv2.imshow("Thresh", thresh)
            cv2.waitKey(0)

        return self.remove_blobs(image, resized, gray, ratio, show_plots), ratio
    
    def remove_blobs(self, full_image, resized_image, gray, ratio, show_plots=False):
        if show_plots:
            cv2.imshow("Thresh2", gray)
            cv2.waitKey(0)

        cnts = cv2.findContours(gray.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]

        # loop over the contours
        
        hsv = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)
        # minv = 1000
        # for c in cnts:
        #     mask = np.zeros(gray.shape,np.uint8)
        #     cv2.drawContours(mask,[c],0,255,-1)
        #     mean_val = np.array(cv2.mean(hsv,mask = mask))
        #     minv = min(mean_val[2], minv)
        # print minv

        for c in cnts:
            mask = np.zeros(gray.shape,np.uint8)
            cv2.drawContours(mask,[c],0,255,-1)
            mean_val = np.array(cv2.mean(hsv,mask = mask))
            if np.max(mean_val) < 100 and cv2.contourArea(c) > 5000:
                continue
            else:
                print cv2.contourArea(c)
                if cv2.contourArea(c) < 5000:
                    cv2.drawContours(gray, [c], -1, (0, 0, 0), -1)
                # else:
                    # pass
        if show_plots:
            cv2.imshow("a", gray)
            cv2.waitKey(0)
        return gray


    def detect_relative_position(self, cur_position, next_position, image, ratio, rect_width=400, rect_height=30, show_plots=False):
        """
        Takes in current and next robot position in pixel space and a full image and returns whether or not the robot is to the right or left.
        """
        cur_position, next_position = np.array(cur_position), np.array(next_position)
        delta = next_position - cur_position

        factor = 100 / np.linalg.norm(np.array(delta))
        next_position = cur_position + factor * np.array(delta)

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


        newshape = (ymax - ymin, xmax - xmin)

        new_image = np.zeros(newshape)
        new_image[xstart:xend,ystart:yend] = image
        new_image = new_image.astype(np.uint8)

        newpts = []
        for i in range(pts.shape[0]):
            pt = np.floor(pts[i,:] + np.array((ystart, xstart))).astype(int)
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
            return float('inf') # Cannot find line
        line_vector = (center_of_mass - np.array(cur_position)).astype(float)
        planned_vector = np.array(next_position) - np.array(cur_position)

        cpdt = np.cross(line_vector, planned_vector)
        print line_vector, np.linalg.norm(line_vector), np.linalg.norm(planned_vector)
        line_vector /= np.linalg.norm(line_vector)
        planned_vector /= np.linalg.norm(planned_vector)

        dpdt = np.dot(line_vector, planned_vector)
        theta = np.arccos(dpdt) * 180 / np.pi
        print "angle", theta, dpdt, line_vector, planned_vector
        print "position", cur_position, next_position

        if theta < 20:
            return 0

        if cpdt < 0:
            return 1 # line is to the right
        else:
            return -1 # line is to the left

    def load_transform_matrix(self, f0="camera_matrix.p"):
        f = open(f0)
        info = pickle.load(f)
        f.close()
        return info

    def get_pixel_from3D(self, position, transform=self.transform, camera_info=self.info, offset=(0,0)):
        if not (self.info and self.transform):
            return

        Trobot = np.zeros((4,4))
        Trobot[:3,:] = np.copy(transform)
        Trobot[3,3] = 1
        Rrobot = np.linalg.inv(Trobot)

        x = np.ones((4,1))

        x[:3,0] = np.squeeze(position)

        cam_frame = np.dot(Rrobot, x)

        Pcam = np.array(camera_info.P).reshape(3,4)

        V = np.dot(Pcam, cam_frame)

        V = np.array((int(V[0]/V[2]), int(V[1]/V[2])))

        V[0] = V[0] + offset[0]
        V[1] = V[1] + offset[1]

        return V

    """
    Queries two points to find the left/right
    0 left
    1 right
    """
    def query(self, cur_position, next_position):
        left_gray, ratio = self.__linedetector_fn(self.image, self.show_plots)
        rel = self.detect_relative_position(cur_position, next_position, left_gray, ratio, show_plots=self.show_plots)
        return rel



"""
if __name__ == "__main__":

    left_image = cv2.imread("left6.jpg")
    c = LineDetector(left_image, show_plots=True)

    cur_position = (1400, 200)
    next_position = (1200, 500)

    print c.query(cur_position, next_position)
"""