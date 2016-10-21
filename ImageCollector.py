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
class ImageCollector(object):

    def __init__(self, 
                 image, 
                 show_plots = False,
                 painted=False):

        self.image = image
        self.show_plots = show_plots
        self.info = {}
        self.transform = self.load_transform_matrix()
        self.offset = (0, 0)

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


    def load_transform_matrix(self, f0="camera_matrix.p"):
        f = open(f0)
        info = pickle.load(f)
        f.close()
        return info

    def get_pixel_from3D(self, position, transform=self.transform, camera_info=self.info, offset=self.offset):
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

    def query(self, position):
        V = self.get_pixel_from3D(position)
        img_size = (300, 300)

        xmin = V[0] - img_size[0]/2
        xmax = V[0] + img_size[0]/2
        if xmin < 0 or xmax > self.image.shape[0]:
            return None
        ymin = V[1] - img_size[1]/2
        ymax = V[1] + img_size[1]/2
        if ymin < 0 or ymax > self.image.shape[1]:
            return None

        return image[xmin:xmax, ymin:ymax]

