import image_geometry
import rospy
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
import cv2
import cv_bridge
import numpy as np
import rospy, scipy.misc

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
import pickle
import sys

"""
This utility function writes detected chess corners in both cameras to file. This can be used for calibration/testing purposes when performing 3D reconstruction.
"""
class ChessDetector:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.left_image = None
        self.right_image = None
        self.info = {'l': None, 'r': None, 'b': None, 'd': None}
        self.plane = None

        #========SUBSCRIBERS========#
        # image subscribers
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
        else:
            self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")


    def left_image_callback(self, msg):
        if rospy.is_shutdown():
            return
        else:
            self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.right_image != None:
            self.process_image()
    


    def process_image(self):
        print "processing image"
        left_gray = cv2.cvtColor(self.left_image,cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(self.right_image,cv2.COLOR_BGR2GRAY)
        ret, left_corners = cv2.findChessboardCorners(left_gray, (6,5), flags=1)
        ret, right_corners = cv2.findChessboardCorners(right_gray, (6,5), flags=1)
        print len(left_corners), len(right_corners)
        left, right, = [], []
        for i in range(len(left_corners)):
            left.append([left_corners[i][0][0], left_corners[i][0][1]])
            right.append([right_corners[i][0][0], right_corners[i][0][1]])
        f = open('calibration_data/camera_chesspts.p', 'w')
        pickle.dump((left_corners, right_corners), f)
        f.close()



if __name__ == "__main__":
    rospy.init_node('ChessDetector')
    a = ChessDetector()
    rospy.spin()
