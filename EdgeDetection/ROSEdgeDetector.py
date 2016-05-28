import rospy
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
import cv2
import cv_bridge
import numpy as np
import rospy
import scipy.misc
import pickle

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

from EdgeDetector2 import *

class ROSEdgeDetector:

    def __init__(self):
        self.right_image = None
        self.left_image = None
        self.rcounter = 0
        self.lcounter = 0
        self.info = {'l': None, 'r': None}
        self.bridge = cv_bridge.CvBridge()


        #========SUBSCRIBERS========#
        # image subscribers
        rospy.init_node('image_saver')
        rospy.Subscriber("/endoscope/left/image_rect_color", Image,
                         self.left_image_callback, queue_size=1)
        rospy.Subscriber("/endoscope/right/image_rect_color", Image,
                         self.right_image_callback, queue_size=1)
        # info subscribers
        rospy.Subscriber("/endoscope/left/camera_info",
                         CameraInfo, self.left_info_callback)
        rospy.Subscriber("/endoscope/right/camera_info",
                         CameraInfo, self.right_info_callback)
        rospy.spin()


    def left_info_callback(self, msg):
        if self.info['l']:
            return
        self.info['l'] = msg

    def right_info_callback(self, msg):
        if self.info['r']:
            return
        self.info['r'] = msg

    def right_image_callback(self, msg):
        # print "right"
        if rospy.is_shutdown():
            return
        self.right_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        self.rcounter += 1


    def left_image_callback(self, msg):
        # print "left"
        if rospy.is_shutdown():
            return
        self.left_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        self.lcounter += 1
        if self.right_image != None:
            self.process_image()

    def process_image(self):
        right = cv2.cvtColor(self.right_image,cv2.COLOR_BGR2GRAY)
        left = cv2.cvtColor(self.left_image, cv2.COLOR_BGR2GRAY)

        workspacer = workspace_mask([215, 1500, 300,  900], plot=False)
        right_edge = get_secant_line(workspacer(segment_edge(right, plot=False, confidence=220, gsigma=20)), 991, 394, plot=False, flag=True)

        x1 = 900
        y1 = right_edge.predict(x1)[0,0]
        p1 = (x1, y1)

        print neigh.predict(p1)

        workspacel = workspace_mask([215, 1500, 300,  900], plot=False)
        left_edge = get_secant_line(workspacel(segment_edge(left, plot=False, confidence=200, gsigma=20)), 991, 394, plot=False, flag=True)

        x2 = x1 + 70
        y2 = left_edge.predict(x2)[0,0]
        
        p2 = (x2, y2)

        print [p1], [p2]
        



if __name__ == "__main__":

    f = open("camera_chesspts.p", 'rb')
    leftpts, rightpts = pickle.load(f)
    f.close()
    leftpts = leftpts.reshape((leftpts.shape[0], leftpts.shape[2]))
    rightpts = rightpts.reshape((rightpts.shape[0], rightpts.shape[2]))
    y = np.r_[0:leftpts.shape[0]]
    from sklearn.neighbors import KNeighborsClassifier
    neigh = KNeighborsClassifier(n_neighbors=1)

    a = ROSEdgeDetector()

