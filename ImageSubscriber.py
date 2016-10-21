import numpy as np
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

class ImageSubscriber:

    def __init__(self, AD=False):
        self.right_image = None
        self.left_image = None
        self.right_timestamp = None
        self.left_timestamp = None
        self.info = {'l': None, 'r': None}
        self.bridge = cv_bridge.CvBridge()


        #========SUBSCRIBERS========#
        # image subscribers
        # rospy.init_node('image_saver', anonymous=True)
        if not AD:
            rospy.Subscriber("/endoscope/left/image_rect_color", Image,
                             self.left_image_callback, queue_size=1)
            rospy.Subscriber("/endoscope/right/image_rect_color", Image,
                             self.right_image_callback, queue_size=1)
            # info subscribers
            rospy.Subscriber("/endoscope/left/camera_info",
                             CameraInfo, self.left_info_callback)
            rospy.Subscriber("/endoscope/right/camera_info",
                             CameraInfo, self.right_info_callback)
        else:
            rospy.Subscriber("/AD/left/image_raw", Image,
                             self.left_image_callback, queue_size=1)
            rospy.Subscriber("/AD/right/image_raw", Image,
                             self.right_image_callback, queue_size=1)
            # info subscribers
            rospy.Subscriber("/AD/left/camera_info",
                             CameraInfo, self.left_info_callback)
            rospy.Subscriber("/AD/right/camera_info",
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

    def dump_camera_info(self, fname):
        f = open(fname, "w+")
        pickle.dump(self.info, f)
        f.close()

if __name__ == "__main__":
    a = ImageSubscriber()
    print "created image subscriber"
