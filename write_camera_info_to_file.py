import rospy, pickle, time
from robot import *
from geometry_msgs.msg import Pose
import numpy as np
import PyKDL
import multiprocessing
import tfx
import image_geometry
from geometry_msgs.msg import PointStamped, Point

"""
Utility file for writing camera intrinsics to file.
"""


def load_camera_info():
    info = {}
    f = open("calibration_data/camera_right.p")
    info['r'] = pickle.load(f)
    f.close()
    f = open("calibration_data/camera_left.p")
    info['l'] = pickle.load(f)
    f.close()
    return info

info = load_camera_info()

l = {}
r = {}

info2 = {}

l['height'] = info['l'].height
l['width'] = info['l'].width
l['distortion_model'] = info['l'].distortion_model
l['D'] = info['l'].D
l['K'] = info['l'].K
l['R'] = info['l'].R
l['P'] = info['l'].P
l['binning_x'] = info['l'].binning_x
l['binning_y'] = info['l'].binning_y

roi = (('x_offset', info['l'].roi.x_offset), ('y_offset', info['l'].roi.y_offset), ('height', info['l'].roi.height), ('width', info['l'].roi.width))

l['roi'] = roi

r['height'] = info['r'].height
r['width'] = info['r'].width
r['distortion_model'] = info['r'].distortion_model
r['D'] = info['r'].D
r['K'] = info['r'].K
r['R'] = info['r'].R
r['P'] = info['r'].P
r['binning_x'] = info['r'].binning_x
r['binning_y'] = info['r'].binning_y

roi = (('x_offset', info['r'].roi.x_offset), ('y_offset', info['r'].roi.y_offset), ('height', info['r'].roi.height), ('width', info['r'].roi.width))

r['roi'] = roi


f = open("calibration_data/camera_right2.p", 'w')
pickle.dump(l, f)
f.close()

f = open("calibration_data/camera_left2", 'w')
pickle.dump(r, f)
f.close()
