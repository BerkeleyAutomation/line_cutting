import rospy, pickle, time
from robot import *
from geometry_msgs.msg import Pose
import numpy as np
import PyKDL
from scipy.interpolate import UnivariateSpline

def load_points():
    """
    Load points in the robot frame that were stored to file.
    """
    lst = []
    f3 = open("calibration_data/psm1_calibration_line_cutting.p", "rb")
    pos1 = pickle.load(f3)
    lst.append(pos1)
    while True:
        try:
            pos1 = pickle.load(f3)
            lst.append(pos1)
        except EOFError:
            f3.close()
            return np.matrix(lst)

def least_squares_plane_normal(points_3d):
    """
    Fit a plane to 3d points
    """
    x_list = points_3d[:,0]
    y_list = points_3d[:,1]
    z_list = points_3d[:,2]

    A = np.concatenate((x_list, y_list, np.ones((len(x_list), 1))), axis=1)
    plane = np.matrix(np.linalg.lstsq(A, z_list)[0]).T

    return plane

def project_onto_plane(plane, point):
    """
    Takes an input point, and modifies the z coordinate to lie on the plane
    """
    pt = np.array(point).reshape(3, 1)
    pt[2, 0] = 1
    new_z = plane * pt
    new_pt = np.array(point).reshape(3, 1)
    new_pt[2, 0] = new_z
    return new_pt.T


if __name__ == "__main__":
    pts = load_points()
    plane = least_squares_plane_normal(pts)
