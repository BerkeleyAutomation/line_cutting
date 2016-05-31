import rospy, pickle, time
from robot import *
from geometry_msgs.msg import Pose
import numpy as np
import PyKDL
import multiprocessing
import tfx
import fitplane
from scipy.interpolate import interp1d
from shape_tracer import plot_points

"""
This file contains utilities that are used for a trajectory following curve cutting model.
"""


def home_robot():
    pos = [0.023580864372, 0.00699340564912, -0.0485527311586]
    psm1.move_cartesian_frame(get_frame_psm1(pos))
    time.sleep(1)

def initialize(pts):
    """
    Initialize both arms to a fixed starting position/rotation.
    """
    home_robot()
    # start_pos = pts[0]
    # start_pos[0,0] -= 0.015
    # start_pos[0,1] += 0.01
    # start_rot = [0.617571885272, 0.59489495214, 0.472153066551, 0.204392867261]
    # start_frame1 = get_frame_psm1(start_pos, start_rot)
    # psm1.move_cartesian_frame(start_frame1)
    # psm1.open_gripper(80)
    # psm1_position = start_pos
    time.sleep(2)
    return

def get_frame_psm1(pos, rot=[0.617571885272, 0.59489495214, 0.472153066551, 0.204392867261]):
    """
    Gets a TFX pose from an input position/rotation for PSM1.
    """
    return tfx.pose(pos, rot)

def cut(closed_angle=2.0, open_angle=80.0, close_time=2.5, open_time=2.35):
    """
    Closes and opens PSM1's grippers.
    """
    psm1.open_gripper(closed_angle)
    time.sleep(close_time)
    psm1.open_gripper(open_angle)
    time.sleep(open_time)

def psm1_translation(translation):
    """
    Translates PSM1 by (x, y, z)
    """
    pos = psm1_position
    pos[0] += translation[0]
    pos[1] += translation[1]
    pos[2] += translation[2]
    psm1.move_cartesian_frame(get_frame_psm1(pos))

def load_robot_points(fname="calibration_data/gauze_pts.p"):
    lst = []
    f3 = open(fname, "rb")
    while True:
        try:
            pos2 = pickle.load(f3)
            lst.append(pos2)
        except EOFError:
            f3.close()
            return np.matrix(lst)

def interpolation(arr, factor):
    """
    Given a matrix of x,y,z coordinates, output a linearly interpolated matrix of coordinates with factor * arr.shape[1] points.
    """
    x = arr[:, 0]
    y = arr[:, 1]
    z = arr[:, 2]
    t = np.linspace(0,x.shape[0],num=x.shape[0])
    to_expand = [x, y, z]
    for i in range(len(to_expand)):
        print t.shape, np.ravel(to_expand[i]).shape
        spl = interp1d(t, np.ravel(to_expand[i]))
        to_expand[i] = spl(np.linspace(0,len(t), len(t)*factor))
    new_matrix = np.matrix(np.r_[0:len(t):1.0/factor])
    for i in to_expand:
        new_matrix = np.concatenate((new_matrix, np.matrix(i)), axis = 0)
    return new_matrix.T[:,1:]

def get_frame_next(pos, nextpos):
    """
    Given two x,y,z coordinates, output a TFX pose that points the grippers to roughly the next position, at pos.
    """
    angle = get_angle(pos, nextpos)
    print angle
    pos[2] -= 0.005
    rotation = [94.299363207+angle, -4.72728031036, 86.1958002688]
    rot = tfx.tb_angles(rotation[0], rotation[1], rotation[2])
    frame = tfx.pose(pos, rot)
    return frame

def get_angle(pos, nextpos):
    """
    Returns angle to nextpos in degrees
    """
    delta = nextpos - pos
    return np.arctan(delta[1]/delta[0]) * 180 / np.pi

if __name__ == '__main__':

    pts = load_robot_points()

    pts = interpolation(pts, 5)

    print pts.shape

    psm1 = robot("PSM1")

    initialize(pts)

    for i in range(pts.shape[0]-1):
        print i
        pos = pts[i,:]
        nextpos = pts[i+1,:]
        cut()
        frame = get_frame_next(np.ravel(pos), np.ravel(nextpos))
        psm1.move_cartesian_frame(frame)
    plot_points()

