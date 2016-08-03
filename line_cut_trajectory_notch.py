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
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
import notch
from geometry_msgs.msg import Point

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
    # home_psm2()
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
def reentry_frame():
    pts=load_robot_points()
    rotation=[0.46428,0.45636,0.51234,0.56008]
    return get_frame_psm1(pts[0],rot=rotation)
def last_pt():
    pts=load_robot_points()
    return pts[-1]
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

def get_frame_next(pos, nextpos, offset=0.003, angle=None):
    """
    Given two x,y,z coordinates, output a TFX pose that points the grippers to roughly the next position, at pos.
    """
    if angle:
        angle = angle
    else:
        angle = get_angle(pos, nextpos)
    print angle
    pos[2] -= offset
    # pos[0] += offset/3.0
    rotation = [94.299363207+angle, -4.72728031036, 86.1958002688]
    rot = tfx.tb_angles(rotation[0], rotation[1], rotation[2])
    frame = tfx.pose(pos, rot)
    return frame

def get_angle(pos, nextpos):
    """
    Returns angle to nextpos in degrees
    """
    delta = nextpos - pos
    theta = np.arctan(delta[1]/delta[0]) * 180 / np.pi
    if delta[0] < 0:
        return theta + 180
    return theta

def grab_gauze():
    """
    Fixed motion for grabbing gauze with PSM2.
    """
    f = open("calibration_data/gauze_grab_pt.p")
    pose = pickle.load(f)
    print pose
    tfx_pose = get_frame_psm1(pose)
    psm2.move_cartesian_frame(tfx_pose)
    print "opening"
    psm2.open_gripper(80)
    time.sleep(2)
    pose[2] -= 0.01
    
    tfx_pose = get_frame_psm1(pose)
    psm2.move_cartesian_frame(tfx_pose)
    print pose
    psm2.open_gripper(-20)
    print "closing"
    time.sleep(2)
    # pose[2] += 0.005
    # print pose
    # tfx_pose = get_frame_psm1(pose)
    # psm2.move_cartesian_frame(tfx_pose)
    # print psm2.get_current_cartesian_position()


def home_psm2():
    psm2.open_gripper(50)
    pos = [-0.0800820928439, 0.0470152232648, -0.063244568979]
    rot = [0.127591711166, 0.986924435718, 0.0258944271904, -0.0950262703941]
    pose = get_frame_psm1(pos, rot)
    psm2.move_cartesian_frame(pose)
    time.sleep(2)

def calculate_xy_error(desired_pos):
    actual_pos = np.ravel(np.array(psm1.get_current_cartesian_position().position))[:2]
    return np.linalg.norm(actual_pos - desired_pos)

def exit():
    psm1.close_gripper()
    time.sleep(2)
    psm1.open_gripper(15)
    time.sleep(2)
    notch.psm1_translation((0, 0.0, 0.02), psm1, psm1.get_current_cartesian_position().orientation)
    home_robot()


if __name__ == '__main__':

    if len(sys.argv) >= 1 and sys.argv[1] == "noisy":
        print "adding gaussian noise"
        noisy = True
    else:
        noisy = False

    nextpospublisher = rospy.Publisher("/cutting/next_position_cartesian", Pose)

    pts = load_robot_points()

    factor = 4

    pts = interpolation(pts, factor)



    print pts.shape

    psm1 = robot("PSM1")
    psm2 = robot("PSM2")

    # initialize(pts)
    # # grab_gauze()
    # angles = []
    # for i in range(pts.shape[0]-1):
    #     pos = pts[i,:]
    #     nextpos = pts[i+1,:]
    #     angle = get_angle(np.ravel(pos), np.ravel(nextpos))
    #     angles.append(angle)

    # for i in range(len(angles)-2):
    #     angles[i] = 0.5 * angles[i] + 0.35 * angles[i+1] + 0.15 * angles[i+2]
    # angles = savgol_filter(angles, factor * (pts.shape[0]/12) + 1, 2)

    # frame = get_frame_next(np.ravel(pts[0,:]), np.ravel(pts[1,:]), offset=0.004, angle = angles[0])
    # psm1.move_cartesian_frame(frame)
    # pt = pts[0,:]
    # notch.cut_notch(pt, psm1)
    # time.sleep(3)

    # if noisy:
    #     pts[:,:2] += np.random.randn(pts.shape[0], 2) * 0.001

    # for i in range(pts.shape[0]-1):
    #     print i
    #     if i != 0:
    #         cut()
    #     pos = pts[i,:]
    #     nextpos = pts[i+1,:]
    #     frame = get_frame_next(np.ravel(pos), np.ravel(nextpos), offset=0.004, angle = angles[i])
    #     nextpos = np.ravel(nextpos)
    #     nextpospublisher.publish(Pose(Point(nextpos[0], nextpos[1], nextpos[2]), frame.orientation))

    #     psm1.move_cartesian_frame(frame)

    #     curpt = np.ravel(np.array(psm1.get_current_cartesian_position().position))
    #     pts[i,:] = curpt
    #     pts[i+1,:2] = savgol_filter(pts[:,:2], 5, 2, axis=0)[i+1,:] #probably going to make a small change to this tomorrow

    # exit()

    pts = load_robot_points(fname="calibration_data/gauze_pts2.p")
    if(last_pt()[0,1]>pts[-1,1]):

        pts[-1]=last_pt()
    factor = 4

    pts = interpolation(pts, factor)



    print pts.shape

    psm1 = robot("PSM1")
    psm2 = robot("PSM2")

    initialize(pts)

    angles = []
    for i in range(pts.shape[0]-1):
        pos = pts[i,:]
        nextpos = pts[i+1,:]
        angle = get_angle(np.ravel(pos), np.ravel(nextpos))
        angles.append(angle)

    for i in range(len(angles)-2):
        angles[i] = 0.5 * angles[i] + 0.35 * angles[i+1] + 0.15 * angles[i+2]
    angles = savgol_filter(angles, factor * (pts.shape[0]/12) + 1, 2)

    # frame = get_frame_next(np.ravel(pts[0,:]), np.ravel(pts[1,:]), offset=0.004, angle = angles[0])
    # psm1.move_cartesian_frame(frame)
    psm1.move_cartesian_frame(reentry_frame())
    notch.cut_notch(reentry_frame().position, psm1)
    notch.psm1_translation((0, 0, 0.004), psm1, psm1.get_current_cartesian_position().orientation)

    time.sleep(3)
    notch.psm1_translation((-.003, 0, 0.0), psm1, psm1.get_current_cartesian_position().orientation)
    if noisy:
        pts[:,:2] += np.random.randn(pts.shape[0], 2) * 0.001

    for i in range(pts.shape[0]-1):
        print i
        if i != 0:
            cut()
        pos = pts[i,:]
        nextpos = pts[i+1,:]
        frame = get_frame_next(np.ravel(pos), np.ravel(nextpos), offset=0.004, angle = angles[i])
        nextpos = np.ravel(nextpos)
        nextpospublisher.publish(Pose(Point(nextpos[0], nextpos[1], nextpos[2]), frame.orientation))
        psm1.move_cartesian_frame(frame)
        curpt = np.ravel(np.array(psm1.get_current_cartesian_position().position))
        pts[i,:] = curpt
        pts[i+1,:2] = savgol_filter(pts[:,:2], 5, 2, axis=0)[i+1,:] #probably going to make a small change to this tomorrow

    exit()