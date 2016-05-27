import rospy, pickle, time
from robot import *
from geometry_msgs.msg import Pose
import numpy as np
import PyKDL
import multiprocessing
import tfx
import fitplane


def initialize():
    """
    Initialize both arms to a fixed starting position/rotation.
    """
    global psm1_position, psm2_position
    psm2_initial = [-0.0912484814441, 0.0794414679811, -0.0736081506167, 0.888720223413, -0.254590620512, -0.302364852004, -0.232240127275]
    psm1_initial = [0.0298111217581, 0.0255537141169, -0.111452040502, 0.617571885272, 0.59489495214, 0.472153066551, 0.204392867261]
    start_frame1 = get_frame(psm1_initial)
    start_frame2 = get_frame2(psm2_initial)
    psm1.move_cartesian_frame(start_frame1)
    # psm2.move_cartesian_frame(start_frame2)
    psm1.open_gripper(80)
    psm2.open_gripper(80)
    psm1_position = psm1_initial
    psm2_position = psm2_initial
    time.sleep(2)
    return

def calibrate():
    """
    Fit a plane to the gauze. Use calibration.py to record these points.
    """
    pts = fitplane.load_points()
    plane = fitplane.least_squares_plane_normal(pts)
    return plane

def get_frame_psm1(pos):
    """
    Gets a TFX pose from an input position/rotation for PSM1.
    """
    pt = fitplane.project_onto_plane(plane, pos[0:3])
    pt[0, 2] += 0.001
    return tfx.pose(pt.tolist(), pos[3:7])


def get_frame_psm2(pos):
    """
    Gets a TFX pose from an input position/rotation for PSM2.
    """
    return tfx.pose(pos[0:3], pos[3:7])

def cut(closed_angle=1.0, open_angle=80.0, close_time=2.5, open_time=2.35):
    """
    Closes and opens PSM1's grippers.
    """
    psm1.open_gripper(closed_angle)
    time.sleep(close_time)
    psm1.open_gripper(open_angle)
    time.sleep(open_time)

def move_to_next(delta_x = 0.0045, delta_y = 0.0010):
    """
    Translates PSM1 to the next cutting position.
    """
    pos = psm1_position
    pos[0] = pos[0] + delta_x
    pos[1] = pos[1] + delta_y
    psm1.move_cartesian_frame(get_frame_psm1(pos))

def psm1_translation(translation):
    """
    Translates PSM1 by (x, y, z)
    """
    pos = psm1_position
    pos[0] += translation[0]
    pos[1] += translation[1]
    pos[2] += translation[2]
    psm1.move_cartesian_frame(get_frame_psm1(pos))

def psm2_translation(translation):
    """
    Translates PSM2 by (x, y, z)
    """
    pos = psm2_position
    pos[0] += translation[0]
    pos[1] += translation[1]
    pos[2] += translation[2]
    psm1.move_cartesian_frame(get_frame_psm2(pos))

def grab_gauze():
    #probably needs to be tuned
    pos = psm2_position
    pos[2] = pos[2] - 0.013
    psm2.move_cartesian_frame(get_frame2(pos))
    psm2.open_gripper(-30)
    time.sleep(2.5)
    pos[2] = pos[2] + 0.01
    psm2.move_cartesian_frame(get_frame2(pos))

if __name__ == '__main__':
    plane = calibrate()

    psm1_position = None
    psm2_position = None

    psm1 = robot("PSM1")
    psm2 = robot("PSM2")

    initialize()

    # grab_gauze()

    for i in range(30):
        cut()
        move_to_next()

