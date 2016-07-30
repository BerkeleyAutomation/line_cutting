import rospy, time, pickle
import numpy as np
import matplotlib.pyplot as plt
from robot import *
import tfx
import sys

def get_frame(pos, rot=[0.77133703427, 0.622022979745, 0.133725384273, 0.0156241426796]):
    """
    Gets a TFX pose from an input position/rotation for PSM1.
    """
    return tfx.pose(pos, rot)

def psm1_translation(translation, psm1, rotation=None):
    """
    Translates PSM1 by (x, y, z)
    """
    pos = psm1.get_current_cartesian_position().position
    pos[0] += translation[0]
    pos[1] += translation[1]
    pos[2] += translation[2]
    if rotation == None:
        psm1.move_cartesian_frame(get_frame(pos))
        time.sleep(2)
    else:
        psm1.move_cartesian_frame(get_frame(pos, rotation))
        time.sleep(2)

def cut_notch(position, psm1, angle=0.0):
    # def home_robot(pos):
    #     psm1.move_cartesian_frame(get_frame(pos))
    #     time.sleep(1)
    # psm1.close_gripper()
    # psm1_translation((0,0,0.01))
    # time.sleep(3)
    # for i in range(4):
    #     psm1_translation((0,0,-0.025))
    #     psm1.open_gripper(80.0)
    #     time.sleep(3)
    #     psm1.close_gripper()
    #     time.sleep(3)
    #     psm1_translation((0,0,0.025))
    # psm1_translation((-0.01, 0, 0))
    # psm1.move_cartesian_frame(get_frame(position))
    psm1.move_cartesian_frame(get_frame(position))
    time.sleep(2)
    psm1.open_gripper(80)
    time.sleep(3)
    rotation = [94.299363207+angle, -4.72728031036, 86.1958002688]
    rot = tfx.tb_angles(rotation[0], rotation[1], rotation[2])
    psm1_translation((0.006,0,-0.009), psm1, rot)
    time.sleep(2)

if __name__ == "__main__":
    psm1 = robot("PSM1")
    pos = psm1.get_current_cartesian_position().position
    cut_notch(pos, psm1)
