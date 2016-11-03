import rospy, pickle, time
from robot import *
from geometry_msgs.msg import Pose
import numpy as np

import multiprocessing
import tfx
import fitplane
from scipy.interpolate import interp1d
from shape_tracer import plot_points
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
import notch
from geometry_msgs.msg import Point
from line_cut_trajectory_notch import get_frame_psm1 as get_frame

class Experiment:
	def __init__(self,robot):
		self.u=None
		self.data=[]
		self.robot=robot
		self.initial_position=self.robot.get_current_cartesian_position()
		self.xi=[]
		self.xf=[]
	

	def translation(self,translation):
	    """
	    Translates PSM1 by (x, y, z)
	    """

	    pos = self.initial_position.position
	    pos[0] += translation[0]
	    pos[1] += translation[1]
	    pos[2] += translation[2]
	    self.robot.move_cartesian_frame(get_frame(pos,self.initial_position.orientation))

	def move(self,u):
		self.u=u
		self.translation(u)
		
	def track(self):
		#xi=get position of every point
		self.move(np.random.randn(3))
		time.sleep(5)
		#xf=get position of every point after a move
		self.data.append((xf,u,xi))
	def reset(self):
		self.robot.move_cartesian_frame(self.initial_position)
def grab_gauze(ex):
	
    pose= ex.initial_position.position
    rot=ex.initial_position.orientation
    pose[2] -= 0.02
    psm2.open_gripper(80.0)
    print "opening"
    print pose
    tfx_pose = get_frame(pose,rot)
    psm2.move_cartesian_frame(tfx_pose,)
    time.sleep(1)
    
    time.sleep(3)
    psm2.open_gripper(2.0)
    print "closing"
    time.sleep(4)
    
    
    
    print "closing"
if __name__ == '__main__':
	psm2=robot("PSM2")
	ex=Experiment(psm2)
	grab_gauze(ex)
	
	# for i in range(10):
	# 	ex.track()
	# 	time.sleep(3)
	# 	ex.reset()


