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
    # time.sleep(2)
    return

def get_frame_psm1(pos, rot=[0.617571885272, 0.59489495214, 0.472153066551, 0.204392867261]):
    """
    Gets a TFX pose from an input position/rotation for PSM1.
    """
    return tfx.pose(pos, rot)

def cut(closed_angle=1.0, open_angle=80.0, close_time=2.5, open_time=2.35):
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
    tfx_pose = get_frame_psm1(pose[:3], pose[3:])
    psm2.move_cartesian_frame(tfx_pose)
    psm2.open_gripper(80)
    time.sleep(2)
    pose[2] -= 0.01
    tfx_pose = get_frame_psm1(pose[:3], pose[3:])
    psm2.move_cartesian_frame(tfx_pose)
    psm2.open_gripper(-30)
    time.sleep(2)
    pose[2] += 0.01
    tfx_pose = get_frame_psm1(pose[:3], pose[3:])
    psm2.move_cartesian_frame(tfx_pose)
    time.sleep(2)

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


if __name__ == '__main__':

    camera = True
    if not camera:
        pts = load_robot_points()
        pts = interpolation(pts, factor)
    else:
        pts = np.matrix([(0.03933971385434714, 0.040324096282248555, -0.12191399999999999), (0.041065839743852244, 0.0404289503245613, -0.12191399999999999), (0.042093653746638177, 0.040506901144839208, -0.12191399999999999), (0.042966197757810888, 0.040370902957900398, -0.12191399999999999), (0.04378523749577707, 0.040034542941245067, -0.12191399999999999), (0.045021815378913671, 0.040090505268590156, -0.12191399999999999), (0.046002491602822763, 0.040390106689518257, -0.121582), (0.047800393883379491, 0.040340130724107852, -0.121582), (0.048637436283946921, 0.04056969337815975, -0.121582), (0.050425418871595892, 0.040362127660212493, -0.121582), (0.051407986744922411, 0.040564525731377574, -0.121582), (0.053185102826851204, 0.040441883482608312, -0.121582), (0.054218523097327449, 0.039897001707474053, -0.120867), (0.055325707727094907, 0.039854151388412508, -0.120867), (0.056596309966645345, 0.039789907620935844, -0.120867), (0.057873769482100788, 0.039724766148624691, -0.120867), (0.059158086273461266, 0.039658726971479048, -0.120867), (0.060470757204612126, 0.039492020167229164, -0.120867), (0.061770502866758906, 0.039423961154205933, -0.120867), (0.063007401080580838, 0.039399152724046115, -0.120867), (0.064306567339257714, 0.039379647416692989, -0.117287), (0.065682529368838991, 0.039315065513233577, -0.117287), (0.067068126733162869, 0.039041856000455274, -0.116274), (0.068403873093811018, 0.038969084037051002, -0.116274), (0.069671629049205569, 0.038940235935135986, -0.116274), (0.071030718897097694, 0.03918218474525715, -0.116274), (0.072348908907236845, 0.039526443574282019, -0.11460099999999999), (0.073891641960851892, 0.039736237704335597, -0.11460099999999999), (0.075899652413250834, 0.039833871154113888, -0.11460099999999999), (0.077173547537389448, 0.039963107723543966, -0.11460099999999999), (0.078434801896487036, 0.039983588617837941, -0.11460099999999999), (0.079838977289040519, 0.03996379107986292, -0.113747), (0.081059059077820711, 0.040086617697230154, -0.113747), (0.0823939824963585, 0.040124363979867295, -0.113747), (0.083631521857969945, 0.040293383032766271, -0.113747), (0.085375017543157655, 0.040468781331665198, -0.113747), (0.086727063968864579, 0.040517740798063993, -0.113747), (0.087885630817637811, 0.041160308656361347, -0.112598), (0.089708517516107089, 0.041290290608841922, -0.112598), (0.091000264601961103, 0.040822001072778516, -0.112598), (0.092295286502606794, 0.041080450637253005, -0.112598), (0.093976262923417, 0.041334079606233268, -0.112598), (0.095557826695890691, 0.041509108609869574, -0.112598), (0.096570969753093511, 0.041699390556381812, -0.11672299999999999), (0.09747516064027871, 0.041828548269155177, -0.11672299999999999), (0.098311657143262884, 0.041737633571649474, -0.11672299999999999), (0.099252528268425844, 0.041910467228030851, -0.11672299999999999), (0.10015493009931305, 0.041991381296711532, -0.11672299999999999), (0.10106427859080616, 0.042071385958960872, -0.11672299999999999), (0.10187817353686818, 0.042088499245428623, -0.11672299999999999), (0.10268805158587999, 0.041925681781139916, -0.11672299999999999), (0.10361566160964222, 0.041954817921642483, -0.11672299999999999), (0.10450934596848643, 0.041974940560487839, -0.11681), (0.10551119310409758, 0.042056290774711985, -0.11681), (0.1065130667878873, 0.042089159659071632, -0.11681), (0.10745261418938505, 0.042248711804066905, -0.11681), (0.10848164229578733, 0.042423459391442681, -0.11681), (0.10946390417546414, 0.042542396616027503, -0.11681), (0.11044935017632221, 0.042612439144188108, -0.11681), (0.11140071333882272, 0.042528055183589944, -0.11681), (0.1124570027657624, 0.042505322499094271, -0.11681), (0.11342128472418478, 0.042613158720895397, -0.11681), (0.11445394333432957, 0.042932864592157061, -0.11681), (0.11621516957743228, 0.042812302525858353, -0.11681), (0.11727597679076172, 0.042775523601841135, -0.11681), (0.11841060057634679, 0.042694058096800436, -0.11681), (0.11951611399584294, 0.042602948706098973, -0.11681), (0.12060785782649175, 0.042451709272057714, -0.123095), (0.12177924078937967, 0.042413909376461029, -0.123095), (0.12282840907042802, 0.042462155042105751, -0.123095), (0.1240682065851814, 0.042318443100762583, -0.123095), (0.12511746071984059, 0.042304744869016579, -0.123095), (0.12591116738667182, 0.042305704894525648, -0.123095), (0.12640574387262399, 0.042265096051974538, -0.123095), (0.12662810460052679, 0.042241327558360638, -0.123095)])
        pts[:,0] += -0.002
        pts[:,1] += 0.006
        first =np.ravel(pts[0,:]).tolist()
        first[0] -= 0.04
        print first
        lst = []
        for i in range(20):
            first = list(first)
            first[0] += 0.0015
            lst.append(first)
            print first
        first = np.matrix(lst)
        pts = np.vstack((first, pts))
    print pts.shape


    psm1 = robot("PSM1")
    psm2 = robot("PSM2")

    #factor used for interpolation, also used for filter length
    factor = 4
    initialize(pts)

    # grab_gauze()

    angles = []
    for i in range(pts.shape[0]-1):
        pos = pts[i,:]
        nextpos = pts[i+1,:]
        angle = get_angle(np.ravel(pos), np.ravel(nextpos))
        angles.append(angle)

    for i in range(len(angles)-2):
        angles[i] = 0.5 * angles[i] + 0.35 * angles[i+1] + 0.15 * angles[i+2]
    angles = savgol_filter(angles, factor * (pts.shape[0]/12) + 1, 2)


    for i in range(pts.shape[0]-1):
        print i
        cut()
        pos = pts[i,:]
        nextpos = pts[i+1,:]
        print pos
        frame = get_frame_next(np.ravel(pos), np.ravel(nextpos), offset=0.004, angle = angles[i])
        psm1.move_cartesian_frame(frame)

        curpt = np.ravel(np.array(psm1.get_current_cartesian_position().position))
        pts[i,:] = curpt
        pts[i+1,:2] = savgol_filter(pts[:,:2], 5, 2, axis=0)[i+1,:] #probably going to make a small change to this tomorrow


        ###plotting code
        # if i % 25 == 0:
        #     for i in range(3):
        #         plt.plot(cpts[:,i])
        #         plt.plot(pts[:,i], c='r')
        #         plt.show()

    # plot_points()

    