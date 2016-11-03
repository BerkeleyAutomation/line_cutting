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
from ImageSubscriber import ImageSubscriber
#from blob_detector import *
#from line_detector import *

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

    camera = False
    if not camera:
        pts = load_robot_points()
        pts = interpolation(pts, 4)
    else:
        a = ImageSubscriber()
        time.sleep(3)
        left_image = a.left_image
        right_image = a.right_image
        plt.imshow(left_image)
        plt.show()
        plt.imshow(right_image)
        plt.show()
        pts = find_line(left_image, right_image)
        # pts = np.matrix([(0.050112891742798482, 0.04901312543771652, -0.11505099100739051), (0.049793270817931432, 0.049638278022463213, -0.11497826424007704), (0.049466206215529836, 0.050398226753803926, -0.1148959169841571), (0.049254280654923835, 0.051205246584987256, -0.11482247111980422), (0.049121669595940332, 0.052259993347573148, -0.11474161122989905), (0.049106308574960449, 0.053540728368865087, -0.11465892243932006), (0.049476845652889456, 0.054921538511152043, -0.11461172067620816), (0.050100150500154587, 0.055423250091638641, -0.1146474736226813), (0.050773893324379267, 0.055722392755393681, -0.11470114790929602), (0.052485545846438407, 0.055823515539892725, -0.11487631434414315), (0.054511476162234201, 0.056203749833716371, -0.11506259960979005), (0.056416602296403773, 0.05652450188412033, -0.11523450128303603), (0.058475228183097694, 0.056949447396538344, -0.11540886591013287), (0.060351734913341434, 0.056797184017991831, -0.11559227666051541), (0.062130500432385813, 0.05675569918129917, -0.115753149862826), (0.063970333492390161, 0.057022854659247023, -0.11589458113322119), (0.06480889088871869, 0.057078149486657043, -0.11596026834723495), (0.064944721786656889, 0.056908805337006485, -0.1159807402500886), (0.065393120198730947, 0.056992491028879946, -0.11601199851033933), (0.065793933652982944, 0.056984423227084122, -0.11604416882967508), (0.065948269532146137, 0.056567699111297172, -0.11607949869179356), (0.06627148511246686, 0.056226849460976766, -0.1161235988857731), (0.066644151529633863, 0.055781543094424839, -0.11617696638540119), (0.066975107074216417, 0.055119425705083172, -0.11623879846300367), (0.067560940715842316, 0.054682051734368436, -0.11630670524209787), (0.068375336048641669, 0.054254214472314308, -0.11638960905759753), (0.069267229715106432, 0.053571274262225181, -0.11649020757099929), (0.070173419826969202, 0.052895995603010573, -0.11658924097758254), (0.07115853295597889, 0.052197232132116467, -0.11669256422820776), (0.072345616860783932, 0.051498333677092847, -0.11680602261673789), (0.073458076876352654, 0.050956178897409332, -0.11690309890177679), (0.074412501934236278, 0.050078619726297895, -0.11700667791986524), (0.075498887786568977, 0.049196937987520756, -0.11711571602989274), (0.076795981979777633, 0.048545782051541586, -0.11722057252261825), (0.07794257587479439, 0.047727895718499183, -0.11732375715549334), (0.079113592822235929, 0.047325049546845292, -0.11740256559680642), (0.080342870536844718, 0.046731763138096197, -0.11749191651343034), (0.081549416025560706, 0.04568737651417467, -0.11760296499198694), (0.082666093479101269, 0.045291644075007867, -0.11767150394729625), (0.083764594638708278, 0.044947282630071014, -0.11773425419094838), (0.08493263481676161, 0.044068064081706021, -0.11782827355242442), (0.086007763484062333, 0.043530796451421508, -0.11789749586361971), (0.086977615440066589, 0.042834030862627624, -0.11797094703369355), (0.088076180856738545, 0.041718950018024392, -0.11807207186717195), (0.08930999112232775, 0.040795282623709302, -0.1181648751194835), (0.090504237865864673, 0.040046454976144313, -0.11824448234370208), (0.091606785239999697, 0.039799546801418326, -0.11828957273249781), (0.092834155363041918, 0.039255501606361265, -0.11835419831946475), (0.094137049111915144, 0.038283331955583096, -0.11844527594652736), (0.095478746565905187, 0.03746620964463844, -0.11852593179119852), (0.096804358863620224, 0.036909303296497303, -0.11858795783522794), (0.098088863654412578, 0.036504752901974954, -0.11863743905279664), (0.09933770369611318, 0.035735625513107619, -0.11870779672541024), (0.10056566526413759, 0.035276333079035646, -0.11875617146276793), (0.10164849664277512, 0.035263810625050955, -0.11877163198920482), (0.10283614811627123, 0.034956769753149715, -0.11880597600352397), (0.10402256623933781, 0.034908000553331743, -0.11882137395813416), (0.10508020448345419, 0.034996759173100064, -0.11882458393370007), (0.10621260517297153, 0.034941908761361704, -0.11883628708212107), (0.10730982738696807, 0.035087629603084078, -0.11883247593479945), (0.10833356871119748, 0.035376791704343033, -0.11881685783675425), (0.10935910421331427, 0.035665718924843839, -0.11879959685631722), (0.11049306993237507, 0.036273858509277015, -0.1187586579586177), (0.11154846781109176, 0.037124088271202081, -0.11869879587179505), (0.11250546212594295, 0.038009886234547999, -0.118634620323817), (0.11355403238766075, 0.039056975761128425, -0.11855678121033905), (0.11452431755905487, 0.039990025267554664, -0.11848520986619948), (0.11557803163916675, 0.040729339706306045, -0.11842464998136322), (0.11668525592704651, 0.041523793244936311, -0.11835743080673668), (0.11770297978952515, 0.042254666479387776, -0.11829344385940561), (0.11886884590418029, 0.043152578809186937, -0.11821318085992605), (0.11990873250895842, 0.044040677897648463, -0.11813284066802705), (0.1208643235538165, 0.044926659566925764, -0.11805179631202256), (0.12179379983144274, 0.045557949781363308, -0.11798776868409744), (0.12282102931179489, 0.04595779099239438, -0.11793667384653513), (0.12385522008119849, 0.046307729396306681, -0.1178869245930019), (0.12484067492042639, 0.046650895172176861, -0.11783672349895186), (0.12590460177277479, 0.047108076497624177, -0.11777369921018058), (0.12686155684115638, 0.047601947713733724, -0.11770859829816768), (0.12787551145123058, 0.047794408049364544, -0.11766214821038579), (0.12838187060021844, 0.047761664036292756, -0.11764783565930331), (0.12862555025109693, 0.047795527176503316, -0.11763700156943713)])
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

    
