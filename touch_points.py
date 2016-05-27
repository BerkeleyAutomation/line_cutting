import rospy, pickle, time
from robot import *
from geometry_msgs.msg import Pose
import numpy as np
import PyKDL
import multiprocessing
import tfx


def get_frame(pos):
    """
    Takes in a 3d Robot frame point and returns a tfx pose
    """
    rot = [0.704583065311, 0.590342398526, 0.387353243821, 0.0708238736684]
    return tfx.pose(pos[0:3], rot)

def camera_to_robot_frame(point, cmat):
    """
    Takes in a 3d point and a rigid transformation matrix, and outputs the result.
    """
    pt = np.ones(4)
    pt[:3] = point
    pred = cmat * np.matrix(pt).T
    return pred

def convertStereo(u, v, disparity, info):
    """
    Converts two pixel coordinates u and v along with the disparity to give PointStamped       
    """
    stereoModel = image_geometry.StereoCameraModel()
    stereoModel.fromCameraInfo(info['l'], info['r'])
    (x,y,z) = stereoModel.projectPixelTo3d((u,v), disparity)

    cameraPoint = PointStamped()
    cameraPoint.header.frame_id = info['l'].header.frame_id
    cameraPoint.header.stamp = time.time()
    cameraPoint.point = Point(x,y,z)
    return cameraPoint

def pixels_to_3D(left_corners, right_corners, info):
    """
    Takes in two lists of pixel coordinates and camera info as a dictionary and outputs 3d points in camera frame.
    """
    pts3d = get_points_3d(left_corners, right_corners)
    self.pts = [(p.point.x, p.point.y, p.point.z) for p in pts3d]
    return self.pts

def get_points_3d(left_points, right_points, info):
    """ this method assumes that corresponding points are in the right order
        and returns a list of 3d points """

    # both lists must be of the same lenghth otherwise return None
    if len(left_points) != len(right_points):
        rospy.logerror("The number of left points and the number of right points is not the same")
        return None

    points_3d = []
    for i in range(len(left_points)):
        a = left_points[i]
        b = right_points[i]
        disparity = abs(a[0]-b[0])
        pt = convertStereo(a[0], a[1], disparity, info)
        points_3d.append(pt)
    return points_3d

def load_camera_info():
    info = {}
    f = open("calibration_data/camera_right.p")
    info['r'] = pickle.load(f)
    f.close()
    f = open("calibration_data/camera_left.p")
    info['l'] = pickle.load(f)
    f.close()
    return info

def load_camera_matrix():
    f3 = open("calibration_data/camera_matrix.p", "rb")
    cmat = pickle.load(f3)
    f3.close()
    return cmat


if __name__ == "__main__":

    #initialization
    psm1 = robot("PSM1")
    cmat = load_camera_matrix()
    info = load_camera_info()

    lpixels, rpixels = [],[]
    info = {}

    pts = pixels_to_3D(lpixels, rpixels, info)

    pts = []

    for point in pts:
        psm1.move_cartesian_frame(camera_to_robot_frame(point))
        time.sleep(2)


