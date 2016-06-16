# import the necessary packages
# from pyimagesearch.shapedetector import ShapeDetector
import argparse
import imutils
import cv2
import numpy as np
import matplotlib.pyplot as plt
import IPython
from ImageSubscriber import ImageSubscriber
import pickle
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
import sys
import time
import image_geometry
import scipy.interpolate
from sklearn.neighbors import KNeighborsClassifier



def contour_detector(image, show_plots = False):
    # resize it to a smaller factor so that
    # the shapes can be approximated better
    resized = imutils.resize(image, width=int(np.ceil(image.shape[1]/2)))
    ratio = image.shape[0] / float(resized.shape[0])

    # convert the resized image to grayscale, blur it slightly,
    # and threshold it
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (25, 25), 0)
    # thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, 11, 2)
    thresh = 255 - thresh

    # edges = cv2.Canny(blurred, 100, 200)
    if show_plots:
        cv2.imshow("Thresh", blurred)
        cv2.waitKey(0)
    hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

    # find contours in the thresholded image 
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]

    # initialize the shape detector
    # sd = ShapeDetector()
    lst = []
    print len(cnts)
    # loop over the contours
    for c in cnts:
        # compute the center of the contour, then detect the name of the
        # shape using only the contour
        M = cv2.moments(c)
        cX = int((M["m10"] / M["m00"]) * ratio) if M["m00"] != 0 else 0
        cY = int((M["m01"] / M["m00"]) * ratio) if M["m00"] != 0 else 0
        # shape = sd.detect(c)
        #find mean color

        mask = np.zeros(gray.shape,np.uint8)
        cv2.drawContours(mask,[c],0,255,-1)
        mean_val = cv2.mean(hsv,mask = mask)

        lst.append([mean_val[:3], (cX, cY), cv2.contourArea(c)])

        # print [mean_val[:3], (cX, cY)]
        if show_plots:
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            cv2.putText(image, "center", (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
              0.5, (255, 255, 255), 2)

            # show the output image
            cv2.imshow("Image", image)
            cv2.waitKey(0)

    # cv2.imshow('image', image)
    # cv2.waitKey(0)
    return lst

def find_correspondences(left, right, disparity_max, disparity_min=0, blob_area_disparity=300, blob_max_area=1000):
    indices = []
    correspondences = []
    for i in range(len(left)):
        # blob_left = [(h,s,v), (cX, cY)], a list of hsv mean value and center of blob pairs in original image
        blob_left = left[i]
        mean_left = blob_left[0]
        center_left = np.array(blob_left[1])

        best_idx = -1
        best_dist = float("inf")
        for j in range(len(right)):
            # check if the current blob in the right list is closer than our threshold radius
            blob_right = right[j]
            mean_right = blob_right[0]
            center_right = np.array(blob_right[1])

            dist = np.linalg.norm(center_left - center_right)

            if dist < disparity_max and dist < best_dist and dist >= disparity_min and blob_left[2] < blob_max_area:
                # check the h value of the means to see if they are with +-10 of each other
                if abs(mean_left[0] - right[j][0][0]) < 10:
                    if abs(center_right[1] - center_left[1]) < 20 and abs(blob_left[2] - blob_right[2]) < blob_area_disparity:
                        best_dist = dist
                        best_idx = j
        indices.append(best_idx)
        if best_idx != -1:
            correspondences.append((tuple(center_left.tolist()), right[best_idx][1]))

    return correspondences

def calculate_disparity(correspondences):
    lst = []
    for c in correspondences:
        left = c[0]
        right = c[1]
        disparity = left[0] - right[0]
        lst.append(disparity)
    return lst

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

def get_points_3d(info, left_points, right_points):
    """ this method assumes that corresponding points are in the right order
        and returns a list of 3d points """

    # both lists must be of the same lenghth otherwise return None
    if len(left_points) != len(right_points):
        return None

    points_3d = []
    for i in range(len(left_points)):
        a = left_points[i]
        b = right_points[i]
        disparity = abs(a[0]-b[0])
        pt = convertStereo(a[0], a[1], disparity, info)
        points_3d.append([pt.point.x, pt.point.y, pt.point.z])
    return np.array(points_3d)

def fit_surface(pts3d):
    points = pts3d[:,:2]
    values = pts3d[:,2]

    grid_x, grid_y = np.mgrid[-0.05:0.05:0.0001, -0.05:0.05:0.0001]
    method = 'cubic'
    grid_z2 = scipy.interpolate.griddata(points, values, (grid_x, grid_y), method='cubic')
    return grid_z2.T

def query_pt(grid_z2, pt):
    x, y = pt
    x = int((x + 0.05) / 0.0001)
    y = int((y + 0.05) / 0.0001)
    print x, y
    if not np.isnan(grid_z2[y, x]):
        return grid_z2[y, x]
    nans = np.isnan(grid_z2)
    pts = []
    vals = []
    for i in range(1000):
        for j in range(1000):
            if nans[i,j]:
                continue
            pts.append([i,j])
            vals.append(grid_z2[i,j])
    vals = np.ceil(np.array(vals) * 1000000)
    return knn_clasifier(pts, vals).predict([(y, x)]) / 1000000

def knn_clasifier(pts, y):
    neigh = KNeighborsClassifier(n_neighbors=1)
    neigh.fit(pts, y)
    return neigh




if __name__ == "__main__":
    # construct the argument parse and parse the arguments
    # ap = argparse.ArgumentParser()
    # ap.add_argument("-i", "--image", required=True,
    #     help="path to the input image")
    # args = vars(ap.parse_args())

    # load the image and 
    # image = cv2.imread(args["image"])

    # left_image = cv2.imread("shapes_and_colors.jpg")
    # right_image = cv2.imread("shapes_and_colors.jpg")
    SHOW_PLOTS = False

    info = {}
    f = open("../calibration_data/camera_left.p", "rb")
    info['l'] = pickle.load(f)
    f.close()

    f = open("../calibration_data/camera_right.p", "rb")
    info['r'] = pickle.load(f)
    f.close()


    left_image = cv2.imread("left0.jpg")
    right_image = cv2.imread("right0.jpg")
    left = contour_detector(left_image, SHOW_PLOTS)
    right = contour_detector(right_image, SHOW_PLOTS)

    correspondences = find_correspondences(left, right, 500, 70)

    disparities = calculate_disparity(correspondences)

    left_pts = [a[0] for a in correspondences]
    right_pts = [a[1] for a in correspondences]
    pts3d = get_points_3d(info, left_pts, right_pts)

    X = [[0], [3.1], [2], [3]]
    y = [0, 0, 1, 1]
    from sklearn.neighbors import KNeighborsClassifier
    neigh = KNeighborsClassifier(n_neighbors=3)
    neigh.fit(X, y) 

    print(neigh.predict([[1.1]]))

    print(neigh.predict_proba([[0.9]]))

    print pts3d
    surf = fit_surface(pts3d)
    pt = [0, 0]
    print query_pt(surf, pt)
    # plt.imshow(surf)
    # plt.show()

