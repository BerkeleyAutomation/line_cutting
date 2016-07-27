# import the necessary packages
# from pyimagesearch.shapedetector import ShapeDetector
import argparse
import imutils
import cv2
import numpy as np
import matplotlib.pyplot as plt
import IPython
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
from statsmodels.nonparametric.kernel_regression import KernelReg


### TO DO: right image/left image preprocessing to increase robustness

def contour_detector(image, show_plots = False, rescale=2):
    # resize it to a smaller factor so that
    # the shapes can be approximated better
    resized = imutils.resize(image, width=int(np.ceil(image.shape[1]/rescale)))
    ratio = image.shape[0] / float(resized.shape[0])

    # convert the resized image to grayscale, blur it slightly,
    # and threshold it
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (25, 25), 0)
    # thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, 19, 2)
    
    # cv2.dilate(thresh, thresh, iterations=1)
    kernel = np.ones((10, 10), np.uint8)
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
    thresh = 255 - closing
    # edges = cv2.Canny(blurred, 100, 200)
    if show_plots:
        cv2.imshow("Thresh", thresh)
        cv2.waitKey(0)
    hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

    # find contours in the thresholded image 
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]

    # initialize the shape detector
    # sd = ShapeDetector()
    lst = []
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
        if cv2.contourArea(c) > 2000 or cv2.contourArea(c) < 100 or mean_val[0] < 90 or mean_val[2] < 70:
            continue
        lst.append([mean_val[:3], (cX, cY), cv2.contourArea(c)])

        # print [mean_val[:3], (cX, cY)]
        if show_plots:
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            cv2.putText(image, "x", (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
              0.5, (255, 255, 255), 2)

            # show the output image
            cv2.imshow("Image", image)
            cv2.waitKey(0)

    # cv2.imshow('image', image)
    # cv2.waitKey(0)
    return lst

def find_correspondences(left, right, disparity_max, disparity_min=0, blob_area_disparity=200, blob_max_area=2000, debugging=False):
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
                if abs(mean_left[0] - right[j][0][0]) < 20 and center_left[0] > center_right[0]:
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
    pts3d = np.array(pts3d)
    x = pts3d[:,0]
    y = pts3d[:,1]
    z = pts3d[:,2]
    return scipy.interpolate.interp2d(x, y, z, kind='linear')

def leftpixels_to_cframe(surf, left_pts, right_pts, pts3d, x, y, knn=False):
    xin = np.array([a[0] for a in left_pts])
    bias = np.ones(len(xin))
    yin = np.array([a[1] for a in left_pts])
    xout = np.array([np.ravel(a)[0] for a in pts3d])
    yout = np.array([np.ravel(a)[1] for a in pts3d])
    A = np.vstack([xin, bias]).T
    m1, c1 = np.linalg.lstsq(A, xout)[0]

    A = np.vstack([yin, bias]).T
    m2, c2 = np.linalg.lstsq(A, yout)[0]

    xnew = m1 * x + c1
    ynew = m2 * y + c2
    zold = surf.f2(xnew, ynew)[0]

    cpoint = np.matrix([(xnew, ynew, zold)])
    pt = np.ones(4)
    pt[:3] = cpoint
    pred = np.ravel(surf.cmat * np.matrix(pt).T)
    if knn:
        return (pred[0], pred[1], surf.query_knn(pred[0], pred[1])[2])
    return (pred[0], pred[1], surf.query(pred[0], pred[1])[2])

def get_surface(left, right):
    info = {}
    f = open("../calibration_data/camera_left.p", "rb")
    info['l'] = pickle.load(f)
    f.close()
    f = open("../calibration_data/camera_right.p", "rb")
    info['r'] = pickle.load(f)
    f.close()

    left_image = cv2.imread(left)
    right_image = cv2.imread(right)
    left = contour_detector(left_image)
    right = contour_detector(right_image)
    correspondences = find_correspondences(left, right, 300, 70)
    print "found ", len(correspondences), " correspondences"
    disparities = calculate_disparity(correspondences)
    left_pts = [a[0] for a in correspondences]
    right_pts = [a[1] for a in correspondences]
    pts3d = get_points_3d(info, left_pts, right_pts)
    oldpts3d = pts3d
    newpts = []
    f3 = open("../calibration_data/camera_matrix.p", "rb")
    cmat = pickle.load(f3)
    f3.close()
    for cpoint in pts3d:
        pt = np.ones(4)
        pt[:3] = cpoint
        pred = cmat * np.matrix(pt).T
        newpts.append(np.ravel(pred).tolist())
    pts3d = newpts
    newlst1 = [] # pts3d
    newlst2 = [] # oldpts3d
    newlst3 = [] # left
    newlst4 = [] # right
    for i in range(len(pts3d)):
        point = pts3d[i]
        if point[2] < -0.140 or point[2] > -0.090:
            continue
        else:
            newlst1.append(pts3d[i])
            newlst2.append(oldpts3d[i].tolist())
            newlst3.append(left_pts[i])
            newlst4.append(right_pts[i])
    pts3d = newlst1
    oldpts3d = np.array(newlst2)
    left_pts = newlst3
    right_pts = newlst4

    surf = fit_surface(pts3d)
    surf2 = fit_surface(oldpts3d)
    return Surface(surf, surf2, pts3d, left_pts, right_pts, oldpts3d)
    
class Surface:

    def __init__(self, f, f2, pts3d, left_pts, right_pts, oldpts3d, safety_check=False):
        self.f = f
        self.f2 = f2
        self.safety_check = safety_check
        self.pts3d = np.matrix(pts3d)
        self.minimum = np.min(self.pts3d[:,2])
        self.maximum = np.max(self.pts3d[:,2])
        self.oldpts3d = oldpts3d
        self.left_pts = left_pts
        self.right_pts = right_pts
        pts2d = []
        ptsz = []
        f3 = open("../calibration_data/camera_matrix.p", "rb")
        self.cmat = pickle.load(f3)
        f3.close()
        
        for pt in pts3d:
            pts2d.append(pt[:2])
            ptsz.append(np.ceil(pt[2] * 1000000))
        self.neigh = KNeighborsClassifier(n_neighbors=2)
        self.neigh.fit(pts2d, ptsz)
        self.f = scipy.interpolate.Rbf(np.matrix(pts3d)[:,0].ravel(), np.matrix(pts3d)[:,1].ravel(), np.matrix(pts3d)[:,2].ravel(), function='linear', epsilon=.1)
        pts3d = np.array(pts3d).T
        print pts3d.shape
        print pts3d[:2,:].shape, pts3d[2,:].shape
        self.f = KernelReg(pts3d[2,:], pts3d[:2,:], 'cc')

    def leftpixels_to_rframe(self, x, y):
        surf = self.f2
        left_pts = self.left_pts
        right_pts = self.right_pts
        pts3d = self.oldpts3d
        xin = np.array([a[0] for a in left_pts])
        bias = np.ones(len(xin))
        yin = np.array([a[1] for a in left_pts])

        xout = np.array([a[0] for a in pts3d])
        yout = np.array([a[1] for a in pts3d])

        A = np.vstack([xin, bias]).T
        m1, c1 = np.linalg.lstsq(A, xout)[0]

        A = np.vstack([yin, bias]).T
        m2, c2 = np.linalg.lstsq(A, yout)[0]

        xnew = m1 * x + c1
        ynew = m2 * y + c2
        cpoint = np.matrix([(xnew, ynew, self.f2(xnew, ynew))])
        pt = np.ones(4)
        pt[:3] = cpoint
        pred = self.cmat * np.matrix(pt).T
        return pred

    def query(self, x, y):
        temp = self.f.fit(np.array((x, y)))[0][0]
        if not self.safety_check:
            return (x, y, temp)
        if temp < self.minimum - 0.02:
            temp = self.query_knn(x, y)[2]
        elif temp > self.maximum + 0.02:
            temp = self.query_knn(x, y)[2]
        print 'asdf', temp
        return (x, y, temp)

    def query_knn(self, x, y):
        return (x, y, (self.neigh.predict([[x, y]]) / 1000000.0)[0])

    def visualize(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        pts3d = np.matrix(self.pts3d)
        f = self.f
        a, b =  np.ravel(np.min(pts3d, axis=0)), np.ravel(np.max(pts3d, axis=0))
        extra_range = 0.0
#         xnew = np.arange(a[0] - extra_range,b[0] + extra_range,0.0001)
#         ynew = np.arange(a[1] - extra_range,b[1] + extra_range,0.0001)
        X, Y = np.mgrid[a[0] + .05 :b[0] - .05 :100j, a[1]:b[1]:100j]
        # znew = f.fit(X.ravel(), Y.ravel())
#         znew = f(xnew, ynew)

        # plt.imshow(znew.reshape((100, 100)), cmap=plt.cm.gist_earth_r,
        #   extent=[a[0], b[0], a[1], b[1]])
        # ax.set_xlim([a[0], b[0]])
        # ax.set_ylim([a[1], b[1]])
        # plt.plot(pts3d[:,0], pts3d[:,1],'o')
        
        # plt.show()  
        





if __name__ == "__main__":

    SHOW_PLOTS = False
    
    left_image = cv2.imread("left86.jpg")
    right_image = cv2.imread("right86.jpg")

    info = {}
    f = open("../calibration_data/camera_left.p", "rb")
    info['l'] = pickle.load(f)
    f.close()

    f = open("../calibration_data/camera_right.p", "rb")
    info['r'] = pickle.load(f)
    f.close()



    surf = get_surface("left86.jpg", "right86.jpg") # specify images

    print surf.pts3d.tolist()
    print np.max(surf.pts3d, axis=0)

    print surf.maximum, surf.minimum
    # print f(-0.0125228, 0.01744704)

    print 'test' + str(leftpixels_to_cframe(surf, surf.left_pts, surf.right_pts, surf.oldpts3d, 80, 276))
    print surf.query(0.065395486704561773, 0.049723773830542037)
    # print pts3d[0]
    surf.visualize()
    plt.imshow(left_image)
    plt.show()
    print "done"
    if SHOW_PLOTS:
        plt.imshow(left_image)
        plt.show()
        plt.imshow(right_image)
        plt.show()


