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
import scipy.ndimage
from blob_detector import *

### TO DO: right image/left image preprocessing to increase robustness

def line_detector_drawn(image, show_plots = False):
    # resize it to a smaller factor so that
    # the shapes can be approximated better
    resized = imutils.resize(image, width=int(np.ceil(image.shape[1]/2)))
    ratio = image.shape[0] / float(resized.shape[0])

    # convert the resized image to grayscale, blur it slightly,
    # and threshold it
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (45, 45), 0)

    # thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, 19, 2)
    kernel1 = np.ones((7, 7), np.uint8)
    thresh = 255 - thresh

    thresh = cv2.dilate(thresh, kernel1, iterations=1)
    kernel2 = np.ones((7, 7), np.uint8)
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel2)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel2)
    thresh = closing
    # if show_plots:
    #     cv2.imshow("Thresh", thresh)
    #     cv2.waitKey(0)

    return remove_blobs(image, resized, thresh, ratio, show_plots), ratio



def line_detector_painted(image, show_plots = False):
    # resize it to a smaller factor so that
    # the shapes can be approximated better
    resized = imutils.resize(image, width=int(np.ceil(image.shape[1]/2)))
    ratio = image.shape[0] / float(resized.shape[0])

    # convert the resized image to grayscale, blur it slightly,
    # and threshold it
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (35, 35), 0)
    # thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, 15, 2)

    kernel = np.ones((5, 5), np.uint8)
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
    thresh = 255 - closing

    if show_plots:
        cv2.imshow("Thresh1", thresh)
        cv2.waitKey(0)
    return remove_blobs(image, resized, thresh, ratio, show_plots), ratio

def remove_blobs(full_image, resized_image, gray, ratio, show_plots=False):
    if show_plots:
        cv2.imshow("Thresh2", gray)
        cv2.waitKey(0)

    cnts = cv2.findContours(gray.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]

    # loop over the contours
    
    hsv = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)
    # minv = 1000
    # for c in cnts:
    #     mask = np.zeros(gray.shape,np.uint8)
    #     cv2.drawContours(mask,[c],0,255,-1)
    #     mean_val = np.array(cv2.mean(hsv,mask = mask))
    #     minv = min(mean_val[2], minv)
    # print minv
    for c in cnts:
        mask = np.zeros(gray.shape,np.uint8)
        cv2.drawContours(mask,[c],0,255,-1)
        mean_val = np.array(cv2.mean(hsv,mask = mask))
        if np.max(mean_val) < 70:
            print 'asdf', mean_val
            continue
        else:
            print cv2.contourArea(c)
            if cv2.contourArea(c) < 1500:
                cv2.drawContours(gray, [c], -1, (0, 0, 0), -1)
            # else:
                # pass
    if show_plots:
        cv2.imshow("a", gray)
        cv2.waitKey(0)
    return gray

def detect_relative_position(cur_position, next_position, image, ratio, rect_width=400, rect_height=50, distance=100, show_plots=False):
    """
    Takes in current and next robot position in pixel space and a full image and returns whether or not the robot is to the right or left.
    """
    cur_position, next_position = np.array(cur_position), np.array(next_position)
    delta = next_position - cur_position

    factor = distance / np.linalg.norm(np.array(delta))
    next_position = cur_position + factor * np.array(delta)

    cur_position, next_position = np.array(cur_position), np.array(next_position)
    delta = next_position - cur_position


    slope = np.array((delta[0], delta[1]))
    if np.linalg.norm(slope) != 0:
        slope =  slope / np.linalg.norm(slope)
    perp_slope = np.array((-delta[1], delta[0]))
    if np.linalg.norm(perp_slope) != 0:
        perp_slope = perp_slope / np.linalg.norm(perp_slope)
    pts = []
    # points defining the corners of a rectangle
    pts.append(next_position + perp_slope * rect_width / 2)
    pts.append(pts[-1] + slope * rect_height/2)
    pts.append(pts[-1] - slope * rect_height)
    pts.pop(-3)
    pts.append(next_position - perp_slope * rect_width / 2)
    pts.append(pts[-1] + slope * rect_height/2)
    pts.append(pts[-1] - slope * rect_height)
    pts.pop(-3)
    pts = np.array([pt.tolist() for pt in pts]) / ratio

    xmax = int(np.ceil(np.max(pts[:,0].tolist() + [image.shape[0]])))
    xmin = int(np.floor(np.min(pts[:,0].tolist() + [0])))

    ymax = int(np.ceil(np.max(pts[:,1].tolist() + [image.shape[1]])))
    ymin = int(np.floor(np.min(pts[:,1].tolist() + [0])))



    xstart = -xmin
    xend = xstart + image.shape[0]

    ystart = -ymin
    yend = ystart + image.shape[1]

    newshape = (xmax - xmin, ymax - ymin)

    new_image = np.zeros(newshape)
    new_image[xstart:xend,ystart:yend] = image
    new_image = new_image.astype(np.uint8)

    newpts = []
    for i in range(pts.shape[0]):
        pt = np.floor(pts[i,:] + np.array((xstart, ystart))).astype(int)
        newpts.append(pt.tolist())
    newpts[2], newpts[3] = newpts[3], newpts[2]

    cv2.drawContours(new_image, [np.array(newpts)], -1, (0,255,0), 3)


    mask = np.zeros(np.array(new_image.shape)[:2],np.uint8)
    cv2.drawContours(mask,[np.array(newpts)],0,255,-1)

    new_image *= mask
    if show_plots:
        plt.imshow(image)
        plt.show()
        plt.imshow(new_image, cmap='Greys_r')
        plt.show()
        plt.imshow(mask, cmap='Greys_r')
        plt.show()
        plt.imshow(new_image[xstart:xend,ystart:yend], cmap='Greys_r')
        plt.show()


    cnts = cv2.findContours(new_image[xstart:xend,ystart:yend], cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]

    dist = float('inf')
    best_center = None
    for c in cnts:
        M = cv2.moments(c)
        cX = int((M["m10"] / M["m00"]) * ratio) if M["m00"] != 0 else 0
        cY = int((M["m01"] / M["m00"]) * ratio) if M["m00"] != 0 else 0
        center = np.array((cX, cY))
        if np.linalg.norm(center - next_position) < dist:
            dist = np.linalg.norm(center - next_position)
            best_center = center
            
    center_of_mass = best_center
    if center_of_mass == None:
        return -1
    line_vector = center_of_mass - np.array(cur_position)
    planned_vector = np.array(next_position) - np.array(cur_position)

    cpdt = np.cross(line_vector, planned_vector)
    if cpdt < 0:
        return 1 # line is to the right
    else:
        return 0 # line is to the left

def detect_new_point_rect(cur_position, image, ratio, heading, rect_width=800, rect_height=25, min_dist=10, max_dist = 1000, show_plots=False):
    cur_position = (np.array(cur_position) / ratio).astype(int)

    dist = min_dist
    delta = np.array((np.cos(heading), np.sin(heading)))
    outer_color = (255, 255, 255)
    found = False

    while dist < max_dist:
        next_position = cur_position - dist * np.array(delta)
        slope = np.array((delta[0], delta[1]))
        if np.linalg.norm(slope) != 0:
            slope =  slope / np.linalg.norm(slope)
        perp_slope = np.array((-delta[1], delta[0]))
        if np.linalg.norm(perp_slope) != 0:
            perp_slope = perp_slope / np.linalg.norm(perp_slope)
        pts = []
        # points defining the corners of a rectangle
        pts.append(next_position + perp_slope * rect_width / 2)
        pts.append(pts[-1] + slope * rect_height/2)
        pts.append(pts[-1] - slope * rect_height)
        pts.pop(-3)
        pts.append(next_position - perp_slope * rect_width / 2)
        pts.append(pts[-1] + slope * rect_height/2)
        pts.append(pts[-1] - slope * rect_height)
        pts.pop(-3)
        pts = np.array([pt.tolist() for pt in pts])

        xmax = int(np.ceil(np.max(pts[:,0].tolist() + [image.shape[1]])))
        xmin = int(np.floor(np.min(pts[:,0].tolist() + [0])))

        ymax = int(np.ceil(np.max(pts[:,1].tolist() + [image.shape[0]])))
        ymin = int(np.floor(np.min(pts[:,1].tolist() + [0])))

        xstart = -ymin
        xend = xstart + image.shape[0]

        ystart = -xmin
        yend = ystart + image.shape[1]

        newshape = (ymax - ymin, xmax - xmin)

        new_image = np.zeros(newshape)
        new_image[xstart:xend,ystart:yend] = image
        new_image = new_image.astype(np.uint8)

        newpts = []
        for i in range(pts.shape[0]):
            pt = np.floor(pts[i,:] + np.array((ystart, xstart))).astype(int)
            newpts.append(pt.tolist())
        newpts[2], newpts[3] = newpts[3], newpts[2]
        cv2.drawContours(new_image, [np.array(newpts)], -1, (0,255,0), 3)

        mask = np.zeros(np.array(new_image.shape)[:2],np.uint8)
        cv2.drawContours(mask,[np.array(newpts)],0,255,-1)

        new_image *= mask
        if show_plots:
            plt.imshow(image)
            plt.show()
            plt.imshow(new_image, cmap='Greys_r')
            plt.show()
            plt.imshow(mask, cmap='Greys_r')
            plt.show()
            plt.imshow(new_image[xstart:xend,ystart:yend], cmap='Greys_r')
            plt.show()

        cnts = cv2.findContours(new_image[xstart:xend,ystart:yend], cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]

        distance = float('inf')
        best_center = None
        for c in cnts:
            M = cv2.moments(c)
            cX = int((M["m10"] / M["m00"]) * ratio) if M["m00"] != 0 else 0
            cY = int((M["m01"] / M["m00"]) * ratio) if M["m00"] != 0 else 0
            center = np.array((cX, cY))
            if np.linalg.norm(center - next_position) < distance:
                distance = np.linalg.norm(center - next_position)
                best_center = center
        if best_center == None:
            dist += 20
        else:
            return best_center
    return


def detect_new_point_circle(cur_position, image, ratio, heading, thickness=20, show_plots=False):
    """
    Takes in current and next robot position in pixel space and a full image and returns the next pixel of the line in pixel space.
    TODO: add something to mask out the gripper itself. Uses a circle mask to detect the next point.
    """
    center = (np.array(cur_position) / ratio).astype(int)

    delta = np.array((np.cos(heading), np.sin(heading))) * thickness
    print delta
    print center
    outer_radii = np.r_[10:1000:20]
    inner_radii = np.r_[0:980:20]

    inner_color = (0,0,0)
    outer_color = (255, 255, 255)

    found = False



    idx = 0
    while not found and outer_radii[idx] < outer_radii[-1]:

        xmin = min(0, center[0] - outer_radii[idx])
        xmax = max(image.shape[0], center[0] + outer_radii[idx])
        ymin = min(0, center[1] - outer_radii[idx])
        ymax = max(image.shape[1], center[1] + outer_radii[idx])
        newshape = (xmax - xmin, ymax - ymin)
        new_image = np.zeros(newshape)
        new_image[-xmin:-xmin + image.shape[0],-ymin:-ymin + image.shape[1]] = image
        new_image = new_image.astype(np.uint8)

        mask = np.zeros(np.array(new_image.shape)[:2],np.uint8)
        new_center = (-xmin + center[0], -ymin + center[1])
        cv2.circle(mask, new_center, outer_radii[idx], outer_color, -1)
        cv2.circle(mask, (int(new_center[0] + delta[0]), int(new_center[1] + delta[1])), inner_radii[idx], inner_color, -1)
        idx += 1
        new_image *= mask

        cnts = cv2.findContours(new_image[-xmin:-xmin + image.shape[0],-ymin:-ymin + image.shape[1]], cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]

        dist = float('inf')
        best_center = None
        for c in cnts:
            found = True
            M = cv2.moments(c)
            cX = int((M["m10"] / M["m00"]) * ratio) if M["m00"] != 0 else 0
            cY = int((M["m01"] / M["m00"]) * ratio) if M["m00"] != 0 else 0
            center = np.array((cX, cY))
            if np.linalg.norm(center - next_position) < dist:
                dist = np.linalg.norm(center - next_position)
                best_center = center

        if show_plots:
            plt.imshow(new_image, cmap='Greys_r')
            plt.show()
            plt.imshow(mask, cmap='Greys_r')
            plt.show()
    return best_center


def get_pixel_from3D(camera_matrix, position, camera_info, offset):
    Trobot = np.zeros((4,4))
    Trobot[:3,:] = np.copy(camera_matrix)
    Trobot[3,3] = 1
    Rrobot = np.linalg.inv(Trobot)

    x = np.ones((4,1))

    x[:3,0] = np.squeeze(position)

    cam_frame = np.dot(Rrobot,x)

    Pcam = np.array(camera_info.P).reshape(3,4)

    V = np.dot(Pcam, cam_frame)

    V = np.array((int(V[0]/V[2]), int(V[1]/V[2])))

    V[0] = V[0] + offset[0]
    V[1] = V[1] + offset[1]

    return V


def plot_points(pts):
    """
    Plots points in robot_frame. Axes may need to be edited.
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    if pts.shape[1] == 0:
        print "no points to show"
        return
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(np.array(pts[:,0]), np.array(pts[:,1]), np.array(pts[:,2]),c='r')
    ax.set_xlim3d(0, 0.2)
    ax.set_ylim3d(-0.1, 0.1)
    ax.set_zlim3d(-0.15,0.05)
    plt.show()

if __name__ == "__main__":

    SHOW_PLOTS = False

    lst = get_surface("left97.jpg", "right97.jpg") # specify images
    surf = lst[0]
    left_pts = lst[1]
    right_pts = lst[2]
    pts3d = lst[3]
    oldpts3d = lst[4]

    left_image = cv2.imread("left97.jpg")
    plt.imshow(np.array(left_image))
    plt.show()

    cur_position = (2000, 500)
    next_position = (1990, 500)

    left_gray, ratio = line_detector_drawn(left_image, SHOW_PLOTS)
    rel = detect_relative_position(cur_position, next_position, left_gray, ratio, show_plots=False)
    print rel

    heading = np.arctan((np.array(cur_position) - np.array(next_position))[1] / (np.array(cur_position) - np.array(next_position))[0])
    traj = []
    pxtraj = []

    for i in range(100):
        cur_position = next_position
        next_position = detect_new_point_rect(cur_position, left_gray, ratio, heading, show_plots=False)
        if next_position == None:
            break
        heading = np.arctan((np.array(cur_position) - np.array(next_position))[1] / (np.array(cur_position) - np.array(next_position))[0])
        pos = leftpixels_to_cframe(surf, left_pts, right_pts, pts3d, next_position[0], next_position[1])
        traj.append(pos)
        pxtraj.append(next_position.tolist())
    print traj
    print pxtraj
    pxtraj = np.matrix(pxtraj)
    plt.scatter(pxtraj[:,0], pxtraj[:,1])
    plt.show()
    plot_points(np.matrix(traj))

