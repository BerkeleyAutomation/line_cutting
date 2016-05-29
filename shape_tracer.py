import cv2
import Tkinter
from Tkinter import *
import rospy, pickle, time
from geometry_msgs.msg import Pose
import multiprocessing
import numpy as np
import sys
from sklearn.neighbors import KNeighborsClassifier

"""
Launching this script creates a GUI that subscribes to PSM1's position_cartesian_current topic and can write this information to file.
"""

def startCallback():
    global prs
    process = multiprocessing.Process(target = start_listening)
    prs.append(process)
    process.start()
    return

def start_listening():
    global sub
    rospy.init_node('listener', anonymous=True)
    sub = rospy.Subscriber('/dvrk/PSM1/position_cartesian_current', Pose, callback_PSM1_actual)
    rospy.spin()

def exitCallback():
    global prs
    for process in prs:
        process.terminate()
    plot_points()
    sys.exit()

def callback_PSM1_actual(data):
    position = data.position
    psm1_pose = [position.x, position.y, position.z]
    print psm1_pose
    f = open("calibration_data/gauze_pts.p", "a")
    pickle.dump(psm1_pose, f)
    f.close()
    sub.unregister()

def load_robot_points():
    lst = []
    f3 = open("calibration_data/gauze_pts.p", "rb")
    pos1 = pickle.load(f3)
    lst.append(pos1)
    while True:
        try:
            pos2 = pickle.load(f3)
            lst.append(pos2)
        except EOFError:
            f3.close()
            return np.matrix(lst)


def plot_points(pts=load_robot_points()):
    """
    Plots points in robot_frame. Axes may need to be edited.
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(np.array(pts[:,0]), np.array(pts[:,1]), np.array(pts[:,2]),c='r')
    # ax.set_xlim3d(-0.1, 0)
    # ax.set_ylim3d(-0.05, 0.05)
    # ax.set_zlim3d(0.1,0.2)
    plt.show()

def knn_clasifier():
    pts = load_robot_points()
    y = np.r_[0:pts.shape[0]]
    neigh = KNeighborsClassifier(n_neighbors=1)
    neigh.fit(pts, y)
    return neigh


if __name__ == '__main__':
    sub = None
    prs = []

    open('calibration_data/gauze_pts.p', 'w+').close()

    top = Tkinter.Tk()
    top.title('Calibration')
    top.geometry('400x200')

    B = Tkinter.Button(top, text="Record Position", command = startCallback)
    D = Tkinter.Button(top, text="Exit", command = exitCallback)

    B.pack()
    D.pack()

    top.mainloop()
