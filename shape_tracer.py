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
    f = open('calibration_data/'+gauze_pts+'.p', "a")
    pickle.dump(psm1_pose, f)
    f.close()
    sub.unregister()

def load_robot_points():
    lst = []
    f3 = open('calibration_data/'+gauze_pts+'.p',"rb")
    while True:
        try:
            pos2 = pickle.load(f3)
            lst.append(pos2)
        except EOFError:
            f3.close()
            return np.matrix(lst)

def startCallback2():
    global prs
    process = multiprocessing.Process(target=start_listening2)
    prs.append(process)
    process.start()
    return

def start_listening2():
    global sub
    rospy.init_node('listener', anonymous=True)
    sub = rospy.Subscriber('/dvrk/PSM2/position_cartesian_current', Pose, callback_PSM2_actual)
    rospy.spin()
def callback_PSM2_actual(data):
    position = data.position
    psm2_pose = [position.x, position.y, position.z]
    print psm2_pose
    f = open('calibration_data/gauze_grab_pt.p', "a")
    pickle.dump(psm2_pose, f)
    f.close()
    sub.unregister()

def plot_points():
    """
    Plots points in robot_frame. Axes may need to be edited.
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    pts=load_robot_points()
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

def knn_clasifier():
    pts = load_robot_points()
    y = np.r_[0:pts.shape[0]]
    neigh = KNeighborsClassifier(n_neighbors=1)
    neigh.fit(pts, y)
    return neigh
def switchCallback():
    global gauze_pts
    plot_points()
    gauze_pts='gauze_pts2'

if __name__ == '__main__':
    sub = None
    prs = []
    gauze_pts='gauze_pts'
    open('calibration_data/'+gauze_pts+'.p', 'w+').close()
    open("calibration_data/gauze_pts2.p", "w+").close()
    open("calibration_data/gauze_grab_pt.p", "w+").close()
    top = Tkinter.Tk()
    top.title('Calibration')
    top.geometry('400x200')

    B = Tkinter.Button(top, text="Record Position PSM1", command = startCallback)
    D=Tkinter.Button(top, text="part 2", command = switchCallback)
    E=Tkinter.Button(top, text="grab point", command = startCallback2)
    F = Tkinter.Button(top, text="Exit", command = exitCallback)

    B.pack()
    D.pack()
    E.pack()
    F.pack()
    top.mainloop()
