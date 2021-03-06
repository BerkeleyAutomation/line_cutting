import Tkinter
from Tkinter import *
import pickle, time
import multiprocessing
import os
from robot import *
import numpy as np
import IPython
from ImageSubscriber import ImageSubscriber
import scipy
import matplotlib.pyplot as plt

def startCallback():
    global record_process, f, f2
    if record_process != None:
        print "You are already recording"
        return
    prs = multiprocessing.Process(target = start_listening)
    # prs.start()
    record_process = prs
    start_listening()

def stopCallback():
    global record_process
    if record_process == None:
        print "Nothing currently recording"
        return
    record_process.terminate()
    record_process = None


def exitCallback():
    global record_process
    if record_process != None:
        record_process.terminate()
    top.destroy()
    f.close()
    f2.close()
    sys.exit()


def start_listening(interval=.01):
    pos1, pos2 = None, None
    grip1, grip2 = None, None
    directory = E.get()
    if not os.path.exists(directory):
        os.makedirs(directory)
        os.makedirs(directory + "/left_endoscope")
        os.makedirs(directory + "/right_endoscope")
        os.makedirs(directory + "/ADleft")
        os.makedirs(directory + "/ADright")
    open(directory + "/psm1.p", "w+").close()
    open(directory + "/psm2.p", "w+").close()

    imgsub = ImageSubscriber()
    adsub = ImageSubscriber(AD=True)
    time.sleep(2)
    count = 0

    while True:
        f = open(directory + "/psm1.p", "a")
        f2 = open(directory + "/psm2.p", "a")
        t = time.time()
        pose1 = psm1.get_current_cartesian_position()
        pose2 = psm2.get_current_cartesian_position()
        pos1 = pose1.position
        pos2 = pose2.position
        rot1 = [pose1.tb_angles.yaw_deg, pose1.tb_angles.pitch_deg, pose1.tb_angles.roll_deg]
        rot2 = [pose2.tb_angles.yaw_deg, pose2.tb_angles.pitch_deg, pose2.tb_angles.roll_deg]

        grip1 = [psm1.get_current_joint_position()[-1] * 180 / np.pi]
        grip2 = [psm2.get_current_joint_position()[-1] * 180 / np.pi]
        one = [t] + list(pos1) + rot1 + list(grip1)
        two = [t] + list(pos2) + rot2 + list(grip2)
        pickle.dump(one, f)
        pickle.dump(two, f2)

        f.close()
        f2.close()

        scipy.misc.imsave(directory + "/left_endoscope/" + str(count) + '.jpg', imgsub.left_image)
        scipy.misc.imsave(directory + "/right_endoscope/" + str(count) + '.jpg', imgsub.right_image)
        scipy.misc.imsave(directory + "/ADleft/" + str(count) + '.jpg', adsub.left_image)
        scipy.misc.imsave(directory + "/ADright/" + str(count) + '.jpg', adsub.right_image)

        count += 1

        time.sleep(interval)

def read_file(fname):
    lst = []
    f3 = open(fname, "rb")
    while True:
        try:
            pos2 = pickle.load(f3)
            lst.append(pos2)
        except EOFError:
            f3.close()
            return np.matrix(lst)



if __name__ == '__main__':

    psm1 = robot("PSM1")
    psm2 = robot("PSM2")

    f, f2 = None, None
    directory = "default"
    record_process = None

    top = Tkinter.Tk()
    top.title('Pose Listener')
    top.geometry('400x200')


    B = Tkinter.Button(top, text="Start Recording", command = startCallback)
    C = Tkinter.Button(top, text="Stop Recording", command = stopCallback)
    D = Tkinter.Button(top, text="Exit", command = exitCallback)
    E = Entry(top)


    B.pack()
    C.pack()
    D.pack()
    E.pack()

    E.delete(0, END)
    E.insert(0, "default")

    top.mainloop()

    print read_file("default/psm1.p").shape
    print read_file("default/psm2.p").shape
