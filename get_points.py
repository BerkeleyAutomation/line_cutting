import cv2
import rospy, pickle, time
from robot import *
from geometry_msgs.msg import Pose
import numpy as np
# import PyKDL
import multiprocessing
import tfx
import fitplane
from scipy.interpolate import interp1d
from shape_tracer import plot_points
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
import notch
from geometry_msgs.msg import Point
from image_saver import ImageSaver
import least_square_circle as sqcirc
from mpl_toolkits.mplot3d import Axes3D
from ImageSubscriber import ImageSubscriber
	
def process_img(fname):
	""" converts image to a binary img and thins a little"""
	img = cv2.imread(fname,1)
	resized=cv2.resize(img,None,fx=.5, fy=.5, interpolation = cv2.INTER_CUBIC)

	print resized.shape
	gray=cv2.cvtColor(resized,cv2.COLOR_BGR2GRAY)
	ret,thresh2 = cv2.threshold(gray,127,255,cv2.THRESH_BINARY_INV)
	kernel = np.ones((8,8),np.uint8)
	closing = cv2.morphologyEx(thresh2, cv2.MORPH_CLOSE, kernel)
	erosion = cv2.erode(closing,kernel,iterations = 1)
	final = cv2.morphologyEx(erosion, cv2.MORPH_CLOSE, kernel)
	return final




def get_raw_points(img):
	"""finds coordinates of all lit up spots"""
	pts=[]
	x=[]
	y=[]
	for i in range(img.shape[0]):
		for j in range(img.shape[1]):
			if img[i][j]==255:
				pts.append([i,j])
				x.append(i)
				y.append(j)
	return pts,x,y
def center_and_radius(x,y):
	#finds radius and center based on circular regression
	x,y,R,residu= sqcirc.leastsq_circle(x,y)
	return x,y,R


def get_circle(x,y,R):
	#uses radius and center to make 100 points to define the circle
	top=np.linspace(0,np.pi,num=8)
	bottom=np.linspace(0,-np.pi,num=8)
	topx,topy=R*np.cos(top)+x,R*np.sin(top)+y
	bottomx,bottomy=R*np.cos(bottom)+x,R*np.sin(bottom)+y
	return topx,topy,bottomx,bottomy


def fit_plane(topx,topy,botx,boty,scale=.5,z=69):#scale and z needs to be experimentally gotten
	"""fits to the frame of the robot and the plane of the gauss"""
	top_pts=[]
	bot_pts=[]
	for i in range(topx.size):
		top_pts.append([topx[i]*scale,topy[i]*scale,z])
		bot_pts.append([botx[i]*scale,boty[i]*scale,z])

	top,bot=np.matrix(top_pts),np.matrix(bot_pts)
	return top,bot

def plot_points(top,bottom):
	
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d') 
	ax.scatter(np.array(top[:,0]),np.array(top[:,1]),np.array(top[:,2]))
	ax.scatter(np.array(bottom[:,0]),np.array(bottom[:,1]),np.array(bottom[:,2]))
	plt.show()
def main():
	a=ImageSubscriber()
	cv2.imwrite('image_utils/left1.jpg',a.left_image)
	processed=process_img('image_utils/left1.jpg')
	#processed=process_img('image_utils/right1.jpg')
	cv2.imshow('processed',processed)
	print processed.shape
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	pts,x,y=get_raw_points(processed)
	xc,yc,R=center_and_radius(x,y)
	print "raw radius=",R
	topx,topy,botx,boty=get_circle(xc,yc,R)
	plt.axis([0,500,0,500])
	plt.scatter(x,y,color='b')
	plt.scatter(topx,topy,c='r')
	plt.scatter(botx,boty,c='r')
	plt.show()
	top,bottom=fit_plane(topx,topy,botx,boty)
	np.savetxt('debug.txt',top)
	plot_points(top,bottom)
	
	
	


if __name__ == '__main__':
	main()