#!/usr/bin/python
from __future__ import print_function
import roslib
roslib.load_manifest('ros_3d_cartography')
import cv2 # OpenCV2 for saving an image
import ModuleDetection
import sys
import os
import rospy # rospy for the subscriber
from sensor_msgs.msg import Image # ROS image message
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge, CvBridgeError # ROS Image message -> OpenCV2 image converter
# Imports for Barrier class
import time
from threading import Thread,Semaphore


class Barrier:
    def __init__(self, n):
        self.n = n
        self.count = 0
        self.mutex = Semaphore(1)
        self.barrier = Semaphore(0)

    def wait(self):
        self.mutex.acquire()
        self.count = self.count + 1
        self.mutex.release()
        if self.count == self.n: self.barrier.release()
        self.barrier.acquire()
        self.barrier.release()


class camera3d:

    def __init__(self):
        print("init")
	#self.image_pub = rospy.Publisher("image_topic_2",Image)
	self.bridge = CvBridge()

        # ROS subscriptions
        self.image_sub = rospy.Subscriber('/rgb_image',Image,self.callback_3d_capture) # To camera only
	self.ros_command = rospy.Subscriber('/WhatEverJoSend', Int32 ,self.callback_command)
	self.commandFlag = 0 # Flag to save 2D image from 3D camera to jpg format
	self.commandImageReady = 0 # Flag telling the 2D image has been saved
	self.synchronisation = Barrier(2) # Create a barrier IOT leave time to save jpg file and perform analysis on relevant files.	
	print("Init done")
	return

    def get_PCL(self):
        # Create call to pcl_2_windows+modify relevant files
	# Files are moved into a folder directly from the function instead
	return

    def imageAnalysis(self, nx, ny):
	# +---------------------------------------+         -----> x.
        # |   nx,ny: |          |                 |         | \
        # |   1,1    |    2,1   |   ...           |         |  \ 
        # |__________|__________|                 |         V   v  z.
        # |          |                            |          y.
        # |   1,2    |                            |
        # |          |                            |
        # |          |                            |
        # |          |                            |
        # +---------------------------------------+

	print("Analysing image")
        ModuleDetection.SplitRosFile('example.txt','/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src/data')
        ModuleDetection.ImageAnalysis(nx,ny,'example') #position on deck, file name (txt AND image)
	print("Image analysed")
	return


    def callback_command(self, msg):
	print("Start callback_command")
	self.commandFlag = 1
	self.synchronisation.wait() # Wait until image is saved (both need to be connecte IOT continue)
	print("Stop waiting")
	if self.commandImageReady == 1:
		print(msg)
		ny = msg
		nx = ny
		self.imageAnalysis(nx,ny)        
	else:
		print("No resquest to save 2D image")

	return

    def callback_3d_capture(self, msg):
	#dump = raw_input("Press Enter to coninue")
	#print("image received from DS325")
        if self.commandFlag == 1:
		try:
                	# Convert ROS image message to OpenCV2
	                cv2_img = self.bridge.imgmsg_to_cv2(msg,"bgr8")
	        except CvBridgeError,e:
        	        print(e)
	        try:
	                cv2.imwrite('/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src/example.jpg',cv2_img)
			self.commandImageReady = 1 # Flag telling that an image has just been saved
        	        cv2.imshow("Image received through ROS",cv2_img)
                	cv2.waitKey(3)
			self.synchronisation.wait() # Wait for callback_command() to connect
	        except:
        	        print("Image not yet ready")
	else: pass
	return	

def main(args):
    #os.system("rosrun ros_depthsense_camera depthsense_camera_node")
    #os.system("rosrun pcl_to_windows pcl_xyzrgb")
    print("Node started")
    ic = camera3d()
    rospy.init_node('camera3d',anonymous=True)
    # Define your image topic
    try:
	rospy.spin() # Spin until crtl+c

    except KeyboardInterrupt:
	print("Shutting down")
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main(sys.argv)
