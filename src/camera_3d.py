#!/usr/bin/python
from __future__ import print_function
import roslib
roslib.load_manifest('ros_3d_cartography')
import cv2 # OpenCV2 for saving an image
from object_detection.ModuleDetection import SplitRosFile,ImageAnalysis
import sys
import os
import time
import rospy # rospy for the subscriber
import time

from sensor_msgs.msg import Image # ROS image message
from std_msgs.msg import String, Int32
from biobot_ros_msgs.msg import IntList
from cv_bridge import CvBridge, CvBridgeError # ROS Image message -> OpenCV2 image converter
# Imports for Barrier class
from threading import Thread,Semaphore
ws_dir = os.path.join('/home','ubuntu','biobot_ros_jtk','src','ros_3d_cartography','src','object_detection')
os.chdir(ws_dir)

class Barrier:
    """ Barrier class is used to wait until the 2D image is 
    correctly saved in jpg format before continuing the analysis.
    To pass the barrier, both function must call it 
    (callback_command & callback_3d_capture)
    """
    def __init__(self, n):
        """ 
        Built the Barrier Object.
        :param n: Number of elements to be connected to the Barrier
                    before continuing the code.
        :return: returns nothing
        """
        self.n = n
        self.count = 0
        self.mutex = Semaphore(1)
        self.barrier = Semaphore(0)

    def wait(self):
        """
        Wait until every called to Barrier were made.
        """
        self.mutex.acquire()
        self.count = self.count + 1
        self.mutex.release()
        if self.count == self.n: self.barrier.release()
        self.barrier.acquire()
        self.barrier.release()


class Camera3d:

    def __init__(self):
        """
        Initialise Camera3d object. It starts relevant subscriber and publisher needed
        for the 3D cartography to work properly.
        

        Publisher: 
        Subscriber: /Do_cartography => Send command to start analysis using the nx/ny value [square number to be analysed]
                    /rgb_image      => 2D image from DS325
        """
        self.node_name = self.__class__.__name__
        print("init")
        #self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.bridge = CvBridge()
        # ROS subscriptions
        self.image_sub = rospy.Subscriber('/rgb_image',Image,self.callback_3d_capture) # To camera only
        self.ros_command = rospy.Subscriber('/Do_Cartography', IntList ,self.callback_command)
        self.commandFlag = 0 # Flag to save 2D image from 3D camera to jpg format
        self.commandImageReady = 0 # Flag telling the 2D image has been saved

        # Cartography publisher (when analysis is done)
        self.done_module = rospy.Publisher('Done_Module', String, queue_size=10)

        self.synchronisation = Barrier(2) # Create a barrier IOT leave time to save jpg file and perform analysis on relevant files.
        print("Init done")
        return

    def imageAnalysis(self, nx, ny):
        """ imageAnalysis calls function from ModuleDetection.
        - SplitRosFile -> split the .txt file into x y z r g b data. Then, an octave file is called
            IOT save those data into one .mat file
            INPUT : file name / path where x y z r g b data will be stored
            OUTPUT: .mat file (example.mat)
        - ImageAnalysis -> Perform 2D and 3D analysis on the current data set (octave file)
            INPUT : Square coordinate (nx / ny) and file (.jpg and .mat ) name. Both name must be the same
            OUTPUT: If something is found, data are stored in the DB

        
         +---------------------------------------+         -----> x.
         |   nx,ny: |          |                 |         | \      
         |   0,0    |    1,0   |   ...           |         |  \     
         |__________|__________|                 |         V   v  z.
         |          |                            |        y.        
         |   0,1    |                            |
         | _________|                            |
         |          |                            |
         |    ...   |                            |
         |          |                            |
         +---------------------------------------+

        """

        print("Analysing image")
        SplitRosFile('example.txt','/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src/data')
        print("IMageAnal")
        ImageAnalysis(nx,ny,'example') #position on deck, file name (txt AND image)
        self.done_module.publish(self.node_name)
        print("Image analysed")
        return


    def callback_command(self, msg):
        """ Callback called each time a demand for the cartography analysis.
        """
        self.commandFlag = 1
        self.synchronisation.wait() # Wait until image is saved (both need to be connecte IOT continue)
        while self.commandImageReady == 0:
            time.sleep(0.1)
        nx = msg.data[0]
        ny = msg.data[1]
        self.imageAnalysis(nx,ny)
        self.commandImageReady = 0

        return

    def callback_3d_capture(self, msg):
        """ Callback called each time an image is available from the DS325
        """
        #print("Received an image from DS325")
        if self.commandFlag == 1:
            try:
                os.remove(os.path.join('/home/ubuntu/.ros/example.jpg'))
            except:
                print('Nothing to delete jpg')
            try:
                os.remove(os.path.join('/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src/data/x'))
                os.remove(os.path.join('/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src/data/y'))
                os.remove(os.path.join('/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src/data/z'))
                os.remove(os.path.join('/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src/data/r'))
                os.remove(os.path.join('/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src/data/g'))
                os.remove(os.path.join('/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src/data/b'))
            except:
                print('Nothing to delete rgbxyz')

            try:
                os.remove(os.path.join('/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src/object_detection/example.mat'))
            except:
                print('Nothing to delete mat')


            try:
                # Convert ROS image message to OpenCV2
                cv2_img = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            except CvBridgeError,e:
                print(e)
            try:
                cv2.imwrite('/home/ubuntu/.ros/example.jpg',cv2_img)
                self.commandImageReady = 1  # Flag telling that an image has just been saved
                #cv2.imshow("Image received through ROS",cv2_img)
                #cv2.waitKey(3)
                while not os.path.isfile('/home/ubuntu/.ros/example.jpg'):
                    time.sleep(0.1)
                self.synchronisation.wait() # Wait for callback_command() to connect
            except:
                print("Image not yet ready")
        self.commandFlag = 0
        return

def main(args):
    ic = Camera3d()
    rospy.init_node('Camera3d',anonymous=True)
    try:
        rospy.spin() # Spin until crtl+c

    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
