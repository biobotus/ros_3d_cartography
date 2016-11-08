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
from cv_bridge import CvBridge, CvBridgeError # ROS Image message -> OpenCV2 image converter

class camera3d:

    def __init__(self):
        # ROS init
	print("start init")
	self.image_pub = rospy.Publisher("image_topic_2",Image)
	self.bridge = CvBridge()

	#self.node_name = self.__class__.__name__
        #rospy.init_node(self.node_name, anonymous=True)
        #self.rate = rospy.Rate(10) # 10Hz

        # ROS subscriptions
        self.image_sub = rospy.Subscriber('/rgb_image',Image,self.callback_3d_capture)
	print("init done")
	return

    def get_PCL(self):
        # Create call to pcl_2_windows+modify relevant files
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


    def callback_3d_capture(self, msg):
	dump = raw_input("Press Enter to coninue")
        #print("Received an image from DS325")
        try:
                # Convert ROS image message to OpenCV2
                cv2_img = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        except CvBridgeError,e:
                print(e)
        try:
                cv2.imwrite('/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src/imageFrom3D.jpeg',cv2_img)
                cv2.imshow("Image window",cv2_img)
                cv2.waitKey(3)
        except:
                print("Image not yet ready")
	self.imageAnalysis(0,0)
def main(args):
    ic = camera3d()
    rospy.init_node('camera3d',anonymous=True)
    # Define your image topic
    nx = 0
    ny = 0
    #ic.imageAnalysis(nx,ny)
    print("bib")
    try:
	rospy.spin() # Spin until crtl+c

    except KeyboardInterrupt:
	print("Shutting down")
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main(sys.argv)
