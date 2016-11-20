#!/usr/bin/python

import rospy
from std_msgs.msg import Int32

def talker():
    pub = rospy.Publisher('WhatEverJoSend', Int32,queue_size=10)
    rospy.init_node('test_talker',anonymous=True)
    empty = raw_input("Press enter to continue")
    msg = 0
    while not rospy.is_shutdown():
        #rospy.loginfo(nx,ny)
        pub.publish(msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
	pass

