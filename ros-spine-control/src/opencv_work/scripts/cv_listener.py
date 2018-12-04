#!/usr/bin/env python

# Simple talker demo that listens to std_msgs/Strings published
# to the 'chatter' topic

import roslib
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

roslib.load_manifest('opencv_work')


def callback(data):
    print 'COM: %s' % str(data.data)
    # rospy.loginfo('X COM: %f', data.data)
    # rospy.loginfo(data)


def listener():

    # Launch node as 'cv_listener' and subscribe to topic, 'cv_data'
    rospy.init_node('cv_listener', anonymous=True)
    rospy.Subscriber('cv_data', numpy_msg(Floats), callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
