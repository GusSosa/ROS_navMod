#!/usr/bin/env python

# Simple talker demo that listens to numpy_msg(Floats) published
# to the 'cv_data' topic

import roslib
import rospy
from opencv_work.msg import SpineState

# This apparently isn't needed when we use catkin.
# roslib.load_manifest('opencv_work')


def callback(data):

    # print(data.rotation)
    # print(data.com1)
    # print(data.com2)
    # rotation_angle = data.data[4]
    # pos_data = data.data[0:4].reshape((2, 2))

    print('Position Data: \n' + str(data.comx) + '\n' + str(data.comy) + '\n' + 'Rotation Angle: \n' + str(data.rotation) + '\n')

    # rospy.loginfo(data)


def listener():

    # Launch node as 'cv_listener' and subscribe to topic, 'cv_data'
    rospy.init_node('cv_listener', anonymous=False)
    rospy.Subscriber('cv_data', SpineState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
