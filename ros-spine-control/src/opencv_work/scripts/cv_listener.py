#!/usr/bin/env python

# Simple talker demo that listens to numpy_msg(Floats) published
# to the 'cv_data' topic

import roslib
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
# import csv
from opencv_work.msg import SpineState

roslib.load_manifest('opencv_work')


def callback(data):

    print(data.rotation)
    print(data.com1)
    print(data.com2)
    # rotation_angle = data.data[4]
    # pos_data = data.data[0:4].reshape((2, 2))

    # print('Position Data: \n' + str(pos_data) + '\n' + 'Rotation Angle: \n' + str(rotation_angle) + '\n')

    # rospy.loginfo(data)


# def csv_writer(full_data, path)

#     with open(path if path is not None else raw_input('Input new file name and press <Enter>')) as csv_file
#         writer = csv.writer(csv_file, delimiter='')
#         for data_line in full_data:
#             writer.writerow(data_line)


def listener():

    # Launch node as 'cv_listener' and subscribe to topic, 'cv_data'
    rospy.init_node('cv_listener', anonymous=True)
    rospy.Subscriber('cv_data', SpineState, callback)
    # rospy.Subscriber('cv_data', numpy_msg(Floats), callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
