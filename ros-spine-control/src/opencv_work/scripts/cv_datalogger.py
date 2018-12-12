#!/usr/bin/env python

# This data logger saves published messages from the computer vision
# framework for the spine state.
# Output is a CSV file with the states recorded, alongside timestamps
# (timestamps are, for now, in seconds of the system clock.)

import roslib
import rospy
import sys
import numpy as np
from opencv_work.msg import SpineState

# This apparently isn't needed when we use catkin.
#roslib.load_manifest('opencv_work')

class CVDataLogger:

    # The function that actually logs the data.
    def log_callback(self, data):
        # print(data.rotation)
        # print(data.com1)
        # print(data.com2)
        # rotation_angle = data.data[4]
        # pos_data = data.data[0:4].reshape((2, 2))
        print('Position Data: \n' + str(data.com1) + '\n' + str(data.com2) + '\n' + 'Rotation Angle: \n' + str(data.rotation) + '\n')

    # a helper function for startup, called by constructor
    def datalogger_startup(self, topic_name, file_name):
        # create the ROS node that will subscribe to the SpineState messages.
        rospy.init_node('cv_datalogger', anonymous=False)
        rospy.Subscriber(topic_name, SpineState, self.log_callback)
        print(file_name)

    # the constructor initializes everything. 
    # Here is where the file handle is created.
    def __init__(self, topic_name, file_name):
        #self.file = self.datalogger_startup(topic_name, file_name)
        self.datalogger_startup(topic_name, file_name)

# the main function: create one of these objects, while parsing the file path
if __name__ == '__main__':
    # the 0-th arg is the name of the file itself, so we want the 1st and 2nd
    logger = CVDataLogger(sys.argv[1], sys.argv[2])
    # We do the spin() in main. It's not appropriate for a constructor.
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

