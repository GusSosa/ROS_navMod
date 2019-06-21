#!/usr/bin/env python

# This data logger saves published messages from the inverse kinematics commands,
# specifically, the state that correlates to a command at a particular time.
# Output is a CSV file with the states recorded, alongside timestamps
# (timestamps are, for now, in seconds of the system clock.)

import roslib
import rospy
import sys
# for brevity
from datetime import date
from datetime import time
from datetime import datetime
import numpy as np
from spine_controller.msg import InvkinControlCommand

class IKDataLogger:

    # Some functions to calculate timestamps/
    # First, milliseconds since midnight.
    # This makes it easiest to correlate timestamps when post-processing,
    # Rounded to an int.
    # Thanks to https://stackoverflow.com/questions/15971308/get-seconds-since-midnight-in-python
    def get_time_since_midnight(self):
        # get the current time (clock ticks of computer)
        now = datetime.now()
        # subtract from midnight. Midnight is 
        midnight = now.replace(hour=0, minute=0, second=0, microsecond=0)
        # total microseconds, according to system clock, since midnight.
        # Is a float.
        usec = (now - midnight).total_seconds()
        # Turn into a in, with microseconds only.
        msec = int(round(usec * 1000))
        return msec

    # This one returns a string of the date and starting time, for use
    # with file naming.
    def get_datetime_string(self):
        # Should be of the form
        # Year-Month-Day_HourMinSec
        t = date.today()
        stampdate = str(t.year) + "-" + str(t.month) + "-" + str(t.day)
        # Now, for the time itself
        now = datetime.now()
        stamptime = now.strftime("%H%M%S")
        # the full datetime timestamp is
        stamp = stampdate + "_" + stamptime
        return stamp

    # The function that actually logs the data.
    def log_callback(self, message):
        # print(data.rotation)
        # print(data.com1)
        # print(data.com2)
        # rotation_angle = data.data[4]
        # pos_data = data.data[0:4].reshape((2, 2))
        # Open the file, append mode, and binary open so we can use savetxt
        f = open(self.filename, 'ab')
        # Construct a comma-separated row to save.
        # Should be timestamp, CoM X, CoM Y, Rotation
        # The CoM is assumed to be the first region of interest, CoM 1, which has two elements.
        #row = str(self.get_time_since_midnight()) + "," + str(data.com1[0]) + "," \
                  #+ str(data.com1[1]) + "," + str(data.rotation) + "\n"
        # the message field we're interested in is the states
        state = np.array(message.invkin_ref_state)
        # Turn the array into a comma separated string
        # Thanks to https://stackoverflow.com/questions/2721521/fastest-way-to-generate-delimited-string-from-1d-numpy-array
        state_chars = np.char.mod('%f', state)
        # combine w/timestamp into a string. needs a newline too
        row = str(self.get_time_since_midnight()) + "," + ",".join(state_chars) + '\n'
        print(row)
        # write the file. It may have been easier to use np.savetxt but whatever
        f.write(row)
        f.close()

    # a helper function for startup, called by constructor
    def datalogger_startup(self, file_folder):
        # hard-code the topic name
        topic_name = 'invkin_tx_commands'
        # create the ROS node that will subscribe to the InvkinControlCommand messages.
        rospy.init_node('invkin_datalogger', anonymous=False)
        rospy.Subscriber(topic_name, InvkinControlCommand, self.log_callback)
        # Create the filename. It's a concatenation of the folder with a descriptive name,
        # and a timestamp.
        filename = file_folder + "/invkin_datalogger_" + self.get_datetime_string() + ".csv"
        print("Running invkin_datalogger node. Writing to file " + filename
              + " from topic " + topic_name)
        # Open the file and put a header.
        f = open(filename, 'w')
        f.write("Invkin Data Logger started on:," + self.get_datetime_string())
        f.write("\n")
        #f.write("Timestamp (millisec since midnight today), CoM X (cm), CoM Y (cm), Rotation (deg)\n")
        f.write("Timestamp (millisec since midnight today),States per body - x y rot\n")
        f.close()
        return filename

    # the constructor initializes everything. 
    # Here is where the file handle is created.
    def __init__(self, file_folder):
        # Save a reference to the file name so the callback can append new data.
        self.filename = self.datalogger_startup(file_folder)

# the main function: create one of these objects, while parsing the file path
# Arguments: topic_name = name of topic (invkin_tx_commands for now)
#            file_folder = folder to create the log. Should NOT end with trailing "/".
if __name__ == '__main__':
    # the 0-th arg is the name of the file itself, so we want the 1st and 2nd
    logger = IKDataLogger(sys.argv[1], )
    # We do the spin() in main. It's not appropriate for a constructor.
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Stopping data collection.\n")
        pass

