#!/usr/bin/python

# NOTE: on Ubuntu 18.04, specifying /usr/bin/env python uses python 3,
# while specifying /usr/bin/python uses python 2.7. We need 2.7.

# invkin_tx_totopic reads in an inverse kinematics calculation file from the tIK library
# and publishes those commands to a topic at a specified interval.
# Usage is:
#	rosrun spine_controller invkin_tx_totopic path-to-file

# Imports:
import rospy
# because we need the command-line arguments
import sys
# We'll also be echoing messages to a ros topic.
# There's a nice message spec: numpy_msg
from rospy.numpy_msg import numpy_msg
import numpy as np
# but that needs to wrap around a standard message type.
# Seems that arrays are in the *MultiArray messages.
# We don't need 64 bits because the invkin output is only to a few decimal places
from std_msgs.msg import Float32MultiArray
# Now, we've got our own message type.
from spine_controller.msg import InvkinControlCommand

# The primary helper function here opens the csv file,
# stores the results in a numpy array, and publishes rows of that array
# to a topic at a hard-coded interval


def tx_to_topic(file_name):
    # A welcome message
    print("Running invkin_tx_totopic with file: " + file_name)
    #print("and python version:")
    # print(sys.version)
    # First, start up the ros node.
    rospy.init_node('invkin_tx_commands', anonymous=False)
    # We need a publisher. Note we're using the numpy message type, wrapping
    # around the standard message type
    #pub = rospy.Publisher('invkin_tx_commands', numpy_msg(Float32MultiArray), queue_size=10)
    pub = rospy.Publisher('invkin_tx_commands', InvkinControlCommand, queue_size=10)
    # Then, read in the csv file.
    # The 3rd row (counting from 0) contains the parameters for this run, and is
    # of mixed type: int, string, int, int, int, string
    # let's try with autodetection of type. Only want 1 row.
    invkin_header = np.genfromtxt(file_name, dtype=None, delimiter=",",
                                  skip_header=3, max_rows=1)
    print("Using an inverse kinematics file with the parameters:")
    print(invkin_header)
    # We need to now pick out the number of cables. That's the 4th column
    # on the csv file. Numpy was giving me some trouble but turning the array into
    # a list seemed to help.
    #print(invkin_header.tolist()[0])
    s = invkin_header.tolist()[3]
    # Then, read the data itself. Starts two rows down from header.
    control_data = np.genfromtxt(file_name, dtype=float, delimiter=",",
                                skip_header=5)
    # Split the data into its two parts: 0:s-1 == invkin, remainder == states.
    #print(control_data)
    invkin_data = control_data[:, 0:s]
    # this is numpy notation for MATLAB equivalent control_data(:, s:end)
    state_data = control_data[:, s:]
    # print(invkin_data.shape)
    #print(state_data)

    # Create a timer object that will sleep long enough to result in
    # a reasonable publishing rate
    r = rospy.Rate(0.5)  # hz
    # We'll keep track of rows: we only want to publish until the end of the CSV file.
    # max timestep is number of rows.
    max_timestep = invkin_data.shape[0]
    # initialize the counter
    current_timestep = 0

    # finishing setup.
    print("File loaded into memory. Ctrl-C to stop output.")

    # We iterate through the array until the end
    # but also need to catch shutdown signals.
    while (current_timestep < max_timestep) and not rospy.is_shutdown():
        # Create the message itself
        #to_publish = Float32MultiArray()
        to_publish = InvkinControlCommand(invkin_control = invkin_data[current_timestep, :], \
        	invkin_ref_state = state_data[current_timestep, :])
        # Put in the numpy array
        #to_publish.data = np.array([0.5, 0.5, 0.5, 0.5])
        #to_publish.data = invkin_data[current_timestep, :]
        # Publish the current_timestep-th message
        pub.publish(to_publish)
        # Echo to the terminal
        print("Timestep " + str(current_timestep) + ", publishing:")
        print(to_publish.invkin_control)
        # increment the counter
        current_timestep += 1
        # sleep until the next output
        r.sleep()


# the main function: just call the helper, while parsing the path to the invkin data file.
if __name__ == '__main__':
    try:
        # the 0-th arg is the name of the file itself, so we want the 1st.
        tx_to_topic(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
