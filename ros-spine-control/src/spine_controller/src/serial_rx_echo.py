#!/usr/bin/python

# NOTE: on Ubuntu 18.04, specifying /usr/bin/env python uses python 3,
# while specifying /usr/bin/python uses python 2.7. We need 2.7.

# serial_rx_echo outputs any lines from a specified serial port out to the terminal.
# Usage:
#	rosrun spine_controller serial_rx_echo path-to-serial-device

# Imports:
import rospy
# because we need the command-line arguments
import sys
# and for the serial/usb/uart via pyserial:
import serial
# We'll also be echoing messages to a ros topic.
from std_msgs.msg import String

# The primary helper function here opens the serial device,
# and iteratively reads lines from it until stopped.
# frustratingly enough, hardware interrupts are difficult on linux, so we poll the device at some interval


def echo_to_terminal(device_name):
    # A welcome message
    print("Running serial_rx_echo node with device: " + device_name)
    #print(" and python version:")
    # print(sys.version)
    # Hard-code a timeout for pyserial. This way, we can capture keyboard interrupts.
    # In seconds, presumably.
    serial_timeout = 1
    # First, start up the ros node.
    rospy.init_node('serial_rx_echo', anonymous=False)
    # We'll publish to a topic as well as echo to the terminal.
    pub = rospy.Publisher('serial_rx_echo', String, queue_size=10)
    # Next, do the serial setup:
    # Hard-coded: our PSoC uses the following baud rate:
    psoc_baud = 115200
    # create the serial port object
    serial_port = serial.Serial(device_name, psoc_baud, timeout=serial_timeout)
    # flush out any old data
    serial_port.reset_input_buffer()
    serial_port.reset_output_buffer()
    # finishing setup.
    print("Opened port, now echoing. Ctrl-C to stop.")

    # Instead of an infinite loop, use ROS's shutdown procedure.
    while not rospy.is_shutdown():
        # blocking-ly read from the port
        # BUT also need to catch keyboard interrupts.
        try:
            from_psoc = serial_port.readline()
            # If timed out, this call returns an empty string.
            # So, don't push anything. The string is overloaded as a boolean here.
            if from_psoc:
                # Echo the input back to the terminal
                print(from_psoc)
                # and publish to the topic.
                pub.publish(from_psoc)
        except rospy.ROSInterruptException:
            # is this ever executed?
            print("Shutting down serial_rx_echo...")
            pass


# the main function: just call the helper, while parsing the serial port path.
if __name__ == '__main__':
    try:
        # the 0-th arg is the name of the file itself, so we want the 1st.
        echo_to_terminal(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
