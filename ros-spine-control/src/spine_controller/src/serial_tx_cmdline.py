#!/usr/bin/python

# NOTE: on Ubuntu 18.04, specifying /usr/bin/env python uses python 3,
# while specifying /usr/bin/python uses python 2.7. We need 2.7.

# serial_tx_cmdline writes lines from the command line to the serial port.
# Usage is:
#	rosrun spine_controller serial_tx_cmdline path-to-serial-device
# and then lines are put to the serial device upon carraige return/newline or whatever.

# Imports:
import rospy
# because we need the command-line arguments
import sys
# and for the serial/usb/uart via pyserial:
import serial
# We'll also be echoing messages to a ros topic.
from std_msgs.msg import String

# The primary helper function here opens the serial device,
# and writes to it from raw_input.


def tx_to_serial(device_name):
    # A welcome message
    print("Running serial_tx_cmdline node with device: " + device_name)
    #print(" and python version:")
    # print(sys.version)
    # Hard-code a timeout for pyserial. Seems recommended, even for tx?
    serial_timeout = 1
    # First, start up the ros node.
    rospy.init_node('serial_tx_cmdline', anonymous=False)
    # We'll publish commands to a topic just in case someone else wants to use them
    pub = rospy.Publisher('serial_tx_cmdline', String, queue_size=10)
    # Next, do the serial setup:
    # Hard-coded: our PSoC uses the following baud rate:
    psoc_baud = 115200
    # create the serial port object, non-exclusive (so others can use it too)
    serial_port = serial.Serial(device_name, psoc_baud, timeout=serial_timeout, exclusive=False)
    # flush out any old data
    serial_port.reset_input_buffer()
    serial_port.reset_output_buffer()
    # finishing setup.
    print("Opened port. Ctrl-C to stop.")

    # Instead of an infinite loop, use ROS's shutdown procedure.
    while not rospy.is_shutdown():
        # request something to send
        try:
            to_psoc = raw_input("Message to send over serial terminal: ")
            # Concatenate a newline so that the psoc calls its command parser
            to_psoc += '\n'
            serial_port.write(to_psoc)
            # and publish to the topic, too.
            pub.publish(to_psoc)
        except KeyboardInterrupt:
            # Check: if we get the shutdown signal...
            # TO-DO: this doesn't seem to be working. Luckily enough, the PSoC
            # doesn't do Ctrl-C as a command, so it's ugly but safe to just not
            # deal with keyboard interrupts and let rospy handle it.
            rospy.signal_shutdown("Shutting down, Ctrl-C received.")



# the main function: just call the helper, while parsing the serial port path.
if __name__ == '__main__':
    try:
        # the 0-th arg is the name of the file itself, so we want the 1st.
        tx_to_serial(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
