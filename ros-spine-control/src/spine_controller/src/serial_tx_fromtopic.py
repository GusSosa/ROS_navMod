#!/usr/bin/python

# NOTE: on Ubuntu 18.04, specifying /usr/bin/env python uses python 3,
# while specifying /usr/bin/python uses python 2.7. We need 2.7.

# serial_tx_fromtopic writes messages from a topic to the serial port.
# Usage is:
#	rosrun spine_controller serial_tx_fromtopic device-name topic-name
# Right now, this subscriber node only works for messages of type Float32MultiArray,
# since that's what's used in the invkin_tx_totopic node.
# We're also doing a publisher here, so we can confirm what's echoed out to the serial.

# Imports:
import rospy
# because we need the command-line arguments
import sys
# and for the serial/usb/uart via pyserial:
import serial
# need to manipulate arrays
import numpy as np
# We'll also be echoing messages to a ros topic.
# There's a nice message spec: numpy_msg
from rospy.numpy_msg import numpy_msg
# but that needs to wrap around a standard message type.
# Seems that arrays are in the *MultiArray messages.
# We don't need 64 bits because the invkin output is only to a few decimal places
#from std_msgs.msg import Float32MultiArray
# Now we've got our own message type.
from spine_controller.msg import InvkinControlCommand
# also need to echo back a string of the formatted output to the serial port.
from std_msgs.msg import String

# Note: in order to do publishing within a callback, we need to pass around
# the publisher object, which is done much easier by making this a class.


class SerialTxFromTopic:

    # The callback function here does all the heavy lifting.
    def serial_tx_callback(self, message):
        # When a message is received:
        # 1) format the string to send to the PSoC
        # 2) actually send the message
        # 3) publish the formatted string back out to another debugging topic

        # The ndarray is in
        #invkin_command = np.array(message.data)
        # Let's do a string with the following format, which seems to work OK on the PSoC:
        # u (rl1) (rl2) ... (rln)
        # For example,
        # u 0.02 0.476 0.87 0.05
        # The PSoC will then parse the first character as a command (u = this is a control input)
        # and then the remaining numbers with spaces between them.

        # Now using our own message type:
        invkin_command = np.array(message.invkin_control)

        # A discussion on the length of the string:
        # It may be bad to do more than a 32-character message over UART, that's pretty long.
        # So if we've got 4 cables with 4 colons and a "u", that leaves (32 - 5)/4 = 6 characters per
        # control input. That's 4 digits after the decimal (since we'll never have more than a 1 meter command.)

        # As of 2018-11-29, the PSoC code has a 128-bit receive buffer for UART strings.
        # That means we can do much longer strings. Arbitrarily, choose 6 decimal places.
        # Check: something something single-precision floating point??
        # (again recalling that our invkin outputs are in meters, which will never be greater than 1,
        # so the floats will always be 0.something.)

        # We seem to be getting some mis-aligned commands.
        # So, before anything else, send out a "clear" every time.
        self.serial_port.write("\n")
        # give the PSoC a moment
        # maybe 20 ms?
        rospy.sleep(0.02)
        self.serial_port.write("c\n")
        rospy.sleep(0.02)
        self.serial_port.write("c\n")
        rospy.sleep(0.02)

        # Thanks to our friends on stackoverflow (https://stackoverflow.com/questions/21008858/formatting-floats-in-a-numpy-array),
        # a nice way to format w/ only certain precision is
        def ik_cmd_formatter(x): return "%.8f" % x
        # The result string will be
        cmd_string = "u"
        # and we can concatenate each float to it.
        for i in range(invkin_command.shape[-1]):
            # Add to the command string, with a preceeding space
            cmd_string += " " + str(ik_cmd_formatter(invkin_command[i]))
        # tack on a newline, since the PSoC requires that.
        cmd_string += "\n"

        # Send the message
        self.serial_port.write(cmd_string)

        # And echo back.
        self.pub.publish(cmd_string)

    # The primary helper function here opens the serial device,
    # subscribes to a topic, writes when new data appears on the topic, and
    # echoes (publishes) its pushed data back out on *another* topic for debugging.
    def serial_tx_startup(self, device_name):
        # A welcome message
        # Hard-coding the topic name, doesn't make sense to need to pass it in each time.
        topic_name = 'invkin_tx_commands'
        print("Running serial_tx_fromtopic node with device " + device_name
              + " and topic " + topic_name)
        #print(" and python version:")
        # print(sys.version)
        # Hard-code a timeout for pyserial. Seems recommended, even for tx?
        serial_timeout = 1
        # First, start up the ros node.
        rospy.init_node('serial_tx_fromtopic', anonymous=False)
        # The main functionality here is a subscriber.
        #sub = rospy.Subscriber(topic_name, Float32MultiArray, self.serial_tx_callback)
        sub = rospy.Subscriber(topic_name, InvkinControlCommand, self.serial_tx_callback)
        # We'll publish commands to a topic just in case someone else wants to use them
        pub = rospy.Publisher('serial_tx_echo', String, queue_size=10)
        # Next, do the serial setup:
        # Hard-coded: our PSoC uses the following baud rate:
        psoc_baud = 115200
        # create the serial port object
        serial_port = serial.Serial(device_name, psoc_baud, timeout=serial_timeout)
        # flush out any old data
        serial_port.reset_input_buffer()
        serial_port.reset_output_buffer()
        # finishing setup.
        print("Opened port, subscriber created. Now echoing from topic to serial.")
        # and return the publisher so that the callback can use it.
        # also needs to have the serial port available to the callback.
        return serial_port, pub

    # The constructor calls a helper to initialize everything, and stores the
    # resulting publisher and serial port object that's created.
    def __init__(self, device_name):
        self.serial_port, self.pub = self.serial_tx_startup(device_name)
        # and that's all.


# the main function: create one of these objects, while parsing the serial port path
# and topic name to subscribe to.
if __name__ == '__main__':
    # the 0-th arg is the name of the file itself, so we want the 1st and 2nd
    # We're making this a class now.
    s_tx = SerialTxFromTopic(sys.argv[1])
    # We do the spin() in main. It's not appropriate for a constructor.
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
