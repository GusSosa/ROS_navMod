#!/usr/bin/env python
# license removed for brevity
import rospy
# because we need the command-line arguments
import sys

from std_msgs.msg import String
from rospy.numpy_msg import numpy_msg

from sensor_msgs.msg import Imu
from turtlesim.msg import Pose
from geometry_msgs.msg import TwistWithCovarianceStamped
# Now we've got our own message type.
from spine_controller.msg import InvkinControlCommand

class SerialTxControlTopic:

  #contador de secuencia por sensor
  seq = 0
  seq2 = 0
  seq3 = 0

  # invkin_ref_state =[device, count])
    # devices:
      # no device:  0
      # laser scan: 1
      # imu:        2
      # wheels:     3

  def invkin_laser_publisher_callback(self, message):
    SerialTxControlTopic.seq += 1
    hello_str = InvkinControlCommand(invkin_control = [message.x, message.y, message.theta, message.linear_velocity, message.angular_velocity], \
          invkin_ref_state =[1, SerialTxControlTopic.seq]) #laser scan = 1
    #hello_str = "invkin_control: 0\n %s" % rospy.get_time()
    #rospy.loginfo(hello_str)
    self.pub.publish(hello_str)

  def invkin_imu_publisher_callback(self, message):
    SerialTxControlTopic.seq2 += 1
    hello_str = InvkinControlCommand(invkin_control = [message.orientation.z, message.orientation.w, message.angular_velocity.z], \
          invkin_ref_state =[2, SerialTxControlTopic.seq2]) #imu = 2
    #hello_str = "invkin_control: 0\n %s" % rospy.get_time()
    #rospy.loginfo(hello_str)
    self.pub.publish(hello_str)

  def invkin_wheel_publisher_callback(self, message):
    SerialTxControlTopic.seq3 += 1
    hello_str = InvkinControlCommand(invkin_control = [message.twist.twist.linear.x, message.twist.twist.angular.z], \
          invkin_ref_state =[3, SerialTxControlTopic.seq3]) #wheel odom = 3
    #hello_str = "invkin_control: 0\n %s" % rospy.get_time()
    #rospy.loginfo(hello_str)
    self.pub.publish(hello_str)


  def invkin_publisher(self, topic_name):
    #inicio el nodo publicador
    rospy.init_node('invkin_publisher', anonymous=True)
    #declaracion del topico donde se va a publicar
    pub = rospy.Publisher('invkin_tx_commands', InvkinControlCommand, queue_size=10)
    #declaracion del topico que se va a escuchar para recibir el movimiento de Wheels encoders
    sub = rospy.Subscriber('/turtle1/pose', Pose, self.invkin_laser_publisher_callback)
    #declaracion del topico que se va a escuchar para recibir el IMU
    sub2 = rospy.Subscriber('/turtle1/sensors/imu', Imu, self.invkin_imu_publisher_callback)
    #declaracion del topico que se va a escuchar para recibir el movimiento de Wheels encoders
    sub3 = rospy.Subscriber('/turtle1/sensors/twist', TwistWithCovarianceStamped, self.invkin_wheel_publisher_callback)
    
        # finishing setup.
  
    print("Running invkin_publisher node in invkin_tx_commands topic")

    # and return the publisher so that the callback can use it.
    # also needs to have the serial port available to the callback.
    return pub


  # The constructor calls a helper to initialize everything, and stores the
  # resulting publisher and serial port object that's created.
  def __init__(self, topic_name):
    self.pub = self.invkin_publisher(topic_name)
    # and that's all.
  

if __name__ == '__main__':
        #invkin_publisher()
        s_tx = SerialTxControlTopic(sys.argv[1])
        #rospy.spin()
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass
        except rospy.ROSInterruptException:
            pass