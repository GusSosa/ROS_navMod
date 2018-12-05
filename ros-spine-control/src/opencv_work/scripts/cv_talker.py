#!/usr/bin/python2.7

# NOTE: on Ubuntu 18.04, specifying /usr/bin/env python uses python 3,
# while specifying /usr/bin/python2.7 uses python 2.7. We need 2.7.

# Simple talker demo that publishes numpy_msg(Floats) messages
# to the 'cv_data' topic

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import roslib
from opencv_object_tracker import tracker_init, tracker_main, tracker_angle
import cv2
import numpy as np
from numpy.linalg import inv

roslib.load_manifest('opencv_work')


def talker():

    # set publish node to 'cv_data' and initialize publisher node as 'cv_talker'
    rospy.init_node('cv_talker', anonymous=True)
    pub = rospy.Publisher('cv_data', numpy_msg(Floats), queue_size=10)
    rate = rospy.Rate(100)  # 10hz

    while not rospy.is_shutdown():

        # wrap everything around a keyboard interrupt catcher
        try:
            # run initalization
            (trackers, args, pix_com, vs, H, fps, key) = tracker_init()

            # while still tracking
            while not key == ord("q"):

                # run tracker
                (pix_com_data, key) = tracker_main(trackers, args, pix_com, vs, fps)
                pix_com_hom = np.append(pix_com_data, [[1, 1]], axis=0)
                true_com = np.linalg.inv(H) * np.transpose(pix_com_hom)
                theta = tracker_angle(pix_com_data)
                true_com = true_com.flatten()

                vert_data = np.append(true_com, theta)
                print(vert_data)

                # publish data
                # rospy.loginfo(vert_data)
                pub.publish(vert_data)
                rate.sleep()

            # if we are using a webcam, release the pointer
            if not args.get("video", False):
                vs.stop()
                print('[END OF TRACKING]')

            # otherwise, release the file pointer
            else:
                vs.release()

            # close all windows and leave loop
            cv2.destroyAllWindows()
            break

        except KeyboardInterrupt:
            print("Exiting.")
            break


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
