#!/usr/bin/python2.7

# NOTE: on Ubuntu 18.04, specifying /usr/bin/env python uses python 3,
# while specifying /usr/bin/python2.7 uses python 2.7. We need 2.7.

# Simple talker demo that publishes numpy_msg(Floats) messages
# to the 'cv_data' topic

import rospy
import roslib
from opencv_object_tracker import tracker_init, tracker_main, tracker_angle
import cv2
import numpy as np
from numpy.linalg import inv
from opencv_work.msg import SpineState

roslib.load_manifest('opencv_work')


def talker():

    # set publish node to 'cv_data' and initialize publisher node as 'cv_talker'
    rospy.init_node('cv_talker', anonymous=False)
    # pub = rospy.Publisher('cv_data', numpy_msg(Floats), queue_size=10)
    pub = rospy.Publisher('cv_data', SpineState, queue_size=10)
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

                # calculate true COM points using homography matrix
                pix_com_hom = np.append(pix_com_data, [[1, 1]], axis=0)

                Q = np.linalg.inv(H)
                pt1 = np.transpose(np.array([pix_com_data[0, :]]))
                pt2 = np.transpose(np.array([pix_com_data[1, :]]))
                pt1h = np.r_[pt1, np.array([[1]])]
                pt2h = np.r_[pt2, np.array([[1]])]
                pt1true = np.dot(Q, pt1h)[0:2, :]
                pt2true = np.dot(Q, pt2h)[0:2, :]
                # true_com = (np.dot(np.linalg.inv(H), pix_com_hom))[0:2, :]

                # calculate angle of rotation of vertebrae
                theta = tracker_angle(np.transpose(pt1true), np.transpose(pt2true))
                # print(theta)
                # append rotation data with COM position data
                # vert_data = np.append(true_com.flatten(), theta)

                # publish data
                message = SpineState(rotation=theta, com1=pt1true, com2=pt2true)
                # message = SpineState(rotation=theta, com1=pix_com_data[0, :], com2=pix_com_data[1, :])
                pub.publish(message)
                rate.sleep()

            # if we are using a webcam, release the pointer
            if not args.get("video", False):
                vs.release()
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
