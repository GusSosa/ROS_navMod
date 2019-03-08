#!/usr/bin/python2.7

# undistort_fisheye.py
# Program to undistort fisheye images, using the K and D matrices input from the calibrate_fisheye.py
# Source: https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0

# Use: Call 'rosrun opencv_work undistort_fisheye.py img_name'
# where 'img_name' does not include the .jpg

import cv2
import numpy as np
import sys
import rospy

# You should replace these 3 lines with the output in calibration step
# NOTE: Need to update to take K and D as inputs
DIM = (640, 480)
K = np.array([[554.4691223393742, 0.0, 296.0677445937175], [0.0, 561.6709234690211, 236.23576688093456], [0.0, 0.0, 1.0]])
D = np.array([[0.03110827900071547], [5.2583990216132435], [-32.665517530927055], [62.26551464746827]])


def undistort(img_path):

    for img_name in img_path:

        while not rospy.is_shutdown():

            # Select image path and get shape
            # NOTE: Need to update generically
            img_path_upd = '/home/jmadden/2d-spine-control-hardware/ros-spine-control/src/opencv_work/scripts/' + str(img_path) + '.jpg'
            img = cv2.imread(img_path_upd)
            h, w = img.shape[:2]

            # map using input matrices and fisheye function, undistort
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
            undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

            # show before and after images
            cv2.imshow('distorted', img)
            cv2.imshow("undistorted", undistorted_img)

            # reset keyboard interrupt
            key = cv2.waitKey(1) & 0xFF

            # if the "q" key is pressed, quit the program
            if key == ord("q"):

                # close all windows and end program
                cv2.destroyAllWindows()
                print('[END]')
                sys.exit()


if __name__ == '__main__':
    for p in sys.argv[1:]:
        undistort(p)
