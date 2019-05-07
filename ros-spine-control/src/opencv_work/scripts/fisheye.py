#!/usr/bin/python2.7

# fisheye.py
# Program to determine the required K and D matrices for cv2.fisheye function,
# using a collection of previously supplied checkerboard images captured from fisheye_img_cap.py,
# and then undistort the web cam video using those matrices
# Source: https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0

# NOTE: Calibration images must be in the same folder as the fisheye.py program

import cv2
import numpy as np
import os
import glob
import rospy
import sys
import random
import imutils


# checkerboard image dimensions per above reference
CHECKERBOARD = (6, 9)
DIM = (640, 480)
# video source: 0=laptop webcam,1=external usb camera
VID = 1
PIX_W = 1500


def calibrate():

    print('[START OF CALIBRATION]')

    subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    _img_shape = None
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    # images = glob.glob('*.jpg')
    images = glob.glob(os.path.dirname(os.path.abspath(__file__)) + '/*.jpg')

    for fname in images:
        # print fname
        img = cv2.imread(fname)
        # print type(fname)
        if _img_shape == None:
            _img_shape = img.shape[:2]
        else:
            assert _img_shape == img.shape[:2], "All images must share the same size."
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH +
                                                 cv2.CALIB_CB_FAST_CHECK +
                                                 cv2.CALIB_CB_NORMALIZE_IMAGE)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), subpix_criteria)
            imgpoints.append(corners)

    N_OK = len(objpoints)
    K = np.zeros((3, 3))
    D = np.zeros((5, 1))
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
    ret, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("Found " + str(N_OK) + " valid images for calibration")
    print("DIM=" + str(_img_shape[::-1]))
    print("K = np.array(" + str(K.tolist()) + ")")
    print("D = np.array(" + str(D.tolist()) + ")")

    mean_error = 0
    for i in xrange(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, D)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    print("total error: {}".format(mean_error / len(objpoints)))
    print('[CALIBRATION COMPLETE]')

    return{'D': D, 'K': K}


def undistort(K, D, frame):

    # # TESTING: taken out when integrated into main tracking program
    # vs = cv2.VideoCapture(VID)

    # while not rospy.is_shutdown():

        # # TESTING: read current web cam frame
        # img = vs.read()[1]

    h, w = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))

    # undistort
    dst = cv2.undistort(frame, K, D, None, newcameramtx)

    # #TESTING: show before and after images
    # img = imutils.resize(img, width=PIX_W)
    # dst = imutils.resize(dst, width=PIX_W)
    # cv2.imshow('distorted', img)
    # cv2.imshow('undistorted', dst)

    # # TESTING: reset keyboard interrupt
    # key = cv2.waitKey(1) & 0xFF

    # TESTING: if the "q" key is pressed, quit the program
    # if key == ord("q"):

    #         # close all windows and end program
    #     vs.release()
    #     cv2.destroyAllWindows()
    #     print('[PROGRAM END]')
    #     sys.exit()

    return(dst)


def test(K, D):
    # test function to re-distort and then un-distort the re-distorted image

    vs = cv2.VideoCapture(VID)

    # generate new distortion matrix
    D_dist = np.zeros((5, 1))
    for i in range(len(D)):
        D_dist[i] = random.random()

    while not rospy.is_shutdown():

        # read current web cam frame
        img = vs.read()[1]
        h, w = img.shape[:2]

        newcameramtx_dist, roi = cv2.getOptimalNewCameraMatrix(K, D_dist, (w, h), 1, (w, h))

        # distort input image with random D matrix
        dst = cv2.undistort(img, K, D_dist, None, newcameramtx_dist)

        # caluclate new K and D matrices for distorted image
        subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp_dst = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        objp_dst[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
        _img_shape = None
        objpoints_dst = []  # 3d point in real world space
        imgpoints_dst = []  # 2d points in image plane.

        gray_dst = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret_dst, corners_dst = cv2.findChessboardCorners(gray_dst, CHECKERBOARD,
                                                         cv2.CALIB_CB_ADAPTIVE_THRESH +
                                                         cv2.CALIB_CB_FAST_CHECK +
                                                         cv2.CALIB_CB_NORMALIZE_IMAGE)

        # If found, add object points, image points (after refining them)
        if ret_dst == True:
            objpoints_dst.append(objp_dst)
            cv2.cornerSubPix(gray_dst, corners_dst, (3, 3), (-1, -1), subpix_criteria)
            imgpoints_dst.append(corners_dst)

        K_dst = np.zeros((3, 3))
        D_dst = np.zeros((5, 1))
        print len(objpoints_dst)
        ret, K_dst, D_dst, rvecs, tvecs = cv2.calibrateCamera(objpoints_dst, imgpoints_dst, gray_dst.shape[::-1], None, None)

        # undistort input image using previously calculated K and D matrices
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
        img_fixed = cv2.undistort(img, K, D, None, newcameramtx)

        # undistort dst image using new calculated K_dst and D_dst matrices
        newcameramtx_dst, roi = cv2.getOptimalNewCameraMatrix(K_dst, D_dst, (w, h), 1, (w, h))
        dst_fixed = cv2.undistort(dst, K_dst, D_dst, None, newcameramtx_dst)

        # show before and after images
        cv2.imshow('original', img)
        cv2.imshow('original K and D', img_fixed)
        cv2.imshow('manually distorted dst image', dst)
        cv2.imshow('fixed dst image', dst_fixed)

        # reset keyboard interrupt
        key = cv2.waitKey(1) & 0xFF

        # if the "q" key is pressed, quit the program
        if key == ord("q"):

            # close all windows and end program
            vs.release()
            cv2.destroyAllWindows()
            print('[PROGRAM END]')
            sys.exit()


if __name__ == '__main__':
    try:
        calibration_data = calibrate()
        [K, D] = [calibration_data['K'], calibration_data['D']]
        if len(sys.argv) < 2:
            undistort(K, D)
        else:
            test(K, D)
    except rospy.ROSInterruptException:
        pass

    # TEST: pass new distoriton and then un-distort
