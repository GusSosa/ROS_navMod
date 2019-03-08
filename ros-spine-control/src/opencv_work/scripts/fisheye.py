#!/usr/bin/python2.7

# fisheye.py
# Program to determine the required K and D matrices for cv2.fisheye function,
# using a collection of previously supplied checkerboard images,
# and then undistort the web cam video using those matrices
# Source: https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0

import cv2
import numpy as np
import os
import glob
import rospy
import sys

DIM = (640, 480)


def calibrate():

    print('[START OF CALIBRATION]')

    CHECKERBOARD = (6, 9)
    subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
    calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW
    # calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW
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
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), subpix_criteria)
            imgpoints.append(corners)

    N_OK = len(objpoints)
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
    rms, _, _, _, _ = \
        cv2.fisheye.calibrate(
            objpoints,
            imgpoints,
            gray.shape[::-1],
            K,
            D,
            rvecs,
            tvecs,
            calibration_flags,
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
        )

    print("Found " + str(N_OK) + " valid images for calibration")
    print("DIM=" + str(_img_shape[::-1]))
    print("K=np.array(" + str(K.tolist()) + ")")
    print("D=np.array(" + str(D.tolist()) + ")")
    print('[CALIBRATION COMPLETE]')

    return{'D': D, 'K': K}


def undistort(K, D):

    # start video stream with webcam
    vs = cv2.VideoCapture(0)

    while not rospy.is_shutdown():

        # read current web cam frame
        img = vs.read()[1]

        # dim2 = None
        # dim3 = None
        # balance = 0.0
        # dim1 = img.shape[:2][::-1]  # dim1 is the dimension of input image to un-distort
        # assert dim1[0] / dim1[1] == DIM[0] / DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
        # if not dim2:
        #     dim2 = dim1
        # if not dim3:
        #     dim3 = dim1
        # scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
        # scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
        # # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
        # new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
        # map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
        # undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

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
            vs.release()
            cv2.destroyAllWindows()
            print('[PROGRAM END]')
            sys.exit()


if __name__ == '__main__':
    try:
        calibration_data = calibrate()
        [K, D] = [calibration_data['K'], calibration_data['D']]
        undistort(K, D)
    except rospy.ROSInterruptException:
        pass
