#!/usr/bin/python2.7

# This module has one function that calculates the homography matrix for a given scene,
# given four points.
# This code is based on Drew Sabelhaus and Saunon Malekshahi's implementation
# of EE206A's lab4 from fall of 2018.
# UPDATE: Modifed by Jacob Madden in spring of 2019

# imports:
import numpy as np
from numpy.linalg import *
import cv2
import rospy
import imutils
import fisheye

# Set desired frame width in pixels
PIX_W = 1200


def calc_H(uv, xy):

    # print 'uv: ' + str(uv)
    # print 'xy: ' + str(xy)
    # print type(uv)
    num_pts = np.size(uv, 0)
    # print num_pts

    # Initialize the A and b matrices
    # There will be 2*N rows, since each point has an x, y position
    A = np.zeros((2 * num_pts, 8))
    # The b vector will be 2N long
    b = np.zeros((2 * num_pts, 1))
    h = np.zeros((8, 1))

    P = np.zeros((2 * num_pts + 1, 9))
    P[2 * num_pts, 8] = 1
    P2 = np.zeros((2 * num_pts + 1, 9))
    P2[2 * num_pts, 8] = 1
    # print P
    b2 = np.zeros((2 * num_pts + 1, 1))
    b2[2 * num_pts, 0] = 1
    # print b2

    # print 'A init: ' + str(A)
    # print np.size(A, 0)
    # print 'b init: ' + str(b)
    # print np.size(b, 0)

    # Loop through and build up A and b based on the points matrix.
    for i in range(num_pts):

        A[i * 2, :] = [xy[i, 0], xy[i, 1], 1, 0, 0, 0, -uv[i, 0] * xy[i, 0], -uv[i, 0] * xy[i, 1]]
        A[i * 2 + 1, :] = [0, 0, 0, xy[i, 0], xy[i, 1], 1, -uv[i, 1] * xy[i, 0], -uv[i, 1] * xy[i, 1]]
        b[i * 2, 0] = uv[i, 0]
        b[i * 2 + 1, 0] = uv[i, 1]

        # 2019-05-15 update: solving PH = 0
        P[i * 2, :] = [-uv[i, 0], -uv[i, 1], -1, 0, 0, 0, xy[i, 0] * uv[i, 0], xy[i, 1] * uv[i, 0], uv[i, 0]]
        P[i * 2 + 1, :] = [0, 0, 0, -uv[i, 0], -uv[i, 1], -1, xy[i, 0] * uv[i, 1], xy[i, 1] * uv[i, 1], uv[i, 1]]

        P2[i * 2, :] = [-xy[i, 0], -xy[i, 1], -1, 0, 0, 0, uv[i, 0] * xy[i, 0], uv[i, 1] * xy[i, 0], xy[i, 0]]
        P2[i * 2 + 1, :] = [0, 0, 0, -xy[i, 0], -xy[i, 1], -1, uv[i, 0] * xy[i, 1], uv[i, 1] * xy[i, 1], xy[i, 1]]

        # # debugging
        # print 'A: ' + str(A)
        # print 'b: ' + str(b)
        # print 'index: ' + str(i)

        # print 'u: ' + str(uv[i, 0])
        # print 'v: ' + str(uv[i, 1])
        # print 'x: ' + str(xy[i, 0])
        # print 'y: ' + str(xy[i, 1])

        # print 'u*x: ' + str(-uv[i, 0] * xy[i, 0])
        # print 'u*y: ' + str(-uv[i, 0] * xy[i, 1])
        # print 'v*x: ' + str(-uv[i, 1] * xy[i, 0])
        # print 'v*y: ' + str(-uv[i, 1] * xy[i, 1])

        # print A[i * 2, :]
        # print A[i * 2 + 1, :]

        # raw_input('break')

    # Find homography vector, h, by using least squares minimization
    # print 'A: ' + str(np.int64(A))
    # print 'b: ' + str(b)
    # Ainv = np.linalg.inv(A)
    # h = np.dot(Ainv, b)
    h = np.linalg.lstsq(A, b, rcond=None)[0]
    # h2 = np.linalg.lstsq(P, b2, rcond=None)[0]
    # h3 = np.linalg.lstsq(P2, b2, rcond=None)[0]

    # h is of the form [h11, h12, h13, h21, h22, h23, h31, h32]
    # but each element is its own one-element array,
    # so we double-index here
    H = np.array([[h[0, 0], h[1, 0], h[2, 0]],
                  [h[3, 0], h[4, 0], h[5, 0]],
                  [h[6, 0], h[7, 0], 1]])
    # H2 = np.array([[h2[0, 0], h2[1, 0], h2[2, 0]],
    #                [h2[3, 0], h2[4, 0], h2[5, 0]],
    #                [h2[6, 0], h2[7, 0], h2[8, 0]]])
    # H3 = np.array([[h3[0, 0], h3[1, 0], h3[2, 0]],
    #                [h3[3, 0], h3[4, 0], h3[5, 0]],
    #                [h3[6, 0], h3[7, 0], h3[8, 0]]])

    # print 'H: ' + str(H)
    # print 'H2: ' + str(H2)
    # print 'H3: ' + str(H3)

    # print(H)
    return(H)


def test_H(H, vs, K, D):
    # Inputs: H = homography matrix, np ndarray 3x3, vs = VideoStream from cv2.
    # a quick function to test out the homography.
    # Like in 206A, we'll test the distance between two representative points.
    # We get two points:
    tot_clicks = 2

    # store the clicked points in an array
    test_pts = []

    # Set up a callback here to get two points.
    def on_mouse_click_for_testing(event, x, y, flag, param):
        # we'll only capture four clicks.
        if len(test_pts) < tot_clicks:
            if(event == cv2.EVENT_LBUTTONUP):
                point = (x, y)
                print "Point Captured: " + str(point)
                test_pts.append(point)

    # Get the positions of the clicks.
    # We want to wait until the user is ready to start.
    # A nifty way to do so is to have them type something into the terminal, then discard what was typed.
    raw_input("\n Please move out of the frame, then press enter to capture the frame that will be used for testing the homography.")
    print("Click two times. Distance will be calculated between these two points.")

    # Next, get the two clicks. Read this frame:
    # grab the current frame, then handle if we are using a
    # VideoStream or VideoCapture object
    # resize the frame (better viewing, consistent with object tracker.
    frame = vs.read()[1]

    # undistort fisheye
    dst = fisheye.undistort(K, D, frame)
    frame = imutils.resize(frame, width=PIX_W)
    dst = imutils.resize(dst, width=PIX_W)

    # show the newly-captured frame
    cv2.imshow("FrameForTesting", dst)

    # Tell OpenCV that it should call 'on_mouse_click' when the user
    # clicks the window. This will add clicked points to our list
    cv2.setMouseCallback("FrameForTesting", on_mouse_click_for_testing, param=1)

    # Finally, loop until we've got enough clicks. Just a blocking wait.
    while len(test_pts) < tot_clicks:
        if rospy.is_shutdown():
            raise KeyboardInterrupt
        # just block
        cv2.waitKey(10)

    # We should now have four elements in the h_pts array now.
    print("Captured points for the distance calculation:")
    print(test_pts)
    # Convert the Python list of points to a NumPy array of the form
    #   | u1 u2 u3 u4 |
    #   | v1 v2 v3 v4 |
    uv = np.array(test_pts).T

    # Invert H
    # print(H)
    Q = np.linalg.inv(H)

    # Then, the x, y points in the world frame.
    # Loop through each point because tensor path is hard.
    # number of points clicked was
    num_pts = np.size(uv, 1)

    # preallocate the xy points
    # it's homogenous, so a 3-vector not a 2-vector
    xy = np.ndarray((3, num_pts))
    testobj = np.zeros((2, 2))

    # plug in for each xy
    for i in range(num_pts):
        # of the two u, v points that were clicked on, are:
        # this point is (homogenous form)
        uv_i = np.r_[uv[:, i], 1]
        # NOTE: This only solves for [alpha*x, alpha*y, alpha]
        xy[:, i] = np.dot(Q, uv_i)
        # Divide out scaling factor and solve for [x, y]
        testobj[:, i] = xy[0:2, i] / xy[2, i]

    # Calculate the distance between these two points
    # the net vector between them is
    # this ONLY works with two clicks!!!
    r = xy[:, 0] - xy[:, 1]
    rtest = testobj[:, 0] - testobj[:, 1]
    disttest = np.linalg.norm(rtest, 2)

    # print 'xy: ' + str(xy)
    # print 'xy_1: ' + str(xy[:, 0])
    # print 'xy_2: ' + str(xy[:, 1])
    # print 'r: ' + str(r)
    # print 'testobj: ' + str(testobj)
    # print 'rtest: ' + str(rtest)
    # print 'disttest: ' + str(disttest)

    # the distance is the norm of the net vector
    dist = np.linalg.norm(r, 2)
    # print dist
    print '\n' + 'Distance between your two points, in cm: ' + str(disttest)

    # close the window.
    cv2.destroyWindow("FrameForTesting")
