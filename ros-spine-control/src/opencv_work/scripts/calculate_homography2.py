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


def calc_H(uv, xy):

    print uv
    print type(uv)
    num_pts = np.size(uv, 1)

    # Initialize the A and b matrices
    # There will be 2*N rows, since each point has an x, y position
    A = np.ndarray((2 * num_pts, 8))
    # The b vector will be 2N long
    b = np.ndarray((2 * num_pts, 1))

    # Loop through and build up A and b based on the points matrix.
    for i in range(0, num_pts):
        # JACOB MADDEN - 2019_04.15
        # cleaned up update
        A[i * 2, :] = np.array([xy[i, 0], xy[i, 1], 1, 0, 0, 0, -uv[0, i] * xy[i, 0], -uv[0, i] * xy[i, 1]])
        A[i * 2 + 1, :] = np.array([0, 0, 0, xy[i, 0], xy[i, 1], 1, -uv[1, i] * xy[i, 0], -uv[1, i] * xy[i, 1]])

    # Find homogrpahy vector, h, by doing A^-1 b
    Ainv = np.linalg.inv(A)
    h = np.dot(Ainv, b)

    # h is of the form [h11, h12, h13, h21, h22, h23, h31, h32]
    # but each element is its own one-element array,
    # so we double-index here
    H = np.array([[h[0, 0], h[1, 0], h[2, 0]],
                  [h[3, 0], h[4, 0], h[5, 0]],
                  [h[6, 0], h[7, 0], 1]])

    # print(H)
    return(H)


def test_H(H, vs):
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
    raw_input("Please move out of the frame, then press enter to capture the frame that will be used for testing the homography.")
    print("Click two times. Distance will be calculated between these two points.")

    # Next, get the two clicks. Read this frame:
    # grab the current frame, then handle if we are using a
    # VideoStream or VideoCapture object
    # resize the frame (better viewing, consistent with object tracker.
    frame = vs.read()[1]
    frame = imutils.resize(frame, width=1000)

    # show the newly-captured frame
    cv2.imshow("FrameForTesting", frame)

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
    print(H)
    Q = np.linalg.inv(H)

    # Then, the x, y points in the world frame.
    # Loop through each point because tensor path is hard.
    # number of points clicked was
    num_pts = np.size(uv, 1)

    # preallocate the xy points
    # it's homogenous, so a 3-vector not a 2-vector
    xy = np.ndarray((3, num_pts))
    
    # plug in for each xy
    for i in range(0, num_pts):
        # of the two u, v points that were clicked on, are:
        # this point is (homogenous form)
        uv_i = np.r_[uv[:, i], 1]
        # then the point in xy is
        xy[:, i] = np.dot(Q, uv_i)

    # Calculate the distance between these two points
    # the net vector between them is
    # this ONLY works with two clicks!!!
    r = xy[:, 0] - xy[:, 1]
    
    # the distance is the norm of the net vector
    dist = np.linalg.norm(r, 2)
    print('Distance between your two points, in cm, is')
    print(dist)
    
    # close the window.
    cv2.destroyWindow("FrameForTesting")
