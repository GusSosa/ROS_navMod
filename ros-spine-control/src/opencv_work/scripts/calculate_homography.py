#!/usr/bin/python2.7

# This module has one function that calculates the homography matrix for a given scene,
# given four points.
# This code is from Drew Sabelhaus and Saunon Malekshahi's implementation
# of EE206A's lab4 from fall of 2018.

# imports:
import numpy as np
from numpy.linalg import *
from imutils.video import VideoStream
import cv2
import rospy

# From EE206A, F'18, a function written by the instructors to check the homography.
# Helper function to check computed homography
# This will draw dots in a grid by projecting x,y coordinates
# of box corners to u,v image coordinates
# We will use it for the grid behind the spine test setup,
# which has 16 squares in each direction at 2cm each.
def check_homography(H, vs, args, nx, ny, length):
	# vs is a VideoStream object
	# H should be a 3x3 numpy.array
	# nx is the number of  in the x direction
	# ny is the number of tiles in the y direction
	# length is the length of one side of a tile
	
	# grab the current frame, then handle if we are using a
	# VideoStream or VideoCapture object
	frame = vs.read()
	frame = frame[1] if args.get("video", False) else frame
	# plot some circles on it
	for i in range(nx+1):
		for j in range(ny+1):
			xbar = np.array([[i*length],[j*length],[1]])
			ubar = np.dot(H,xbar).T[0]
			u = np.int(ubar[0]/ubar[2])
			v = np.int(ubar[1]/ubar[2])
			print 'Dot location: ' + str((u,v))
			cv2.circle(frame, (u,v), 5, 0, -1)
	cv2.imshow('Check Homography', frame)
	# A pause.
	raw_input("Homography-transformed grid displayed. Press enter to continue.")
	# then destroy the window.
	cv2.destroyWindow("Check Homography")

# This function takes in two sets of points, one in the camera frame and one 
# in the global frame, and calculates the transformation (homography) between them.
# and outputs a 3x3 numpy ndarray.
# uv are the camera points, and xy are the global frame points.
def calc_H(uv, xy):

	# # Convert the Python list of points to a NumPy array of the form
	# #   | u1 u2 u3 u4 |
	# #   | v1 v2 v3 v4 |
	# uv = np.array(points).T

	# This is placeholder code that will draw a 4 by 3 grid in the corner of
	# the image
	#nx = 4
	#ny = 3
	#H = np.eye(3)

	# Debugging: look at the points array
	#print(points)
	# and the number of points is then
	#print(np.size(points, 0))
	#num_pts = points.shape[0]
	num_pts = np.size(uv, 1)

	# Initialize the A and b matrices
	# There will be 2*N rows, since each point has an x, y position
	A = np.ndarray((2*num_pts, 8))
	# The b vector will be 2N long
	b = np.ndarray((2*num_pts, 1))

	# Loop through and build up A and b based on the points matrix.
	for i in range(0, num_pts):
	  # debugging
	  #print(num_pts)
	  #print(i)
	  #print(uv)
	  #print(np.)
	  # The u,v for point i are points(i, 0) and points(i, 1)
	  #u_i = uv[i, 0]
	  #v_i = uv[i, 1]
	  # flip indexing
	  u_i = uv[0, i]
	  v_i = uv[1, i]
	  # The row indices for point i are
	  row_u_i = i*2
	  row_v_i = row_u_i + 1
	  # for b, directly plug in each point
	  b[row_u_i, 0] = u_i
	  b[row_v_i, 0] = v_i
	  # Each row of A for this pixel is the following.
	  # Note that since we've hard-coded the xy array, there can only
	  # be 4 uv to click here (eg. num_pts better equal 4, OR ELSE!)
	  # for ease,
	  x_i = xy[i,0]
	  y_i = xy[i,1]
	  A_row_u = np.array([x_i, y_i, 1, 0, 0, 0, -u_i*x_i, -u_i*y_i])
	  A_row_v = np.array([0, 0, 0, x_i, y_i, 1, -v_i*x_i, -v_i*y_i])
	  # now plug in to the pre-allocated A matrix
	  A[row_u_i, :] = A_row_u
	  A[row_v_i, :] = A_row_v

	# We can then find h by doing A^-1 b
	# debugging
	#print(A)
	#print(A.shape)
	Ainv = np.linalg.inv(A)
	h = np.dot(Ainv, b)
	# h is of the form [h11, h12, h13, h21, h22, h23, h31, h32]
	# but each element is accidentally its own one-element array,
	# so we double-index here. TO-DO: fix this and make it cleaner.
	# so H is
	H = np.array([[h[0,0], h[1,0], h[2,0]], 
	              [h[3,0], h[4,0], h[5,0]],
	              [h[6,0], h[7,0], 1]])
	#for part 3 of the lab, output H,
	#so we can hard-code it here later.
	#print(H)
	return(H)
	# The H we got from one representative test was
	# H = np.array([[3.95587655, 1.93812798, 171.],
	#             [0.02141018, -0.24739812, 421.],
	#             [7.7015021e-05, 0.0047355, 1.]])
	# Invert H
	#Q = np.linalg.inv(H)

def test_H(H, vs, args):
	# Inputs: H = homography matrix, np ndarray 3x3, vs = VideoStream from cv2.
	# a quick function to test out the homography.
	# Like in 206A, we'll test the distance between two representative points.
	# We get two points:
	tot_clicks = 2
	# store the clicked points in an array
	test_pts = []
	# Set up a callback here to get two points.
	def on_mouse_click_for_testing(event,x,y,flag,param):
		# we'll only capture four clicks.
		if len(test_pts) < tot_clicks:
			if(event == cv2.EVENT_LBUTTONUP):
				point = (x,y)
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
	frame = vs.read()
	frame = frame[1] if args.get("video", False) else frame
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
	r = xy[:,0] - xy[:,1]
	# the distance is the norm of the net vector
	dist = np.linalg.norm(r, 2)
	print('Distance between your two points, in cm, is')
	print(dist)
	# close the window.
	cv2.destroyWindow("FrameForTesting")
