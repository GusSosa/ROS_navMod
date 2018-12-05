#!/usr/bin/python2.7

# This module has one function that calculates the homography matrix for a given scene,
# given four points.
# This code is from Drew Sabelhaus and Saunon Malekshahi's implementation
# of EE206A's lab4 from fall of 2018.

# imports:
import numpy as np

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
	# so H is
	H = np.array([[h[0], h[1], h[2]], 
	              [h[3], h[4], h[5]],
	              [h[6], h[7], 1]])
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