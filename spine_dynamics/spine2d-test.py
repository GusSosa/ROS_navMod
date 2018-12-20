import unittest as test
import numpy as np
import spine2d as spine

if __name__ == "__main__":
    # define:
    # parameters: [k , c, mass]
    # initial: [x, xdot, y, ydot, theta, thetadot]
    # Final_pos : []
    # Attachment points : [s1, s2, s3] ... Will change to 4 later (using one data for two att. points)
    
    # Check that acc calculations are correct.. set final and initial positions equal and force should be zero
    # Set gravity to TRUE and y acc should be -9.81
    
    param = np.array([20, 20, 1])
    initial = [0.5, 0, 0, 0, 0, 0]
    att = np.zeros((3,2))
    
    force_vector = np.array([0,0,0,0])
    final_pos = np.array(initial) + 0
    
    gravity = True
    spine1 = spine2d(param, initial, att, True)
    spine1.length_inputs(force_vector, final_pos)
    fx , fy = spine1.force_value()
    xa, ya, ta = spine1.acc()
    print(xa, ya, ta)
    
    