import numpy as np
import spine2d
import sim 
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # define:
    # param: [k , c, mass]
    # initial: [x, xdot, y, ydot, theta, thetadot]
    # Att : [s1, s2, s3] ... Will change to 4 later (using one data for two att. points)
    
    # Check that acc calculations are correct.. set final and initial positions equal and force should be zero
    # Set gravity to TRUE and y acc should be -9.81
    
    param = np.array([-30, 3, 1])
    initial = np.array([[0.5, 0, 0, 0, 0, 0]])
    att = np.array([[0, 0],
                   [-.5, .5],
                   [-.5, -.5]])
    
    # force vector : [n1-s1, n1-s2, n2-s1, n2-s3]
    # Final_pos : [x, y, theta]
    force_vector = np.array([0,0,0,0])
    final_pos = np.array([0.6, 0.1, 0])
    
    gravity = True # False: no gravity, True: gravity
    # Instantiate spine
    spine1 = spine2d.Spine(param, initial, att, gravity)
    # Calculate inputs
    spine1.length_inputs(force_vector, final_pos)
    
    # Simulator
    sim1 = sim.Simulation(spine1, 0.01, 20)
    sim1.run()
    results = sim1.get_results()
    
    # Plot x, y ----> results size = time steps X 6 where [x, xdot, y, ydot, theta, thetadot]
    plt.plot(results[:,0], results[:,2])
    # red dot is final state
    plt.plot(results[-1,0], results[-1,2], 'ro')
    plt.title('Spine com position')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.show()
    print(results.shape)