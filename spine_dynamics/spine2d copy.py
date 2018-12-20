import numpy as np


class Spine:
    
    def __init__(self, parameters, initial, attachment_points, g = True):
        self.barlength = 0.5
        self.parameters = parameters
        self.states = initial
        self.inputs = None
        self.attachments = attachment_points
        self.rod = self.bar_length()
        self.bool = g
    
    def size(self):
        return len(self.states)
    def length_inputs(self, force_vector, final_positions): 
        x = final_positions[0]
        y = final_positions[1]
        th = final_positions[2]
        barlength = self.barlength

        s1 = self.attachments[0,:]
        s2 = self.attachments[1,:]
        s3 = self.attachments[2,:]

        # define nodes
        n1 = np.array([x + np.cos(2 * np.pi/3 + th) * barlength, y + np.sin(2 * np.pi/ 3 + th) * barlength])
        n2 = np.array([x + np.cos(-2 * np.pi/ 3 + th) * barlength, y + np.sin(-2 * np.pi/ 3 + th) * barlength])
        l2_dist_vector = np.array([np.linalg.norm(np.array(n1 - s1)), 
                                   np.linalg.norm(np.array(n1 - s2)), 
                                   np.linalg.norm(np.array(n2 - s1)), 
                                   np.linalg.norm(np.array(n2 - s3))])
        
        kay = np.ones(4) * 1/self.parameters[0]
        K_inv = np.diag(kay)

        self.inputs = l2_dist_vector - np.dot(force_vector, K_inv)
        return None
        
    def node1(self) :
        barlength = self.barlength
        delta = 2 * np.pi/ 3 
        state_t = self.current_state()
        x, y, theta = state_t[0], state_t[2], state_t[4]
        x1dot, x2dot, thetadot = state_t[1], state_t[3], state_t[5]
        n1 = np.array([x + np.cos(2 * np.pi/3 + theta) * barlength, y + np.sin(2 * np.pi/ 3 + theta) * barlength])
        n1_dot = np.array([x1dot - barlength * np.sin(delta + theta) * thetadot,
                   x2dot + barlength * np.cos(delta + theta) * thetadot])
        return n1, n1_dot

    def node2(self) :
        barlength = self.barlength
        delta = 2 * np.pi/ 3 
        state_t = self.current_state()
        x, y, theta = state_t[0], state_t[2], state_t[4]
        x1dot, x2dot, thetadot = state_t[1], state_t[3], state_t[5]
        
        n2 = np.array([x + np.cos(-2 * np.pi/3 + theta) * barlength, y + np.sin(-2 * np.pi/ 3 + theta) * barlength])
        n2_dot = np.array([x1dot - barlength * np.sin(-delta + theta) * thetadot,
                   x2dot + barlength * np.cos(-delta + theta) * thetadot])
        return n2, n2_dot


    def force_value(self) :
        n1 , n1ddt = self.node1()
        n2 , n2ddt = self.node2()
        state_t = self.current_state()
        # [n1, n1, n2, n2]; [s1, s2, s1, s3]
        Node = np.array([n1,n1, n2, n2])
        ddt_Node = np.array([n1ddt,n1ddt, n2ddt, n2ddt]).T
        s1 = self.attachments[0,:]
        s2 = self.attachments[1,:]
        s3 = self.attachments[2,:]
    
        ref = np.array([s1,s2,s1,s3])
        length_norms = np.linalg.norm((Node - ref), axis = 1)
        # (1, 4) size each
        spring_comp = self.parameters[0] * (length_norms - self.inputs)
        
        damping_comp = np.zeros((1,4))
        # Hard coded ... maybe fix later
        for i in range(4) :
            damping_comp[0, i] = (Node - ref)[i,:] @ ddt_Node[:,i]
        damping_comp = -self.parameters[1] * damping_comp @ np.linalg.inv(np.diag(length_norms))
    
        direction_components = (Node - ref)

        x_component =   ((spring_comp + damping_comp) @ np.diag(direction_components[:,0])) @np.linalg.inv(np.diag(length_norms))
        y_component = ((spring_comp + damping_comp) @ np.diag(direction_components[:,1])) @ np.linalg.inv(np.diag(length_norms))

        return x_component, y_component

    def gravity(self):
        return 9.81 if self.bool == True else 0
    
    def current_state(self) :
        st = self.states
        if (st.shape[0] < 2) :
            last_state =  self.states[0]
        else :
            last_state =  st[-1]
        return(last_state)

    def acc(self):
        n1 , n1ddt = self.node1()
        n2 , n2ddt = self.node2()
        # [n1, n1, n2, n2]; [s1, s2, s1, s3]
        Node = np.array([n1, n1, n2, n2]) 
        state_t = self.current_state()
        CoM = np.array([state_t[0], state_t[2]])
        relative_position = Node - CoM

        fx, fy = self.force_value()
        acc_x = 1 / self.parameters[2] * np.sum(fx)
        acc_y = 1 / self.parameters[2] * np.sum(fy) - self.gravity()
        acc_th = (relative_position[:,0] @ (fy.T + self.gravity())) - (relative_position[:,1]@ fx.T)
        
        return acc_x, acc_y, acc_th[0]


    def bar_length(spacing = None) :
        return spacing