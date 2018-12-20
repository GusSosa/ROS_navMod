import numpy as np

class Simulation:
    
    def __init__(self, spine2d, dt, horizon) :
        self.spine = spine2d
        self.dt = dt
        self.time = horizon
        self.new_state = None
        self.results = False
    
    # Get the acceleration values
    def get_acc(self) :
        return self.spine.acc()
    
    def update_state(self):
        self.spine.states = np.concatenate((self.spine.states, self.new_state), axis=0)
    
    def current_state(self):
        return self.spine.current_state()
        
    def integrate_step(self) :
        x_acc, y_acc, th_acc = self.get_acc()
        state = self.current_state()
        x, y, th = state[0], state[2], state[4]
        xd, yd, thd = state[1], state[3], state[5]
        
        xd_t = xd + self.dt * x_acc
        yd_t = yd + self.dt * y_acc
        thd_t = thd + self.dt * th_acc
        
        x_t = x +  0.5 * xd_t * self.dt
        y_t = y + 0.5 * yd_t * self.dt
        th_t = th + 0.5 * thd_t * self.dt
        
        self.new_state = np.array([[x_t, xd_t, y_t, yd_t, th_t, thd_t]])
        self.update_state()
    
    def get_results(self):
        if (self.results) :
            return self.spine.states
        else :
            print("have not run simulations")
            
    def run(self):
        self.results = True
        N = int(self.time / self.dt)
        
        for i in range(N):
            self.integrate_step()


        
        