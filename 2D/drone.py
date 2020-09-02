import numpy as np
import numpy.random as rand

class Drone:
    g = 9.81 # gravity

    def __init__(self, mass, l_f, l_r, state=0):
        self.state = state
        self.mass = mass
        self.l_f = l_f
        self.l_r = l_r

    def update(self,state,u,dt):
        nextState = self.dynamics(state,u,dt)
        self.state = nextState
        return nextState

    def dynamics(self,state,u,dt):
        y = state[0]
        y_dot = state[1]
        y_dot_new = y_dot - dt*self.mass*self.g
        y_new = y + dt*y_dot_new
        state_new = np.array([y_new,y_dot_new])
        return state_new

    def randStart(self,xMax,yMax):
        randState0 = np.array([[rand.uniform(0,xMax,1)],
                              [rand.uniform(0,yMax,1)],
                              [rand.random()*2*np.pi],
                              [rand.random()*5], # cap x_dot_0 at 5 m/s
                              [rand.random()*5], # cap y_dot_0 at 5 m/s
                              [rand.random()]]) # cap theta_dot_0 at 1 rad/s2
        return randState0

    def PID(self, state, desiredState, K):
        u = 
        return u
