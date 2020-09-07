import numpy as np
import numpy.random as rand

class Drone:
    g = 9.81 # gravity
    fMax = 200 # maximum force

    def __init__(self, m, l_f, l_r, h, w, state=0):
        self.state = state # x, xdot, y, ydot, theta, thetadot
        self.m = m
        self.h = h
        self.l_f = l_f
        self.l_r = l_r
        self.w = w
        self.lastError = 0
        self.totalError = 0
        self.I = (self.w*(self.l_f+self.l_r)**3)/12

    def update(self,state,u,dt):
        nextState = self.dynamics(state,u,dt)
        self.state = nextState
        return nextState

    def dynamics(self,state,u,dt):
        x = state[0]
        x_dot = state[1]
        y = state[2]
        y_dot = state[3]
        theta = state[4]
        theta_dot = state[5]
        x_dot_new = x_dot + dt*(-np.sin(theta)*(u[0]+u[1])/self.m)
        y_dot_new = y_dot + dt*(-self.g+np.cos(theta)*(u[0]+u[1])/self.m)
        theta_dot_new = theta_dot + dt*(self.l_r*(u[1]-u[0])/self.I - self.I*theta_dot**2)
        x_new = x + dt*x_dot_new
        y_new = y + dt*y_dot_new
        theta_new = theta + dt*theta_dot_new
        state_new = np.array([x_new,x_dot_new,
                              y_new,y_dot_new,
                              theta_new,theta_dot_new])
        return state_new

    def randStart(self,xMax,yMax):
        randState0 = np.array([[rand.uniform(0,xMax,1)],
                              [rand.uniform(0,yMax,1)],
                              [rand.random()*2*np.pi],
                              [rand.random()*5], # cap x_dot_0 at 5 m/s
                              [rand.random()*5], # cap y_dot_0 at 5 m/s
                              [rand.random()]]) # cap theta_dot_0 at 1 rad/s2
        return randState0

    def PID(self, currentState, desiredState, K, dt):
        #u = np.array([1,1])
        error = desiredState-currentState
        self.totalError += error
        errorDot = error - self.lastError
        u = K[0]*error + K[1]*self.totalError + K[2]*errorDot
        for i in u:
            if (i>self.fMax):
                i = self.fMax
        self.lastError = error
        return u
