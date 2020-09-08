import numpy as np
import numpy.random as rand

class Drone:
    g = 9.81 # gravity
    fMax = 200 # maximum force

    def __init__(self, m, l_f, l_r, h, w, state_init):
        self.state = state_init # x, xdot, y, ydot, theta, thetadot
        self.m = m
        self.h = h
        self.l_f = l_f
        self.l_r = l_r
        self.w = w
        self.lastError = 0
        self.totalError = 0
        self.I = (self.w*(self.l_f+self.l_r)**3)/12
        self.theta_des_last = self.state[4]

    def update(self,state,u,dt):
        nextState = self.dynamics(state,u,dt)
        self.state = nextState
        return nextState

    def dynamics(self,state,u,dt):
        # linearized about x,y,theta = 0, thrust = mg, torque = 0
        x = state[0]
        x_dot = state[1]
        y = state[2]
        y_dot = state[3]
        theta = state[4]
        theta_dot = state[5]
        x_dot_new = x_dot + dt*(-theta*self.g) #(-theta*u[0]/self.m)
        y_dot_new = y_dot + dt*(-self.g+u[0]/self.m)
        theta_dot_new = theta_dot + dt*(u[1]/self.I)
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

    def feedbackLin(self, currentState, desiredState, K, dt):
        u=[0,0]
        y_des = desiredState[2]
        y_dot_des = desiredState[3]
        accel_y_command = K[0]*(y_des - currentState[2]) + K[1]*(y_dot_des - currentState[3])
        thrust = self.m*self.g + self.m*accel_y_command
        if thrust>self.fMax:
            thrust = self.fMax
        elif thrust<0:
            thrust = 0

        x_des = desiredState[0]
        x_dot_des = desiredState[1]
        accel_x_command = K[2]*(x_des - currentState[0]) + K[3]*(x_dot_des - currentState[1])

        theta_des = -accel_x_command/self.g
        theta_dot_des = (theta_des-self.theta_des_last)/dt
        alpha_command = K[4]*(theta_des - currentState[4]) + K[5]*(theta_dot_des - currentState[5])
        torque = self.I*alpha_command
        """
        if np.absolute(torque)>self.l_f*self.fMax/2 and torque > 0:
            torque = self.l_f*self.fMax/2
        else:
            torque = -self.l_f*self.fMax/2
            """
        self.theta_des_last = theta_des
        u = np.array([thrust,torque])
        return u
