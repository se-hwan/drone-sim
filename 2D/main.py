import numpy as np
from drone import Drone

"""
INITIALIZE
"""
# Drone parameters
mass = 3 # mass of drone
l_f = 0.25 # distance from center to front rotor
l_r = 0.25 # distance from center to back rotor
n_states = 6 # number of states (x, y, theta, x_dot, y_dot, theta_dot)
n_inputs = 2 # number of inputs (w1, w2)

# Simulation parameters
dt = 0.01 # time steps for simulation
time_end = 1.5 # duration of simulation
t = np.arange(0,time_end+dt,dt)
n_points = int(time_end/dt)
states = np.zeros((n_states,n_points)) # states over simulation time
statesDes = np.zeros((n_states,n_points)) # desired state (default to 0)
inputs = np.zeros((n_inputs,n_points)) # inputs over simulation time

# Initialization
drone = Drone(mass,l_f,l_r)
state_0 = np.array([5,5,np.pi/4,2,2,0.5]) # initial state
for i in range(n_points):
    statesDes[:,i]=np.array([10,10,0,0,0,0])
#state_0 = drone.randStart(xMax, yMax) # random initial state with xMax, yMax
states[:,0] = state_0
K = np.array([10,1,1]) # PID gain matrix



for i in range(1,len(t)-1):
    next_u = drone.PID(states[:,i-1],statesDes[:,i-1],K)
    next_state = drone.update(states[:,i-1],next_u,dt)
    states[:,i]=next_state
