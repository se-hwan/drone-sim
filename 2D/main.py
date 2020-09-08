import numpy as np
import matplotlib.pyplot as plt
from drone import Drone
from matplotlib.patches import Circle
import time
import matplotlib.animation as animation
from animation import animate

"""
INITIALIZE
"""
# Drone parameters
mass = 5 # mass of drone
h = 0.4 # height of drone
l_f = 0.5 # distance from center to front rotor
l_r = 0.5 # distance from center to back rotor
w = 1 # width, needed for inertia
n_states = 6 # number of states (x, xdot, y, ydot, theta, thetadot)
n_inputs = 2 # number of inputs (f1, f2)

# Simulation parameters
dt = 0.005 # time steps for simulation
time_end = 5. # duration of simulation
t = np.arange(0,time_end+dt,dt)
n_points = int(time_end/dt)
states = np.zeros((n_states,n_points)) # states over simulation time
statesDes = np.zeros((n_states,n_points)) # desired state (default to 0)
inputs = np.zeros((n_inputs,n_points)) # inputs over simulation time

# Initialization
state_0 = np.array([5,0,5,0,0,0]) # initial state
#state_0 = drone.randStart(xMax, yMax) # random initial state with xMax, yMax
drone = Drone(mass,l_f,l_r,h,w,state_0)
states[:,0] = state_0

# Set desired states over time
for i in range(n_points):
    statesDes[:,i]=np.array([10,0,10,0,0,0]) # (go to x=10, y=10, and stay stationary)

# Set gain matrix
K = np.array([1,1,5,5,15,15]) # y, ydot, x, xdot, theta, thetadot

# Solve simulation
for i in range(1,len(t)-1):
    next_u = drone.feedbackLin(states[:,i-1],statesDes[:,i-1],K, dt)
    next_state = drone.update(states[:,i-1],next_u,dt)
    states[:,i]=next_state

# animation of path
animate(states,drone,dt,time_end)
