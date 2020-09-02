import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import time
import matplotlib.animation as animation
# state is just y, y_dot

def update(state,m,dt=0.01,g=9.81):
    y = state[0]
    y_dot = state[1]
    y_dot_new = y_dot - dt*m*g
    y_new = y + dt*y_dot_new
    state_new = np.array([y_new,y_dot_new])
    return state_new


time_end = 1.5
dt = 0.01
t = np.arange(0,time_end+dt,dt)
m = 3

n_points = int(time_end/dt)
states=np.zeros((2,n_points))
state_0 = np.array([20,20])
states[:,0] = state_0



for i in range(1,len(t)-1):
    next_state = update(states[:,i-1],m,dt)
    states[:,i]=next_state

fig, ax = plt.subplots()
def animate(i):
    p=plt.plot(t[0:i],states[0,0:i],color='b')  # update the data.


ani = animation.FuncAnimation(fig, animate, n_points, interval=20)
plt.show()
#plt.plot(time,states[0,:])
