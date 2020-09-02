import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import time
import matplotlib.animation as animation
from drone import Drone


time_end = 1.5
dt = 0.01
t = np.arange(0,time_end+dt,dt)
m = 3

n_points = int(time_end/dt)
states=np.zeros((2,n_points))
state_0 = np.array([20,20])
states[:,0] = state_0

drone = Drone(3,.5,.5,state_0)


for i in range(1,len(t)-1):
    next_state = drone.update(states[:,i-1],dt)
    states[:,i]=next_state

fig, ax = plt.subplots()
def animate(i):
    fig.clear()
    plt.plot(t[0:i],states[0,0:i],color='b')  # update the data.
    simTime = "{:.2f}"
    plt.text(.15,.9,"Sim time: "+simTime.format(i*dt),horizontalalignment='center',
             verticalalignment='center', transform=ax.transAxes)

ani = animation.FuncAnimation(fig, animate, n_points+1, interval=20, repeat = False)
plt.show()
