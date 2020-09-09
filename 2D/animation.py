import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib.animation as animation
#from drone import Drone

def animate(states, drone, dt, time_end):

    t = np.arange(0,time_end+dt,dt) # timesteps array
    n_points = int(time_end/dt) # number of points

    fig = plt.figure()
    ax = plt.axes(xlim=(0,5), ylim=(0,5))
    droneFig = plt.Rectangle((0,0),drone.l_f+drone.l_r,drone.h,fc='k')

    def initPlot():
        droneFig.xy = (states[0,0]-drone.l_f, states[2,0]-drone.h/2)
        droneFig.angle = np.degrees(states[4,0])
        ax.add_patch(droneFig)
        return droneFig,

    def draw(i):
        #fig.clear()
        x = states[0,i]
        y = states[2,i]
        theta = states[4,i]
        droneFig.xy = (x-drone.l_f, y-drone.h/2)
        droneFig.angle = np.degrees(theta)

        return droneFig,

    ani = animation.FuncAnimation(fig, draw, n_points-1, init_func = initPlot, blit=True,interval=10,repeat = True)
    plt.show()
    ani.save('feedbackLin.gif',writer='imagemagick')
