# importing libraries
import numpy as np
import time
import matplotlib.pyplot as plt
from math import pi

class Environment_1d: 
    def __init__(self, goal, initial_pose, mass, stepsize, screenSize=[-10,10]):
        self.mass = mass
        self.goal = goal
        self.ip = initial_pose
        self.t = stepsize

        self._S = np.array([
            [1, self.t, pow(self.t,2)/2],
            [0, 1, self.t]
        ])
        self.screenSize = screenSize


    def S(self,s):
        return np.dot(self._S, s)


    def update_plot(self, s):
        # updating the value of x and y
        self.line1.set_xdata(s)
        self.line1.set_ydata(0)

        self.ax.set_xlim([-10+s, 10+s])

    
        # re-drawing the figure
        self.fig.canvas.draw()

        # to flush the GUI events
        self.fig.canvas.flush_events()
        time.sleep(0.1)

   
if __name__ == '__main__':
    e = Environment_1d(10, 0, 10, .1)

    ip = np.array([0,0,0]).reshape((3,1))
    s = e.S(ip)

    # enable interactive mode
    plt.ion()

    # creating subplot and figure
    fig = plt.figure()
    ax = fig.add_subplot(111)
    line1, = ax.plot(ip[0], 0 , marker="o", markersize=20, markeredgecolor="red", markerfacecolor="green")
    ax.set_xlim([-10,10])
    ax.set_ylim([-10,10])

    # setting labels
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.title("Updating plot...")

    for _ in range(50):
        # updating the value of x and y
        line1.set_xdata(s[0])
        line1.set_ydata(0)

        print(s[0])

        ax.set_xlim([-10+s[0], 10+s[0]])

    
        # re-drawing the figure
        fig.canvas.draw()

        # to flush the GUI events
        fig.canvas.flush_events()
        time.sleep(0.1)


        # define method for updateing s[2]
        # ...
        if _ < 25:
            s = np.append(s,4)
        else:
            s = np.append(s,-10)


        s = e.S(s.reshape(3,1))



    
