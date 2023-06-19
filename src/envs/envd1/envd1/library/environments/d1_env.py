from math import pi
import matplotlib.pyplot as plt
import numpy as np
import time


class env():
    def __init__(self, init_pose, final_pose, frame_rate, follow=False, external=lambda: 0, internal=lambda:1):
        self.ip = init_pose
        self.gp = final_pose
        
        self.fr = frame_rate
        
        self.st = np.array([[1, self.fr, pow(self.fr,2)/2],
                            [0, 1, self.fr]])
        
        self.R = external
        self.m = internal
        
        print(self.R)
        
        self.pion = plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.line1, = self.ax.plot(self.ip, 0, marker="o", markersize=20, markeredgecolor="red", markerfacecolor="green")
        self.line2, = self.ax.plot(self.gp, 0, marker="o", markersize=10, markeredgecolor="red", markerfacecolor="blue")
        
        self.prettifiyPlot()
        self.follow = follow
        
    
    def S(self, cp):
        cp[-1] = (cp[-1] - self.R())*self.m()
        return np.dot(self.st, cp)
    
    
    def prettifiyPlot(self):
        # setting labels
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        plt.title("Updating plot...")

        self.ax.set_xlim([-10,10])
        self.ax.set_ylim([-10,10])
        
    def render(self, p):
        self.ax.set_xlim([-10+p,10+p]) if self.follow else None
        
        self.line1.set_xdata(p)
        self.line1.set_ydata(0)

        # re-drawing the figure
        self.fig.canvas.draw()
        
        # self.ax.set_ylim([-10,10])

        # to flush the GUI events
        self.fig.canvas.flush_events()
        time.sleep(self.fr)
        
    def getA(self,action):
        value = 0
        if action==0:
            value = -1
        if action==1:
            value = 0
        if action==2:
            value = 1
        # return (value + self.R)
        return value
    
    
if __name__ == '__main__':
    a = 4
    e = env(0,5,.1)
    
    observation = np.array([e.ip,0,a])
    
    # # print(s)
    for _ in range(50):
        
        state = e.S(observation)
        
        e.render(state[0])
        
        observation = np.array([state[0],state[1],a]) if state[0] < e.gp else np.array([state[0],state[1],-a])
        
        
    # e = env(0,5,.1, external=2)
    # e.S(np.array([e.ip,0,e.getA(0)]))
    