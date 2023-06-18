import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from math import pi
import time

# from li  import DeepQNetwork
import numpy as np
# from ..environments.d1_env import env
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import axes3d


class MyNode(Node):
    def __init__(self, external, internal):
        super().__init__('envd1') 
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.subscription_ = self.create_subscription(Odometry, 'output', self.listener_callback, 10)
        self.timer_ = self.create_timer(.14, self.timer_callback)
        self.count_ = 0
        
        # self.env = gym.make('MountainCar-v0', render_mode="human")

        # self.obs = self.env.reset()
        # self.done = False
        # self.total_reward = 0

        num_of_actions = 3

        ep = [0,0,0,0,0,0]

        self.e = []
        self.a_error = []
        self.state = []
        self.state_ = []


        for i in range(6):
            self.e.append(env(ep[i],5,.1, follow=True, external=external, internal=internal))
            self.state.append(self.e[i].S(np.array([ep[i],0,0])))
            self.state_.append(self.e[i].S(np.array([ep[i],0,0])))
            self.a_error.append(np.zeros(num_of_actions))

        

        self.act = [0,0,0,0,0,0]

    # main loop... send over data on new pose estimate
    def timer_callback(self):
        self.call_environment()

        # msg = String()
        # msg.data = 'Hello World: %d' % self.count_
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.count_ += 1


        # print("-------")
        # print('pose x: ', self.state[0][1])
        # print('pose y: ', self.state[1][1])
        # print('pose z: ', self.state[2][1])
        # print('pose thx: ', self.state[3][1])
        # print('pose thy: ', self.state[4][1])
        # print('pose thz: ', self.state[5][1])
        # print("-------")

        data = Odometry()
        data.pose.pose.position.x = float(self.state[0][0])
        data.pose.pose.position.y = float(self.state[1][0])
        data.pose.pose.position.z = float(self.state[2][0])
        data.pose.pose.orientation.x = float(self.state[3][0])
        data.pose.pose.orientation.y = float(self.state[4][0])
        data.pose.pose.orientation.z = float(self.state[5][0])

        data.twist.twist.linear.x = float(self.state[0][1])
        data.twist.twist.linear.y = float(self.state[1][1])
        data.twist.twist.linear.z = float(self.state[2][1])
        data.twist.twist.angular.x = float(self.state[3][1])
        data.twist.twist.angular.y = float(self.state[4][1])
        data.twist.twist.angular.z = float(self.state[5][1])

        self.publisher_.publish(data)

    # convert action into new position in environment
    def call_environment(self):
        # select a random action
        # self.action = self.act

        # print(self.act)

        # take the action and observe the next state and reward
        # self.obs, self.reward, self.done, self.info,_ = self.env.step(int(self.action)+1)
        print("----------------")
        for i in range(6):
            a_idx = int(self.act[i]+1)
            # print("this is AAAAAAAAAA_INDEX: ", a_idx)
            self.state_[i] = self.e[i].S(np.array([self.state[i][0],
                                                self.state[i][1],
                                                self.act[i]+self.a_error[i][a_idx]])) ## +self.a_error[i][a_idx]
            
            if (self.state_[i][0] - self.state[i][0]) == 0:
                aa = 0
            else:
                aa =  (self.state_[i][1]**2 - self.state[i][1]**2) / (2* (self.state_[i][0] - self.state[i][0]))

            self.a_error[i][a_idx] += self.act[i]-aa

            print("state: ", self.state[i] , "  state_: ", self.state_[i])
            print("a_error: ", i, ": ", self.a_error[i][2], " ==? " , aa)

            self.state[i] = np.array(self.state_[i]).T

            self.e[i].render(self.state[i][0])
        
        print("---------------e-")
        # self.e[0].render(self.state[0][0])

        # render the environment
        # self.env.render()

    # get action from controller
    def listener_callback(self, msg):
        # pose = list(msg.pose.pose.position) + list(msg.pose.pose.orientation)
        # for i in range(6):
        #     self.act[i] = pose[i]


        # print("------")
        # print("x thrust: ", msg.pose.pose.position.x)
        # print("y thrust: ", msg.pose.pose.position.y)
        # print("z thrust: ", msg.pose.pose.position.z)
        # print("------")
        
        self.act[0] = msg.pose.pose.position.x
        self.act[1] = msg.pose.pose.position.y
        self.act[2] = msg.pose.pose.position.z
        self.act[3] = msg.pose.pose.orientation.x
        self.act[4] = msg.pose.pose.orientation.y
        self.act[5] = msg.pose.pose.orientation.z


def R(): # external effects
    return 0

def m(): # internal effects
    return 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode(R,m)
    rclpy.spin(node)
    
    node.env.close()
    node.destroy_node()


    rclpy.shutdown()



class env:
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
    main()


# print(gym)

# # create the environment
# env = gym.make('MountainCar-v0', render_mode="human")

# # reset the environment
# obs = env.reset()

# # run a few episodes
# for episode in range(3):
#     obs = env.reset()
#     done = False
#     total_reward = 0
#     while not done:
#         # select a random action
#         action = env.action_space.sample()

#         # take the action and observe the next state and reward
#         obs, reward, done, info,_ = env.step(action)

#         # add the reward to the total reward for the episode
#         total_reward += reward

#         # render the environment
#         env.render()

#     print(f"Episode {episode+1} reward: {total_reward}")

# # close the environment
# env.close()

