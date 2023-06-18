import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import gym

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.subscription_ = self.create_subscription(Odometry, 'output', self.listener_callback, 10)
        self.timer_ = self.create_timer(.07, self.timer_callback)
        self.count_ = 0
        
        self.env = gym.make('MountainCar-v0', render_mode="human")

        self.obs = self.env.reset()
        self.done = False
        self.total_reward = 0

        self.p = 0
        self.v = 0

        self.act = 0

    def timer_callback(self):
        self.call_environment()

        # msg = String()
        # msg.data = 'Hello World: %d' % self.count_
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.count_ += 1

        data = Odometry()
        data.pose.pose.position.x = float(self.p)
        data.twist.twist.linear.x = float(self.v)

        self.publisher_.publish(data)

    def call_environment(self):
        # select a random action
        self.action = self.act

        print(self.act)

        # take the action and observe the next state and reward
        self.obs, self.reward, self.done, self.info,_ = self.env.step(int(self.action)+1)

        # print(self.obs)
        self.p = self.obs[0]
        self.v = self.obs[1]

        # add the reward to the total reward for the episode
        self.total_reward += self.reward

        # render the environment
        self.env.render()

    def listener_callback(self, msg):
        self.act = msg.pose.pose.position.x
        # self.get_logger().info('Received: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    
    node.env.close()
    node.destroy_node()


    rclpy.shutdown()

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

