import socket
import json
from time import sleep

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np

class MyNode(Node):
    def __init__(self,client):
        super().__init__('dvl_client')
        self.odom = self.create_publisher(Odometry,'DVL_ODOM',10)
        self.doppler = self.create_publisher(Odometry,'DVL_DOPPLER',10)
        # self.odom = rospy.Publisher("DVL_ODOM", Odometry, queue_size=100)
        # self.doppler = rospy.Publisher(
        #      "/DVL_DOPPLER", Odometry, queue_size=100)
        
        self.timer_ = self.create_timer(.014, self.timer_callback)

        self.dvl = client
        self.dead_reckon = Odometry()
        self.doppler_data = Odometry()

        self.last_time = 0
        self.lastx = 0
        self.lasty = 0
        self.lastz = 0
        self.lastr = 0
        self.lastp = 0
        self.lastya = 0
        self.dt = 1000000000000

    def determine_loops(self,angle):
        i = 0
        if angle != abs(angle): # angle is negative
            while True:
                if angle < 0:
                    angle+=2*np.pi
                    i-=1
                else:
                    return i
        else:
            while True:
                if angle > 2*np.pi:
                    angle-=2*np.pi
                    i+=1
                else:
                    return i



    def timer_callback(self):
        data = json.loads(self.dvl.get().decode("utf-8"))
        
        # print(data)

        # self.dead_reckon.header.stamp = rospy.Time.now()
        # self.doppler_data.header.stamp = rospy.Time.now()

        # print('mason')
        if (data['type'] == 'velocity'):
            if (data['vx'] != 0):
                self.doppler_data.twist.twist.linear.x = data['vx']
                self.doppler_data.twist.twist.linear.y = data['vy']
                self.doppler_data.twist.twist.linear.z = data['vz']
                # print(data['vx'])
                self.doppler.publish(self.doppler_data)

        elif (data['type'] == 'position_local'):
            print(self.get_clock().now().nanoseconds)
            self.dead_reckon.header.stamp = self.get_clock().now().to_msg()
            # self.dead_reckon.pose.pose.position.x = data['x']
            # self.dead_reckon.pose.pose.position.y = data['y']
            self.dead_reckon.pose.pose.position.x = data['x']*np.cos(data['yaw']) - data['y']*np.sin(data['yaw'])
            self.dead_reckon.pose.pose.position.y = data['x']*np.sin(data['yaw']) + data['y']*np.cos(data['yaw'])
            self.dead_reckon.pose.pose.position.z = data['z']
            self.dead_reckon.pose.pose.orientation.x = data['roll']
            self.dead_reckon.pose.pose.orientation.y = data['pitch']
            print('message: ', self.dead_reckon.header.stamp)
            
            yaw = data['yaw']*np.pi/180
            if (yaw != abs(yaw)):
                yaw += 2*np.pi

            if (self.lastya
             - yaw) > np.pi: # from 6 to 0
                yaw += 2*np.pi
            elif (self.lastya - yaw) < -np.pi: # from 0 to 6
                yaw -= 2*np.pi

            self.dead_reckon.pose.pose.orientation.z = float(yaw)
            self.dead_reckon.pose.pose.orientation.w = float(self.determine_loops(yaw))

            self.dead_reckon.twist.twist.linear.x = (data['x'] - self.lastx)/self.dt
            self.dead_reckon.twist.twist.linear.y = (data['y'] - self.lasty)/self.dt
            self.dead_reckon.twist.twist.linear.z = (data['z'] - self.lastz)/self.dt
            self.dead_reckon.twist.twist.angular.x = (data['roll'] - self.lastr)/self.dt
            self.dead_reckon.twist.twist.angular.y = (data['pitch'] - self.lastp)/self.dt
            self.dead_reckon.twist.twist.angular.z = (yaw - self.lastya)/self.dt

            self.lastx = data['x']
            self.lasty = data['y']
            self.lastz = data['z']
            self.lastr = data['roll']
            self.lastp = data['pitch']
            self.lastya = yaw

            time_now = self.get_clock().now().nanoseconds
            self.dt = (time_now - self.last_time)/1000000000
            # print((time_now - self.last_time)/1000000000)
            self.last_time = time_now 
            

            self.odom.publish(self.dead_reckon)

        


class DVLclient():
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port

    def connect(self):
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((self.ip, self.port))
            self.s.settimeout(1)
        except socket.error as err:
            sleep(1)
            self.connect()

    def get(self):
        raw_data = b''

        while True:
            rec = self.s.recv(1)
            # print(rec)
            if rec == b'\n':
                break
            raw_data += rec
        # print(raw_data)
        return (raw_data)

    def run(self):
        while True:
            self.get()

def main():
    rclpy.init(args=None)
    TCP_IP = "192.168.2.95"
    TCP_PORT = 16171

    client = DVLclient(TCP_IP, TCP_PORT)
    client.connect()

    node = MyNode(client)
    rclpy.spin(node)
    node.env.close()
    node.destroy_node()


    rclpy.shutdown()

    # ros.run()

if __name__ == '__main__':
    main()