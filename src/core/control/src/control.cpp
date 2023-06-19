#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"

#include "../lib/ControllerTemplates/Controller.hpp"
#include "srv_interfaces/srv/create_path.hpp"
#include "srv_interfaces/srv/goal_waypoint_request.hpp"
#include "srv_interfaces/srv/goal_waypoint_reached.hpp"
#include "srv_interfaces/srv/path_waypoint_send.hpp"

#include "../lib/ControllerTemplates/TestController.hpp"
#include "../lib/ControllerTemplates/MountainCarController_DeepQ.hpp"
#include "../lib/ControllerTemplates/MemoryDynamics.hpp"
#include "../lib/ControllerTemplates/PID.hpp"

using namespace std;

template <const int DOF, class C>
class Control : public rclcpp::Node {
    public:
    Control() : Node("control") {

        // controller
        c = new C();

        // create client to ask path planner for next waypoint
        getPathwp_client = this->create_client<srv_interfaces::srv::PathWaypointSend>("goalpose");

        // create publisher to send output commands
        output_pub = this->create_publisher<nav_msgs::msg::Odometry>("output", 10);

        // create a client to let master know when goal pose has been reached
        goalwp_reached = this->create_client<srv_interfaces::srv::GoalWaypointReached>("goal_waypoint_reached");

        // create a server to accept goal pose request from master
        goalwp_request = this->create_service<srv_interfaces::srv::GoalWaypointRequest>("goal_waypoint",
                                                                                        std::bind(&Control::get_goalwp_cb,
                                                                                                  this,
                                                                                                  std::placeholders::_1,
                                                                                                  std::placeholders::_2));

        // create subsciber to recieve current pose
        pose_sub = this->create_subscription<nav_msgs::msg::Odometry>("pose", 
                                                                      10, 
                                                                      std::bind(&Control::get_pose_cb, 
                                                                                this, 
                                                                                std::placeholders::_1));

        // create client to ask for path from path planner
        client_waypoint_request = this->create_client<srv_interfaces::srv::CreatePath>("create_path");

        timer_ = this->create_wall_timer(150ms, 
                                         std::bind(&Control::control_control_loop, this));

        cout << "constructed\n";
        
    }

    ~Control() {delete c;};

    private:

    // get goal waypoint from master
    void get_goalwp_cb(const srv_interfaces::srv::GoalWaypointRequest::Request::SharedPtr request,
                       srv_interfaces::srv::GoalWaypointRequest::Response::SharedPtr response) 
    {
        // callback to master for getting next waypoint

        goalwp << request->x, request->y, request->z,
                  request->roll, request->pitch, request->yaw;

        response->received = true;

        cout << "recieved goal waypoint from master\n";
        cout << "requesting for path\n";

        cout << "goal from master: " << goalwp<<endl;
        // cout<<"current: "<<c->p_current<<endl;

        // c->wp_reached = false;
        // ask for next goal waypoint from path planner
        auto _request = std::make_shared<srv_interfaces::srv::CreatePath::Request>();
        // goal position
        _request->g_x = goalwp(0,0);
        _request->g_y = goalwp(1,0);
        _request->g_z = goalwp(2,0);
        _request->g_roll = goalwp(3,0);
        _request->g_pitch = goalwp(4,0);
        _request->g_yaw = goalwp(5,0);
        // current position
        _request->c_x = c->p_current(0,0);
        _request->c_y = c->p_current(1,0);
        _request->c_z = c->p_current(2,0);
        _request->c_roll = c->p_current(3,0);
        _request->c_pitch = c->p_current(4,0);
        _request->c_yaw = c->p_current(5,0);

        // auto future = client_waypoint_request->async_send_request(request);
        client_waypoint_request->async_send_request(_request, 
                                                    std::bind(&Control::create_path_cb,
                                                              this,
                                                              std::placeholders::_1));
    }

    void create_path_cb(rclcpp::Client<srv_interfaces::srv::CreatePath>::SharedFuture future) {
        // call back from path planner after it has successfully created a path
        cout << "returned from pathplanner\n";
        auto response = future.get();
        if (response) {
            cout << "path recieved\n";
            cout << "control loop on\n";
            not_at_goal_wp = true;
        }
    }


    // store current pose in controller
    void get_pose_cb(const nav_msgs::msg::Odometry::SharedPtr recieve_data) {
        // callback to slam that updates current pose for controller

        // const float init_offset = 5;

        const auto pose  = recieve_data->pose.pose;
        const auto vel = recieve_data->twist.twist;

        

        Eigen::Matrix<float, 6, 1> pose_vector;

        if (abs(c->p_goal(5,0) - pose.orientation.z) < abs(c->p_goal(5,0) - pose.orientation.w)) {
            // w has negative and z stays positive
            cout << "from z\n";
            pose_vector << pose.position.x, 
                        pose.position.y, 
                        pose.position.z, 
                        pose.orientation.x, 
                        pose.orientation.y, 
                        pose.orientation.z;

        }
        else {
            cout << "from w\n";
            pose_vector << pose.position.x, 
                        pose.position.y, 
                        pose.position.z, 
                        pose.orientation.x, 
                        pose.orientation.y, 
                        pose.orientation.w;
        }

        

        // pose_vector << pose.position.x, 
        //                pose.position.y, 
        //                pose.position.z, 
        //                pose.orientation.x, 
        //                pose.orientation.y, 
        //                .1;

        cout << "pose from slam: " << endl << pose_vector << endl;
        
        Eigen::Matrix<float, 6, 1> vel_vector;

        vel_vector << vel.linear.x, vel.linear.y, vel.linear.z,
                      vel.angular.x, vel.angular.y, vel.angular.z;

        c->add_pose_velocity_current(pose_vector, 
                                     vel_vector);
        // cout << "pose: " << c->p_goal << endl;

    }

    // node control loop
    void control_control_loop() {

        if (not_at_goal_wp) { // set to true once a path is created by path planner
            this->_compute_action();
        }

    }

    void _compute_action () {

        // generate a new waypoint
        // NOTE: it is the job of the path planner to determine if a 
        // new path needs to be computed... it is the job of the controller 
        // to determine if a the goal pose has been reached
        if (c->wp_reached) { 
            c->wp_reached = false;
            got_mp = false;
            cout << "asking for midpoint\n";

            // take care of any reset on controller
            // that needs to happen before acting on next waypoint
            c->exit_function();
            
            auto _request = std::make_shared<srv_interfaces::srv::PathWaypointSend::Request>();
            getPathwp_client->async_send_request(_request, 
                                                 std::bind(&Control::waypoint_request_cb,
                                                           this,
                                                           std::placeholders::_1));
        }

        if (got_mp) {
            cout << "controller computing\n";

            // compute controller output
            // must specify criterion for goal pose and current pose meeting each other
            c->compute_output(); 

            if (c->output.sum() != 0) {
                cout << "controller output: "<<c->output<<endl;    
            }

            // send controller output to ros
            auto output = nav_msgs::msg::Odometry();
            output.pose.pose.position.x = c->output(0,0);
            output.pose.pose.position.y = c->output(1,0);
            output.pose.pose.position.z = c->output(2,0);
            output.pose.pose.orientation.x = c->output(3,0);
            output.pose.pose.orientation.y = c->output(4,0);
            output.pose.pose.orientation.z = c->output(5,0);

            output_pub->publish(output);

            // cout << c->output << endl;
        }

        // not_at_goal_wp = false; //////////////////////////////// remove after test
    }

    void waypoint_request_cb(rclcpp::Client<srv_interfaces::srv::PathWaypointSend>::SharedFuture future) {
        // callback to path planner for getting next midpoint
        got_mp=true;
        auto response = future.get();
        
        if (response) {
            // store controller goal pose
            cout << "got midpoint from path planner or at end\n" << response->x;

            if (response->done) {
                cout << "control loop off\n";

                // send controller output to ros
                auto output = nav_msgs::msg::Odometry();
                output.pose.pose.position.x = 0;
                output.pose.pose.position.y = 0;
                output.pose.pose.position.z = 0;
                output.pose.pose.orientation.x = 0;
                output.pose.pose.orientation.y = 0;
                output.pose.pose.orientation.z = 0;

                output_pub->publish(output);

                not_at_goal_wp = false;

                // notify master that we have reached goal pose
                auto request = std::make_shared<srv_interfaces::srv::GoalWaypointReached::Request>();
                request->reached = true;
                // auto future = goalwp_reached->async_send_request(request);
                goalwp_reached->async_send_request(request, 
                                                std::bind(&Control::goal_pose_reached_cb,
                                                        this,
                                                        std::placeholders::_1));
            }

            Eigen::Matrix<float, 6, 1> goal_pose_vector;

            goal_pose_vector << response->x, response->y, response->z,
                                response->roll, response->pitch, response->yaw;
            
            cout << "goal in controller: " << goal_pose_vector << endl;

            c->add_goal_pose(goal_pose_vector);

            // Eigen::Matrix<float, 6, 1> goal_vel_vector;

            // goal_vel_vector << response->vx, response->vy, response->vz,
            //                    response->vroll, response->vpitch, response->vyaw;      

            // c->add_pose_velocity_goal(goal_pose_vector, goal_vel_vector);


        } else {
            cout << "Failure\n";
            exit(1);
        }
    }

    void goal_pose_reached_cb(rclcpp::Client<srv_interfaces::srv::GoalWaypointReached>::SharedFuture future) {
        auto response = future.get();
        if (response) {           
            if (!response->understood) {
                cout << "no more waypoints from master\n";
            }
        }
    }

    bool got_mp = false;
    Controller<DOF> *c;
    Eigen::Matrix<float, 6, 1> goalwp;
    bool not_at_goal_wp = false;
    float wp_reached_linear_tolerance = .01;
    float wp_reached_angular_tolerance = .01;

    rclcpp::Client<srv_interfaces::srv::CreatePath>::SharedPtr client_waypoint_request;
    rclcpp::Service<srv_interfaces::srv::GoalWaypointRequest>::SharedPtr goalwp_request;
    rclcpp::Client<srv_interfaces::srv::GoalWaypointReached>::SharedPtr goalwp_reached;
    rclcpp::Client<srv_interfaces::srv::PathWaypointSend>::SharedPtr getPathwp_client;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    const int DOF = 6;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Control<DOF, PID_C<DOF>>>());
    rclcpp::shutdown();

    return 0;
}



/*
import gym
import numpy as np
import random
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam

env = gym.make('MountainCarContinuous-v0')
state_size = env.observation_space.shape[0]
action_size = env.action_space.shape[0]

class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=2000)
        self.gamma = 0.95
        self.epsilon = 1.0
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 0.001
        self.model = self._build_model()

    def _build_model(self):
        model = Sequential()
        model.add(Dense(24, input_dim=self.state_size, activation='relu'))
        model.add(Dense(24, activation='relu'))
        model.add(Dense(self.action_size, activation='linear'))
        model.compile(loss='mse', optimizer=Adam(lr=self.learning_rate))
        return model

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        if np.random.rand() <= self.epsilon:
            return self.env.action_space.sample()
        act_values = self.model.predict(state)
        return act_values[0]

    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        for state, action, reward, next_state, done in minibatch:
            target = reward
            if not done:
                target = (reward + self.gamma *
                      np.amax(self.model.predict(next_state)[0]))
            target_f = self.model.predict(state)
            target_f[0][action] = target
            self.model.fit(state, target_f, epochs=1, verbose=0)
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def load(self, name):
        self.model.load_weights(name)

    def save(self, name):
        self.model.save_weights(name)

agent = DQNAgent(state_size, action_size)

n_episodes = 1000
batch_size = 32

for i in range(n_episodes):
    state = env.reset()
    state = np.reshape(state, [1, state_size])
    for t in range(500):
        action = agent.act(state)
        next_state, reward, done, _ = env.step(action)
        next_state = np.reshape(next_state, [1, state_size])
        agent.remember(state, action, reward, next_state, done)
        state = next_state
        if done:
            print("episode: {}/{}, score: {}".format(i, n_episodes, t))
            break
    if len(agent.memory) > batch_size:
        agent.replay(batch_size)



*/