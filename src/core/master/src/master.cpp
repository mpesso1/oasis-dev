#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "srv_interfaces/srv/goal_waypoint_request.hpp"
#include "srv_interfaces/srv/goal_waypoint_reached.hpp"
#include "srv_interfaces/srv/key_commands.hpp"
#include "std_msgs/msg/string.hpp"

#include "../lib/MasterTemplates/MasterTemplate.hpp"
#include "../lib/Masters/WaypointReader.hpp"

using namespace std;


template <class M>
class Master : public rclcpp::Node {
  public:

  Master() : Node("master") {

    MasterTemp = new M();

    // start command subscriber
    start_command = this->create_subscription<std_msgs::msg::String>("start",
                                                                     10,
                                                                     std::bind(&Master::start_command_sub_cb,
                                                                               this,
                                                                               std::placeholders::_1));

    // create server to get notified whenever goal wp has been reached
    goal_wp_reached_server = this->create_service<srv_interfaces::srv::GoalWaypointReached>("goal_waypoint_reached",
                                                                                            std::bind(&Master::goal_wp_reached_server_cb, 
                                                                                                      this,
                                                                                                      std::placeholders::_1,
                                                                                                      std::placeholders::_2));

    // create client to send out goal waypoint GoalWaypointReached
    goal_wp_request_client = this->create_client<srv_interfaces::srv::GoalWaypointRequest>("goal_waypoint");


    timer_ = this->create_wall_timer(150ms, std::bind(&Master::master_control_loop, this));
  }

  ~Master() {delete MasterTemp;}

  private:

  void start_command_sub_cb(const std_msgs::msg::String::SharedPtr request) {
    cout << "recieved start command\n";

    // give controller new goal wp
    auto _request = std::make_shared<srv_interfaces::srv::GoalWaypointRequest::Request>();
    
    for (size_t i=0;i<6;i++){
      cout <<MasterTemp->current_step[i]<<endl;
    }
    
    // fill in request information
    _request->x = MasterTemp->current_step[0];
    _request->y = MasterTemp->current_step[1];
    _request->z = MasterTemp->current_step[2];
    _request->roll = MasterTemp->current_step[3];
    _request->pitch = MasterTemp->current_step[4];
    _request->yaw = MasterTemp->current_step[5];

    // make request to server
    // auto future = goal_wp_request_client->async_send_request(_request);
    goal_wp_request_client->async_send_request(_request, 
                                                std::bind(&Master::sent_goal_pose_cb,
                                                          this,
                                                          std::placeholders::_1));
  }

  void goal_wp_reached_server_cb(const srv_interfaces::srv::GoalWaypointReached::Request::SharedPtr request,
                                 srv_interfaces::srv::GoalWaypointReached::Response::SharedPtr response)
  {
    if (MasterTemp->step_idx < MasterTemp->num_of_steps) { // s := start key was pressed
      

      response->understood = true;

      cout << "sending next waypoint\n";


      for (size_t i=0;i<6;i++){
        cout <<MasterTemp->current_step[i]<<endl;
      }
      
      // give controller new goal wp
      auto _request = std::make_shared<srv_interfaces::srv::GoalWaypointRequest::Request>();
      
      // fill in request information
      _request->x = MasterTemp->current_step[0];
      _request->y = MasterTemp->current_step[1];
      _request->z = MasterTemp->current_step[2];
      _request->roll = MasterTemp->current_step[3];
      _request->pitch = MasterTemp->current_step[4];
      _request->yaw = MasterTemp->current_step[5];

      // make request to server
      // auto future = goal_wp_request_client->async_send_request(_request);
      goal_wp_request_client->async_send_request(_request, 
                                                 std::bind(&Master::sent_goal_pose_cb,
                                                           this,
                                                           std::placeholders::_1));
      
    } else {
      response->understood = false;
    }
  }

  void sent_goal_pose_cb(rclcpp::Client<srv_interfaces::srv::GoalWaypointRequest>::SharedFuture future) {
    auto response = future.get();
    if (response) {
      // cout << "successfully sent goal pose\n";
      MasterTemp->step_idx += 1;
      if (MasterTemp->step_idx < MasterTemp->num_of_steps) {
        MasterTemp->current_step = MasterTemp->steps[MasterTemp->step_idx];
      }

    }
  }

  void master_control_loop() {
    if (MasterTemp->need_new_steps) {
      MasterTemp->plan_steps();
      MasterTemp->need_new_steps = false;
    }

  }

  MasterTemplate *MasterTemp;

  rclcpp::Service<srv_interfaces::srv::GoalWaypointReached>::SharedPtr goal_wp_reached_server;
  rclcpp::Client<srv_interfaces::srv::GoalWaypointRequest>::SharedPtr goal_wp_request_client;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_command;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Master<WaypointReader>>());
  rclcpp::shutdown();
  return 0;
}