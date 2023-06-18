#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tutorial_interfaces/srv/goal_pose.hpp"
#include "srv_interfaces/srv/create_path.hpp"
#include "srv_interfaces/srv/path_waypoint_send.hpp"

#include "../lib/PlannerTemplates/PathPlanner.h"
#include "../lib/PlannerTemplates/_goPath.h"
#include "../lib/PlannerTemplates/noPath.h"


using namespace std;

template <const int DOF, class P>
class PathPLanner : public rclcpp::Node
{
public:
  PathPLanner() : Node("pathplanner")
  {
    // create path planner
    PathPlanner = new P();

    // // subscribe to odometry
    // odom = this->create_subscription<nav_msgs::msg::Odometry>(
    //   "odom", 10, std::bind(&PathPLanner::cb, this, std::placeholders::_1));

    
    sendWayPoint = this->create_service<srv_interfaces::srv::PathWaypointSend>("goalpose",
                                                                             std::bind(&PathPLanner::sendWayPointcb, 
                                                                             this, 
                                                                             std::placeholders::_1,
                                                                             std::placeholders::_2));

    // create service to send next waypoint to controller after computing path
    create_path = this->create_service<srv_interfaces::srv::CreatePath>("create_path",
                                                                        std::bind(&PathPLanner::create_path_cb, 
                                                                        this,
                                                                        std::placeholders::_1,
                                                                        std::placeholders::_2));
    cout << "constructed\n";
  }

  ~PathPLanner() {delete PathPlanner;};

private:

  void create_path_cb(const srv_interfaces::srv::CreatePath::Request::SharedPtr request,
                      srv_interfaces::srv::CreatePath::Response::SharedPtr response) 
  {

    Eigen::Matrix<float, 1, 6> goal_pose;
    goal_pose << request->g_x, request->g_y, request->g_z, request->g_roll,
                 request->g_pitch, request->g_yaw;

    PathPlanner->add_goal_pose(goal_pose);

    Eigen::Matrix<float, 1, 6> current_pose;
    current_pose << request->c_x, request->c_y, request->c_z, request->c_roll,
                    request->c_pitch, request->c_yaw;

    PathPlanner->add_init_pose(current_pose);

    Eigen::Matrix<float, 1, 6> goal_vel;
    goal_vel << request->g_vx, request->g_vy, request->g_vz, request->g_vroll,
                request->g_vpitch, request->g_vyaw;

    PathPlanner->add_goal_vel(goal_vel);

    Eigen::Matrix<float, 1, 6> current_vel;
    current_vel << request->c_vx, request->c_vy, request->c_vz, request->c_vroll,
                   request->c_vpitch, request->c_vyaw;

    PathPlanner->add_init_vel(current_vel);
    
    PathPlanner->setup();
    PathPlanner->compute();
    PathPlanner->store_results();

    path_length = PathPlanner->path_trans.rows();
    wp_idx = 1;

  }

  void sendWayPointcb(const srv_interfaces::srv::PathWaypointSend::Request::SharedPtr request,
                      srv_interfaces::srv::PathWaypointSend::Response::SharedPtr response) 
  {
    cout << "----------\n";
    // cout << "wp index: " << wp_idx << endl;
    // cout << "path length: " << path_length << endl;
    // cout << "path trans: " << endl << PathPlanner->path_trans << endl;
    // cout << "path angular: " << endl << PathPlanner->path_angular << endl;
    // cout << "trans size: " << PathPlanner->path_trans.cols() << " x " << PathPlanner->path_trans.rows() << endl;
    cout << "ang size: " << PathPlanner->path_angular.cols() << " x " << PathPlanner->path_angular.rows() << endl;
    cout << wp_idx << endl;
    cout << "--------e-\n";

    
    
    if (wp_idx < path_length) {
      cout << "x: " << PathPlanner->path_trans(wp_idx,0) << endl;
      response->done = false;
      response->x = PathPlanner->path_trans(wp_idx,0);
      response->y = PathPlanner->path_trans(wp_idx,1);
      response->z = PathPlanner->path_trans(wp_idx,2);
      if (PathPlanner->path_angular.cols() == 1) {
        if (PathPlanner->path_angular(wp_idx,0) != abs(PathPlanner->path_angular(wp_idx,0))) {
          response->yaw = PathPlanner->path_angular(wp_idx,0);
          cout << "thz: " << PathPlanner->path_angular(wp_idx,0) << endl;
        }
        else {
          response->yaw = PathPlanner->path_angular(wp_idx,0);
          cout << "thz: " << PathPlanner->path_angular(wp_idx,0) << endl;
        }
      }
      else {
        response->roll = PathPlanner->path_angular(wp_idx,0); // response->roll = 1; 
        response->pitch = PathPlanner->path_angular(wp_idx,1);
        response->yaw = PathPlanner->path_angular(wp_idx,2);
      }
      wp_idx += 1;
    } else {
      response->x = PathPlanner->path_trans(wp_idx-1,0);
      response->y = PathPlanner->path_trans(wp_idx-1,1);
      response->z = PathPlanner->path_trans(wp_idx-1,2);
      if (PathPlanner->path_angular.cols() == 1) {
        response->yaw = PathPlanner->path_angular(wp_idx-1,0);
      }
      else {
        response->roll = PathPlanner->path_angular(wp_idx-1,0); // response->roll = 1; 
        response->pitch = PathPlanner->path_angular(wp_idx-1,1);
        response->yaw = PathPlanner->path_angular(wp_idx-1,2);
      }
      response->done = true;
    }

  }
  // void cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  // {
  //   // RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
  // }

  size_t wp_idx = 1;
  size_t path_length = 0;
  pp::PathPlanner<DOF> *PathPlanner;
  // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom;
  rclcpp::Service<srv_interfaces::srv::PathWaypointSend>::SharedPtr sendWayPoint;
  rclcpp::Service<srv_interfaces::srv::CreatePath>::SharedPtr create_path;
};


int main(int argc, char** argv) {
  const int DOF = 6;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPLanner<DOF, _goPath<DOF>>>());
  rclcpp::shutdown();
  return 0;
}