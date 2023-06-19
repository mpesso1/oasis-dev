#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "../lib/SimulatorTemplate/Simulator.hpp"
#include "../lib/SimulatorTemplate/SkipPose.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std;

template <const int DOF, class R>
class Robot : public rclcpp::Node {
    public:
    Robot() : Node("robot") {

        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        Sim = new R();

        output = this->create_subscription<nav_msgs::msg::Odometry>("output",
                                                                    10,
                                                                    std::bind(&Robot::output_cb,
                                                                              this,
                                                                              std::placeholders::_1));
        
        odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        timer_ = this->create_wall_timer(150ms, 
                                         std::bind(&Robot::robot_control_loop,
                                                   this));
    }

    ~Robot() {delete Sim;}

    private:

    void robot_control_loop() {

        Sim->compute_dynamics();

        cout << "odom in robot: " << Sim->odom << endl;


        auto odometry = nav_msgs::msg::Odometry();
        odometry.header.stamp = this->now();
        odometry.header.frame_id = "odom";
        odometry.child_frame_id = "base_link";
        odometry.pose.pose.position.x = Sim->odom(0,0);
        odometry.pose.pose.position.y = Sim->odom(1,0);
        odometry.pose.pose.position.z = Sim->odom(2,0);
        odometry.pose.pose.orientation.x = Sim->odom(3,0);
        odometry.pose.pose.orientation.y = Sim->odom(4,0);
        odometry.pose.pose.orientation.z = Sim->odom(5,0);

        odom->publish(odometry);

        auto tfm = geometry_msgs::msg::TransformStamped();
        tfm.header.stamp = this->now();
        tfm.header.frame_id = "odom";
        tfm.child_frame_id = "base_link";
        tfm.transform.translation.x = odometry.pose.pose.position.x;
        tfm.transform.translation.y = odometry.pose.pose.position.y;
        tfm.transform.translation.z = odometry.pose.pose.position.z;
        tfm.transform.rotation = odometry.pose.pose.orientation;
        tf_broadcaster->sendTransform(tfm);

    }

    void output_cb(const nav_msgs::msg::Odometry::SharedPtr received_data) {
        const auto pose = received_data->pose.pose;
        const auto vel = received_data->twist.twist;

        Eigen::Matrix<float, DOF, 1> pose_vector;

        pose_vector << pose.position.x, pose.position.y, pose.position.z,
                       pose.orientation.x, pose.orientation.y, pose.orientation.z;

        Sim->input = pose_vector;
        // cout << "output: " << Sim->input <<endl;
        
    }

    Simulator<DOF>* Sim;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr output;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};


int main(int argc, char** argv) {

    const int DOF = 6;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot<DOF,SkipPose<DOF>>>());
    rclcpp::shutdown();


    return 0;
}