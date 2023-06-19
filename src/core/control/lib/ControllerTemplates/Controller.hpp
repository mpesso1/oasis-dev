#ifndef CONTROLLER
#define CONTROLLER

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"

#include <algorithm>


using namespace std;

template <const int DOF>
class Controller {
    private:
    public:

    Controller() {};
    virtual ~Controller(){cout << "Destructing Controller\n";};

    // variables
    Eigen::Matrix<float, DOF, 1> p_goal = Eigen::ArrayXXf::Zero(DOF, 1); // goal pose
    Eigen::Matrix<float, DOF, 1> p_current = Eigen::ArrayXXf::Zero(DOF, 1); // current pose

    Eigen::Matrix<float, DOF, 1> v_goal = Eigen::ArrayXXf::Zero(DOF, 1); // goal velocity
    Eigen::Matrix<float, DOF, 1> v_current = Eigen::ArrayXXf::Zero(DOF, 1); // current velocity

    Eigen::Matrix<float, DOF, 1> output; // output from controller


    virtual void add_goal_pose(Eigen::Matrix<float, DOF, 1> &pose) {p_goal = pose;};
    virtual void add_current_pose(Eigen::Matrix<float, DOF, 1> &pose) {p_current = pose;};
    virtual void add_goal_velocity(Eigen::Matrix<float, DOF, 1> &vel) {v_goal = vel;};
    virtual void add_current_velocity(Eigen::Matrix<float, DOF, 1> &vel) {v_current = vel;};

    virtual void add_pose_velocity_goal(Eigen::Matrix<float, DOF, 1> &pose, 
                                        Eigen::Matrix<float, DOF, 1> &vel) {
                                            this->add_goal_pose(pose);
                                            this->add_goal_velocity(vel);
                                        };
    virtual void add_pose_velocity_current(Eigen::Matrix<float, DOF, 1> &pose, 
                                            Eigen::Matrix<float, DOF, 1> &vel) {
                                                this->add_current_pose(pose);
                                                this->add_current_velocity(vel);
                                            };

    virtual void clear_goal_pose(Eigen::Matrix<float, DOF, 1> &pose) {this->p_goal = Eigen::ArrayXXf::Zero(DOF,1);};
    virtual void clear_current_pose(Eigen::Matrix<float, DOF, 1> &pose) {this->p_current = Eigen::ArrayXXf::Zero(DOF,1);};
    virtual void clear_goal_velocity(Eigen::Matrix<float, DOF, 1> &vel) {this->v_goal = Eigen::ArrayXXf::Zero(DOF,1);};
    virtual void clear_current_velocity(Eigen::Matrix<float, DOF, 1> &vel) {this->v_current = Eigen::ArrayXXf::Zero(DOF,1);};

    virtual void clear_pose_velocity_goal(Eigen::Matrix<float, DOF, 1> &pose, 
                                        Eigen::Matrix<float, DOF, 1> &vel) {
                                            this->clear_goal_pose(pose);
                                            this->clear_goal_velocity(vel);
                                        };
    virtual void clear_pose_velocity_current(Eigen::Matrix<float, DOF, 1> &pose, 
                                            Eigen::Matrix<float, DOF, 1> &vel) {
                                                this->clear_current_pose(pose);
                                                this->clear_current_velocity(vel);
                                            };


    // put specific initialization in this function
    virtual void initialize() { this->print_no_derived_class(); };

    // put specific computation in this function and assign it to output
    // responsible for setting wp_reached to true once error has between current pose and final pose under tolerance
    virtual void compute_output() { this->print_no_derived_class(); };   

    // put specific code that should be exicuted after reaching goal
    virtual void exit_function() { this->print_no_derived_class(); };


    void print_no_derived_class() {cout << "Derived Class Not Computing Anything\n";};

    bool wp_reached = true;

    float linear_tolerance = 0.3;
    float angular_tolerance = 0.12;

};



#endif // CONTROLLER