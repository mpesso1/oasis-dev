#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <iostream>
#include <Eigen/Dense>

using namespace std;

template <const int DOF>
class Simulator {
    public:
    Simulator() : input(Eigen::ArrayXXf::Zero(DOF, 1)),
                  odom(Eigen::ArrayXXf::Zero(DOF, 1))
     {}
    ~Simulator() {}

    virtual void compute_dynamics() { print_no_derived(); }

    void print_no_derived() { cout << "no derived class\n";}

    Eigen::Matrix<float, DOF, 1> input;
    Eigen::Matrix<float, DOF, 1> odom;
};

#endif // SIMULATOR