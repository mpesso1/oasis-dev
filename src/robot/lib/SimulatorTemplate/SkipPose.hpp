#ifndef SKIPPOSE_HPP
#define SKIPPOSE_HPP

#include "Simulator.hpp"

template <const int DOF>
class SkipPose : public Simulator<DOF> {
    public:
    SkipPose() {}
    ~SkipPose() {}

    void compute_dynamics() {
        cout << "-----\n";
        cout << "current odom: " << this->odom <<endl;
        cout << "input: " << this->input <<endl;
        cout << "-----\n";
        this->odom += this->input;
    }
};


#endif // SKIPPOSE