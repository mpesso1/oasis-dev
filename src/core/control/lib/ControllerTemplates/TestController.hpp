#ifndef TESTCONTROLLER_HPP
#define TESTCONTROLLER_HPP

#include "Controller.hpp"

template <const int DOF>
class TestController : public Controller<DOF> {
    public:
    TestController() {}
    ~TestController() {}

    void compute_output() {
        cout << "computing output\n";
        if (below_tolerance()) {
            this->wp_reached = true;
            return;
        }
        // computer ouput

        // store output in output
        this->output << this->p_goal(0,0), this->p_goal(1,0), this->p_goal(2,0),
                        this->p_goal(3,0), this->p_goal(4,0), this->p_goal(5,0);
    }

    bool below_tolerance() {
        Eigen::Matrix<float, DOF, 1> diff;

        diff = this->p_current - this->p_goal;
        diff = diff.cwiseAbs();
        // float diff_sum = diff.sum();

        auto linear = diff.block(0,0,3,1);
        auto angular = diff.block(0,0,3,1);

        if (linear.sum() < this->linear_tolerance && 
            angular.sum() < this->angular_tolerance) {
            
            std::cout << "linear: " << linear << std::endl;
            std::cout << "angular: " << angular << std::endl;
            
            std::cout << "below tolerance\n";
            return true;
        }

        return false;
    }


};


#endif // TESTCONTROLLER