#ifndef MASTERTEMPLATE_HPP
#define MASTERTEMPLATE_HPP

#include <iostream>

using namespace std;

class MasterTemplate {
    public:
    MasterTemplate() {};
    ~MasterTemplate() {};

    virtual void plan_steps() {this->not_using_derived();};

    virtual void release_steps() {this->not_using_derived();};

    virtual void take_step() {this->not_using_derived();};

    void not_using_derived() {cout << "not using derived class \n";}

    vector<vector<float>> steps;
    vector<float> current_step;
    size_t num_of_steps;
    size_t step_idx = 0;
    bool need_new_steps=true;
};


#endif // MASTERTEMPLATE