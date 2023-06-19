#ifndef PIDHPP
#define PIDHPP

#include <iostream>
#include "Controller.hpp"
#include <cmath>

using namespace std;

class PIDImpl
{
    public:
        PIDImpl( float dt, 
                 float max, 
                 float min, 
                 float Kp, 
                 float Kd, 
                 float Ki );

        ~PIDImpl();
        float calculate( float setpoint, float pv );
        void reset();

    private:
        float _dt;
        float _max;
        float _min;
        float _Kp;
        float _Kd;
        float _Ki;
        float _pre_error;
        float _integral;
};

class PID {
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID( float dt, float max, float min, float Kp, float Kd, float Ki );

        // Returns the manipulated variable given a setpoint and current process value
        float calculate( float setpoint, float pv );
        void reset() {pimpl->reset();};
        ~PID();



    private:
        PIDImpl *pimpl;
};

PID::PID( float dt, float max, float min, float Kp, float Kd, float Ki )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}
float PID::calculate( float setpoint, float pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID() 
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl( float dt, float max, float min, float Kp, float Kd, float Ki ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
}

void PIDImpl::reset() {
    _integral = 0;
    _pre_error = 0;
}

float PIDImpl::calculate( float setpoint, float pv )
{
    
    // Calculate error
    float error = setpoint - pv;

    // Proportional term
    float Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    float Iout = _Ki * _integral;

    // Derivative term
    float derivative = (error - _pre_error) / _dt;
    float Dout = _Kd * derivative;

    // cout << "::  " << Pout <<" , " << Iout << " , " << Dout << endl;
    // Calculate total output
    float output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}

template <const int DOF>
class PID_C : public Controller<DOF> {
    public:
    PID_C() {
        
        pids(0,0) = new PID(0.1,300,-300,250,370,0);  //200,275
        pids(1,0) = new PID(0.1,300,-300,250,370,0); // 300, 320
        pids(2,0) = new PID(0.1,300,-300,150,370,30); // 105,145,0
        pids(3,0) = new PID(0.1,300,-300,10,0,0);
        pids(4,0) = new PID(0.1,300,-300,10,0,0);
        pids(5,0) = new PID(0.1,300,-300,68,10,30); // 50,10,20

        // pids(0,0) = new PID(0.1,300,-300,250,450,0);  //200,275
        // pids(1,0) = new PID(0.1,300,-300,250,450,0); // 300, 320
        // pids(2,0) = new PID(0.1,300,-300,170,370,30); // 105,145,0
        // pids(3,0) = new PID(0.1,300,-300,10,0,0);
        // pids(4,0) = new PID(0.1,300,-300,10,0,0);
        // pids(5,0) = new PID(0.1,300,-300,65,40,30); // 50,10,20

        // pids(0,0) = new PID(0.1,300,-300,1,0,0);  //200,275
        // pids(1,0) = new PID(0.1,300,-300,1,0,0);
        // pids(2,0) = new PID(0.1,300,-300,1,0,0); // 125, 90
        // pids(3,0) = new PID(0.1,300,-300,1,0,0);
        // pids(4,0) = new PID(0.1,300,-300,1,0,0);
        // pids(5,0) = new PID(0.1,300,-300,1,0,0);    
    };
    ~PID_C() {};

    // PID* pidx;
    // PID* pidy;
    // PID* pidz;
    // PID* pidr;
    // PID* pidp;
    // PID* pidya;


    Eigen::Matrix<PID*,DOF,1> pids;

    void exit_function() {
        for (int i=0;i<DOF;i++){
            if (i != 2) {
                pids(i,0)->reset();
            }
        }
    }

    void compute_output() {
        if (below_tolerance()) {
            this->wp_reached = true;
            return;
        }
        // // computer ouput
        // float input_data[2] = {this->p_current(0,0),this->v_current(0,0)};


        for (int i=0;i<DOF;i++){
            this->output(i,0) = pids(i,0)->calculate(this->p_goal(i,0),
                                                    this->p_current(i,0));
        // if (i == 2) {
        //     cout << "z stuff\n";
        //     cout << this->p_goal(2,0) << endl;
        //     cout << this->p_current(2,0) << endl;
        // }
                                        
        
        }
    }
    
    bool below_tolerance() {
        Eigen::Matrix<float, DOF, 1> diff;

        // Eigen::Matrix<float,DOF,1> fuckyou;
        // fuckyou(0,0) = 5;
        // fuckyou(1,0) = 5;
        // fuckyou(2,0) = 5;
        // fuckyou(3,0) = 5;
        // fuckyou(4,0) = 5;
        // fuckyou(5,0) = 5;

        diff = (this->p_current) - this->p_goal;
        // cout << "true controller error: " << diff << endl;
        if (this->output.sum() != 0) {
            cout << "current pose: " << this->p_current << endl;
            cout << "goal pose: " << this->p_goal << endl;
        }

        diff = diff.cwiseAbs();
        // float diff_sum = diff.sum();

        auto linear = diff.block(0,0,3,1);
        auto angular = diff.block(0,0,3,1);



        if (
            linear(0,0) < this->linear_tolerance && 
            linear(1,0) < this->linear_tolerance &&
            linear(2,0) < this->linear_tolerance &&
            // false &&
            // angular(0,0) < this->angular_tolerance &&
            // angular(1,0) < this->angular_tolerance &&
            abs(this->p_current(5,0) - this->p_goal(5,0)) < this->angular_tolerance
            ) {
            
            std::cout << "below tolerance\n";
            cout << "current pose: " << this->p_current(5,0) << endl;
            cout << "goal pose: " << this->p_goal(5,0) << endl;
            cout << "angular error: "<< angular(2,0) << endl;

            for (int i=0;i<DOF;i++){
                this->output(i,0) = 0;
            }

            return true;
        
        }

        // if (linear.sum() < this->linear_tolerance && 
        //     angular.sum() < this->angular_tolerance) {
            

        // }

        return false;
    }

    private:

};

#endif // PIDHPP