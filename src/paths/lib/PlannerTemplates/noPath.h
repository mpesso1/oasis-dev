#ifndef NOPATH
#define NOPATH

template <const int DOF>
class noPath : public pp::PathPlanner<DOF> {
private:
protected:
public:

    noPath() {}

    void setup() override {};

    void compute() override {
        this->path_trans.resize(2,3);
        this->path_angular.resize(2,3);
        std::cout << "goal pose in nopath compute: " << this->goal_pose << std::endl;
        this->path_trans.row(0) << this->goal_pose(0,0), this->goal_pose(0,1), this->goal_pose(0,2);
        this->path_trans.row(1) << this->goal_pose(0,0), this->goal_pose(0,1), this->goal_pose(0,2);
        this->path_angular.row(0) << this->goal_pose(0,3), this->goal_pose(0,4), this->goal_pose(0,5);
        this->path_angular.row(1) << this->goal_pose(0,3), this->goal_pose(0,4), this->goal_pose(0,5);
        // cout << this->path_trans << endl;
    };

    void store_results() override {};

};

#endif // NOPATH