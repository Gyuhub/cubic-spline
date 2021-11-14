#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

class Trajectory{
public:
    Trajectory();
    ~Trajectory();
    void initialize();
    void initializeStart(Matrix3d start_rot_mat, Vector3d ang_vel, Vector3d ang_acc);
    void initializeGoal(Matrix3d goal_rot_mat);
private:
    double _dt, _start_time, _goal_time;
    Vector3d _ang_vel, _ang_acc;
    Vector4d _a, _b, _c, _x;
};