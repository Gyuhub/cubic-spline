#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

class Trajectory{
public:
    Trajectory(int a) : _a(a){cout << "Trajectory init! a is " << _a << '\n';}
    void initialize() {
    std::cout << "Trajectry initialize!\n";}

private:
    double _a;
};