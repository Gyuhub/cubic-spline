#pragma once
#include "trajectory.h"

class Child : public Trajectory
{
public:
    Child(int a) : Trajectory(a){cout << "Child init!! a is" << a << '\n';}
    void Cinitialize() {std::cout << "child init!\n";};
};