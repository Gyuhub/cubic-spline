#include <math.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
// #include "trajectory.h"
// #include "child.h"

using namespace std;
using namespace Eigen;

#define DEG2RAD (0.01745329251994329576923690768489)
#define RAD2DEG 1/DEG2RAD
#define PI 3.1415926535897932384626433

Matrix3d _R[5];
Vector4d _q[5];
double _dt = 0.002, _time = 0.0;
double _time_step = 1.0, _time_start = 0.0, _time_end = 4.0;
Vector4d _a[4], _b[4], _c[4], _x;
Vector3d _a0, _w0;

Matrix4d Omega(Vector4d x)
{
    Matrix4d mat;
    mat << x(3) ,  x(2), -x(1), -x(0)
        , -x(2) ,  x(3),  x(0), -x(1)
        ,  x(1) , -x(0),  x(3), -x(2)
        ,  x(0) ,  x(1),  x(2),  x(3);
    return mat;
}

Matrix3d Theta(Vector4d x)
{
    Matrix3d mat;
    mat << (x(3)*x(3) + x(0)*x(0) - x(1)*x(1) - x(2)*x(2)) , 2.0 * (x(0)*x(1) - x(2)*x(3))                    , 2.0 * (x(0)*x(2) + x(1)*x(3))
         , 2.0 * (x(0)*x(1) + x(2)*x(3))                   , (x(3)*x(3) - x(0)*x(0) + x(1)*x(1) - x(2)*x(2) ) , 2.0 * (x(1)*x(2) - x(0)*x(3))
         , 2.0 * (x(0)*x(2) - x(1)*x(3))                   , 2.0 * (x(1)*x(2) + x(0)*x(3))                    , (x(3)*x(3) - x(0)*x(0) - x(1)*x(1) + x(2)*x(2) );
    return mat;
}

int main() {
    double theta0_ = 0.0 * DEG2RAD, theta1_ = 90.0 * DEG2RAD, theta2_ = -45.0 * DEG2RAD, theta3_ = 90.0 * DEG2RAD, theta4_ = 90.0 * DEG2RAD;
    _R[0] << cos(theta0_) , -sin(theta0_) ,  0
        ,  sin(theta0_) ,  cos(theta0_) ,  0
        ,   0           ,   0           ,  1;
    _R[1] << cos(theta1_) ,   0           ,  sin(theta1_)
        ,  0            ,   1           ,  0
        , -sin(theta1_) ,   0           ,  cos(theta1_);
    _R[2] << cos(theta2_) ,   0           ,  sin(theta2_)
        ,  0            ,   1           ,  0
        , -sin(theta2_) ,   0           ,  cos(theta2_);
    _R[3] << 1            ,   0           ,  0
        ,  0            ,   cos(theta3_), -sin(theta3_)
        ,  0            ,   sin(theta3_),  cos(theta3_);
    _R[4] << cos(theta4_) , -sin(theta4_) ,  0
        ,  sin(theta4_) ,  cos(theta4_) ,  0
        ,   0           ,   0           ,  1;
    _R[0] = _R[0];
    _R[1] = _R[0] * _R[1];
    _R[2] = _R[0] * _R[1] * _R[2];
    _R[3] = _R[0] * _R[1] * _R[2] * _R[3];
    _R[4] = _R[0] * _R[1] * _R[2] * _R[3] * _R[4];
    Quaterniond q0, q1, q2, q3, q4, q;
    q0 = _R[0];
    q1 = _R[1];
    q2 = _R[2];
    q3 = _R[3];
    q4 = _R[4];
    q = _R[4];
    // q = (_R[0] * _R[1] * _R[2] * _R[3] * _R[4]);
    cout << "Target Rotation Matrix to Quaternion\n0 : " << q0.coeffs().transpose() << "\n1 : " << q1.coeffs().transpose() << "\n2 : " << q2.coeffs().transpose() << "\n3 : " << q3.coeffs().transpose() << "\n4 : " << q4.coeffs().transpose() << '\n';
    cout << "Total Rotation Matrix from 0 to 4\nTotal : " << q.coeffs().transpose() << '\n';
    ///////////////////////////////////////////////////////////////////////////////////
    ////////////////////////// Preprocessing for i = 0 to n ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////
    for (int i = 1; i <= 4; i++)
    {
        double phi = ((_R[i-1]*_R[i]).trace() - 1.0) / 2.0;
        Matrix3d skew = _R[i-1].transpose()*_R[i] - _R[i].transpose()*_R[i-1];
        Vector3d psi;
        psi(0) = skew(2, 1);
        psi(1) = skew(0, 2);
        psi(2) = skew(1, 0);
        _q[i].head(3) = psi * sqrt((1.0 - phi) / 2.0) / psi.norm();
        _q[i](3) = sqrt((1.0 + phi) / 2.0);
    }
    //////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Initialization /////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////
    _c[0] = Eigen::Vector4d::Zero(4);
    Vector4d b0;
    b0 = Omega(_c[0]) * _c[0];
    _b[0](3) = -2.0 * b0(3) / 4.0;
    _b[0].head(3) = (_a0 - b0.head(3) * 2) / 4.0;
    Vector4d z;
    z.setZero();
    z(3) = 1.0;
    _a[0] = _q[1] - _b[0] - _c[0] - z;
    ofstream quaternion, rotmat;
    quaternion.open("/home/kist/cubic-spline/quaternion.txt");
    rotmat.open("/home/kist/cubic-spline/rotmat.txt");
    ///////////////////////////////////////////////////////////////////////////////////
    //////////////////////////// Iteration for i = 2 to n /////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////
    for (int i = 2; i <= 4; i++)
    {
        Vector4d s_, t_, u_; s_.setZero(); t_.setZero(); u_.setZero();
        s_ = _q[i];
        t_ = 3 * _a[i - 2] + 2 * _b[i - 2] + _c[i - 2];
        u_ = 6 * _a[i - 2] + 2 * _b[i - 2];
        _c[i - 1] = Omega(s_) * t_;
        _b[i - 1] = (Omega(t_) * t_ + Omega(s_) * u_ - Omega(_c[i - 1]) * _c[i - 1]) / 2.0;
        _a[i - 1] = s_ - _b[i - 1] - _c[i - 1] - z;
        // cout << i << ": a[" << _a[i -1].transpose() << "], b[" << _b[i - 1].transpose() << "], c[" << _c[i - 1].transpose() << "]\n\n";
    }
    ///////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////// Result //////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////
    Matrix3d R;R.setZero();
    for (double t = _time; _time <= _time_end; _time = _time + _dt)
    {
        if (0 <= _time && _time <= 1.0)
        {
            double tau = (_time - 0) / (1.0 - 0.0);
            _x = _a[0] * std::pow(tau, 3.0) + _b[0] * std::pow(tau, 2.0) + _c[0] * tau + z;
            double x_square = _x.transpose() * _x;
            R = _R[0] * Theta(_x) / x_square;
        }
        else if (1.0 < _time && _time <= 2.0)
        {
            double tau = (_time - 1.0) / (2.0 - 1.0);
            _x = _a[1] * std::pow(tau, 3.0) + _b[1] * std::pow(tau, 2.0) + _c[1] * tau + z;
            double x_square = _x.transpose() * _x;
            R = _R[1] * Theta(_x) / x_square;
        }
        else if (2.0 < _time && _time <= 3.0)
        {
            double tau = (_time - 2.0) / (3.0 - 2.0);
            _x = _a[2] * std::pow(tau, 3.0) + _b[2] * std::pow(tau, 2.0) + _c[2] * tau + z;
            double x_square = _x.transpose() * _x;
            R = _R[2] * Theta(_x) / x_square;
        }
        else if (3.0 < _time && _time <= 4.0)
        {
            double tau = (_time - 3.0) / (4.0 - 3.0);
            _x = _a[3] * std::pow(tau, 3.0) + _b[3] * std::pow(tau, 2.0) + _c[3] * tau + z;
            double x_square = _x.transpose() * _x;
            R = _R[3] * Theta(_x) / x_square;
        }
        Quaterniond q;
        q = R;
        rotmat << _time << '\t' << R(0, 0) << ' ' << R(1, 0) << ' ' << R(2, 0) << ' ' << R(0, 1) << ' ' << R(1, 1) << ' ' << R(2, 1) << ' ' << R(0, 2) << ' ' << R(1, 2) << ' ' << R(2, 2) << '\n';
        quaternion << _time << '\t' << q.coeffs().transpose() << '\n';
    }
    rotmat.close();
    quaternion.close();

    // Quaterniond q;
    // q = Matrix3d::Identity(3, 3);
    // cout << q.coeffs().transpose() << '\n';
    // cout << q.toRotationMatrix() << '\n';
    return 0;
}