#include <eigen3/Eigen/Core>
#include <transform.hpp>

Eigen::Matrix4d transform::get_transform(double q, double d, double a, double alpha){

    Eigen::Matrix4d T;
    T << cos(q), -sin(q)*cos(alpha), sin(q)*sin(alpha), a*cos(q),
         sin(q), cos(q)*cos(alpha), -cos(q)*sin(alpha), a*sin(q),
         0, sin(alpha), cos(alpha), d,
         0, 0, 0, 1;
    return T;
}