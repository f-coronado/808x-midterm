#include <eigen3/Eigen/Core>
#include <jacobian.hpp>


Eigen::Matrix<double, 3, 6> jacobian::create_Jx(Eigen::Matrix<double, 3, 1> J_v1,
                           Eigen::Matrix<double, 3, 1> J_v2,
                           Eigen::Matrix<double, 3, 1> J_v3,
                           Eigen::Matrix<double, 3, 1> J_v4,
                           Eigen::Matrix<double, 3, 1> J_v5,
                           Eigen::Matrix<double, 3, 1> J_v6){
    Eigen::Matrix<double, 3, 6> Jx;
    Jx << J_v1, J_v2, J_v3, J_v4, J_v5, J_v6;
    return Jx;
    }