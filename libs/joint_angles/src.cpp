#include <iostream>
#include <joint_angles.hpp>

// Include any relevant libraries or headers for your robot control interface

Eigen::Matrix<double, 6, 1> joint_angles::velocity_IK(
    Eigen::Matrix<double, 1, 6> q_joint, Eigen::Matrix<double, 6, 1> x_dot,
    Eigen::Matrix<double, 6, 6> J) {
  Eigen::Matrix<double, 6, 1> q_dot = J.inverse() * x_dot;
  return q_dot;
}
