#include <eigen3/Eigen/Core>
#include <iostream>
#include <jacobian.hpp>
#include <symengine/symbol.h>
#include <symengine/symengine_exception.h>
using SymEngine::Expression;
using SymEngine::symbol;
using Eigen::MatrixXd;
#include <symengine/derivative.h>
#include <transform.hpp>
#include <joint_angles.hpp>



int main() {
  std::cout << "Demo..." << std::endl;
  jacobian jacobian;
  transform transform;
  // joint_angles joint_angles;

  Eigen::Matrix<double, 1, 6> q_joint;
  q_joint << 0, 0, M_PI/2, 0, M_PI, 0;

  Eigen::Matrix<double, 1, 6> x_dot;
  x_dot << 0, -4*M_PI*sin(M_PI/2), 4*M_PI*cos(M_PI/2), 0, 0, 0;
  std::cout << "\nUsing x_dot: " << x_dot << "\nand q_joint: " << q_joint << "\n";

  Eigen::Matrix4d T_01 = transform.get_transform(q_joint(0,0), transform.d1, 0, M_PI/2);
  Eigen::Matrix4d T_12 = transform.get_transform(q_joint(0,1), 0, 0, -M_PI/2);
  Eigen::Matrix4d T_23 = transform.get_transform(0, transform.d3, transform.a3, -M_PI/2);
  Eigen::Matrix4d T_34 = transform.get_transform(q_joint(0,3), 0, -transform.a3, M_PI/2);
  Eigen::Matrix4d T_45 = transform.get_transform(q_joint(0,4), transform.d5, 0, M_PI/2);
  Eigen::Matrix4d T_56 = transform.get_transform(q_joint(0,5), 0, transform.a3, -M_PI/2);
  Eigen::Matrix4d T_6n = transform.get_transform(1, -transform.d7, 0, 0);

  Eigen::Matrix4d T_02 = T_01 * T_12;
  Eigen::Matrix4d T_03 = T_02 *T_23;
  Eigen::Matrix4d T_04 = T_02 * T_23 * T_34;
  Eigen::Matrix4d T_05 = T_04 * T_45;
  Eigen::Matrix4d T_06 = T_05 * T_56;
  Eigen::Matrix4d T_0n = T_06 * T_6n;
  std::cout << "Transformation from base to end effector is: \n" << T_0n << std::endl;

  Eigen::Matrix <double, 3, 1> x_p;
  x_p << T_0n(0, 3), T_0n(1, 3), T_0n(2, 3);
  std::cout << "x_p:\n" << x_p << std::endl;

  

}
