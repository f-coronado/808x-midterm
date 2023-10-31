#include <gtest/gtest.h>
#include <transform.hpp>
#include <jacobian.hpp>
#include <joint_angles.hpp>

TEST(jacobianTest, getJ){

  jacobian jacobian;
  Eigen::Matrix<double, 6, 6> expectedJ;
  expectedJ << -8.8, -49.3, 0, -8.8,  1.26751e-15, 0,
                0, -10.9, 0, 5.38845e-16, 20.7, 0,
                0, 0, 8.8, 0, 8.8, 0,
                0, 0, -0, -0, 1, 0,
                0, -1, -0, -0, -6.12323e-17, -0,
                1, 0, 1, 1, 0, 1;

  Eigen::Matrix<double, 6, 6> J = jacobian.j_matrix(0, 0, M_PI/2, 0, M_PI/2, 0);

  EXPECT_TRUE(J.isApprox(expectedJ, 1e-4));
};

TEST(jointAnglesTest, getQdot){

  joint_angles angles;
  jacobian jacobian;
  
  Eigen::Matrix<double, 6, 1> x_dot;
  x_dot << 0, -4*M_PI*sin(M_PI/2), 4*M_PI*cos(M_PI/2), 0, 0, 0;
  Eigen::Matrix<double, 1, 6> initial_angles(6);
  initial_angles << double(0), double(0), double(M_PI/2), double(0), double(M_PI/2), double(0);
  Eigen::Matrix<double, 6, 6> J = jacobian.j_matrix(0, 0, M_PI/2, 0, M_PI/2, 0);

  Eigen::Matrix<double, 6, 1> q_dot = angles.velocity_IK(initial_angles, x_dot, J);

}
