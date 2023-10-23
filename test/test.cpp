#include <gtest/gtest.h>
#include <transform.hpp>
#include <jacobian.hpp>

TEST(TransformTest, GetTransformFn) {
  double q = .5*M_PI;
  double d = 9;
  double a = 5;
  double alpha = 0;
  transform transform;
  Eigen::Matrix4d T = transform.get_transform(q, d, a, alpha);

  Eigen::Matrix4d expectedT;
    expectedT << cos(q), -sin(q)*cos(alpha), sin(q)*sin(alpha), a*cos(q),
                sin(q), cos(q)*cos(alpha), -cos(q)*sin(alpha), a*sin(q),
                0, sin(alpha), cos(alpha), d,
                0, 0, 0, 1;
  EXPECT_TRUE(T.isApprox(expectedT, 1e-4));
  };

TEST(JacobianTest, GetJx){

  jacobian jacobian;
  Eigen::Matrix<double, 3, 1> J_v;
  J_v << 1,
         0,
         1;
  Eigen::Matrix<double, 3, 1> J_w;
  J_w << 2,
         0,
         2;
  Eigen::Matrix<double, 3, 6> expectedJ;
  expectedJ << J_v, J_v, J_v, J_w, J_w, J_w;

  Eigen::Matrix<double, 3, 6> Jx = jacobian.create_Jx(J_v, J_v, J_v, J_w, J_w, J_w);

  EXPECT_TRUE(Jx.isApprox(expectedJ, 1e-4));
};