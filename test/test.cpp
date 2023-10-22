#include <gtest/gtest.h>
#include <transform.hpp>
#include <cmath>

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
  std::cout << "\nexpectedT:" << expectedT << std::endl;
  std::cout << "\nT:" << T << std::endl;

  EXPECT_TRUE(T.isApprox(expectedT, 1e-4));
  };
