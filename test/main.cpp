#include <gtest/gtest.h>

#include "lib.hpp"
#include <transform.hpp>
#include <jacobian.hpp>
#include <joint_angles.hpp>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}