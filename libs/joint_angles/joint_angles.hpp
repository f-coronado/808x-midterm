#pragma once
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <vector>

/**
 * @brief Class for controlling a robot manipulator's joint angles.
 */
class joint_angles {
public:

    /**
     * @brief Finds the velocity inverse kinematics
     * @param q_joint Initial joint angles
     * @param x_dot The desired end effector velocity vector
     * @param J The panda arm jacobian matrix
     */
    Eigen::Matrix<double, 6, 1> velocity_IK(Eigen::Matrix<double, 1, 6> q_joint,
                                        Eigen::Matrix<double, 6, 1> x_dot,
                                        Eigen::Matrix<double, 6, 6> J
                                        );
};