#pragma once
#include <eigen3/Eigen/Core>
#include <symengine/subs.h>
#include <symengine/symbol.h>

/**
 * @file jacobian.hpp
 * @brief Jacobian class for generating Jacobian matrices in a robotic arm controller.
 * @author Fabrizzio Coronado
 * @date 2023
 * @copyright Copyright (c) 2023 by Fabrizzio Coronado.
 */
class jacobian {
private:
    Eigen::Matrix<double, 3, 1> J_v1; /**< Partial derivative of the transformation from base to end effector position with respect to joint 1 */
    Eigen::Matrix<double, 3, 1> J_v2; /**< Partial derivative of the transformation from base to end effector position with respect to joint 2 */
    Eigen::Matrix<double, 3, 1> J_v3; /**< Partial derivative of the transformation from base to end effector position with respect to joint 3 */
    Eigen::Matrix<double, 3, 1> J_v4; /**< Partial derivative of the transformation from base to end effector position with respect to joint 4 */
    Eigen::Matrix<double, 3, 1> J_v5; /**< Partial derivative of the transformation from base to end effector position with respect to joint 5 */
    Eigen::Matrix<double, 3, 1> J_v6; /**< Partial derivative of the transformation from base to end effector position with respect to joint 6 */
    Eigen::Matrix<double, 3, 1> Z_0; /**< A 3x1 matrix (column vector) */
    Eigen::Matrix<double, 6, 6> J; /**< The Jacobian matrix (6x6) */
    Eigen::Matrix<double, 6, 6> J_inv; /**< The inverse of the Jacobian matrix (6x6) */
    Eigen::Matrix<double, 6, 1> q_dot; /**< A 6x1 matrix representing joint velocities */

public:
    /**
     * @brief Constructs a 3x6 Jacobian matrix using six 3x1 vectors.
     * @param J_v1 Partial derivative of transformation from base to end effector position with respect to joint 1.
     * @param J_v2 Partial derivative of transformation from base to end effector position with respect to joint 2.
     * @param J_v3 Partial derivative of transformation from base to end effector position with respect to joint 3.
     * @param J_v4 Partial derivative of transformation from base to end effector position with respect to joint 4.
     * @param J_v5 Partial derivative of transformation from base to end effector position with respect to joint 5.
     * @param J_v6 Partial derivative of transformation from base to end effector position with respect to joint 6.
     * @return A 3x6 Jacobian matrix.
     */
    Eigen::Matrix<double, 3, 6> create_Jx(Eigen::Matrix<double, 3, 1> J_v1,
                                          Eigen::Matrix<double, 3, 1> J_v2,
                                          Eigen::Matrix<double, 3, 1> J_v3,
                                          Eigen::Matrix<double, 3, 1> J_v4,
                                          Eigen::Matrix<double, 3, 1> J_v5,
                                          Eigen::Matrix<double, 3, 1> J_v6);

    /**
     * @brief Computes the velocity inverse kinematics.
     * @param q_joint The joint angles of the manipulator (6x1 matrix).
     * @param x_dot The velocity of the end effector (6x1 matrix).
     * @param J The Jacobian matrix (6x6) used in the computation.
     * @return A 6x1 matrix representing the joint velocities.
     */
    Eigen::Matrix<double, 6, 1> velocity_IK(Eigen::Matrix<double, 1, 6> q_joint,
                                            Eigen::Matrix<double, 6, 1> x_dot,
                                            Eigen::Matrix<double, 6, 6> J);
};
