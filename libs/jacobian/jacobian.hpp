#pragma once
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Dense>

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
    double q1;
    double q2;
    double q3;
    double q4;
    double q5;
    double q6;

    Eigen::Matrix <double, 6, 6> j_matrix(double q1, 
                                        double q2, 
                                        double q3, 
                                        double q4, 
                                        double q5, 
                                        double q6);
};