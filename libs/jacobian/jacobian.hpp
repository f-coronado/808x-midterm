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
 *
 * This code is licensed under the MIT License.
 */
class jacobian {
public:
    double q1; /**< joint 1 angle */
    double q2; /**< joint 1 angle */
    double q3; /**< joint 1 angle */
    double q4; /**< joint 1 angle */
    double q5; /**< joint 1 angle */
    double q6; /**< joint 1 angle */

    /**
     * @brief Calculates the jacobian with user defined angles
     * @param q1 Joint 1 angle
     * @param q2 Joint 2 angle
     * @param q3 Joint 3 angle
     * @param q4 Joint 4 angle
     * @param q5 Joint 5 angle
     * @param q6 Joint 6 angle
     */
    Eigen::Matrix <double, 6, 6> j_matrix(double q1, 
                                        double q2, 
                                        double q3, 
                                        double q4, 
                                        double q5, 
                                        double q6);
};