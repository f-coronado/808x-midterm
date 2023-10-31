#include "jacobian.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

/**
 * @file jacobian.hpp
 * @brief Jacobian class for generating Jacobian matrices in a robotic arm controller.
 * @author Fabrizzio Coronado
 * @date 2023
 * @copyright Copyright (c) 2023 by Fabrizzio Coronado.
 */

Eigen::Matrix <double, 6, 6> jacobian::j_matrix(double q1, 
                                            double q2, 
                                            double q3, 
                                            double q4, 
                                            double q5, 
                                            double q6){
    Eigen::Matrix <double, 6, 6> J;
    J(0, 0)= 31.6*sin(q1)*sin(q2) + 8.8*sin(q1)*sin(q6)*sin(q2-q4) - \
    20.7*sin(q1)*sin(q6)*cos(q5)*cos(q2-q4)-20.7*sin(q1)*sin(q2-q4)*cos(q6) + \
    38.4*sin(q1)*sin(q2-q4) - 8.8*sin(q1)*cos(q2) - 8.8*sin(q1)*cos(q5)*cos(q6)*cos(q2-q4) - \
    20.7*sin(q5)*sin(q6)*cos(q1) - 8.8*sin(q5)*cos(q1)*cos(q6);
    J(1, 0) = -20.7*sin(q1)*sin(q5)*sin(q6) - 8.8*sin(q1)*sin(q5)*cos(q6) - \
    31.6*sin(q2)*cos(q1) - 8.8*sin(q6)*sin(q2-q4)*cos(q1) + \
    20.7*sin(q6)*cos(q1)*cos(q5)*cos(q2-q4)*cos(q1)*cos(q6) - 38.4*sin(q2-q4)*cos(q1) + \
    8.8*cos(q1)*cos(q2) + 8.8*cos(q1)*cos(q5)*cos(q6)*cos(q2-q4) - 8.8*cos(q1)*cos(q2-q4);
    J(2, 0) = 0;
    J(3, 0) = 0;
    J(4, 0) = 0;
    J(5, 0) = 1;

    J(0, 1) = (-8.8*sin(q2) - 20.7*sin(q6)*sin(q2 - q4)*cos(q4) - 8.8*sin(q6)*cos(q2 - q4) -\
    8.8*sin(q2 - q4)*cos(q4)*cos(q6) + 8.8*sin(q2 - q4) - 31.6*cos(q2) + \
    20.7*cos(q6)*cos(q2 - q4) - 38.4*cos(q2 - q4))*cos(q1);
    J(1, 1) = -8.8*sin(q2) - 20.7*sin(q6)*sin(q2 - q4)*cos(q5) - 8.8*sin(q6)*cos(q2 - q4) - \
    8.8*sin(q2 - q4)*cos(q5)*cos(q6) + 8.8*sin(q2 - q4) - 31.6*cos(q2) + \
    20.7*cos(q6)*cos(q2 - q4) - 38.4*cos(q2 - q4)*sin(q1);
    J(2, 1) = -31.6*sin(q2) - 8.8*sin(q6)*sin(q2 - q4) + 20.7*sin(q6)*cos(q5)*cos(q2 - q4) + \
    20.7*sin(q2 - q4)*cos(q6) - 38.4*sin(q2 - q4) + 8.8*cos(q2) + \
    8.8*cos(q5)*cos(q6)*cos(q2 - q4) - 8.8*cos(q2 - q4);
    J(3, 1) = sin(q1);
    J(4, 1) = -cos(q1);
    J(5, 1) = 0;

    J(0, 2) = (20.7*sin(q6)*sin(q2 - q4)*cos(q5) + 8.8*sin(q6)*cos(q2 - q4) + \
    8.8*sin(q2 - q4)*cos(q5)*cos(q6) - 8.8*sin(q2 - q4) - 20.7*cos(q6)*cos(q2 - q4) + \
    38.4*cos(q2 - q4))*sin(q1);
    J(1, 2) = (20.7*sin(q6)*sin(q2 - q4)*cos(q5) + 8.8*sin(q6)*cos(q2 - q4) + \
    8.8*sin(q2 - q4)*cos(q5)*cos(q6) - 8.8*sin(q2 - q4) - 20.7*cos(q6)*cos(q2 - q4) + \
    38.4*cos(q2 - q4))*sin(q1);
    J(2, 2) = 8.8*sin(q6)*sin(q2 - q4) - 20.7*sin(q6)*cos(q5)*cos(q2 - q4) - \
    20.7*sin(q2 - q4)*cos(q6) + 38.4*sin(q2 - q4) - 8.8*cos(q5)*cos(q6)*cos(q2 - q4) + \
    8.8*cos(q2 - q4);
    J(3, 2) = -sin(q2)*cos(q1);
    J(4, 2) = -sin(q1)*sin(q2);
    J(5, 2) = cos(q2);


    J(0, 3) = -(sin(q1)*cos(q5) + sin(q5)*cos(q1)*cos(q2 - q4))*(20.7*sin(q6) + 8.8*cos(q6));
    J(1, 3) = -(sin(q1)*sin(q5)*cos(q2 - q4) - cos(q1)*cos(q5))*(20.7*sin(q6) + 8.8*cos(q6));
    J(2, 3) = -(20.7*sin(q6) + 8.8*cos(q6))*sin(q5)*sin(q2 - q4);
    J(3, 3) = -sin(q2 - q4)*cos(q1);
    J(4, 3) = -sin(q1)*sin(q2 - q4);
    J(5, 3) = cos(q2 - q4);

    J(0, 4) = 8.8*(sin(q1)*sin(q5) - cos(q1)*cos(q5)*cos(q2 - q4))*sin(q6) - \
    20.7*(sin(q1)*sin(q5) - cos(q1)*cos(q5)*cos(q2 - q4))*cos(q6) - \
    20.7*sin(q6)*sin(q2 - q4)*cos(q1) - 8.8*sin(q2 - q4)*cos(q1)*cos(q6);
    J(1, 4) = -8.8*(sin(q1)*cos(q5)*cos(q2 - q4) + sin(q5)*cos(q1))*sin(q6) + \
    20.7*(sin(q1)*cos(q5)*cos(q2 - q4) + sin(q5)*cos(q1))*cos(q6) - \
    20.7*sin(q1)*sin(q6)*sin(q2 - q4) - 8.8*sin(q1)*sin(q2 - q4)*cos(q6);
    J(2, 4) = -8.8*sin(q6)*sin(q2 - q4)*cos(q5) + 20.7*sin(q6)*cos(q2 - q4) + \
    20.7*sin(q2 - q4)*cos(q5)*cos(q6) + 8.8*cos(q6)*cos(q2 - q4);
    J(3, 4) = sin(q1)*cos(q5) + sin(q5)*cos(q1)*cos(q2 - q4);
    J(4, 4) = sin(q1)*sin(q5)*cos(q2 - q4) - cos(q1)*cos(q5);
    J(5, 4) = sin(q5)*sin(q2 - q4);

    J(0, 5) = 0;
    J(1, 5) = 0;
    J(2, 5) = 0;
    J(3, 5) = (sin(q1)*sin(q5) - cos(q1)*cos(q5)*cos(q2 - q4))*sin(q6) - sin(q2 - q4)*cos(q1)*cos(q6);
    J(4, 5) = -(sin(q1)*cos(q5)*cos(q2 - q4) + sin(q5)*cos(q1))*sin(q6) - sin(q1)*sin(q2 - q4)*cos(q6);
    J(5, 5) = -sin(q6)*sin(q2 - q4)*cos(q5) + cos(q6)*cos(q2 - q4);
    return J;
}