#pragma once
#include <eigen3/Eigen/Core>

/**
 * @brief jacobian class contains members used for creating jacobian matrices
 * controller.
 */
class jacobian {
    private:
        Eigen::Matrix<double, 3, 1> J_v1;
        Eigen::Matrix<double, 3, 1> J_v2;
        Eigen::Matrix<double, 3, 1> J_v3;
        Eigen::Matrix<double, 3, 1> J_v4;
        Eigen::Matrix<double, 3, 1> J_v5;
        Eigen::Matrix<double, 3, 1> J_v6;
        Eigen::Matrix<double, 3, 1> Z_0;


    public:
        
        /**
        * @brief Method that constructs a 3x6 vector once passed 6 3x1 vectors.
        * @param J_v1 the partial derivative of the tranformation from the base to end effector psotion wrt joint 1
        * @param J_v2 the partial derivative of the tranformation from the base to end effector psotion wrt joint 2
        * @param J_v3 the partial derivative of the tranformation from the base to end effector psotion wrt joint 3
        * @param J_v4 the partial derivative of the tranformation from the base to end effector psotion wrt joint 4
        * @param J_v5 the partial derivative of the tranformation from the base to end effector psotion wrt joint 5
        * @param J_v6 the partial derivative of the tranformation from the base to end effector psotion wrt joint 6       
        */
        Eigen::Matrix<double, 3, 6> create_Jx(Eigen::Matrix<double, 3, 1> J_v1,
                                  Eigen::Matrix<double, 3, 1> J_v2,
                                  Eigen::Matrix<double, 3, 1> J_v3,
                                  Eigen::Matrix<double, 3, 1> J_v4,
                                  Eigen::Matrix<double, 3, 1> J_v5,
                                  Eigen::Matrix<double, 3, 1> J_v6);

        /**
        * @brief Method returns the velocity inverse kinematics
        * @param q_joint the joint angles of the manipulator
        * @param x_dot the velocity of the end effector
        */
        Eigen::Matrix<double, 6, 1> velocity_IK(Eigen::Matrix<double, 1, 6> q_joint,
                                                Eigen::Matrix<double, 6, 1> x_dot
                                                Eigen::Matrix<double, 6, 6> J
                                                );

};