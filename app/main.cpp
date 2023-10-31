#include <eigen3/Eigen/Core>
#include <iostream>
using Eigen::MatrixXd;
#include <jacobian.hpp>
#include <joint_angles.hpp>

int main() {

    std::cout << "Welcome to the Inverse Kinematics Demo \n\n" 
    <<"Would you like to set your own desired end effector velocity vector" 
    << "and initial joint angles or use predefined values? (y/n):" << std::endl;

    char userInput;
    while (true) {
        std::cin >> userInput;
        if (userInput == 'y' || userInput == 'n') {
            break;  // Valid input, exit the loop
        } else {
            std::cout << "Please enter 'y' for yes or 'n' for no: ";
        }
    }

    jacobian jacobianInstance;
    joint_angles robot;
    Eigen::Matrix<double, 6, 1> x_dot;
    Eigen::Matrix<double, 1, 6> initial_angles(6);


    if (userInput == 'n'){
        x_dot << 0, -4*M_PI*sin(M_PI/2), 4*M_PI*cos(M_PI/2), 0, 0, 0;
        initial_angles << double(0), double(0), double(M_PI/2), double(0), double(M_PI/2), double(0);

        std::cout << "Using EE velocity vector: \n" << x_dot << "\nWith initial joint angles:"
        << initial_angles << std::endl;
        Eigen::Matrix<double, 6, 6> jacobianM = jacobianInstance.j_matrix(double(0), \
                                                                        double(0), \
                                                                        double(M_PI/2), \
                                                                        double(0), \
                                                                        double(M_PI/2), \
                                                                        double(0));
        std::cout << "\n jacobian matrix is as follows: \n" << jacobianM << std::endl;

        // q_dot are the joint velocities we are solving for
        Eigen::Matrix<double, 6, 1> q_dot = robot.velocity_IK(initial_angles, x_dot, jacobianM);
        std::cout << "\nThe required joint angle velocities are:\n " << q_dot << std::endl;
    }
    else{
        std::cout << "Please enter 6 values for your desired end effector velocities: " << std::endl;
        for (int i = 0; i < 6; ++i){
            std::cin >> x_dot[i];
        }
        std::cout << "You entered:\n" << x_dot;

        std::cout << "\nPlease enter 6 values for your desired initial joint angles: " << std::endl;
        for (int i = 0; i < 6; ++i){
            std::cin >> initial_angles[i];
        }
        std::cout << "You entered:\n" << initial_angles;

        Eigen::Matrix<double, 6, 6> jacobianM = jacobianInstance.j_matrix(double(0), \
                                                                        double(0), \
                                                                        double(M_PI/2), \
                                                                        double(0), \
                                                                        double(M_PI/2), \
                                                                        double(0));
        std::cout << "\njacobian matrix is as follows: \n" << jacobianM << std::endl;

        // q_dot are the joint velocities we are solving for
        Eigen::Matrix<double, 6, 1> q_dot = robot.velocity_IK(initial_angles, x_dot, jacobianM);
        std::cout << "\nThe required joint angle velocities are:\n " << q_dot << std::endl;
    }

    return 0;
}