#include <eigen3/Eigen/Core>

class jacobian {
    private:
        Eigen::Matrix<double, 3, 1> J_v1;
        Eigen::Matrix<double, 3, 1> J_v2;
        Eigen::Matrix<double, 3, 1> J_v3;
        Eigen::Matrix<double, 3, 1> J_v4;
        Eigen::Matrix<double, 3, 1> J_v5;
        Eigen::Matrix<double, 3, 1> J_v6;

    public:
        Eigen::Matrix<double, 3, 6> create_Jx(Eigen::Matrix<double, 3, 1> J_v1,
                                  Eigen::Matrix<double, 3, 1> J_v2,
                                  Eigen::Matrix<double, 3, 1> J_v3,
                                  Eigen::Matrix<double, 3, 1> J_v4,
                                  Eigen::Matrix<double, 3, 1> J_v5,
                                  Eigen::Matrix<double, 3, 1> J_v6);
};