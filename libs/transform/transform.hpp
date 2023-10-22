#include <eigen3/Eigen/Core>


class transform{
    private:
        int tool_length; // cm
        double a3;
        double d1;
        double d3;
        double d5;
        double d7;

    public:
        Eigen::Matrix4d get_transform(int q, double d, double a, double alpha);
        
};