#include <eigen3/Eigen/Core>


class transform{
    private:
        int tool_length; // cm
        double a3;
        double d1;
        double d3;
        double d5;
        double d7;
        double q;
        double d;
        double a;
        double alpha;

    public:
        Eigen::Matrix4d get_transform(double q, double d, double a, double alpha);
        
};