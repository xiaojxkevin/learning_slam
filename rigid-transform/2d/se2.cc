#include <iostream>
#include <Eigen/Core>
#include "sophus/se2.hpp"

int main() {

    // x, y, angle
    double theta = 0.463171;
    Eigen::Vector2d t(-0.146303, 0.521450);
    Eigen::Matrix3d T;
    T << cos(theta), -sin(theta), t[0],
            sin(theta), cos(theta), t[1],
            0, 0, 1;
    std::cout << T << std::endl;
    Sophus::SE2d mat(theta, t);
    std::cout << mat.matrix() << std::endl;
    Eigen::VectorXd se2 = mat.log();
    std::cout << "\n";
    std::cout << se2 << std::endl;

    return 0;
}