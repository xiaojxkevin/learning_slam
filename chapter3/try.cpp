#define _USE_MATH_DEFINES
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

int main() {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix());
    T.pretranslate(Eigen::Vector3d(0, 20, 15));
    std::cout << "Rotation Matrix:\n" << T.matrix() << std::endl;
    Eigen::Vector3d p = T * Eigen::Vector3d(0, 10, 0);
    std::cout << "Vector:\n" << p << std::endl;
    return 0;
}

