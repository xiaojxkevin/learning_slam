#include <Eigen/Core>
#include <iostream>

constexpr unsigned MATRIX_SIZE = 10;

int main() {
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> m = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    Eigen::Matrix3d target = m.block(0, 0, 3, 3); 
    target = Eigen::Matrix3d::Identity();
    std::cout << target << std::endl;
    return 0;
}