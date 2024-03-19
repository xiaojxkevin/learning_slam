#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Sparse>

// g++ pose-graph/project/optimize.cc -o ./opt -I /usr/include/eigen3/

constexpr int NUM_NODES = 12;
typedef Eigen::Vector<double, 3 * NUM_NODES> state_vector;
typedef Eigen::Vector<double, 3 * NUM_NODES> measurement_vector;
typedef Eigen::Vector<double, 3 * NUM_NODES> coefficient_vector;
typedef Eigen::Matrix<double, 3, 3 * NUM_NODES> J_matrix;
typedef Eigen::Matrix<double, 3 * NUM_NODES, 3 * NUM_NODES> H_matrix;

inline measurement_vector read_data(const std::string& file_path)
{
    std::ifstream file(file_path);
    std::string line;
    state_vector x;
    for (int i = 0; i != 3; ++i)
    {
        getline(file, line);
        std::istringstream iss(line);
        double value;
        int j = 0;
        while (iss >> value)
        {
            x[3 * j + i] = value;
            j += 1;
        }
    }
    file.close();
    return x;
}

inline Eigen::Vector3d t2v(const Eigen::Matrix3d& T)
{
    Eigen::Matrix2d tmp;
    tmp << T(0, 0), T(0, 1), T(1, 0), T(1, 1);
    Eigen::Rotation2Dd R;
    R.fromRotationMatrix(tmp);
    Eigen::Vector3d v(T(0, 2), T(1, 2), R.angle());
    return v;
}

inline Eigen::Matrix3d v2t(const Eigen::Vector3d& v)
{
    double theta = v[2];
    Eigen::Matrix3d T;
    T << cos(theta), -sin(theta), v[0],
        sin(theta), cos(theta), v[1],
        0, 0, 1;
    return T;
}

inline state_vector compute_nodes(const measurement_vector& z)
{
    state_vector nodes;
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    auto node = t2v(T);
    nodes.block<3, 1>(0, 0) = node;
    for (int i = 0; i != NUM_NODES - 1; ++i)
    {
        Eigen::Vector3d relative_transform = z.block<3, 1>(3 * i, 0);
        T *=  v2t(relative_transform);
        auto new_state = t2v(T);
        nodes.block<3, 1>((i+1)*3, 0) = new_state;
    }
    return nodes;
}

inline Eigen::Vector3d compute_tilde_z_ij(int i, int j, const state_vector& x)
{
    Eigen::Vector3d x_i = x.block<3, 1>(3 * i, 0);
    auto T_i = v2t(x_i);
    Eigen::Vector3d x_j = x.block<3, 1>(3 * j, 0);
    auto T_j = v2t(x_j);
    auto T_ij = T_i.inverse() * T_j;
    auto tilde_z_ij = t2v(T_ij);
    return tilde_z_ij;
}

inline Eigen::Vector3d compute_eij(int i, int j, const state_vector& x, const measurement_vector& z)
{
    Eigen::Vector3d tilde_z_ij = compute_tilde_z_ij(i, j, x), 
                    z_ij = z.block<3, 1>(3 * i, 0);
    return tilde_z_ij - z_ij;
}

inline void compute_gradients(int i, int j, const state_vector& x, 
                        Eigen::Matrix3d& A_ij, Eigen::Matrix3d& B_ij)
{
    double theta_i = x(3 * i + 2, 0);
    Eigen::Vector2d t_i = x.block<2, 1>(3 * i, 0);
    Eigen::Vector2d t_j = x.block<2, 1>(3 * j, 0);
    Eigen::Matrix2d inv_Ri, paritial_Ri_theta;
    inv_Ri << cos(theta_i), sin(theta_i),
                -sin(theta_i), cos(theta_i);
    paritial_Ri_theta << -sin(theta_i), cos(theta_i),
                            -cos(theta_i), -sin(theta_i);
    A_ij.block<2, 2>(0, 0) = -inv_Ri;
    A_ij.block<2, 1>(0, 2) = paritial_Ri_theta * (t_j - t_i);
    A_ij(2, 2) = -1;
    B_ij.block<2, 2>(0, 0) = inv_Ri;
}

inline void compute_J_ij(int i, int j, J_matrix& J, 
                const Eigen::Matrix3d& A_ij, const Eigen::Matrix3d& B_ij)
{
    J.block<3, 3>(0, 3 * i) = A_ij;
    J.block<3, 3>(0, 3 * j) = B_ij;
}

inline void compute_H_ij(int i, int j, H_matrix& H, 
                const Eigen::Matrix3d& A_ij, const Eigen::Matrix3d& B_ij)
{
    H.block<3, 3>(i*3, i*3) = A_ij.transpose() * A_ij;
    H.block<3, 3>(i*3, j*3) = A_ij.transpose() * B_ij;
    H.block<3, 3>(j*3, i*3) = B_ij.transpose() * A_ij;
    H.block<3, 3>(j*3, j*3) = B_ij.transpose() * B_ij;
}

inline void compute_b_ij(int i, int j, coefficient_vector& b, const Eigen::Vector3d& e_ij, 
                    const Eigen::Matrix3d& A_ij, const Eigen::Matrix3d& B_ij)
{
    b.block<3, 1>(3 * i, 0) = A_ij.transpose() * e_ij;
    b.block<3, 1>(3 * j, 0) = B_ij.transpose() * e_ij;
}

int main() {
    // The value of z would be edges Z1_2, Z2_3, ..., Z11_12, Z12_1
    // which is the Lie-algebra for relative poses
    measurement_vector z = read_data("./pose-graph/project/hw1_data.txt");
    state_vector x = compute_nodes(z);
    state_vector dx = state_vector::Ones();
    coefficient_vector b;
    H_matrix H;

    auto start = std::chrono::high_resolution_clock::now(); 

    while (dx.norm() >= 1e-3) // TODO: to find the condition
    {
        b = coefficient_vector::Zero();
        H = H_matrix::Zero();   
        for (int i = 0; i != NUM_NODES; ++i)
        {
            int j = (i+1) % NUM_NODES;
            Eigen::Matrix3d A_ij = Eigen::Matrix3d::Zero(), B_ij = Eigen::Matrix3d::Identity();
            compute_gradients(i, j, x, A_ij, B_ij);
            Eigen::Vector3d eij = compute_eij(i, j, x, z);
            // std::cout << compute_eij(i, j, x, z) << std::endl;
            compute_H_ij(i, j, H, A_ij, B_ij);
            compute_b_ij(i, j, b, eij, A_ij, B_ij);
        }
        // std::cout << H << std::endl;
        // keep the first node fixed
        H.block<3, 3>(0, 0) += Eigen::Matrix3d::Identity();

        Eigen::SimplicialLDLT<H_matrix> solver;
        solver.compute(H);
        dx = solver.solve(-b);
        x += dx;
        break;
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Execution time: " << duration.count() << " microseconds" << std::endl;

    return 0;
}
