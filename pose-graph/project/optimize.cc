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
typedef Eigen::SparseMatrix<double> H_matrix;


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

inline void compute_H_ij(int i, int j, H_matrix& H, 
                const Eigen::Matrix3d& A_ij, const Eigen::Matrix3d& B_ij)
{
    for (int k = 0; k != 3; ++k)
    {
        for (int l = 0; l != 3; ++l)
        {
            H.coeffRef(i*3+k, i*3+l) += (A_ij.transpose() * A_ij)(k, l);
            H.coeffRef(i*3+k, j*3+l) += (A_ij.transpose() * B_ij)(k, l);
            H.coeffRef(j*3+k, i*3+l) += (B_ij.transpose() * A_ij)(k, l);
            H.coeffRef(j*3+k, j*3+l) += (B_ij.transpose() * B_ij)(k, l);
        }
    }
}

inline void compute_b_ij(int i, int j, coefficient_vector& b, const Eigen::Vector3d& e_ij, 
                    const Eigen::Matrix3d& A_ij, const Eigen::Matrix3d& B_ij)
{
    b.block<3, 1>(3 * i, 0) += A_ij.transpose() * e_ij;
    b.block<3, 1>(3 * j, 0) += B_ij.transpose() * e_ij;
}

inline void save_H(unsigned int k, const std::string& file_folder, const H_matrix& matrix)
{
    std::string intString = std::to_string(k);
    intString = std::string(3 - intString.length(), '0') + intString;
    std::string save_h_filename = file_folder + intString + ".txt";
    std::ofstream file(save_h_filename);
    if (!file.is_open()) 
    {
        std::cerr << "Error opening file " << save_h_filename << std::endl;
        return;
    }
    for (int k = 0; k < matrix.outerSize(); ++k) 
    {
        for (H_matrix::InnerIterator it(matrix, k); it; ++it) 
        {
            file << it.row() << " " << it.col() << " " << it.value() << std::endl;
        }
    }
    file.close();
}

inline void save_final_state_tum(std::string& filename, state_vector& x)
{
    std::ofstream file(filename);
    file << std::fixed << std::setprecision(6);
    for (int i = 0; i != NUM_NODES; ++i)
    {
        int idx = 3 * i;
        double theta = x(idx+2, 0);
        file << i << " "
            << x(idx,0) << " " << x(idx+1, 0) << " " << 0.0 << " "
            << 0 << " " << 0 << " " << sin(theta/2) << " " << cos(theta/2) << std::endl;
    }
}

int main() {
    // set some run-time constants
    ////////////////////////////////////////////////////
    std::string raw_data_path = "./pose-graph/project/hw1_data.txt";
    double convergence_criterion = 1e-9;
    std::string save_state_path = "./pose-graph/project/poses/opt_tum.txt";
    std::string save_h_folder = "pose-graph/project/information_matrix/";
    ////////////////////////////////////////////////////

    measurement_vector z = read_data(raw_data_path);
    state_vector x = compute_nodes(z);
    coefficient_vector b;
    H_matrix H(3 * NUM_NODES, 3 * NUM_NODES);
    unsigned int iterations(1);

    auto start = std::chrono::high_resolution_clock::now(); 

    while (iterations <= 999)
    {
        b.setZero();
        H.setZero();
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
        // save_H(iterations, save_h_folder, H);

        // keep the first node fixed
        for (int i = 0; i != 3; ++i)
                H.coeffRef(i, i) += 1.0;

        // solve the equation
        Eigen::SimplicialCholesky<H_matrix> chol(H);
        Eigen::VectorXd dx = chol.solve(-b);
        if (chol.info() != Eigen::Success) {
            // decomposition failed
            std::cerr << "Decomposition failed!" << std::endl;
            return 1;
        }
        if (dx.norm() < convergence_criterion)
            break;
        x += dx;
        // std::cout << dx.norm() << std::endl;
        ++iterations;
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Finished with " << iterations << " iterations\n";
    std::cout << "Execution time: " << duration.count() / 1e6 << " s" << std::endl;

    for (int i = 0; i != 3; ++i)
        H.coeffRef(i, i) -= 1.0;
    save_H(iterations, save_h_folder, H);
    save_final_state_tum(save_state_path, x);

    return 0;
}
