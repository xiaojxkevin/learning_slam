#include <iostream>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/se2.h>

constexpr int NUM_NODES = 12;
typedef Eigen::Vector<double, 3 * NUM_NODES> state_vector;
typedef Eigen::Vector<double, 3 * NUM_NODES> measurement_vector;

inline measurement_vector read_raw_data(const std::string& file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "Can't open " << file_path << std::endl;
    }
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

// Custom vertex representing an absolute pose in 3D space
// It contains [x, y, theta]
class VertexPose: public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl() // initialize
    {
        _estimate.setZero();
    }
    virtual void oplusImpl( const double* update ) // update
    {
        _estimate += Eigen::Vector3d(update[0], update[1], update[2]);
    }
    virtual bool read( std::istream& in ) {return false;}
    virtual bool write( std::ostream& out ) const {return false;}
};

class EdgeRelativePose: public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexPose, VertexPose> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeRelativePose() {}
    // compute eij
    virtual void computeError() override
    {
        const Eigen::Vector3d& v1 = static_cast<const VertexPose*>(_vertices[0])->estimate();
        const Eigen::Vector3d& v2 = static_cast<const VertexPose*>(_vertices[1])->estimate();
        g2o::SE2 T1, T2;
        T1.fromVector(v1);  T2.fromVector(v2);
        auto relativePose = T1.inverse() * T2;
        Eigen::Vector3d v(relativePose[0], relativePose[1], relativePose[2]);
        _error = relativePose.toVector() - _measurement;
    }
    virtual bool read( std::istream& in ) {return false;}
    virtual bool write( std::ostream& out ) const {return false;}
};


int main() {
    ////////////////////////////////////////////////////
    std::string raw_data_path = "/home/jinxi/codes/learning_slam/pose-graph/project/hw1_data.txt";
    int iterations = 7;
    std::string save_state_path = "/home/jinxi/codes/learning_slam/pose-graph/project/poses/g2o_tum.txt";
    ////////////////////////////////////////////////////
    measurement_vector z = read_raw_data(raw_data_path);
    state_vector x = compute_nodes(z);
    // std::cout << z.transpose() << std::endl;
    // std::cout << x.transpose() << std::endl;

    // create a solver
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 3>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));

    // Create optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // Add vertices (absolute poses)
    for (int i = 0; i != 12; ++i)
    {
        Eigen::Vector3d state = x.block<3, 1>(3 * i, 0);
        auto vertex = new VertexPose();
        vertex->setId(i);
        if (i == 0)
            vertex->setFixed(true);
        vertex->setEstimate(state); // Initial estimate
        optimizer.addVertex(vertex);
    }
    // check if set correctly
    for (int i = 0; i < 12; ++i) {
        VertexPose* v = static_cast<VertexPose*>(optimizer.vertex(i));
        std::cout << "Pose " << i << ": " << v->estimate().transpose() << std::endl;
    }

    // Add edges (relative pose measurements)
    for (int i = 0; i != 12; ++i) 
    {
        int j = (i + 1) % 12;
        auto edge = new EdgeRelativePose();
        edge->setId(i);
        edge->setVertex(0, optimizer.vertex(i));
        edge->setVertex(1, optimizer.vertex(j));
        Eigen::Vector3d m = z.block<3, 1>(3 * i, 0);
        edge->setMeasurement(m);
        edge->setInformation(Eigen::Matrix3d::Identity()); // Set information matrix
        optimizer.addEdge(edge);
    }

    auto start = std::chrono::high_resolution_clock::now(); 

    // Perform optimization
    optimizer.initializeOptimization();
    optimizer.optimize(iterations);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "\nExecution time: " << duration.count() / 1e6 << " s" << std::endl;

    // Retrieve optimized poses
    state_vector new_x;
    for (int i = 0; i < 12; ++i) 
    {
        VertexPose* v = static_cast<VertexPose*>(optimizer.vertex(i));
        auto x_ij = v->estimate();
        new_x.block<3, 1>(3 * i, 0) = x_ij;
    }
    save_final_state_tum(save_state_path, new_x);

    return 0;
}
