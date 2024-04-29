#include <iostream>
#include <chrono>
#include <stdexcept>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include "Lie.h"

unsigned NUM_NODES(0);
// [quat, translation, scale] == [qw, qx, qy, qz, x, y, z, s]
typedef std::vector<Vector8d> NODES;
// index pairs for edges
typedef std::pair<unsigned, unsigned> INDEX_PAIR;
// Define the list to store all edges
typedef std::vector<Vector8d> EDGES;

inline NODES read_nodes(const std::string& file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open())
        throw std::invalid_argument("no such file!\n");
    std::string line;
    getline(file, line);
    NUM_NODES = std::stod(line);
    NODES nodes;
    std::cout << "Number of Nodes: " <<  NUM_NODES << std::endl;
    for (int i = 0; i != NUM_NODES; ++i)
    {
        Vector8d node;
        getline(file, line);
        std::istringstream iss(line);
        unsigned time_stamp;
        iss >> time_stamp;
        iss >> node(4) >> node(5) >> node(6)
            >> node(1) >> node(2) >> node(3) >> node(0)
            >> node(7);
        // represent nodes in world frame
        node = Sim3Inv(node);
        nodes.push_back(node);
    }
    file.close();
    return nodes;
}

inline EDGES read_edges(const std::string& file_path, 
                        std::vector<INDEX_PAIR>& edge_pairs)
{
    std::ifstream file(file_path);
    if (!file.is_open())
        throw std::invalid_argument("no such file!\n");
    std::string line;
    getline(file, line);
    double num_edges = std::stod(line);
    if (num_edges != NUM_NODES)
        throw std::invalid_argument("Not matched!\n");
    EDGES edges;
    std::cout << "Number of Edges: " <<  num_edges << std::endl;
    for (int i = 0; i != num_edges; ++i)
    {
        INDEX_PAIR pair;
        Vector8d edge;
        getline(file, line);
        std::istringstream iss(line);
        iss >> pair.first; iss >> pair.second;
        edge_pairs.push_back(pair);
        iss >> edge(4) >> edge(5) >> edge(6)
            >> edge(1) >> edge(2) >> edge(3) >> edge(0)
            >> edge(7);
        edges.push_back(edge);
    }
    // to make the last edge be transformation from id100 to id0
    INDEX_PAIR last_pair = edge_pairs[num_edges - 1];
    edge_pairs[num_edges - 1].first = last_pair.second;
    edge_pairs[num_edges - 1].second = last_pair.first;
    edges[num_edges - 1] = Sim3Inv(edges[num_edges - 1]);
    file.close();
    return edges;
}

inline void save_opt_nodes(std::string& filename, const NODES& nodes)
{
    std::ofstream file(filename);
    std::cout << "*** saving new nodes to file " << filename << std::endl;
    file << std::fixed << std::setprecision(6);
    for (int i = 0; i != NUM_NODES; ++i)
    {
        auto node = nodes[i];
        Eigen::Vector3d trans(node(4), node(5), node(6));
        Eigen::Quaterniond quat(node(0), node(1), node(2), node(3));
        Eigen::Matrix3d rot(quat);
        rot *= node(7);
        Eigen::Quaterniond s_quat(rot);
        file << i << " "
            << node(4) << " " << node(5) << " " << node(6) << " "
            << s_quat.x() << " " << s_quat.y() << " " << s_quat.z() << " " << s_quat.w() << std::endl;
    }
}

// Custom vertex representing an absolute pose in 3D space
class VertexPose: public g2o::BaseVertex<7, Vector7d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl() // initialize
    {
        _estimate.setZero();
    }
    virtual void oplusImpl( const double* update ) // update
    {
        Vector7d delta_pose;
        for (int i = 0; i != 7; ++i)
            delta_pose(i) = update[i];
        Vector8d Sim_delta = Sim3Exp(delta_pose);
        Vector8d Sim_esti = Sim3Exp(_estimate);
        _estimate = Sim3Log(Tdot(Sim_delta, Sim_esti));
    }
    virtual bool read( std::istream& in ) {return false;}
    virtual bool write( std::ostream& out ) const {return false;}
};

class EdgeRelativePose: public g2o::BaseBinaryEdge<7, Vector7d, VertexPose, VertexPose> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeRelativePose() {}
    // compute eij
    virtual void computeError() override
    {
        const Vector7d& v1 = static_cast<const VertexPose*>(_vertices[0])->estimate();
        const Vector7d& v2 = static_cast<const VertexPose*>(_vertices[1])->estimate();
        Vector8d relative_pose = Tdot(Sim3Inv(Sim3Exp(v2)), Sim3Exp(v1));
        _error = Sim3Log(relative_pose) - _measurement;
    }
    virtual bool read( std::istream& in ) {return false;}
    virtual bool write( std::ostream& out ) const {return false;}
};


int main() {
    ////////////////////////////////////////////////////

    std::string raw_nodes_path = "/home/jinxi/codes/learning_slam/pose-graph/scale-aware/data/scale_drift_circle/nodes.txt";
    std::string raw_edges_path = "/home/jinxi/codes/learning_slam/pose-graph/scale-aware/data/scale_drift_circle/edges.txt";
    std::string save_state_path = "/home/jinxi/codes/learning_slam/pose-graph/scale-aware/poses/scale_drift/opt_edges.txt";
    const unsigned iterations(10);

    ////////////////////////////////////////////////////

    NODES nodes = read_nodes(raw_nodes_path);
    std::vector<INDEX_PAIR> edge_pairs;
    EDGES edges = read_edges(raw_edges_path, edge_pairs);
    // std::cout << nodes[nodes.size()-1].transpose() << std::endl;
    // std::cout << edge_pairs[edge_pairs.size()-1].first << "  " << edge_pairs[edge_pairs.size()-1].second << std::endl;
    // std::cout << edges[edge_pairs.size()-1].transpose() << std::endl;
    

    // create a solver
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<7, 7>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));

    // Create optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // Add vertices (absolute poses)
    for (int i = 0; i != NUM_NODES; ++i)
    {
        Vector7d sim_node = Sim3Log(nodes[i]);
        auto vertex = new VertexPose();
        vertex->setId(i);
        if (i == 0)
            vertex->setFixed(true);
        vertex->setEstimate(sim_node); // Initial estimate
        optimizer.addVertex(vertex);
    }
    // check if set correctly
    for (int i = 0; i < 10; ++i) {
        VertexPose* v = static_cast<VertexPose*>(optimizer.vertex(i));
        std::cout << "Node " << i << ": " << v->estimate().transpose() << std::endl;
    }

    Eigen::Matrix<double, 7, 7> info;
    info.setIdentity();
    // Add edges (relative pose measurements)
    for (int i = 0; i != NUM_NODES; ++i) 
    {
        auto edge = new EdgeRelativePose();
        unsigned l(edge_pairs[i].first), r(edge_pairs[i].second);
        edge->setId(i);
        edge->setVertex(0, optimizer.vertex(l));
        edge->setVertex(1, optimizer.vertex(r));
        Vector7d sim_meas = Sim3Log(edges[i]);
        edge->setMeasurement(sim_meas);
        edge->setInformation(info); // Set information matrix
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
    NODES new_x;
    for (int i = 0; i != NUM_NODES; ++i) 
    {
        VertexPose* v = static_cast<VertexPose*>(optimizer.vertex(i));
        auto x_ij = v->estimate();
        new_x.push_back(Sim3Exp(x_ij));
    }
    save_opt_nodes(save_state_path, new_x);

    return 0;
}
