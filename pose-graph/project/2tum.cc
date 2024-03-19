#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iomanip>

// g++ pose-graph/project/2tum.cc -o pose-graph/project/2tum -I /usr/include/eigen3/ && ./pose-graph/project/2tum

void print_vector(int a, int b, std::vector<std::vector<double>> *data)
{
    for (int i = 0; i != a; ++i)
    {
        for (int j = 0; j != b; ++j)
            std::cout << (*data)[i][j] << "  ";
        std::cout << "\n";
    }
}

void write2tum(double time_stamp, Eigen::Matrix3d& rotation, Eigen::Vector3d& translation, std::ofstream& file)
{
    if (file.is_open()) 
    {
        Eigen::Quaterniond quat = Eigen::Quaterniond(rotation);
        file << std::fixed << std::setprecision(6);
        file << time_stamp << " ";
        file << translation[0] << " " <<translation[1] << " " << translation[2] << " ";
        auto r = quat.coeffs();
        file << r[0] << " " << r[1] << " " << r[2] << " " << r[3] << std::endl;
    } 
    else 
        std::cerr << "Error: File is not open." << std::endl;
}

int main() {
    ////////////////////////////////////////////////////
    std::string raw_data_path = "./pose-graph/project/hw1_data.txt";
    std::string save_path = "./pose-graph/project/poses/slam_tum.txt";
    ////////////////////////////////////////////////////

    std::ifstream data_file(raw_data_path);
    std::string line;
    std::vector<std::vector<double>> data(12, std::vector<double>(3));
    // import data
    for (int i = 0; i != 3; ++i)
    {
        getline(data_file, line);
        std::istringstream iss(line);
        double value;
        int j = 0;
        while (iss >> value)
        {
            data[j][i] = value;
            j += 1;
        }
    }
    // print_vector(12, 3, &data);
    data_file.close();

    // absolute .tum format
    std::ofstream absolute_out(save_path);
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d tranlation(0, 0, 0);
    write2tum(0, rotation, tranlation, absolute_out);
    for (int i = 0; i != 12; ++i)
    {
        double theta = data[i][2];
        Eigen::Vector3d t(data[i][0], data[i][1], 0);
        Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
        r.block<2, 2>(0,0) << cos(theta), -sin(theta), sin(theta), cos(theta);
        tranlation = rotation * t + tranlation;
        rotation = rotation * r;
        write2tum((double)(i+1), rotation, tranlation, absolute_out);
    }
    absolute_out.close();

    return 0;
}
