#ifndef __CREATE_MODEL_NODE_HPP__
#define __CREATE_MODEL_NODE_HPP__

#include <unistd.h>
#include <iostream>
#include <filesystem>

#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <rclcpp/rclcpp.hpp>

#include "geometry_tools/geometry_operations.hpp"

class CreateModelNode : public rclcpp::Node
{
private:
    gz::transport::Node _node;
    std::string world_name;
    std::string model_name;
    std::string model_file;
    std::vector<double> model_pose;
    double model_quaternion[QUATERNION_ALL];
    std::vector<double> model_euler;

    // __FILE__: gives the location of the file
    // std::filesystem::current_path(): Returns the path where the file is run
    std::filesystem::path file_path = __FILE__;
    std::filesystem::path target_path;

public:
    CreateModelNode();
    ~CreateModelNode() = default;
    void init();
    void findSimulationPath();
    void declareParameters();
    void printDisplay() const;
};

#endif