#ifndef __CONTROLLER_NODE_HPP__
#define __CONTROLLER_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"


class Controller: public rclcpp::Node{
public:
  Controller();

private:
};

#endif