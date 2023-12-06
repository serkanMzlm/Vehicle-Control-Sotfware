#include "controller.hpp"
using namespace std::placeholders;

Controller::Controller(): Node("controller_node"){
  // keyboard_pub = this->create_publisher<twistMsg>("cmd_vel", 10);
}


int main(int argc, char ** args){
  rclcpp::init(argc, args);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}