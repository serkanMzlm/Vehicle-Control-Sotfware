#include "controller.hpp"
using namespace std::placeholders;

Controller::Controller(): Node("controller_node"), is_ready(true){
  keyboard_pub = this->create_publisher<twistMsg>("cmd_vel", 10);
}

void Controller::joyCallback(const int32Msg msg){
}

int main(int argc, char ** args){
  rclcpp::init(argc, args);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}