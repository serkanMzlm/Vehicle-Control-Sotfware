#include "controller_node.hpp"
using namespace std::placeholders;

Controller::Controller(): Node("controller_node"){
  initTopic();
}

void Controller::commandCallback(const joyMsg msg){
  data.linear.x = msg.axes[0];
  data.angular.z = msg.axes[1];
  pub.joy->publish(data);
}

void Controller::initTopic(){
  sub.joy = this->create_subscription<joyMsg>("command_data", 10, std::bind(
                                &Controller::commandCallback, this, _1));

  pub.joy = this->create_publisher<twistMsg>("cmd_vel", 10);
}


int main(int argc, char ** args){
  rclcpp::init(argc, args);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}