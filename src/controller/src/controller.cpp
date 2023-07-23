#include "controller.hpp"
using namespace std::placeholders;

Controller::Controller(): Node("controller_node"), is_ready(true){
  // this->declare_parameter("a_scale", 0.1);
  // this->declare_parameter("l_scale", 1);
  
  speed.data[A_SCALE] = 1.0;
  speed.data[L_SCALE] = 1.0;
  // speed.data[A_SCALE] = this->get_parameter("a_scale").as_double();
  // speed.data[L_SCALE] = this->get_parameter("l_scale").as_double();
  speed.data[ANGULAR] = 0;
  speed.data[LINEAR]  = 0;

  keyboard_sub = this->create_subscription<int32Msg>("/input_cmd", 10,
                std::bind(&Controller::joyCallback, this, _1));
  keyboard_pub = this->create_publisher<twistMsg>("cmd_vel", 10);
}

void Controller::joyCallback(const int32Msg msg){
  twistMsg data;
  RCLCPP_INFO(this->get_logger(), "------");

  speed.data[LINEAR] = 0;
  speed.data[ANGULAR] = 0;
  is_ready = true;

  switch(msg.data){
      case KEYCODE_W:
        RCLCPP_DEBUG(this->get_logger(), "UP");
        speed.data[LINEAR] = 1.0;
        break;
      case KEYCODE_A:
        RCLCPP_DEBUG(this->get_logger(), "LEFT");
        speed.data[ANGULAR] = 1.0;
        break;
      case KEYCODE_S:
        RCLCPP_DEBUG(this->get_logger(), "STOP");
        speed.data[LINEAR] = 0;
        speed.data[ANGULAR] = 0;
        break;
      case KEYCODE_D:
        RCLCPP_DEBUG(this->get_logger(), "RIGHT");
        speed.data[ANGULAR] = -1.0;
        break;
      case KEYCODE_X:
        RCLCPP_DEBUG(this->get_logger(), "DOWN");
        speed.data[LINEAR] = -1.0;
        break;
      default:
        RCLCPP_DEBUG(this->get_logger(), "None: %x", msg.data);
        is_ready = false;
        break;
    }
    
    if(is_ready){
      data.angular.z = speed.data[A_SCALE] * speed.data[ANGULAR];
      data.linear.x  = speed.data[L_SCALE] * speed.data[LINEAR];
      keyboard_pub->publish(data);
    }
}

int main(int argc, char ** args){
  rclcpp::init(argc, args);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}