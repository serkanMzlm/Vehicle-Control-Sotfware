#ifndef __CONTROLLER_NODE_HPP__
#define __CONTROLLER_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"



using int32Msg = std_msgs::msg::Int32;
using twistMsg = geometry_msgs::msg::Twist;

typedef enum {
  LINEAR, ANGULAR, L_SCALE, A_SCALE, SPEED_ALL_DATA
} Joy_e;

typedef union{
  struct{
    float linear;
    float angular;
    float l_scale;
    float a_scale;
  }Joy_s;
  float data[SPEED_ALL_DATA];
}Joy_t;

class Controller: public rclcpp::Node{
public:
  Controller();
  void joyCallback(int32Msg);

private:
  Joy_t speed;
  bool is_ready;
  rclcpp::Subscription<int32Msg>::SharedPtr keyboard_sub;
  rclcpp::Publisher<twistMsg>::SharedPtr keyboard_pub;
};

#endif