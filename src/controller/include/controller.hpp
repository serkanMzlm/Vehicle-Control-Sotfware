#ifndef __CONTROLLER_HPP__
#define __CONTROLLER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define KEYCODE_W 87 
#define KEYCODE_A 65
#define KEYCODE_S 83
#define KEYCODE_D 68
#define KEYCODE_X 88

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