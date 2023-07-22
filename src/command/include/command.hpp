#ifndef __COMMAND_HPP__
#define __COMMAND_HPP__
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <memory.h>
#include <unistd.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

struct termios cooked, raw;
using twistMsg = geometry_msgs::msg::Twist;

auto logger = rclcpp::get_logger("logger");

typedef enum {LINEAR, ANGULAR, L_SCALE, A_SCALE, SPEED_ALL_DATA};
typedef union{
  struct{
    float linear;
    float angular;
    float l_scale;
    float a_scale;
  };
  float data[SPEED_ALL_DATA];
}Joy_t;

class Teleop: public rclcpp::Node{
public:
  Teleop();
  void keyLoop();

private:
  Joy_t speed;
  rclcpp::Publisher<twistMsg>::SharedPtr twist_pub_;
};

void quit(int sig);
#endif