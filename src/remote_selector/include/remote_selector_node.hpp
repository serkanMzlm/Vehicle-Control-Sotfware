#ifndef __REMOTE_SELECTOR_NODE_HPP__
#define __REMOTE_SELECTOR_NODE_HPP__

#include <fcntl.h>   
#include <errno.h> 
#include <unistd.h>  
#include <termios.h> 
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32.hpp"

#include "remote_selector_type.hpp"
#include "geometry_utils.hpp"

using joyMsg = sensor_msgs::msg::Joy;
using int32Msg = std_msgs::msg::Int32;

typedef struct{
    rclcpp::Publisher<joyMsg>::SharedPtr joy;
} Pub_t;

typedef struct{
    rclcpp::Subscription<joyMsg>::SharedPtr joy;
    rclcpp::Subscription<int32Msg>::SharedPtr keyboard;
} Sub_t;

typedef struct{
    rclcpp::TimerBase::SharedPtr serial;  
    rclcpp::TimerBase::SharedPtr keyboard; 
} Timer_t;


class RemoteSelector: public rclcpp::Node{
private:
    Sub_t sub;
    Pub_t pub;
    Timer_t timer;

    Rx_t rx_data;
    Device_t dev;
    joyMsg joy_data;
    std::string control_unit;

    std::chrono::steady_clock::duration elapsed_time ;
    std::chrono::steady_clock::time_point last_msg_timestamp_;

public:
    RemoteSelector();
    ~RemoteSelector();
    void initParam();
    void controlSelection();

    bool initPort();
    bool serialConfig();
    void serialDataRead();

    void keyboardCallback(const int32Msg msg);
    void timeOutKeyboard();

    void joyCallback(const joyMsg msg);
    int buttonChange(int button_data);
};

#endif