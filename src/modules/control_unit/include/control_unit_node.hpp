#ifndef __CONTOL_UNIT_NODE_HPP__
#define __CONTOL_UNIT_NODE_HPP__

#include <fcntl.h>   
#include <errno.h> 
#include <unistd.h>  
#include <termios.h> 
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32.hpp"

#include "control_unit_type.hpp"
#include "tic_toc.hpp"
#include "num_tools.hpp"


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


class ControlUnit: public rclcpp::Node{
private:
    Sub_t sub;
    Pub_t pub;
    Timer_t timer;

    Rx_t rx_data;
    Device_t dev;
    joyMsg joy_data;
    std::string control_unit;
    TicToc tictoc;

public:
    ControlUnit();
    ~ControlUnit();
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