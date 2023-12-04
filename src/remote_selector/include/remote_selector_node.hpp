#ifndef __REMOTE_SELECTOR_NODE_HPP__
#define __REMOTE_SELECTOR_NODE_HPP__

#include <fcntl.h>   
#include <errno.h> 
#include <unistd.h>  
#include <termios.h> 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32.hpp"

using joyMsg = sensor_msgs::msg::Joy;
using int32Msg = std_msgs::msg::Int32;

class RemoteSelector: public rclcpp::Node{
private:
    std::string control_unit;

    std::chrono::steady_clock::time_point last_msg_timestamp_;
    std::chrono::steady_clock::duration elapsed_time ;

public:
    RemoteSelector();
    ~RemoteSelector();
    void initParam();
};

#endif