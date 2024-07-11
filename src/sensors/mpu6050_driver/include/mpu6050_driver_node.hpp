#ifndef __MPU6050_DRIVER_NODE_HPP__
#define __MPU6050_DRIVER_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include  "mpu6050.hpp"

using ImuMsg = sensor_msgs::msg::Imu;

class MPU6050Driver: public rclcpp::Node{
public:
    MPU6050Driver();
    ~MPU6050Driver();
    void handleInput();
private:
    rclcpp::Publisher<ImuMsg>::SharedPtr imu_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<MPU6050> mpu6050;
    // size_t count_;
    // uint64_t updateTime;
    void declareParameters();
};
 #endif