#ifndef __ESTIMATOR_NODE__
#define __ESTIMATOR_NODE__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "estimator_node_type.hpp"

using ImuMsg = sensor_msgs::msg::Imu;

class EstimatorNode: public rclcpp::Node
{
private:
    float imu_data[ALL_IMU];
    rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub;
public:
    EstimatorNode();
    ~EstimatorNode();
    void imuCallback(const ImuMsg::SharedPtr msg);
};

#endif