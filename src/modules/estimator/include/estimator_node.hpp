#ifndef __ESTIMATOR_NODE__
#define __ESTIMATOR_NODE__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "filter_tools/complementary_filter.hpp"
#include "filter_tools/low_high_pass_filter.hpp"
#include "filter_tools/mahony_orientation_filter.hpp"
#include "math_tools/time_utils.hpp"

using ImuMsg = sensor_msgs::msg::Imu;

class EstimatorNode: public rclcpp::Node
{
private:   
    IMUData_s imu;
    TicToc tictoc;
    rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub;
public:
    EstimatorNode();
    ~EstimatorNode();
    void imuCallback(const ImuMsg::SharedPtr msg);
    float normalizeByGravity(float data);
};

#endif