#ifndef __ESTIMATOR_NODE__
#define __ESTIMATOR_NODE__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "estimator_node_type.hpp"
#include "complementary_filter.hpp"
#include "filters.hpp"

using ImuMsg = sensor_msgs::msg::Imu;

class EstimatorNode: public rclcpp::Node
{
private:
    float prev_theta = 0.0;
    float prev_phi = 0.0f;

    float theta_g = 0.0f;
    float phi_g = 0.0f;
    
    IMUData_s imu;
    rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub;
public:
    EstimatorNode();
    ~EstimatorNode();
    void imuCallback(const ImuMsg::SharedPtr msg);
    void accelToEuler(IMUData_s imu);
    void gyroToEuler(IMUData_s imu);
    float normalizeByGravity(float data);
};

#endif