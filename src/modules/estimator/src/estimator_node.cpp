#include "estimator_node.hpp"

using namespace std::placeholders;

EstimatorNode::EstimatorNode():Node("estimator_node")
{
    imu_sub = this->create_subscription<ImuMsg>("/imu", 10,
                                                std::bind(&EstimatorNode::imuCallback, this, _1));
}

EstimatorNode::~EstimatorNode()
{

}

void EstimatorNode::imuCallback(const ImuMsg::SharedPtr msg)
{
    imu.gx = msg->angular_velocity.x;
    imu.gy = msg->angular_velocity.y;
    imu.gz = msg->angular_velocity.z;
    imu.ax = normalizeByGravity(msg->linear_acceleration.x);
    imu.ay = normalizeByGravity(msg->linear_acceleration.y);
    imu.az = normalizeByGravity(msg->linear_acceleration.z);
    // accelToEuler(imu);
    gyroToEuler(imu);
}

void EstimatorNode::accelToEuler(IMUData_s data)
{
    float theta_m = atan2(data.ax,data.az) * RAD_TO_DEG;
    float phi_m = atan2(data.ay, data.az) * RAD_TO_DEG;

    float current_theta = lowPassFilter(theta_m, prev_theta, 0.9);
    float current_phi = lowPassFilter(phi_m, prev_phi, 0.9);

    prev_phi = current_phi;
    prev_theta = current_theta;

    RCLCPP_INFO_STREAM(this->get_logger(), "Pitch: " << static_cast<int>(current_theta) 
                                        << " Roll: " << static_cast<int>(current_phi));
}

void EstimatorNode::gyroToEuler(IMUData_s imu)
{
    theta_g += imu.gx * 0.05;
    phi_g += imu.gy * 0.05;
    RCLCPP_INFO_STREAM(this->get_logger(), "Pitch: " << static_cast<int>(theta_g) 
                                        << " Roll: " << static_cast<int>(phi_g));
}

float EstimatorNode::normalizeByGravity(float data)
{
    return data / GRAVITY;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EstimatorNode>());
    rclcpp::shutdown();
    return 0;
}