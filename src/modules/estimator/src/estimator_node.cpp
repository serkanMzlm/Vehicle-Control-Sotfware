#include "estimator_node.hpp"

using namespace std::placeholders;

EstimatorNode::EstimatorNode():Node("estimator_node")
{
    tictoc.tic();
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
    complementaryFilter(imu, static_cast<float>(tictoc.toc() / 1000)); // dt -> second
    tictoc.tic();
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