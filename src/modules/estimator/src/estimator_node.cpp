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
    imu_data[GYRO_X] = msg->angular_velocity.x;
    imu_data[GYRO_Y] = msg->angular_velocity.y;
    imu_data[GYRO_Z] = msg->angular_velocity.z;
    imu_data[ACC_X] = msg->linear_acceleration.x;
    imu_data[ACC_Y] = msg->linear_acceleration.y;
    imu_data[ACC_Z] = msg->linear_acceleration.z;

    // RCLCPP_INFO_STREAM(this->get_logger(), "Gyro: " << imu_data[GYRO_X] << " "
    //                             <<  imu_data[GYRO_Y] << " " << imu_data[GYRO_Z]);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Acc: " << imu_data[ACC_X] << " "
    //                             <<  imu_data[ACC_Y] << " " << imu_data[ACC_Z]);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EstimatorNode>());
    rclcpp::shutdown();
    return 0;
}