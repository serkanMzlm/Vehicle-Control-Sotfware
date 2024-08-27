#include "commander_node.hpp"

CommanderNode::CommanderNode() : Node("commander_node")
{
    tf_vehicle = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    declareParameters();
    initTopic();
    RCLCPP_INFO(this->get_logger(), "Commaned Started");
}

void CommanderNode::initTopic()
{
}

void CommanderNode::printDisplay()
{
    this->declare_parameter("vehicle_dimensions", std::vector<double>(VEC_DIM_ALL, 0.0));
    this->declare_parameter("lidar_rules", std::vector<double>(SENSOR_RULES_ALL, 0.0));
    this->declare_parameter<double>("max_angular_velocity", 1.0);
    this->declare_parameter<double>("max_linear_velocity", 1.0);

    vehicle_dimensions = this->get_parameter("vehicle_dimensions").as_double_array();
    lidar_rules = this->get_parameter("lidar_rules").as_double_array();
    angular_velocity_limit = this->get_parameter("max_angular_velocity").as_double();
    linear_velocity_limit = this->get_parameter("max_linear_velocity").as_double();

    for (int i = 0; i < VEL_ALL; i++)
    {
    first_point[i].x = 0.0;
    first_point[i].y = 0.0;
    first_point[i].z = 0.0;
    }

    calculateAvoidanceRules();

    lidar_pose[0] = (float)lidar_rules[X_POS];
    lidar_pose[1] = (float)lidar_rules[Y_POS];
    lidar_pose[2] = (float)lidar_rules[Z_POS];
}

void CommanderNode::declareParameters()
{
}

void CommanderNode::joyCallback(const joyMsg::SharedPtr msg)
{
    data.twist.linear.x = msg->axes[0];
    data.twist.angular.z = msg->axes[1];
}

void CommanderNode::pointCloudCallback(const pointCloudMsg::SharedPtr msg)
{
    pcl_conversions::toPCL(*msg, data.pcl_cloud);
    pcl::fromPCLPointCloud2(data.pcl_cloud, data.pcl_xyz_cloud);
}

void CommanderNode::odometryCallback(const odometryNavMsg::SharedPtr msg)
{
    if (msg->child_frame_id == "marble_husky/base_link")
    {
        return;
    }

    data.state.position.x = msg->pose.pose.position.x;
    data.state.position.y = msg->pose.pose.position.y;
    data.state.position.z = msg->pose.pose.position.z;

    data.state.quaternion.q[0] = msg->pose.pose.orientation.x;
    data.state.quaternion.q[1] = msg->pose.pose.orientation.y;
    data.state.quaternion.q[2] = msg->pose.pose.orientation.z;
    data.state.quaternion.q[3] = msg->pose.pose.orientation.w;

    quaternionToEuler(data.state.quaternion.q, data.state.orientation.pose);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommanderNode>());
    rclcpp::shutdown();
}