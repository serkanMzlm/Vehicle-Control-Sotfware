#include "commander_node.hpp"

using namespace std::placeholders;

CommanderNode::CommanderNode() : Node("commander_node")
{
    tf_vehicle = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    declareParameters();
    initTopic();
    RCLCPP_INFO(this->get_logger(), "Commaned Started");
}

void CommanderNode::initTopic()
{
    sub.joy = this->create_subscription<joyMsg>("velocity", 10, std::bind(&CommanderNode::joyCallback, this, _1));
    sub.cloud = this->create_subscription<pointCloudMsg>("lidar", 100, std::bind(&CommanderNode::pointCloudCallback, this, _1));
    sub.nav_odom = this->create_subscription<odometryNavMsg>("/model/marble_husky/odometry", 10,
                                                             std::bind(&CommanderNode::odometryCallback, this, _1));

    pub.joy = this->create_publisher<twistMsg>("cmd_vel", 10);
    pub.markers = this->create_publisher<markerArrayMsg>("marker_visulation", 100);
    pub.vehicle_path = this->create_publisher<navPathMsg>("vehicle_path", 100);

    timer_.visual = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&CommanderNode::visualization, this));
}

void CommanderNode::printDisplay()
{
    RCLCPP_INFO(this->get_logger(), "safety_distance: %.2f", obs_data.safety_dist);
    RCLCPP_INFO(this->get_logger(), "distance_limits: %.2f", obs_data.dist_limit);
    RCLCPP_INFO(this->get_logger(), "vehicle_radius: %.2f", obs_data.veh_radius);
    RCLCPP_INFO(this->get_logger(), "vehicle_fov: %f", obs_data.fov);
    RCLCPP_INFO(this->get_logger(), "linear velocity limit: %lf", obs_data.linear_limit);
    RCLCPP_INFO(this->get_logger(), "angular velocity limit: %lf", obs_data.angular_limit);
}

void CommanderNode::declareParameters()
{
    this->declare_parameter("vehicle_dimensions", std::vector<double>(VEC_DIM_ALL, 0.0));
    this->declare_parameter("lidar_rules", std::vector<double>(SENSOR_RULES_ALL, 0.0));
    this->declare_parameter<double>("max_angular_velocity", 1.0);
    this->declare_parameter<double>("max_linear_velocity", 1.0);

    vehicle_dimensions = this->get_parameter("vehicle_dimensions").as_double_array();
    lidar_rules = this->get_parameter("lidar_rules").as_double_array();
    obs_data.linear_limit = this->get_parameter("max_angular_velocity").as_double();
    obs_data.angular_limit = this->get_parameter("max_linear_velocity").as_double();

    calculateAvoidanceRules();

    obs_data.sensor_pose[0] = (float)lidar_rules[X_POS];
    obs_data.sensor_pose[1] = (float)lidar_rules[Y_POS];
    obs_data.sensor_pose[2] = (float)lidar_rules[Z_POS];
    printDisplay();
}

void CommanderNode::calculateAvoidanceRules()
{
    float x = powf((vehicle_dimensions[WIDTH] / 2.0f), 2);
    float y = powf((vehicle_dimensions[LENGTH] / 2.0f), 2);
    obs_data.veh_radius = sqrtf(x + y);

    obs_data.dist_limit = lidar_rules[MAX_DIS] - OFFSET;
    obs_data.safety_dist = obs_data.dist_limit - obs_data.veh_radius;
    obs_data.fov = static_cast<int>(RAD2DEG(atan2(vehicle_dimensions[WIDTH], vehicle_dimensions[LENGTH])));
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

    quaternionToEuler(data.state.quaternion.q, data.state.orientation.angle);
}

void CommanderNode::visualization()
{
    
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommanderNode>());
    rclcpp::shutdown();
}