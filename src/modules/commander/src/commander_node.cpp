#include "commander_node.hpp"

using namespace std::placeholders;

CommanderNode::CommanderNode() : Node("commander_node")
{
    avoidance = std::make_shared<Avoidance>();
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
    RCLCPP_INFO(this->get_logger(), "safety_distance: %.2f", avoidance->safety_dist);
    RCLCPP_INFO(this->get_logger(), "distance_limits: %.2f", avoidance->dist_limit);
    RCLCPP_INFO(this->get_logger(), "vehicle_radius: %.2f", avoidance->vehicle_radius);
    RCLCPP_INFO(this->get_logger(), "vehicle_fov: %f", avoidance->fov);
    RCLCPP_INFO(this->get_logger(), "linear velocity limit: %lf", avoidance->linear_limit);
    RCLCPP_INFO(this->get_logger(), "angular velocity limit: %lf", avoidance->angular_limit);
}

void CommanderNode::declareParameters()
{
    this->declare_parameter("vehicle_dimensions", std::vector<double>(VEC_DIM_ALL, 0.0));
    this->declare_parameter("lidar_rules", std::vector<double>(SENSOR_RULES_ALL, 0.0));
    this->declare_parameter<std::string>("frame_id", "world");
    this->declare_parameter<double>("max_angular_velocity", 1.0);
    this->declare_parameter<double>("max_linear_velocity", 1.0);

    frame_id = this->get_parameter("frame_id").as_string();
    avoidance->vehicle_dimensions = this->get_parameter("vehicle_dimensions").as_double_array();
    avoidance->sensor_rules = this->get_parameter("lidar_rules").as_double_array();
    avoidance->angular_limit = this->get_parameter("max_angular_velocity").as_double();
    avoidance->linear_limit = this->get_parameter("max_linear_velocity").as_double();

    tf_vehicle = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    avoidance->init();

    printDisplay();
}

void CommanderNode::joyCallback(const joyMsg::SharedPtr msg)
{
    velocity.linear.x = msg->axes[0];
    velocity.angular.z = msg->axes[1];

    if (velocity.linear.x != 0 || velocity.angular.z != 0)
    {
        avoidance->updateVelocity(velocity.linear.x, velocity.angular.z);
    }

    pub.joy->publish(velocity);
}

void CommanderNode::pointCloudCallback(const pointCloudMsg::SharedPtr msg)
{
    pcl_conversions::toPCL(*msg, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, pcl_xyz_pc);
    avoidance->updateHistogram(pcl_xyz_pc);
}

void CommanderNode::odometryCallback(const odometryNavMsg::SharedPtr msg)
{
    if (msg->child_frame_id == "marble_husky/base_link")
    {
        return;
    }

    state.position.x = msg->pose.pose.position.x;
    state.position.y = msg->pose.pose.position.y;
    state.position.z = msg->pose.pose.position.z;

    state.quaternion.q[0] = msg->pose.pose.orientation.x;
    state.quaternion.q[1] = msg->pose.pose.orientation.y;
    state.quaternion.q[2] = msg->pose.pose.orientation.z;
    state.quaternion.q[3] = msg->pose.pose.orientation.w;

    quaternionToEuler(state.quaternion.q, state.orientation.angle);
}

void CommanderNode::visualization()
{
    geometry_msgs::msg::PoseStamped pose_stamped;
    geometry_msgs::msg::TransformStamped t;

    t = visualizationTf2(state, frame_id);
    tf_vehicle->sendTransform(t);

    if (visualizationPath(pose_stamped, state.position, frame_id))
    {
        vehicle_path.header = t.header;
        vehicle_path.poses.push_back(pose_stamped);
        pub.vehicle_path->publish(vehicle_path);
    }

    if (false)
    {
        for (int i = 0; i < 3; i++)
        {
            markerMsg marker;
            visualizationMarker(marker, velocity.linear.x, velocity.angular.z, i);
            marker_array.markers.push_back(marker);
            marker.action = markerMsg::DELETEALL;
        }

        pub.markers->publish(marker_array);
        marker_array.markers.clear();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommanderNode>());
    rclcpp::shutdown();
}