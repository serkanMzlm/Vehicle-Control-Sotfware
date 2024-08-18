#include "commander_node.hpp"
using namespace std::placeholders;

Commander::Commander() : Node("commander_node")
{
  tf_vehicle = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  declareParameters();
  initTopic();
}

void Commander::calculateAvoidanceRules()
{
  float x = powf((vehicle_dimensions[WIDTH] / 2.0f), 2);
  float y = powf((vehicle_dimensions[LENGTH] / 2.0f), 2);
  vehicle_radius = sqrtf(x + y);

  distance_limits = lidar_rules[MAX_DIS] - OFFSET;
  safety_distance = distance_limits - vehicle_radius;
  vehicle_fov = static_cast<int>(RAD2DEG(atan2(vehicle_dimensions[WIDTH], vehicle_dimensions[LENGTH])));

  RCLCPP_INFO(this->get_logger(), "safety_distance: %.2f", safety_distance);
  RCLCPP_INFO(this->get_logger(), "distance_limits: %.2f", distance_limits);
  RCLCPP_INFO(this->get_logger(), "vehicle_radius: %.2f", vehicle_radius);
  RCLCPP_INFO(this->get_logger(), "vehicle_fov: %d", vehicle_fov);
  RCLCPP_INFO(this->get_logger(), "linear velocity limit: %lf", linear_velocity_limit);
  RCLCPP_INFO(this->get_logger(), "angular velocity limit: %lf", angular_velocity_limit);
}

void Commander::commandCallback(const joyMsg msg)
{
  data.linear.x = msg.axes[0];
  data.angular.z = msg.axes[1];
  obstacleAvoidance();
}

void Commander::obstacleAvoidance()
{
  updateVelocity(data.linear.x, data.angular.z);
  makerCallback();
  pub.joy->publish(data);
}

void Commander::pointCloudCallback(const pointCloudMsg &msg)
{
  pcl_conversions::toPCL(msg, pcl_data.cloud);
  pcl::fromPCLPointCloud2(pcl_data.cloud, pcl_data.xyz_cloud);
  centerData(pcl_data.xyz_cloud);
}

void Commander::makerCallback()
{
  for (int i = 0; i < VEL_ALL; i++)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "marble_husky/base_link/front_laser";
    marker.ns = "markers" + std::to_string(i);
    marker.type = markerMsg::ARROW;
    marker.id = i;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = i % 3 == 0 ? 1.0 : 0.0;
    marker.color.g = i % 3 == 1 ? 1.0 : 0.0;
    marker.color.b = i % 3 == 2 ? 1.0 : 0.0;
    marker.points.push_back(first_point[i]);
    marker.points.push_back(last_point[i]);
    marker_array.markers.push_back(marker);
    marker.action = markerMsg::DELETEALL;
  }

  pub.markers->publish(marker_array);
  marker_array.markers.clear();
}

void Commander::declareParameters()
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

void Commander::initTopic()
{
  sub.joy = this->create_subscription<joyMsg>("velocity", 10, std::bind(&Commander::commandCallback, this, _1));
  sub.cloud = this->create_subscription<pointCloudMsg>("lidar", 100, std::bind(&Commander::pointCloudCallback, this, _1));
  sub.nav_odom = this->create_subscription<odometryNavMsg>("/model/marble_husky/odometry", 10,
                                                           std::bind(&Commander::odometryCallback, this, _1));

  pub.joy = this->create_publisher<twistMsg>("cmd_vel", 10);
  pub.markers = this->create_publisher<markerArrayMsg>("marker_visulation", 100);
}

void Commander::odometryCallback(const odometryNavMsg::SharedPtr msg)
{
  if(msg->child_frame_id == "marble_husky/base_link") 
  {
    return;
  }
  
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "marble_husky/base_link/front_laser";
  t.child_frame_id = "base_link";

  t.transform.translation.x = msg->pose.pose.position.x / 10.0;
  t.transform.translation.y = msg->pose.pose.position.y / 10.0;
  t.transform.translation.z = msg->pose.pose.position.z / 10.0;

  // euler to quaternion
  // tf2::Quaternion q;
  // q.setRPY(msg->pose.orientation.x,
  //          msg->pose.orientation.y,
  //          msg->pose.orientation.z);

  t.transform.rotation.x = msg->pose.pose.orientation.x;
  t.transform.rotation.y = msg->pose.pose.orientation.y;
  t.transform.rotation.z = msg->pose.pose.orientation.z;
  t.transform.rotation.w = msg->pose.pose.orientation.w;

  tf_vehicle->sendTransform(t);
}

int main(int argc, char **args)
{
  rclcpp::init(argc, args);
  rclcpp::spin(std::make_shared<Commander>());
  rclcpp::shutdown();
  return 0;
}