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

// will be corrected
void Commander::makerCallback()
{
  // for (int i = 0; i < VEL_ALL; i++)
  // {
  //   visualization_msgs::msg::Marker marker;
  //   marker.header.frame_id = "marble_husky/base_link/front_laser";
  //   marker.ns = "markers" + std::to_string(i);
  //   marker.type = markerMsg::ARROW;
  //   marker.id = i;
  //   marker.pose.orientation.w = 1.0;
  //   marker.scale.x = 0.4;
  //   marker.scale.y = 1;
  //   marker.scale.z = 1;
  //   marker.color.a = 1.0;
  //   marker.color.r = i % 3 == 0 ? 1.0 : 0.0;
  //   marker.color.g = i % 3 == 1 ? 1.0 : 0.0;
  //   marker.color.b = i % 3 == 2 ? 1.0 : 0.0;
  //   float first_point_[3] = {first_point[i].x, first_point[i].y, first_point[i].z};
  //   float last_point_[3] = {last_point[i].x, last_point[i].y, last_point[i].z};

  //   transformation(first_point_, global_state.orientation.pose, global_state.position.pose);
  //   transformation(last_point_, global_state.orientation.pose, global_state.position.pose);

  //   first_point[i].x = first_point_[0];
  //   first_point[i].y = first_point_[1]; 
  //   first_point[i].z = 0; 
  //   last_point[i].x = last_point_[0];
  //   last_point[i].y = last_point_[1]; 
  //   last_point[i].z = 0; 

  //   marker.points.push_back(first_point[i]);
  //   marker.points.push_back(last_point[i]);
  //   marker_array.markers.push_back(marker);
  //   marker.action = markerMsg::DELETEALL;
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
  pub.vehicle_path = this->create_publisher<navPathMsg>("vehicle_path", 100);

  ros_time.visual = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Commander::visualization, this));
}

void Commander::odometryCallback(const odometryNavMsg::SharedPtr msg)
{
  if (msg->child_frame_id == "marble_husky/base_link")
  {
    return;
  }

  global_state.position.x = msg->pose.pose.position.x / 10.0;
  global_state.position.y = msg->pose.pose.position.y / 10.0;
  global_state.position.z = msg->pose.pose.position.z / 10.0;

  global_state.q[0] = msg->pose.pose.orientation.x;
  global_state.q[1] = msg->pose.pose.orientation.y;
  global_state.q[2] = msg->pose.pose.orientation.z;
  global_state.q[3] = msg->pose.pose.orientation.w;

  quaternionToEuler(global_state.q, global_state.orientation.pose);
}

void Commander::visualization()
{
  geometry_msgs::msg::PoseStamped pose_stamped;
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "marble_husky/base_link/front_laser";
  t.child_frame_id = "base_link";

  t.transform.translation.x = global_state.position.x;
  t.transform.translation.y = global_state.position.y;
  t.transform.translation.z = global_state.position.z;

  // euler to quaternion
  // tf2::Quaternion q;
  // q.setRPY(msg->pose.orientation.x,
  //          msg->pose.orientation.y,
  //          msg->pose.orientation.z);

  t.transform.rotation.x = global_state.q[0];
  t.transform.rotation.y = global_state.q[1];
  t.transform.rotation.z = global_state.q[2];
  t.transform.rotation.w = global_state.q[3];
  tf_vehicle->sendTransform(t);

  if (pre_pose[0] == global_state.position.x &&
      pre_pose[1] == global_state.position.y &&
      pre_pose[2] == global_state.position.z)
  {
    return;
  }
  pre_pose[0] = global_state.position.x;
  pre_pose[1] = global_state.position.y;
  pre_pose[2] = global_state.position.z;

  pose_stamped.header = t.header;
  pose_stamped.pose.position.x = t.transform.translation.x;
  pose_stamped.pose.position.y = t.transform.translation.y;
  pose_stamped.pose.position.z = t.transform.translation.z;

  path_.header = t.header;
  path_.poses.push_back(pose_stamped);
  pub.vehicle_path->publish(path_);
}

int main(int argc, char **args)
{
  rclcpp::init(argc, args);
  rclcpp::spin(std::make_shared<Commander>());
  rclcpp::shutdown();
  return 0;
}