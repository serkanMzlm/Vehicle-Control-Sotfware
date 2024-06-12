#include "commander_node.hpp"
using namespace std::placeholders;

Commander::Commander() : Node("commander_node")
{
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
  RCLCPP_INFO(this->get_logger(), "safety_distance: %.2f", safety_distance);
  RCLCPP_INFO(this->get_logger(), "distance_limits: %.2f", distance_limits);
  RCLCPP_INFO(this->get_logger(), "vehicle_radius: %.2f", vehicle_radius);
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
  for (size_t i = 0; i < pcl_data.xyz_cloud.size(); i++)
  {
    if (std::isinf(std::abs(pcl_data.xyz_cloud.points[i].x)) ||
        std::isnan(std::abs(pcl_data.xyz_cloud.points[i].x)))
    {
      pcl_data.xyz_cloud.points[i].x = 0.0f;
      pcl_data.xyz_cloud.points[i].y = 0.0f;
      pcl_data.xyz_cloud.points[i].z = 0.0f;
    }
  }

  detectObject(pcl_data.xyz_cloud);
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
  vehicle_dimensions = this->get_parameter("vehicle_dimensions").as_double_array();
  lidar_rules = this->get_parameter("lidar_rules").as_double_array();

  for (int i = 0; i < VEL_ALL; i++)
  {
    first_point[i].x = 0.0;
    first_point[i].y = 0.0;
    first_point[i].z = 0.0;
  }

  calculateAvoidanceRules();
}

void Commander::initTopic()
{
  sub.joy = this->create_subscription<joyMsg>("velocity", 10, std::bind(&Commander::commandCallback, this, _1));
  sub.cloud = this->create_subscription<pointCloudMsg>("lidar", 100, std::bind(&Commander::pointCloudCallback, this, _1));

  pub.joy = this->create_publisher<twistMsg>("cmd_vel", 10);
  pub.markers = this->create_publisher<markerArrayMsg>("marker_visulation", 100);
}

int main(int argc, char **args)
{
  rclcpp::init(argc, args);
  rclcpp::spin(std::make_shared<Commander>());
  rclcpp::shutdown();
  return 0;
}