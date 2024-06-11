#ifndef __CONTROLLER_NODE_HPP__
#define __CONTROLLER_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "obstacle_avoidance.hpp"
#include "commander_type.hpp"

using joyMsg = sensor_msgs::msg::Joy;
using int32Msg = std_msgs::msg::Int32;
using twistMsg = geometry_msgs::msg::Twist;
using pointCloudMsg = sensor_msgs::msg::PointCloud2;
using markerArrayMsg = visualization_msgs::msg::MarkerArray;

typedef struct
{
  rclcpp::Subscription<joyMsg>::SharedPtr joy;
  rclcpp::Subscription<pointCloudMsg>::SharedPtr cloud;
} sub_t;

typedef struct
{
  rclcpp::Publisher<markerArrayMsg>::SharedPtr markers;
  rclcpp::Publisher<twistMsg>::SharedPtr joy;
} pub_t;

class Commander : public rclcpp::Node, public ObstacleAvoidance
{
private:
  sub_t sub;
  pub_t pub;
  twistMsg data;
  pcl_t pcl_data;
  markerArrayMsg marker_array;

public:
  Commander();
  void calculateAvoidanceRules();
  void commandCallback(const joyMsg msg);
  void obstacleAvoidance();
  void pointCloudCallback(const pointCloudMsg &);
  void declareParameters();
  void initTopic();
  void makerCallback();
};

#endif