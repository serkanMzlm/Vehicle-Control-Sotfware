#ifndef __CONTROLLER_NODE_HPP__
#define __CONTROLLER_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "avoidance/obstacle_avoidance.hpp"
#include "commander_type.hpp"

using joyMsg = sensor_msgs::msg::Joy;
using int32Msg = std_msgs::msg::Int32;
using twistMsg = geometry_msgs::msg::Twist;
using pointCloudMsg = sensor_msgs::msg::PointCloud2;
using markerArrayMsg = visualization_msgs::msg::MarkerArray;
using odometryNavMsg = nav_msgs::msg::Odometry;
using poseStampedMsg = geometry_msgs::msg::PoseStamped;
using navPathMsg = nav_msgs::msg::Path;

typedef struct
{
  rclcpp::Subscription<joyMsg>::SharedPtr joy;
  rclcpp::Subscription<pointCloudMsg>::SharedPtr cloud;
  rclcpp::Subscription<odometryNavMsg>::SharedPtr nav_odom;
} sub_t;

typedef struct
{
  rclcpp::TimerBase::SharedPtr visual;
} rosTime_t;

typedef struct
{
  rclcpp::Publisher<markerArrayMsg>::SharedPtr markers;
  rclcpp::Publisher<navPathMsg>::SharedPtr vehicle_path;
  rclcpp::Publisher<twistMsg>::SharedPtr joy;
} pub_t;

class Commander : public rclcpp::Node, public ObstacleAvoidance
{
private:
  sub_t sub;
  pub_t pub;
  rosTime_t ros_time;
  twistMsg data;
  pcl_t pcl_data;
  markerArrayMsg marker_array;
  navPathMsg path_;
  float pre_pose[3] = {0.0, 0.0, 0.0};
  State_t global_state;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_vehicle;

public:
  Commander();
  void calculateAvoidanceRules();
  void commandCallback(const joyMsg msg);
  void obstacleAvoidance();
  void pointCloudCallback(const pointCloudMsg &);
  void odometryCallback(const odometryNavMsg::SharedPtr);
  void visualization();
  void declareParameters();
  void initTopic();
  void makerCallback();
};

#endif