#ifndef __CONTROLLER_NODE_HPP__
#define __CONTROLLER_NODE_HPP__

#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using joyMsg   = sensor_msgs::msg::Joy;
using int32Msg = std_msgs::msg::Int32;
using twistMsg = geometry_msgs::msg::Twist;
using pointCloudMsg   = sensor_msgs::msg::PointCloud2;
using markerArrayMsg  = visualization_msgs::msg::MarkerArray;

typedef struct{
    rclcpp::Subscription<joyMsg>::SharedPtr joy;
    rclcpp::Subscription<pointCloudMsg>::SharedPtr cloud;
} sub_t;

typedef struct{
    rclcpp::Publisher<twistMsg>::SharedPtr joy;
    rclcpp::Publisher<markerArrayMsg>::SharedPtr markers;
} pub_t;

class Controller: public rclcpp::Node{
private:
  sub_t sub;
  pub_t pub;
  twistMsg data;

  void initTopic();
public:
  Controller();
  void commandCallback(const joyMsg msg);
};

#endif