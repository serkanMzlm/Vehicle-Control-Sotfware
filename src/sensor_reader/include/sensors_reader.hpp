#ifndef __SENSORS_READER_HPP__
#define __SENSORS_READER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef sensor_msgs::msg::PointCloud2 pointCloudMsg;
typedef sensor_msgs::msg::Imu imuMsg;

using namespace std::placeholders;
class SensorsReader: public rclcpp::Node{
public:
  SensorsReader();
  void calbackPointCloud(pointCloudMsg);
  void calbackImu(imuMsg);
private:
  pointCloudMsg cloud_data;
  imuMsg imu_data;
  rclcpp::Subscription<imuMsg>::SharedPtr imu_sub;
  rclcpp::Subscription<pointCloudMsg>::SharedPtr point_cloud_sub;
  rclcpp::Publisher<pointCloudMsg>::SharedPtr point_cloud_pub;
};
#endif