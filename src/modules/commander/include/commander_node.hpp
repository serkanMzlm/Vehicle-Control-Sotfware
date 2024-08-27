#ifndef __CONTROLLER_NODE_HPP__
#define __CONTROLLER_NODE_HPP__

#include "commander_types.hpp"

typedef struct
{
    pcl::PCLPointCloud2 pcl_cloud;
    pcl::PointCloud<pcl::PointXYZ> pcl_xyz_cloud; 
    twistMsg twist;
    markerArrayMsg marker_array;
    navPathMsg path;
    State_t state;
} Data_t;

typedef struct
{
    rclcpp::Subscription<joyMsg>::SharedPtr joy;
    rclcpp::Subscription<pointCloudMsg>::SharedPtr cloud;
    rclcpp::Subscription<odometryNavMsg>::SharedPtr nav_odom;
} Sub_t;

typedef struct
{
    rclcpp::Publisher<markerArrayMsg>::SharedPtr markers;
    rclcpp::Publisher<navPathMsg>::SharedPtr vehicle_path;
    rclcpp::Publisher<twistMsg>::SharedPtr joy;
} Pub_t;

typedef struct
{
    rclcpp::TimerBase::SharedPtr visual;
} RosTime_t;

class CommanderNode : public rclcpp::Node
{
private:
    Data_t data;
    Sub_t sub;
    Pub_t pub;
    RosTime_t timer_;

    float pre_pose[3] = {0.0, 0.0, 0.0};
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_vehicle;
public:
    CommanderNode();
    void initTopic();
    void printDisplay();
    void declareParameters();
    void joyCallback(const joyMsg::SharedPtr msg);
    void pointCloudCallback(const pointCloudMsg::SharedPtr msg);
    void odometryCallback(const odometryNavMsg::SharedPtr msg);
};

#endif