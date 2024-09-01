#ifndef __CONTROLLER_NODE_HPP__
#define __CONTROLLER_NODE_HPP__

#include "avoidance/avoidance.hpp"
#include "commander_types.hpp"
#include "commander_visualization.hpp"

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
    rclcpp::Publisher<pointCloudMsg>::SharedPtr cloud;
} Pub_t;

typedef struct
{
    rclcpp::TimerBase::SharedPtr visual;
} RosTime_t;

class CommanderNode : public rclcpp::Node
{
private:
    State_t state;
    Sub_t sub;
    Pub_t pub;
    RosTime_t timer_;

    pcl::PCLPointCloud2 pcl_pc;
    pointXYZMsg pcl_xyz_pc;
    pointCloudMsg ros_pc;

    twistMsg velocity;
    markerArrayMsg marker_array;
    navPathMsg vehicle_path;

    std::string frame_id;
    std::shared_ptr<Avoidance> avoidance;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_vehicle;

public:
    CommanderNode();
    void initTopic();
    void printDisplay();
    void declareParameters();

    void joyCallback(const joyMsg::SharedPtr msg);
    void pointCloudCallback(const pointCloudMsg::SharedPtr msg);
    void odometryCallback(const odometryNavMsg::SharedPtr msg);

    void visualization();
};

#endif