#ifndef __COMMANDER_TYPES_HPP__
#define __COMMANDER_TYPES_HPP__

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

#include "math_tools/math_operations.hpp"
#include "geometry_tools/geometry_operations.hpp"
#include "geometry_tools/transformation.hpp"

using joyMsg = sensor_msgs::msg::Joy;
using int32Msg = std_msgs::msg::Int32;
using navPathMsg = nav_msgs::msg::Path;
using twistMsg = geometry_msgs::msg::Twist;
using odometryNavMsg = nav_msgs::msg::Odometry;
using pointCloudMsg = sensor_msgs::msg::PointCloud2;
using poseStampedMsg = geometry_msgs::msg::PoseStamped;
using markerArrayMsg = visualization_msgs::msg::MarkerArray;

typedef union
{
    struct
    {
        float x;
        float y;
        float z;
    };
    float pose[3];
} Position_t;

typedef union
{
    struct
    {
        float roll;
        float pitch;
        float yaw;
    };
    float angle[3];
} Orientation_t;

typedef union
{
    struct
    {
        float x;
        float y;
        float z;
        float w;
    };
    float q[4];
} Quaternion_t;

typedef struct
{
    Position_t position;
    Orientation_t orientation;
    Quaternion_t quaternion;
} State_t;

#endif