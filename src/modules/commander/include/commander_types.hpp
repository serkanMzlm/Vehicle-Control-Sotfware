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
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "math_tools/math_operations.hpp"
#include "geometry_tools/geometry_operations.hpp"
#include "geometry_tools/transformation.hpp"

#include <pcl_conversions/pcl_conversions.h>

#define HORIZONTAL 360  // phi
#define VERTICAL 181    // theta
#define OFFSET 0.1f

using joyMsg = sensor_msgs::msg::Joy;
using int32Msg = std_msgs::msg::Int32;
using navPathMsg = nav_msgs::msg::Path;
using twistMsg = geometry_msgs::msg::Twist;
using odometryNavMsg = nav_msgs::msg::Odometry;
using pointCloudMsg = sensor_msgs::msg::PointCloud2;
using poseStampedMsg = geometry_msgs::msg::PoseStamped;
using markerArrayMsg = visualization_msgs::msg::MarkerArray;

typedef enum
{
  MAX_DIS,
  MIN_DIS,
  X_POS,
  Y_POS,
  Z_POS,
  SENSOR_RULES_ALL
} SensorRules_e;

typedef enum
{
  WIDTH,
  LENGTH,
  HEIGHT,
  VEC_DIM_ALL
} VehicleDimensions_s;

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

typedef struct
{
    float linear_limit;
    float angular_limit;
    float veh_radius;
    float fov;
    float safety_dist;
    float dist_limit;
    float sensor_pose[3];
} ObstacleData_t;

typedef struct
{
    pcl::PCLPointCloud2 pcl_cloud;
    pcl::PointCloud<pcl::PointXYZ> pcl_xyz_cloud; 
    twistMsg twist;
    markerArrayMsg marker_array;
    navPathMsg path;
    State_t state;
} Data_t;

#endif