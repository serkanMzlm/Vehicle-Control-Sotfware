#ifndef __OBSTACLE_AVOIDANCE_HPP__
#define __OBSTACLE_AVOIDANCE_HPP__

#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "geometry_msgs/msg/point.hpp"

#include "commander_type.hpp"


using markerMsg = visualization_msgs::msg::Marker;
using pointXYZMsg = pcl::PointCloud<pcl::PointXYZ>;
using pointMsg = geometry_msgs::msg::Point;
using pointIndicesMsg = std::vector<pcl::PointIndices>;

typedef struct
{
    pcl::PCLPointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZ> xyz_cloud; 
} pcl_t;

class ObstacleAvoidance
{
private:
    std::array<float, HORIZONTAL> histogram;

protected:
    std::vector<double> vehicle_dimensions;
    std::vector<double> lidar_rules;
    float vehicle_radius = 0.0f;
    float safety_distance = 0.0f;
    float distance_limits = 0.0f;
    pointMsg first_point[VEL_ALL];
    pointMsg last_point[VEL_ALL];

public:
    void clearHistogram();
    void polarObstacleDensity(float *);
    void maskPolarHistogram(Coordinate_t);
    void detectObject(pointXYZMsg &);
    void filterPointCloud(const pointXYZMsg::Ptr &);
    void printPointCloud(const pointXYZMsg::Ptr &);
    float calculateDistance(float, int);
    float avoidanceDistance(float, int);
    void updateVelocity(double &, double &);
    void getClusterPoint(pointIndicesMsg &, pointXYZMsg &);
};

#endif