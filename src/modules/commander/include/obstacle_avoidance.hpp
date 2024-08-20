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
    int vehicle_fov = 0;
    pointMsg first_point[VEL_ALL];
    pointMsg last_point[VEL_ALL];
    float angle[3] = {0.0, 0.0, 0.0};
    float lidar_pose[3];
    float normalized_phi = 0.0f;
    double linear_velocity_limit;
    double angular_velocity_limit;

public:
    void clearHistogram();
    void polarObstacleDensity(float *);
    void maskPolarHistogram(Coordinate_t);
    float calculateError(float, int);
    void updateVelocity(double &, double &);
    void centerData(pointXYZMsg &);
    void printHistogram();
    void printPointCloud(const pointXYZMsg::Ptr &);
    void printClusters(const std::vector<pcl::PointIndices>&, const pointXYZMsg::Ptr&);
};

#endif