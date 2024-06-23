#include "obstacle_avoidance.hpp"

void ObstacleAvoidance::updateVelocity(double &linear_x, double &linear_w)
{
    float error = 0.0f;
    float critical_zone = (safety_distance - vehicle_radius);
    printHistogram();
    // if (linear_x == 0.0f && linear_w == 0.0f)
    // {
        // return;
    // }

    // for (int phi = 0; phi < HORIZONTAL; phi++)
    // {
    //     int angle = phi % 10;

    //     if(histogram[phi] >= lidar_rules[MAX_DIS] || histogram[phi] <= vehicle_radius)
    //     {
    //         continue;
    //     }

    //     // printHistogram();
    //     if ((histogram[angle] < critical_zone) || (histogram[359 - angle] < critical_zone))
    //     {
    //         std::cout << histogram[angle] << "   " << histogram[359 - angle] << "  " << critical_zone << std::endl;
    //         linear_x = 0.0;
    //         int left_force = 0.0f;
    //         int right_force = 0.0f;

    //         for (int i = 60; i < 110; i++)
    //         {
    //             left_force += histogram[i];
    //             right_force += histogram[360 - i];
    //         }
    //         linear_w = left_force >= right_force ? 0.5 : -0.5;
    //     }

    //     if (std::abs(linear_x * cosf(DEG2RAD(phi))) > 0)
    //     {
    //         // std::cout << error << std::endl;
    //         // error += avoidanceDistance(histogram[phi], phi);
    //     }
    // }
    // if (abs(linear_x) > OFFSET && abs(error) > OFFSET)
    // {
    //     linear_w = error;
    // }

    last_point[LINEAR_V].x = linear_x;

    first_point[ANGULAR_V].x = last_point[LINEAR_V].x;
    last_point[ANGULAR_V].x = last_point[LINEAR_V].x;
    last_point[ANGULAR_V].y = linear_w;

    last_point[RESULT_V].x = last_point[LINEAR_V].x;
    last_point[RESULT_V].y = last_point[ANGULAR_V].y;
}

float ObstacleAvoidance::calculateError(float distance, float setpoint, int angle)
{

    return 0.0f;
    // float kForce = 1.414213562;
    // return distance * (abs(cos(DEG2RAD(angle))) + abs(sin(DEG2RAD(angle)))) / kForce;
    //    float kForce = -0.1f;
    // return kForce * cosf(DEG2RAD(angle)) * sinf(DEG2RAD(angle)) / distance;
}

void ObstacleAvoidance::detectObject(pointXYZMsg &cloud_data)
{
    pointXYZMsg::Ptr cloud_m(new pointXYZMsg);
    *cloud_m = cloud_data;

    pointIndicesMsg cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance(vehicle_radius);
    tree->setInputCloud(cloud_m);
    ec.setMinClusterSize(2);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_m);
    ec.extract(cluster_indices);
    getClusterPoint(cluster_indices, cloud_data);
}

void ObstacleAvoidance::getClusterPoint(pointIndicesMsg &indices_c, pointXYZMsg &cloud_c)
{
    clearHistogram();
    float cartesian[] = {0, 0, 0};
    for (pointIndicesMsg::const_iterator it = indices_c.begin(); it != indices_c.end(); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            float angle[3] = {0.0, 0.0, 0.0};
            float lidar_pose[3] = {(float)lidar_rules[X_POS], (float)lidar_rules[Y_POS], (float)lidar_rules[Z_POS]};
            float center_data[3] = {cloud_c.points[*pit].x, cloud_c.points[*pit].y, cloud_c.points[*pit].z};
            transformation(center_data, angle, lidar_pose);

            // cartesian[X] = cloud_c.points[*pit].x;
            // cartesian[Y] = cloud_c.points[*pit].y;
            // cartesian[Z] = cloud_c.points[*pit].z;

            cartesian[X] = center_data[0];
            cartesian[Y] = center_data[1];
            cartesian[Z] = center_data[2];

            polarObstacleDensity(cartesian);
        }
    }
    // printHistogram();
}

void ObstacleAvoidance::polarObstacleDensity(float *cc_data)
{
    Coordinate_t spherical;
    cartesian2Spherical(cc_data, spherical.pos); // PHI - THETA - RADIUS
    // std::cout << "cc: " << cc_data[X] << " - " << cc_data[Y] << " - " << cc_data[Z] << std::endl;
    // std::cout << "sc: " << spherical.pos[THETA] << " - " << spherical.pos[PHI] << " - " << spherical.pos[RADIUS] << std::endl;
    maskPolarHistogram(spherical);
}

void ObstacleAvoidance::maskPolarHistogram(Coordinate_t spherical)
{
    bool flag_distance_limits = spherical.pos[RADIUS] <= lidar_rules[MIN_DIS] || spherical.pos[RADIUS] >= lidar_rules[MAX_DIS];
    bool flag_angle_limit = spherical.pos[THETA] >= MAX_THETA_ANGLE || spherical.pos[THETA] <= MIN_THETA_ANGLE;
    if (flag_distance_limits || flag_angle_limit)
    {
        return;
    }

    if(histogram[spherical.pos[PHI]] > lidar_rules[MAX_DIS])
    {
        histogram[spherical.pos[PHI]] = spherical.pos[RADIUS];
    }
    else
    {
        histogram[spherical.pos[PHI]] = fminf(spherical.pos[RADIUS], histogram[spherical.pos[PHI]]);
    }
}

void ObstacleAvoidance::clearHistogram()
{
    histogram.fill(lidar_rules[MAX_DIS] + OFFSET);
}

void ObstacleAvoidance::printHistogram()
{
    int i = 0;
    for (const auto &value : histogram)
    {
#ifdef FULL_HISTOGRAM
        if ((i % 30) == 0)
        {
            std::cout << std::endl;
        }
        std::cout << std::fixed << std::setprecision(1) << (value == -1 ? 0.0 : value) << " ";
#else
    // if(value < lidar_rules[MAX_DIS])
    std::cout << i << ". " <<  std::fixed << std::setprecision(1) << value << std::endl;
#endif
        i++;
    }
    std::cout << std::endl << std::endl;
}

void ObstacleAvoidance::printPointCloud(const pointXYZMsg::Ptr &cloud_in)
{
    int i = 0;
    for (const auto &point : *cloud_in)
    {
        std::cout << i << ". x: " << point.x << ", y: " << point.y << ", z: " << point.z << std::endl;
        i++;
    }

    std::cout << std::endl << std::endl;
}

void ObstacleAvoidance::printClusters(const std::vector<pcl::PointIndices> &cluster_indices, const pointXYZMsg::Ptr &cloud)
{
    std::cout << "Clusters:" << std::endl;
    int cluster_id = 0;
    for (const auto &indices : cluster_indices)
    {
        std::cout << "Cluster " << cluster_id << ": ";
        for (const auto &index : indices.indices)
        {
#ifndef DEBUG_POINT
            std::cout << index << ", ";
#else 
            std::cout << "  x: " << cloud->points[index].x
                      << ", y: " << cloud->points[index].y
                      << ", z: " << cloud->points[index].z << std::endl;
#endif
        }
        std::cout << std::endl << std::endl;
        cluster_id++;
    }
}