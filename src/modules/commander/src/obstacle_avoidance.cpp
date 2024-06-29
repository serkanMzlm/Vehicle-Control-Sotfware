#include "obstacle_avoidance.hpp"

void ObstacleAvoidance::updateVelocity(double &linear_x, double &linear_w)
{
    float error = 0.0f;
    float critical_zone = (safety_distance - vehicle_radius) * 0.8;

    if (linear_x == 0.0f && linear_w == 0.0f)
    {
        return;
    }

    for (int phi = 0; phi < HORIZONTAL; phi++)
    {
        normalized_phi = constrainAngle(phi, -180);
        int front_angle = phi % 10;
        if (histogram[phi] >= lidar_rules[MAX_DIS] || histogram[phi] <= vehicle_radius)
        {
            continue;
        }

        if (((histogram[front_angle] < critical_zone) || (histogram[359 - front_angle] < critical_zone)) && linear_x > 0 )
        {
            linear_x = 0.0f;
            int left_force = 0.0f;
            int right_force = 0.0f;

            for (int i = 60; i < 110; i++)
            {
                left_force += histogram[i];
                right_force += histogram[360 - i];
            }

            linear_w = left_force >= right_force ? 0.5 : -0.5;
        }

        if (((histogram[180 + front_angle] < critical_zone) || (histogram[180 + front_angle] < critical_zone)) && linear_x < 0)
        {
            linear_x = 0.0f;
            int left_force = 0.0f;
            int right_force = 0.0f;

            for (int i = 60; i < 110; i++)
            {
                left_force += histogram[i];
                right_force += histogram[360 - i];
            }

            linear_w = left_force >= right_force ? -0.5 : 0.5;
        }

        else if(((normalized_phi <= vehicle_fov) && (normalized_phi >= -vehicle_fov)) && linear_x > 0)
        {
            error += calculateError(histogram[phi], linear_x, phi);
        }

        if(((normalized_phi >= (180 - vehicle_fov)) || (normalized_phi <= (-180 + vehicle_fov))) && linear_x < 0) 
        {
            error += calculateError(histogram[phi], linear_x, phi);
        }
    }


    if (abs(linear_x) > OFFSET && abs(error) > OFFSET)
    {
        linear_w = error;
        // std::cout << "error: " << error << std::endl;
    }


    linear_x = constrainValue(linear_x, -1.0, 1.0);
    linear_w = constrainValue(linear_w, -0.5, 0.5);

    last_point[LINEAR_V].x = linear_x;

    first_point[ANGULAR_V].x = last_point[LINEAR_V].x;
    last_point[ANGULAR_V].x = last_point[LINEAR_V].x;
    last_point[ANGULAR_V].y = linear_w;

    last_point[RESULT_V].x = last_point[LINEAR_V].x;
    last_point[RESULT_V].y = last_point[ANGULAR_V].y;
}

float ObstacleAvoidance::calculateError(float distance, float velocity, int angle)
{
    float kForce = -0.15;
    float error = kForce * cosf((DEG2RAD(angle))) * sinf((DEG2RAD(angle))) / distance;

    return error;
}

void ObstacleAvoidance::centerData(pointXYZMsg &data)
{
    clearHistogram();
    float center_data[] = {0, 0, 0};
    bool is_empty = true;

    for (size_t i = 0; i < data.size(); i++)
    {
        if (std::isinf(std::abs(data.points[i].x)) ||
            std::isnan(std::abs(data.points[i].x)))
        {
            continue;
        }
        else
        {
            center_data[0] = data.points[i].x;
            center_data[1] = data.points[i].y;
            center_data[2] = data.points[i].z;

            transformation(center_data, angle, lidar_pose);
            polarObstacleDensity(center_data);
            is_empty = false;
        }
    }

    if (is_empty)
    {
        // std::cout << "data is empty.\n";
    }
}

void ObstacleAvoidance::polarObstacleDensity(float *cc_data)
{
    Coordinate_t spherical;
    cartesian2Spherical(cc_data, spherical.pos); // PHI - THETA - RADIUS
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

    if (histogram[spherical.pos[PHI]] > lidar_rules[MAX_DIS])
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
        if (value < lidar_rules[MAX_DIS])
            std::cout << i << ". " << std::fixed << std::setprecision(1) << value << std::endl;
#endif
        i++;
    }
    std::cout << std::endl;
}

void ObstacleAvoidance::printPointCloud(const pointXYZMsg::Ptr &cloud_in)
{
    int i = 0;
    for (const auto &point : *cloud_in)
    {
        std::cout << i << ". x: " << point.x << ", y: " << point.y << ", z: " << point.z << std::endl;
        i++;
    }

    std::cout << std::endl;
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
        std::cout << std::endl;
        cluster_id++;
    }
}