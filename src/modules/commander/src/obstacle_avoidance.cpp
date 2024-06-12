#include "obstacle_avoidance.hpp"

void ObstacleAvoidance::updateVelocity(double &linear_x, double &linear_w)
{
    float error = 0.0f;
    for (int phi = 0; phi < HORIZONTAL; phi++)
    {
        if(histogram[phi] != -1)
            std::cout << phi << ": " << histogram[phi] << std::endl;
        int angle = phi % 15;
        if (histogram[phi] >= lidar_rules[MIN_DIS] || histogram[phi]  <= lidar_rules[MIN_DIS])
        {
            continue;
        }
        if ((histogram[angle] < 0.3f) || (histogram[359 - angle] < 0.3f))
        {
            linear_x = 0.0;
            int left_force = 0.0f;
            int right_force = 0.0f;
            for (int i = 60; i < 110; i++)
            {
                left_force += histogram[i];
                right_force += histogram[360 - i];
            }
            linear_w = left_force >= right_force ? 0.5 : -0.5;
            linear_x = 0.0;
        }
        if (histogram[phi] <= calculateDistance(vehicle_radius, phi))
        {
            continue;
        }
        if (std::abs(linear_x * cosf(DEG2RAD * phi)) > 0)
        {
            // std::cout << error << std::endl;
            error += avoidanceDistance(histogram[phi], phi);
        }
    }
    std::cout << "--------------------" << std::endl;
    if (abs(linear_x) > OFFSET && abs(error) > OFFSET)
    {
        linear_w = error;
    }

    last_point[LINEAR_V].x = linear_x;

    first_point[ANGULAR_V].x = last_point[LINEAR_V].x;
    last_point[ANGULAR_V].x = last_point[LINEAR_V].x;
    last_point[ANGULAR_V].y = linear_w;

    last_point[RESULT_V].x = last_point[LINEAR_V].x;
    last_point[RESULT_V].y = last_point[ANGULAR_V].y;
}

float ObstacleAvoidance::calculateDistance(float distance, int angle)
{   
    float kForce = 1.414213562;
    return distance * (abs(cos(DEG2RAD * angle)) + abs(sin(DEG2RAD * angle))) / kForce;
}

float ObstacleAvoidance::avoidanceDistance(float distance, int angle)
{
    float kForce = -0.1f;
    return kForce * cosf(DEG2RAD * angle) * sinf(DEG2RAD * angle) / distance;
}

void ObstacleAvoidance::detectObject(pointXYZMsg &cloud_data)
{
    pointIndicesMsg cluster_indices;
    pointXYZMsg::Ptr cloud_m(new pointXYZMsg);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    *cloud_m = cloud_data;
    ec.setClusterTolerance(vehicle_dimensions[WIDTH]);
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
            float lidar_pose[3] = {lidar_rules[X_POS], lidar_rules[Y_POS], lidar_rules[Z_POS]};
            float center_data[3] = {cloud_c.points[*pit].x, cloud_c.points[*pit].y, cloud_c.points[*pit].z};
            transformation(center_data, angle, lidar_pose);

            cartesian[X] = center_data[0];
            cartesian[Y] = center_data[1];
            cartesian[Z] = center_data[2];
            polarObstacleDensity(cartesian);
        }
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
    bool flag_angle_limit = spherical.pos[THETA] >= MAX_PHI_ANGLE || spherical.pos[THETA] <= MIN_PHI_ANGLE;
    if (flag_distance_limits || flag_angle_limit)
    {
        return;
    }

    histogram[spherical.pos[PHI]] = spherical.pos[RADIUS];
}

void ObstacleAvoidance::clearHistogram()
{
    histogram.fill(-1.0f);
}