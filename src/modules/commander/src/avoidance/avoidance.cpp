#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include "avoidance/avoidance.hpp"
#include "geometry_tools/transformation.hpp"
#include "math_tools/math_operations.hpp"

Avoidance::Avoidance() : Histogram()
{
}

void Avoidance::init()
{
    sensor_pose[0] = static_cast<float>(sensor_rules[X_POS]);
    sensor_pose[1] = static_cast<float>(sensor_rules[Y_POS]);
    sensor_pose[2] = static_cast<float>(sensor_rules[Z_POS]);
    calculateAvoidanceRules();
}

void Avoidance::calculateAvoidanceRules()
{
    float x = powf((vehicle_dimensions[WIDTH] / 2.0f), 2);
    float y = powf((vehicle_dimensions[LENGTH] / 2.0f), 2);
    vehicle_radius = sqrtf(x + y);

    dist_limit = sensor_rules[MAX_DIS] - OFFSET;
    safety_dist = dist_limit - vehicle_radius;
    critical_zone = (safety_dist - vehicle_radius) * 0.8;
    fov = static_cast<int>(2 * RAD2DEG(atan2(vehicle_dimensions[WIDTH], vehicle_dimensions[LENGTH])));
}

void Avoidance::updateHistogram(pointXYZMsg &data)
{
    setZero();

    for (size_t i = 0; i < data.size(); i++)
    {
        if (std::isinf(std::abs(data.points[i].x)) || std::isnan(std::abs(data.points[i].x)))
        {
            continue;
        }

        dataCenter(data.points[i].x, data.points[i].y, data.points[i].z);
    }

    setHistogram();
}

void Avoidance::dataCenter(float x, float y, float z)
{
    static float pose[3];
    pose[0] = x;
    pose[1] = y;
    pose[2] = z;

    transformation(pose, sensor_orientation, sensor_pose);
    polarObstacleDensity(pose);
}

void Avoidance::polarObstacleDensity(float *pose)
{
    Position_t spherical;
    cartesianToSpherical(pose, spherical.pose);

    if (spherical.pose[RADIUS] > dist_limit || spherical.pose[RADIUS] <= vehicle_radius)
    {
        return;
    }

    maskPolarHistogram(static_cast<int>(spherical.pose[THETA]),
                       static_cast<int>(spherical.pose[PHI]),
                       spherical.pose[RADIUS]);
}

void Avoidance::updateVelocity(double &linear_x, double &angular_z)
{
    if (isEmpty())
    {
        return;
    }

    if (linear_x > 0)
        direction_of_movement = 1;
    if (linear_x < 0)
        direction_of_movement = -1;

    for (int i = 0; i < static_cast<int>(fov); i++)
    {
        int phi_ = 0;
        if (direction_of_movement == 1)
        {
            phi_ = wrapAngleTo360(i - (static_cast<int>(fov) / 2));
        }
        else
        {
            phi_ = wrapAngleTo360(i + (180 - (static_cast<int>(fov) / 2)));
        }

        for (int j = 80; j < 100; j++)
        {
            if (getHistogramDist(j, phi_) > dist_limit || getHistogramDist(j, phi_) <= vehicle_radius)
            {
                continue;
            }

            if (getHistogramDist(j, phi_) < critical_zone && linear_x != 0.0f)
            {
                linear_x = 0.0f;
                angular_z = compareForceValues() ? -angular_limit * direction_of_movement : angular_limit * direction_of_movement;
            }

            error += calculateError(getHistogramDist(j, phi_), phi_);
        }
    }

    if (abs(linear_x) > OFFSET && abs(error) > OFFSET)
    {
        angular_z = error;
    }

    linear_x = constrain(linear_x, -linear_limit, linear_limit);
    angular_z = constrain(angular_z, -angular_limit, angular_limit);

    error = 0.0f;
}

float Avoidance::calculateError(float distance, int angle)
{
    float kForce = -0.13;
    float error = kForce * cosf((DEG2RAD(angle))) * sinf((DEG2RAD(angle))) / distance;

    return error;
}
