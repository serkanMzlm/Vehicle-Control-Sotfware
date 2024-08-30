#include "avoidance/avoidance.hpp"
#include "geometry_tools/transformation.hpp"

Avoidance::Avoidance(int res) : Histogram(res)
{
    setLimit();
}

void Avoidance::setLimit()
{
    max_theta = 120;
    min_theta = 60;
    max_dist = 4.0f;
    min_dist = 0.2f;
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
}

void Avoidance::setSensorState(float state[3])
{
    sensor_pose[0] = state[0];
    sensor_pose[1] = state[1];
    sensor_pose[2] = state[2];
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
    maskPolarHistogram(static_cast<int>(spherical.pose[THETA]),
                       static_cast<int>(spherical.pose[PHI]),
                       spherical.pose[RADIUS]);
}


