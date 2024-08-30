#include "avoidance/histogram.hpp"

#define RESOLUTION 1
#define PHI_VALUE (360 / RESOLOTION)
#define THETA_VALUE (180 / RESOLUTION)

Histogram::Histogram(int res)
{
    resolution = checkResolution(res);
    theta = 180 / resolution;
    phi = 360 / resolution;
    dist_.resize(theta, phi);
}

void Histogram::setDist(int x, int y, float value)
{
    wrapIndex(x, y);
    dist_(x, y) = value;
}

float Histogram::getDist(int x, int y)
{
    wrapIndex(x, y);
    return dist_(x, y);
}

void Histogram::wrapIndex(int &x, int &y)
{
    x = x % theta;
    if (x < 0)
    {
        x += theta;
    }

    y = y % phi;
    if (y < 0)
    {
        y += phi;
    }
}

int Histogram::checkResolution(int res)
{
    if (res == 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("hitogram"), "Resolution 0 cannot be entered");
        return 1;
    }

    if (180 % res != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("hitogram"), "The number entered must be divisible by 180 and 360");
        return 1;
    }

    return res;
}

void Histogram::setZero()
{
    dist_.fill(0.f);
}

bool Histogram::isEmpty() const
{
    for (int i = 0; (i < theta); i++)
    {
        for (int j = 0; (j < phi); j++)
        {
            if (dist_(i, j) > FLT_MIN)
            {
                return false;
            }
        }
    }
    return true;
}

void Histogram::maskPolarHistogram(int theta, int phi, float radius)
{
    if (theta > max_theta || theta < min_theta)
    {
        return;
    }

    if (getDist(theta, phi) <= min_dist)
    {
        setDist(theta, phi, radius);
    }
    else
    {
        setDist(theta, phi, fminf(getDist(theta, phi), radius));
    }
    
}