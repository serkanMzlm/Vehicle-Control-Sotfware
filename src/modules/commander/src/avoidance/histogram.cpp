#include "avoidance/histogram.hpp"

Histogram::Histogram()
{
    dist_.resize(THETA_MAX_VALUE + 1, PHI_MAX_VALUE);
    setZero();
    setHistogram();
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

void Histogram::setHistogram()
{
    histogram = dist_;
    prev_force[0] = force[0];
    prev_force[1] = force[1];
}

float Histogram::getHistogramDist(int x, int y)
{
    wrapIndex(x, y);
    return histogram(x, y);
}

void Histogram::wrapIndex(int &x, int &y)
{
    x = x % THETA_MAX_VALUE;
    if (x < 0) x += THETA_MAX_VALUE;

    y = y % PHI_MAX_VALUE;
    if (y < 0) y += PHI_MAX_VALUE;
}

void Histogram::setZero()
{
    dist_.fill(0.f);
    force[0] = 0.0f;
    force[1] = 0.0f;
}

bool Histogram::isEmpty() const
{
    for (int i = 0; (i < THETA_MAX_VALUE); i++)
    {
        for (int j = 0; (j < PHI_MAX_VALUE); j++)
        {
            if (histogram(i, j) > FLT_MIN)
            {
                return false;
            }
        }
    }
    return true;
}

bool Histogram::isIndexLimit(int theta, int phi) const
{
    if(theta > THETA_MAX_VALUE || theta < START_ANGLE)
    {
        return false;
    }

    if(phi >= PHI_MAX_VALUE || phi < START_ANGLE)
    {
        return false;
    }

    return true;
}

bool Histogram::compareForceValues()
{
    return prev_force[0] > prev_force[1];
}

void Histogram::maskPolarHistogram(int theta, int phi, float radius)
{
    if (!isIndexLimit(theta, phi))
    {
        return;
    }
    if (getDist(theta, phi) <= 0.0f)
    {
        setDist(theta, phi, radius);
    }
    else
    {
        setDist(theta, phi, fminf(getDist(theta, phi), radius));
    }

    if(phi > (90 - FORCE_OFFSET_ANGLE) && phi < (90 + FORCE_OFFSET_ANGLE))
    {
        force[0] += getDist(theta, phi);
    }

    if(phi > (270 - FORCE_OFFSET_ANGLE) && phi < (270 + FORCE_OFFSET_ANGLE))
    {
        force[1] += getDist(theta, phi);
    }
}