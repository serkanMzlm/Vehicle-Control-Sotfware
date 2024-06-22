#include "geometric_operations.hpp"

void cartesian2Spherical(float *cart_data, float *spe_data)
{
    // Calculate radius
    float x2 = powf(cart_data[X], 2);
    float y2 = powf(cart_data[Y], 2);
    float z2 = powf(cart_data[Z], 2);
    spe_data[RADIUS] = sqrtf(x2 + y2 + z2);

    // Calculate theta (angle from z-axis)
    if (spe_data[RADIUS] != 0.0f)
    {
        spe_data[THETA] = RAD2DEG(acosf(cart_data[Z] / spe_data[RADIUS]));
    }
    else
    {
        spe_data[THETA] = 0.0f;
    }
    spe_data[THETA] = static_cast<int>(std::round(spe_data[THETA]));

    // Calculate phi (angle from x-axis in the xy-plane)
    spe_data[PHI] = RAD2DEG(atan2f(cart_data[Y], cart_data[X]));
    spe_data[PHI] = static_cast<int>(std::round(spe_data[PHI]));
    spe_data[PHI] = constrainAngle(spe_data[PHI], 0);
}

void spherical2Cartesian(float *spe_data, float *cart_data)
{
    // Convert spherical coordinates to Cartesian coordinates
    float theta_rad = DEG2RAD(spe_data[THETA]);
    float phi_rad = DEG2RAD(spe_data[PHI]);

    cart_data[X] = spe_data[RADIUS] * sin(theta_rad) * cos(phi_rad);
    cart_data[Y] = spe_data[RADIUS] * sin(theta_rad) * sin(phi_rad);
    cart_data[Z] = spe_data[RADIUS] * cos(theta_rad);
}

float constrainAngle(float angle, float min_angle)
{
    if (min_angle != 0.0f && min_angle != -180.0f)
    {
        min_angle = 0.0f;
    }

    angle = fmod(angle - min_angle, 360);

    if (angle < 0)
    {
        angle += 360;
    }

    return angle + min_angle;
}