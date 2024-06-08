#include "geometry_utils.hpp"

Eigen::Matrix3f R_x(float phi)
{
    Eigen::Matrix3f rx;
    rx << 1, 0, 0,
        0, cos(phi), -sin(phi),
        0, sin(phi), cos(phi);
    return rx;
}

Eigen::Matrix3f R_y(float beta)
{
    Eigen::Matrix3f ry;
    ry << cos(beta), 0, sin(beta),
        0, 1, 0,
        -sin(beta), 0, cos(beta);
    return ry;
}

Eigen::Matrix3f R_z(float theta)
{
    Eigen::Matrix3f rz;
    rz << cos(theta), -sin(theta), 0,
        sin(theta), cos(theta), 0,
        0, 0, 1;
    return rz;
}

Eigen::Matrix3f calculateRotationMatrix(float angle[])
{
    return R_z(angle[YAW]) * R_y(angle[PITCH]) * R_x(angle[ROLL]);
}

Eigen::Matrix3f calculateRotationMatrix(float roll, float pitch, float yaw)
{
    return R_z(yaw) * R_y(pitch) * R_x(roll);
}

void rotasyonTransformation(float *out[], float angle[])
{
    Eigen::Matrix<float, 3, 1> input;
    Eigen::Matrix<float, 3, 1> result;

    input << *out[0], *out[1], *out[2];
    result = calculateRotationMatrix(angle) * input;

    *out[0] = result(0, 0);
    *out[1] = result(1, 0);
    *out[2] = result(2, 0);
}

void rotasyonTransformation(float *out[], float roll, float pitch, float yaw)
{
    Eigen::Matrix<float, 3, 1> input;
    Eigen::Matrix<float, 3, 1> result;

    input << *out[0], *out[1], *out[2];
    result = calculateRotationMatrix(roll, pitch, yaw) * input;

    *out[0] = result(0, 0);
    *out[1] = result(1, 0);
    *out[2] = result(2, 0);
}

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
        spe_data[THETA] = acosf(cart_data[Z] / spe_data[RADIUS]) * RAD2DEG;
    }
    else
    {
        spe_data[THETA] = 0.0f;
    }
    spe_data[THETA] = static_cast<int>(std::round(spe_data[THETA]));

    // Calculate phi (angle from x-axis in the xy-plane)
    spe_data[PHI] = atan2f(cart_data[Y], cart_data[X]) * RAD2DEG;
    spe_data[PHI] = static_cast<int>(std::round(spe_data[PHI]));
    spe_data[PHI] = constrainAngle(spe_data[PHI], 0);
}

void spherical2Cartesian(float *spe_data, float *cart_data)
{
    // Convert spherical coordinates to Cartesian coordinates
    float theta_rad = spe_data[THETA] * DEG2RAD;
    float phi_rad = spe_data[PHI] * DEG2RAD;

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