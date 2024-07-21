#include "geometric_operations.hpp"

void eulerToQuaternion(double* euler, double *quaternion)
{
    double roll = euler[0];
    double pitch = euler[1];
    double yaw = euler[2];

    // Calculate the half angles
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    quaternion[0] = sr * cp * cy - cr * sp * sy; // x
    quaternion[1] = cr * sp * cy + sr * cp * sy; // y
    quaternion[2] = cr * cp * sy - sr * sp * cy; // z
    quaternion[3] = cr * cp * cy + sr * sp * sy; // w

}

void quaternionToEuler(double *quaternion, double* euler)
{
    double x = quaternion[0];
    double y = quaternion[1];
    double z = quaternion[2];
    double w = quaternion[3];

    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    euler[0] = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        euler[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler[1] = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    euler[2] = std::atan2(siny_cosp, cosy_cosp);
}

void cartesianToSpherical(float *cart_data, float *spe_data)
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

void sphericalToCartesian(float *spe_data, float *cart_data)
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