#ifndef __GEOMETRY_UTILS_HPP__
#define __GEOMETRY_UTILS_HPP__

#include <cmath>
#include <Eigen/Dense>

#define F2P(x) (1000 / x)
#define DEG2RAD (M_PI / 180.0f)
#define RAD2DEG (180.0f / M_PI)

typedef enum
{
    X,
    Y,
    Z,
    ALL_CC
} Cartesian_coordinate_e;

typedef enum
{
    RADIUS,
    THETA,
    PHI,
    ALL_SC
} Cylindrical_coordinate_e;

typedef enum
{
    ROLL,
    PITCH,
    YAW,
    R_ALL
} Rotation_angle_t;

float capAngle(float angle);
float normalizeAngle(float angle);

Eigen::Matrix3f R_x(float phi);
Eigen::Matrix3f R_y(float beta);
Eigen::Matrix3f R_z(float theta);

Eigen::Matrix3f calculateRotationMatrix(float angle[]);
Eigen::Matrix3f calculateRotationMatrix(float roll, float pitch, float yaw);

void rotasyonTransformation(float *out[], float angle[]);

void cartesian2Spherical(float *cart_data, float *spe_data);
void spherical2Cartesian(float *spe_data, float *cart_data);

#endif