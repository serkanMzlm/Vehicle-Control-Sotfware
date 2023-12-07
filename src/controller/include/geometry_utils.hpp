#ifndef __GEOMETRY_UTILS_HPP__
#define __GEOMETRY_UTILS_HPP__

#include "controller_type.hpp"
#include <Eigen/Dense>


float capAngle(float angle);
float normalizeAngle(float angle);

Eigen::Matrix3f R_x(float phi);
Eigen::Matrix3f R_y(float beta);
Eigen::Matrix3f R_z(float theta);

Eigen::Matrix3f calculateRotationMatrix(float angle[]);
Eigen::Matrix3f calculateRotationMatrix(float roll, float pitch, float yaw);

void rotasyonTransformation(float* out[], float angle[]);

void cartesian2Spherical(float* cart_data, float* spe_data);
void spherical2Cartesian(float* spe_data, float* cart_data);

#endif