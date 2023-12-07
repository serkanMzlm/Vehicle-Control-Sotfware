#include "geometry_utils.hpp"
#include <cmath>

float capAngle(float angle) {
  angle = fmod(angle + 180.0, 360.0);
  while (angle < 0) {
    angle += 360.0;
  }
  return angle - 180.0;
}

float normalizeAngle(float angle){
    while(angle < 0)    { angle = angle + 360; } 
    while(angle >= 360) { angle = angle - 360; }
    return angle;
}

Eigen::Matrix3f R_x(float phi){
    Eigen::Matrix3f rx; 
    rx <<   1, 0, 0,
            0, cos(phi), -sin(phi),
            0, sin(phi), cos(phi);
    return rx;
}

Eigen::Matrix3f R_y(float beta){
    Eigen::Matrix3f ry;
    ry << cos(beta), 0, sin(beta),
          0, 1, 0,
         -sin(beta), 0, cos(beta);
    return ry;
}

Eigen::Matrix3f R_z(float theta){
    Eigen::Matrix3f rz;
    rz << cos(theta), -sin(theta), 0,
          sin(theta), cos(theta), 0,
          0, 0, 1;
    return rz;
}

Eigen::Matrix3f calculateRotationMatrix(float angle[]){
    return R_z(angle[YAW]) * R_y(angle[PITCH]) * R_x(angle[ROLL]);
}

Eigen::Matrix3f calculateRotationMatrix(float roll, float pitch, float yaw){
    return R_z(yaw) * R_y(pitch) * R_x(roll);
}

void rotasyonTransformation(float* out[], float angle[]){
	Eigen::Matrix <float, 3, 1> input;
    Eigen::Matrix <float, 3, 1> result;

	input << *out[0] , *out[1], *out[2];
	result = calculateRotationMatrix(angle) * input;

	*out[0] = result(0, 0);
	*out[1] = result(1, 0);
	*out[2] = result(2, 0);
}

void cartesian2Spherical(float* cart_data, float* spe_data){
    spe_data[RADIUS] = sqrtf(powf(cart_data[X], 2) + powf(cart_data[Y], 2) + powf(cart_data[Z], 2));
    spe_data[THETA] = acosf(cart_data[Z] / spe_data[RADIUS]) * RAD2DEG;
    spe_data[THETA] = static_cast<int>(std::round(spe_data[THETA]));
    spe_data[PHI] = std::atan2(cart_data[Y], cart_data[X]) * RAD2DEG;
    spe_data[PHI] = static_cast<int>(std::round(spe_data[PHI]));
    spe_data[PHI] = normalizeAngle(spe_data[PHI]);

}

void spherical2Cartesian(float* spe_data, float* cart_data){
   cart_data[X] = spe_data[RADIUS] * sin(spe_data[THETA] * DEG2RAD) * cos(spe_data[PHI] * DEG2RAD);
   cart_data[Y] = spe_data[RADIUS] * sin(spe_data[THETA] * DEG2RAD) * sin(spe_data[PHI] * DEG2RAD);
   cart_data[Z] = spe_data[RADIUS] * cos(spe_data[THETA] * DEG2RAD);
}