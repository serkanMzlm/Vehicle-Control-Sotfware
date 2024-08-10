#ifndef __MAHONY_ORIENTATION_FILTER_HPP
#define __MAHONY_ORIENTATION_FILTER_HPP

#include "estimator_node_type.hpp"

void mahonyOrientationFilter(IMUData_s imu, float dt);

#endif