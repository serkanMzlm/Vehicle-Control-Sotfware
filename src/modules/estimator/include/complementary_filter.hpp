#ifndef __COMPLEMENTRAY_FILTER__
#define __COMPLEMENTRAY_FILTER__

#include <cmath>

#include "num_tools.hpp"
#include "estimator_node_type.hpp"
#include "geometric_operations.hpp"
#include "filters.hpp"

void complementaryFilter(IMUData_s imu, float dt);
void accelToEuler(IMUData_s imu);
void gyroToEuler(IMUData_s imu, float dt);

#endif