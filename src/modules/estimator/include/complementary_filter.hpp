#ifndef __COMPLEMENTRAY_FILTER__
#define __COMPLEMENTRAY_FILTER__

#include <cmath>

#include "num_tools.hpp"
#include "estimator_node_type.hpp"
#include "geometric_operations.hpp"

void complementaryFilter(IMUData_s imu, float dt);

#endif