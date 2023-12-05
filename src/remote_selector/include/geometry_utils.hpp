#ifndef __GEOMETRY_UTILS_HPP__ 
#define __GEOMETRY_UTILS_HPP__

#include <cmath>
#include "remote_selector_type.hpp"

#define OFFSET_EXCEPTION(X) (abs(X) > OFFSET ? X : 0.0f) 

double map(double data, double in_min, double in_max, double out_min, double out_max);

#endif