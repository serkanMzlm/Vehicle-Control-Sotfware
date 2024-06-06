#include "num_tools.hpp"
#include <cmath>

double map(double data, double in_min, double in_max,
           double out_min, double out_max)
{
  return ((((data - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min);
}

float constrain(float value, const float min_value, const float max_value)
{
  return fminf(max_value, fmaxf(min_value, value));
}