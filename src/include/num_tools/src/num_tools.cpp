#include <cmath>
#include <cstdint>

#include "num_tools.hpp"

double mapValue(double data, double in_min,
                double in_max, double out_min, double out_max)
{
  if (in_max == in_min)
  {
    return out_min;
  }

  // Calculate the proportion of 'data' within the input range
  double proportion = (data - in_min) / (in_max - in_min);

  double mapped_value = proportion * (out_max - out_min) + out_min;
  return mapped_value;
}

float constrainValue(float value, const float min_value, const float max_value)
{
  return fminf(max_value, fmaxf(min_value, value));
}

float fastInverseSqrt(float number)
{
  union 
  {
    float f;
    uint32_t i;
  } conv = { .f = number };

  // Step 1: Initial Value and Bit Manipulation
  conv.i = THREEHALFS - (conv.i >> 1);

  // Step 2: Newton-Raphson Iteration
  conv.f *= 1.5F - (number * 0.5F * conv.f * conv.f);
  return conv.f;
}