#ifndef __CONTROLLER_TYPE_HPP__
#define __CONTROLLER_TYPE_HPP__

#include <cmath>
#include "geometric_operations.hpp"


#define VERTICAL 181    // theta
#define HORIZONTAL 360  // phi
#define OFFSET 0.1f

#define ANGLE_OFFSET 5
#define MAX_PHI_ANGLE (90 + ANGLE_OFFSET)
#define MIN_PHI_ANGLE (90 - ANGLE_OFFSET)

#define ERROR (-1)
#define OK (1)

#define DETECT_RANGE(X) abs(std::pow((cosf(DEG2RAD * X)), 3))
#define OFFSET_EXCEPTION(X) (abs(X) > OFFSET ? X : 0.0f) 

typedef enum
{
  LINEAR_V,
  ANGULAR_V,
  RESULT_V,
  VEL_ALL
} Velocity_e;

typedef enum
{
  WIDTH,
  LENGTH,
  HEIGHT,
  VEC_DIM_ALL
} Vehicle_Dimensions_s;

typedef enum
{
  MAX_DIS,
  MIN_DIS,
  X_POS,
  Y_POS,
  Z_POS,
  SENSOR_RULES_ALL
} Sensor_rules_e;

typedef struct
{
  union
  {
    struct
    {
      float x;
      float y;
      float z;
    };
    struct
    {
      float radius;
      float theta;
      float phi;
    };
    float pos[3];
  };
} Coordinate_t;

typedef union
{
  struct
  {
    float linear;
    float angular;
    float result;
  };
  float data[VEL_ALL];
} Joy_t;

#endif