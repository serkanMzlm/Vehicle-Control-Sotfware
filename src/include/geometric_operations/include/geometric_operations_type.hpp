#ifndef __GEOMETRIC_OPERATIONS_TYPE_HPP__
#define __GEOMETRIC_OPERATIONS_TYPE_HPP__

typedef enum
{
    X,
    Y,
    Z,
    W,
    QUATERNION_ALL
} Quaternion_e;

typedef enum
{
    RADIUS,
    THETA,
    PHI,
    CYL_COORD_ALL
} Cylindrical_coordinate_e;

typedef enum
{
    ROLL,
    PITCH,
    YAW,
    EULER_ALL
} Euler_e;

#endif