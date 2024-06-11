#ifndef __GEOMETRIC_OPERATIONS_TYPE_HPP__
#define __GEOMETRIC_OPERATIONS_TYPE_HPP__


typedef enum
{
    X,
    Y,
    Z,
    CART_COORD_ALL
} Cartesian_coordinate_e;

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
    ROT_ANGLE_ALL
} Rotation_angle_t;

#endif