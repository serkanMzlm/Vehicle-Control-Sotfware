#ifndef __ESTIMATO_NODE_TYPE_HPP__
#define __ESTIMATO_NODE_TYPE_HPP__

#define GRAVITY 9.8f

typedef enum
{
    AXIS_X,
    AXIS_Y,
    AXIS_Z,
    ALL_AXIS
} Axis_e;

typedef struct
{
    float ax, ay, az; // Acc data
    float gx, gy, gz; // Gyro data
} IMUData_s;

#endif