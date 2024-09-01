#include "estimator/estimator.hpp"

Estimator::Estimator()
{
    tictoc.tic();
}

void Estimator::imuCallback(const ImuMsg::SharedPtr msg)
{
    imu.gx = msg->angular_velocity.x;
    imu.gy = msg->angular_velocity.y;
    imu.gz = msg->angular_velocity.z;
    imu.ax = normalizeByGravity(msg->linear_acceleration.x);
    imu.ay = normalizeByGravity(msg->linear_acceleration.y);
    imu.az = normalizeByGravity(msg->linear_acceleration.z);
    complementaryFilter(imu, 12.5); // dt -> second
    tictoc.tic();
}

float Estimator::normalizeByGravity(float data)
{
    return data / GRAVITY;
}