#include "complementary_filter.hpp"

const float alpha = 0.98f;  //filter coefficient
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

void complementaryFilter(IMUData_s imu, float dt)
{
    if (dt == 0.0f)
    {
        return;
    }

    // float accelRoll = atan2(imu.values.ay, imu.values.az) * RAD_TO_DEG;
    // float accelPitch = atan2(-imu.values.ax, imu.values.az) ;

}

