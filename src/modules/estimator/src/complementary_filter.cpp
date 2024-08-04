#include "complementary_filter.hpp"

const float alpha = 0.98f;  //filter coefficient
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

float gyro_roll = 0.0f;
float gyro_pitch = 0.0f;
float gyro_yaw = 0.0f;

float prev_phi = 0.0;    // Roll
float prev_theta = 0.0;  // Pitch

void complementaryFilter(IMUData_s imu, float dt)
{
    if (dt == 0.0f)
    {
        return;
    }

    accelToEuler(imu);
    gyroToEuler(imu, dt);

    roll = alpha * gyro_roll + (1.0f - alpha) * prev_phi;
    pitch = alpha * gyro_pitch + (1.0f - alpha) * prev_theta;
    
}

void accelToEuler(IMUData_s data)
{
    float measure_theta = atan2(data.ax,data.az) * RAD_TO_DEG;
    float measure_phi = atan2(data.ay, data.az) * RAD_TO_DEG;

    float current_theta = lowPassFilter(measure_theta, prev_theta, 0.75);
    float current_phi = lowPassFilter(measure_phi, prev_phi, 0.75);

    prev_phi = current_phi;
    prev_theta = current_theta;

    // RCLCPP_INFO_STREAM(this->get_logger(), "Pitch: " << static_cast<int>(current_theta) 
    //                                     << " Roll: " << static_cast<int>(current_phi));
}

void gyroToEuler(IMUData_s imu, float dt)
{
    gyro_roll += imu.gx * dt;
    gyro_pitch += imu.gy * dt;
    gyro_yaw += imu.gz * dt;
    // RCLCPP_INFO_STREAM(this->get_logger(), "Pitch: " << static_cast<int>(theta_g) 
    //                                     << " Roll: " << static_cast<int>(phi_g));
}