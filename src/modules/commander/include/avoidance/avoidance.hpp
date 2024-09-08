#ifndef __AVOIDANCE_HPP__
#define __AVOIDANCE_HPP__

#include "avoidance/histogram.hpp"
#include "commander_types.hpp"

class Avoidance : public Histogram
{
private:
    float sensor_pose[3] = {0.0, 0.0, 0.0};
    float sensor_orientation[3] = {0.0, 0.0, 0.0};
    float error = 0.0f;
    float critical_zone;
    int direction_of_movement = 1; // 1 forward, -1 back
    laserScanMsg

    rclcpp::Logger avoidance_logger = rclcpp::get_logger("avoidance");

public:
    std::vector<double> sensor_rules;
    std::vector<double> vehicle_dimensions;
    float vehicle_radius;
    float safety_dist;
    float dist_limit;
    float linear_limit;
    float angular_limit;
    float fov;

public:
    Avoidance();
    ~Avoidance() = default;

    void init();
    void calculateAvoidanceRules();
    void updateHistogram(pointXYZMsg &data);
    void dataCenter(float x, float y, float z);
    void polarObstacleDensity(float *pose);
    void updateVelocity(double &linear_x, double &angular_z);
    float calculateError(float dist, int phi);
};

#endif