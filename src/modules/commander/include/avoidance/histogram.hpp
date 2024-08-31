#ifndef __HISTOGRAM_HPP__
#define __HISTOGRAM_HPP__

#include <cmath>
#include <vector>
#include <float.h>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#define THETA_MAX_VALUE 180
#define PHI_MAX_VALUE 380
#define START_ANGLE 0
#define FORCE_OFFSET_ANGLE 20

class Histogram
{
private:
    Eigen::MatrixXf dist_;
    Eigen::MatrixXf histogram;
    float prev_force[2] = {0.0f, 0.0f};    // left - right
    float force[2] = {0.0f, 0.0f};    

public:
    Histogram();
    ~Histogram() = default;
    void setDist(int x, int y, float value);
    float getDist(int x, int y);
    void setHistogram();
    float getHistogramDist(int x, int y);
    void wrapIndex(int &x, int &y);
    void setZero();
    bool isEmpty() const;
    bool isIndexLimit(int theta, int phi) const;
    bool compareForceValues();
    void maskPolarHistogram(int theta, int phi, float radius);
};

#endif