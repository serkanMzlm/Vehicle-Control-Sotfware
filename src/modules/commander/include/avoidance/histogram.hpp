#ifndef __HISTOGRAM_HPP__
#define __HISTOGRAM_HPP__

#include <cmath>
#include <vector>
#include <float.h>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

class Histogram
{
private:
    Eigen::MatrixXf dist_;
    int resolution;
    int theta;
    int phi;

protected:
    float max_dist, min_dist;
    int max_theta, min_theta;

public:
    Histogram(int res = 1);
    ~Histogram() = default;
    void setDist(int x, int y, float value);
    float getDist(int x, int y);
    void wrapIndex(int &x, int &y);
    int checkResolution(int res);
    void setZero();
    bool isEmpty() const;
    void maskPolarHistogram(int theta, int phi, float radius);
};

#endif