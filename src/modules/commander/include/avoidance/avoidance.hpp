#ifndef __AVOIDANCE_HPP__
#define __AVOIDANCE_HPP__

#include "avoidance/histogram.hpp"
#include "commander_types.hpp"

class Avoidance : public Histogram
{
private:
    float sensor_pose[3] = {0.0, 0.0, 0.0};
    float sensor_orientation[3] = {0.0, 0.0, 0.0};

public:
    Avoidance(int res = 1);
    ~Avoidance() = default;
    void setLimit();
    void updateHistogram(pointXYZMsg &data);
    void setSensorState(float state[3]);
    void dataCenter(float x, float y, float z);
    void polarObstacleDensity(float *pose);
};

#endif