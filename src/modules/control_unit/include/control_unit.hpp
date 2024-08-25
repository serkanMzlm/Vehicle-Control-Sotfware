#ifndef __CONTROL_UNIT_HPP__ 
#define __CONTROL_UNIT_HPP__

#include "control_unit_type.hpp"

using namespace std::placeholders;

class ControlUnit
{
protected:
    joyMsg joy_data;
    std::shared_ptr<rclcpp::Node> _node;
    rclcpp::Publisher<joyMsg>::SharedPtr joy_pub;
    Param_t params;

protected:
    virtual bool setup() = 0;
    virtual void initTopic() = 0;
    virtual void publishVelocity() = 0;

public:
    ControlUnit(std::shared_ptr<rclcpp::Node> node, Param_t params)
    : _node(node), params(params) {
        joy_data.axes.resize(4);
        joy_data.buttons.resize(1);
    }
    ~ControlUnit() = default;
};

#endif