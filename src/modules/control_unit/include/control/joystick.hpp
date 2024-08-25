#ifndef __JOYSTICK_CONTROL__ 
#define __JOYSTICK_CONTROL__ 

#include "control_unit.hpp"

class JoystickControl: public ControlUnit
{
private:
    bool setup();
    void initTopic();
    void publishVelocity();

    rclcpp::Subscription<joyMsg>::SharedPtr joy_sub;

public:
    JoystickControl(std::shared_ptr<rclcpp::Node> node, Param_t params);
    ~JoystickControl() = default;
    void joyCallback(const joyMsg::SharedPtr msg);
};

#endif